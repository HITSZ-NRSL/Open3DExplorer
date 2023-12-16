/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <tf/tf.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>


#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

// For TF
#include"../../../include/Converter.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

#define pub_debug_image 1

ros::Publisher map_points_publisher_;
ros::Publisher pose_publisher_;
ros::Publisher gps_pose_publisher_;
ros::Publisher path_publisher_;
ros::Publisher gps_path_publisher_;
nav_msgs::Path orb_path_;
nav_msgs::Path gps_path_;
image_transport::Publisher rendered_image_publisher_;
ros::Subscriber gps_sub_;

// image_transport::ImageTransport image_transport_;


ros::Time current_frame_time_;
std::string map_frame_id_param_ = "world";
std::string camera_frame_id_param_ = "camera_link";
// std::string camera_frame_id_param_ = "d435i_link";    // for active_localization dataset
bool publish_tf_param_ = true;
bool publish_pose_param_ = true;
bool publish_pointcloud_param_ = true;

int min_observations_per_point_ = 2;


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){lastStamp = ros::Time::now(); frameCounter = 0;}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;

private:
    ros::Time lastStamp;
    int frameCounter;
};

void GPSCallback (const nav_msgs::OdometryConstPtr& gps_);
ORB_SLAM2::System* SLAM;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    SLAM = new ORB_SLAM2::System(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    
    // message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/d435i/aligned_depth_to_color/image_raw", 1);
    // message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/d435i/color/image_raw", 1);
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    // add pub topics
#if pub_debug_image
    image_transport::ImageTransport image_transport (nh);
    rendered_image_publisher_ = image_transport.advertise ("/orb_slam2_rgbd/debug_image", 1);
#endif

    map_points_publisher_ = nh.advertise<sensor_msgs::PointCloud2> ("/orb_slam2_rgbd/map_points", 1);
    pose_publisher_ = nh.advertise<geometry_msgs::PoseStamped> ("/orb_slam2_rgbd/pose", 1);
    path_publisher_ = nh.advertise<nav_msgs::Path> ("/orb_slam2_rgbd/path", 1);

    gps_pose_publisher_ = nh.advertise<geometry_msgs::PoseStamped> ("/ground_truth/camera_state/pose", 1);
    gps_sub_ = nh.subscribe<nav_msgs::Odometry>("/ground_truth/camera_state", 10, &GPSCallback);
    // gps_sub_ = nh.subscribe<nav_msgs::Odometry>("/novatel/oem7/odom", 10, &GPSCallback);
    gps_path_publisher_ = nh.advertise<nav_msgs::Path> ("/orb_slam2_rgbd/gps_path", 1);


    ros::spin();

    // Stop all threads
    SLAM->Shutdown();

    // Save camera trajectory
    SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}


sensor_msgs::PointCloud2 MapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> map_points) {
  if (map_points.size() == 0) {
    std::cout << "Map point vector is empty!" << std::endl;
  }

  sensor_msgs::PointCloud2 cloud;

  const int num_channels = 3; // x y z

  cloud.header.stamp = current_frame_time_;
  cloud.header.frame_id = map_frame_id_param_;
  cloud.height = 1;
  cloud.width = map_points.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = { "x", "y", "z"};
  for (int i = 0; i<num_channels; i++) {
  	cloud.fields[i].name = channel_id[i];
  	cloud.fields[i].offset = i * sizeof(float);
  	cloud.fields[i].count = 1;
  	cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

	unsigned char *cloud_data_ptr = &(cloud.data[0]);

  float data_array[num_channels];
  for (unsigned int i=0; i<cloud.width; i++) {
    if (map_points.at(i)->nObs >= min_observations_per_point_) {
      data_array[0] = map_points.at(i)->GetWorldPos().at<float> (2); //x. Do the transformation by just reading at the position of z instead of x
      data_array[1] = -1.0* map_points.at(i)->GetWorldPos().at<float> (0); //y. Do the transformation by just reading at the position of x instead of y
      data_array[2] = -1.0* map_points.at(i)->GetWorldPos().at<float> (1); //z. Do the transformation by just reading at the position of y instead of z
      //TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

      memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
    }
  }

  return cloud;
}

void PublishMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points) {
  sensor_msgs::PointCloud2 cloud = MapPointsToPointCloud (map_points);
  map_points_publisher_.publish (cloud);
}

#if pub_debug_image
void PublishRenderedImage (cv::Mat image) {
  std_msgs::Header header;
  header.stamp = current_frame_time_;
  header.frame_id = map_frame_id_param_;
  const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  rendered_image_publisher_.publish(rendered_image_msg);
}
#endif

tf::Transform TransformFromMat (cv::Mat position_mat) {
  cv::Mat rotation(3,3,CV_32F);
  cv::Mat translation(3,1,CV_32F);

  rotation = position_mat.rowRange(0,3).colRange(0,3);
  translation = position_mat.rowRange(0,3).col(3);


  tf::Matrix3x3 tf_camera_rotation (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2),
                                    rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2),
                                    rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2)
                                   );

  tf::Vector3 tf_camera_translation (translation.at<float> (0), translation.at<float> (1), translation.at<float> (2));

  // Coordinate transformation matrix from orb coordinate system to ros coordinate system
  const tf::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                    -1, 0, 0,
                                     0,-1, 0);

  // const tf::Matrix3x3 tf_orb_to_ros (1, 0, 0,
  //                                    0, 1, 0,
  //                                    0, 0, 1);

  //Transform from orb coordinate system to ros coordinate system on camera coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  //Inverse matrix
  tf_camera_rotation = tf_camera_rotation.transpose();
  tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

  //Transform from orb coordinate system to ros coordinate system on map coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  return tf::Transform (tf_camera_rotation, tf_camera_translation);
}

void PublishPositionAsTransform (cv::Mat position) {
  if(publish_tf_param_){
      tf::Transform transform = TransformFromMat (position);
      static tf::TransformBroadcaster tf_broadcaster;
      tf_broadcaster.sendTransform(tf::StampedTransform(transform, current_frame_time_, map_frame_id_param_, camera_frame_id_param_));
  }
}

void PublishPositionAsPoseStamped (cv::Mat position) {
  tf::Transform grasp_tf = TransformFromMat (position);
  tf::Stamped<tf::Pose> grasp_tf_pose(grasp_tf, current_frame_time_, map_frame_id_param_);
  geometry_msgs::PoseStamped pose_msg;
  tf::poseStampedTFToMsg (grasp_tf_pose, pose_msg);
/********************Rotation the trajectory****************/
  // tf::Vector3 ori_translation (pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
  // // cout << "x: " << ori_translation.getX() << " y: " << ori_translation.getY() << " z: " << ori_translation.getZ()<<endl;

  // // active path
  // // const tf::Matrix3x3 tf_bias (-0.99542383, 0.01234577,  -0.09475749,
  // //                               0.03760268, 0.96222441, -0.26964834,
  // //                               0.08784895, -0.27197752, -0.95828534);

  // // passive path slope
  // // const tf::Matrix3x3 tf_bias (-0.99999244, 0.00326016,  0.00211764,
  // //                              0.0028536,   0.98549717, -0.16966787,
  // //                              -0.00264008, -0.16966055, -0.98549903);

  // // passive path plane
  // const tf::Matrix3x3 tf_bias (0.9612617, 0.2756374,  0.0,
  //                              0.2756374, 0.9612617, -0.0,
  //                              -0.0, -0.0, 1);


  // // const tf::Matrix3x3 tf_bias (0.0, -1,  0.0000000,
  // //                                   1,  0.0,  0.0000000,
  // //                                   0.0000000,  0.0000000,  1.0000000);

  // ori_translation = tf_bias*ori_translation;
  // // cout << "x: " << ori_translation.getX() << " y: " << ori_translation.getY() << " z: " << ori_translation.getZ()<<endl;

  // pose_msg.pose.position.x = ori_translation.getX();
  // pose_msg.pose.position.y = ori_translation.getY();
  // pose_msg.pose.position.z = ori_translation.getZ();
/********************Rotation the trajectory****************/

  pose_publisher_.publish(pose_msg);
}

// publish Path
void PublishPositionAsPath (cv::Mat position) {
  tf::Transform grasp_tf = TransformFromMat (position);
  tf::Stamped<tf::Pose> grasp_tf_pose(grasp_tf, current_frame_time_, map_frame_id_param_);
  geometry_msgs::PoseStamped pose_msg;
  tf::poseStampedTFToMsg (grasp_tf_pose, pose_msg);
  
  std_msgs::Header header;
  header.stamp = current_frame_time_;
  header.frame_id = map_frame_id_param_;

  orb_path_.header = header;
  // orb_path_.header.frame_id = map_frame_id_param_;
  // pose_msg.pose.position.z = 0;

  orb_path_.poses.push_back(pose_msg);
  path_publisher_.publish(orb_path_);

}

static int gps_count = 0;
float gps_bias_x = 0;
float gps_bias_y = 0;
float gps_bias_z = 0;
float gps_bias_ow = 0;
float gps_bias_ox = 0;
float gps_bias_oy = 0;
float gps_bias_oz = 0;
// publish GPSPath
void GPSCallback (const nav_msgs::OdometryConstPtr& gps_) {
  std_msgs::Header header = gps_->header;
  
  current_frame_time_ = gps_->header.stamp;
  // // publish gps path
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = header;
  pose_stamped.header.frame_id = "world";
  // pose_stamped.pose = gps_->pose.pose;

  // slope
  // pose_stamped.pose.position.x = (gps_->pose.pose.position.x)+1.5;
  // pose_stamped.pose.position.y = (gps_->pose.pose.position.y)-0.05;
  // pose_stamped.pose.position.z = (gps_->pose.pose.position.z)-0.79;

  // // others


  // tf::Vector3 ori_translation_bias(2.50112812e+06, 8.05082700e+05, 3.34976473e+01);

  // pose_stamped.pose.position.x = (gps_->pose.pose.position.x) - ori_translation_bias.getX();
  // pose_stamped.pose.position.y = (gps_->pose.pose.position.y) - ori_translation_bias.getY();
  // pose_stamped.pose.position.z = (gps_->pose.pose.position.z) - ori_translation_bias.getZ();

  // pose_stamped.pose.orientation.w = (gps_->pose.pose.orientation.w);
  // pose_stamped.pose.orientation.x = (gps_->pose.pose.orientation.x);
  // pose_stamped.pose.orientation.y = (gps_->pose.pose.orientation.y);
  // pose_stamped.pose.orientation.z = (gps_->pose.pose.orientation.z);

  if(gps_count<100){
    gps_bias_x = (gps_->pose.pose.position.x);
    gps_bias_y = (gps_->pose.pose.position.y);
    gps_bias_z = (gps_->pose.pose.position.z);
    gps_bias_ow = (gps_->pose.pose.orientation.w);
    gps_bias_ox = (gps_->pose.pose.orientation.x);
    gps_bias_oy = (gps_->pose.pose.orientation.y);
    gps_bias_oz = (gps_->pose.pose.orientation.z);
    pose_stamped.pose.position.x = -((gps_->pose.pose.position.x) - gps_bias_x);
    pose_stamped.pose.position.y = ((gps_->pose.pose.position.y) - gps_bias_y);
    pose_stamped.pose.position.z = (gps_->pose.pose.position.z) - gps_bias_z;

    pose_stamped.pose.orientation.w = (gps_->pose.pose.orientation.w) - gps_bias_ow;
    pose_stamped.pose.orientation.x = (gps_->pose.pose.orientation.x) - gps_bias_ox;
    pose_stamped.pose.orientation.y = (gps_->pose.pose.orientation.y) - gps_bias_oy;
    pose_stamped.pose.orientation.z = (gps_->pose.pose.orientation.z) - gps_bias_oz;

    gps_count++;
    // cout << "gps_count: " << gps_count << endl;
    // cout << "x: " << gps_bias_x << " y: " << gps_bias_y << " z: " << gps_bias_z << " ow: " << gps_bias_ow << " ox: " << gps_bias_ox << " oy: " << gps_bias_oy << " oz: " << gps_bias_oz << endl;
  }
  else{
    pose_stamped.pose.position.x = ((gps_->pose.pose.position.x) - gps_bias_x);
    pose_stamped.pose.position.y = ((gps_->pose.pose.position.y) - gps_bias_y);
    pose_stamped.pose.position.z = ((gps_->pose.pose.position.z) - gps_bias_z);

    // tf::Vector3 ori_translation (pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z);
    // // cout << "x: " << ori_translation.getX() << " y: " << ori_translation.getY() << " z: " << ori_translation.getZ()<<endl;

    // // // slope -20 0816 slope
    // const tf::Matrix3x3 tf_bias (-0.5000000, -0.8660254,  0.0000000,
    //                               0.8660254,  -0.5000000,  0.0000000,
    //                                   0.0000000,  0.0000000,  1.0000000);

    // ori_translation = tf_bias*ori_translation;
    // // cout << "x: " << ori_translation.getX() << " y: " << ori_translation.getY() << " z: " << ori_translation.getZ()<<endl;

    // pose_stamped.pose.position.x = ori_translation.getX();
    // pose_stamped.pose.position.y = ori_translation.getY();
    // pose_stamped.pose.position.z = ori_translation.getZ();

    // cout << "x: " << pose_stamped.pose.position.x << " y: " << pose_stamped.pose.position.y << " z: " << pose_stamped.pose.position.z << endl;

    pose_stamped.pose.orientation.w = (gps_->pose.pose.orientation.w);
    pose_stamped.pose.orientation.x = (gps_->pose.pose.orientation.x);
    pose_stamped.pose.orientation.y = (gps_->pose.pose.orientation.y);
    pose_stamped.pose.orientation.z = (gps_->pose.pose.orientation.z);

    // cout << "x: " << pose_stamped.pose.position.x << " y: " << pose_stamped.pose.position.y << " z: " << pose_stamped.pose.position.z << " ow: " << pose_stamped.pose.orientation.w << " ox: " << pose_stamped.pose.orientation.x << " oy: " << pose_stamped.pose.orientation.y << " oz: " << pose_stamped.pose.orientation.z << endl;

  }
  gps_pose_publisher_.publish(pose_stamped);

  gps_path_.header = header;
  gps_path_.header.frame_id = "world";
  gps_path_.poses.push_back(pose_stamped);
  gps_path_publisher_.publish(gps_path_);
  
}


void Update(cv::Mat position) {
  // cv::Mat position = SLAM->GetCurrentPosition();
  
  if (!position.empty()) {
    // PublishPositionAsTransform (position);

    if (publish_pose_param_) {
      PublishPositionAsPoseStamped (position);
      PublishPositionAsPath (position);
    }
  }

  PublishRenderedImage (SLAM->DrawCurrentFrame());

  if (publish_pointcloud_param_) {
    PublishMapPoints (SLAM->GetAllMapPoints());
    // PublishMapPoints (SLAM->GetListMapPoints());
    // PublishMapPoints (SLAM->GetTrackedMapPoints());
  }

}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    frameCounter++;
    // Print frame rate every x second
    if((cv_ptrRGB->header.stamp - lastStamp).toSec() >= 5.0)
    {
        float fps = frameCounter / (cv_ptrRGB->header.stamp - lastStamp).toSec();
        lastStamp = cv_ptrRGB->header.stamp;
        ROS_INFO("Frames per second: %f", fps);
        frameCounter = 0;
    }

    if(Tcw.empty())
        return;
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);

    // Publish tf transform
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = cv_ptrRGB->header.stamp;

    // current_frame_time_ = cv_ptrRGB->header.stamp;

    transformStamped.header.frame_id = "world"; 
    transformStamped.child_frame_id = camera_frame_id_param_;
    
    tf::Transform transform = TransformFromMat (Tcw);
    // transformStamped.transform = transform;
    
    transformStamped.transform.translation.x = transform.getOrigin().getX();
    transformStamped.transform.translation.y = transform.getOrigin().getY();
    transformStamped.transform.translation.z = transform.getOrigin().getZ();
    transformStamped.transform.rotation.x = transform.getRotation().getX();
    transformStamped.transform.rotation.y = transform.getRotation().getY();
    transformStamped.transform.rotation.z = transform.getRotation().getZ();
    transformStamped.transform.rotation.w = transform.getRotation().getW();

    br.sendTransform(transformStamped);
    
    Update(Tcw);
}


