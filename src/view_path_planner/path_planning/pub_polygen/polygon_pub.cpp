#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <log4cxx/logger.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <string.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int16.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <chrono>
#include <iostream>
#include <pcl_ros/impl/transforms.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

using namespace std;
using namespace octomap;

#define def_octo_generator 1

#if def_octo_generator
octomap::ColorOcTree *_octree;
#else
octomap::OcTree *_octree;
#endif

nav_msgs::Path Travel_Path_visualization;
ros::Publisher Travel_Path_publisher_visual;
ros::Publisher Odometry_publisher_visual;
ros::Publisher pub_cloud_;

geometry_msgs::PolygonStamped myPolygon;

float explore_range_blx;
float explore_range_bly;
float explore_range_urx;
float explore_range_ury;

void orbpose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {

  geometry_msgs::PoseStamped pose_msg;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "world";

  Travel_Path_visualization.header = msg->header;

  pose_msg.pose.position = msg->pose.position;

/********************Rotation the trajectory****************/
  tf::Vector3 ori_translation (pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
//   // // cout << "x: " << ori_translation.getX() << " y: " << ori_translation.getY() << " z: " << ori_translation.getZ()<<endl;

// //   ours-wop-2022-07-28-22-21-18-startInPoor-Video.orig.bag
//   const tf::Matrix3x3 tf_bias( 0.98037413,  0.19307704,  0.03984743,
// -0.19315064,  0.98116701, -0.00203107,
// -0.03948914, -0.00570535,  0.99920371);

//   ours-wop-2022-07-28-22-21-18-startInPoor-Video.orig.bag
//   const tf::Matrix3x3 tf_bias( 0.98037413,  0.19307704,  0.03984743,
// -0.19315064,  0.98116701, -0.00203107,
// -0.03948914, -0.00570535,  0.99920371);

// ours-wop-2022-07-28-21-21-56-startInPoor-Video.orig.bag
// const tf::Matrix3x3 tf_bias( 0.98386073,  0.17165267,  0.05053135,
// -0.17209117,  0.98507108,  0.00442618,
// -0.04901721, -0.01305074,  0.99871267);

// ours-wop-2022-07-28-19-54-20-fianlInPoor-failed-Video.orig.bag
// const tf::Matrix3x3 tf_bias(9.94104183e-01,  1.04159067e-01, -3.01291015e-02,
// -1.04215672e-01,  9.94554673e-01, -3.10284430e-04,
// 2.99327198e-02,  3.44837962e-03,  9.99545967e-01);

const tf::Matrix3x3 tf_bias( 0.99968187,  0.00396064,  0.02490948,
-0.0040982,  0.99997661,  0.0054766,
-0.0248872,  -0.00557695,  0.99967471);

  ori_translation = tf_bias*ori_translation;
//   // // cout << "x: " << ori_translation.getX() << " y: " << ori_translation.getY() << " z: " << ori_translation.getZ()<<endl;

  pose_msg.pose.position.x = ori_translation.getX();
  pose_msg.pose.position.y = ori_translation.getY();
  pose_msg.pose.position.z = ori_translation.getZ();
/********************Rotation the trajectory****************/

  Travel_Path_visualization.poses.push_back(pose_msg);
  Travel_Path_publisher_visual.publish(Travel_Path_visualization);

  nav_msgs::Odometry pose_msg_odom;
  header.stamp = ros::Time::now();
  header.frame_id = "world";

  pose_msg_odom.header = msg->header;

  pose_msg_odom.pose.pose.position.x = ori_translation.getX();
  pose_msg_odom.pose.pose.position.y = ori_translation.getY();
  pose_msg_odom.pose.pose.position.z = ori_translation.getZ();
  pose_msg_odom.pose.pose.orientation = msg->pose.orientation;

  Odometry_publisher_visual.publish(pose_msg_odom);

}

void CameraState_Cb(const nav_msgs::Odometry::ConstPtr &msg){

  geometry_msgs::PoseStamped pose_msg;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "world";

  Travel_Path_visualization.header = msg->header;

  pose_msg.pose.position = msg->pose.pose.position;
/********************Rotation the trajectory****************/
  tf::Vector3 ori_translation (pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
//   // // cout << "x: " << ori_translation.getX() << " y: " << ori_translation.getY() << " z: " << ori_translation.getZ()<<endl;

  // ours-wop-2022-07-28-19-54-20-fianlInPoor-failed-Video.orig.bag
// const tf::Matrix3x3 tf_bias(0.98642941, -0.16411114, -0.00495447,
//  0.16407105,  0.98641896, -0.00763664,
//  0.00614044,  0.00672012,  0.99995857);


//   ori_translation = tf_bias.inverse()*ori_translation;
//   // // cout << "x: " << ori_translation.getX() << " y: " << ori_translation.getY() << " z: " << ori_translation.getZ()<<endl;

  pose_msg.pose.position.x = ori_translation.getX();
  pose_msg.pose.position.y = ori_translation.getY();
  pose_msg.pose.position.z = ori_translation.getZ();
/********************Rotation the trajectory****************/

  Travel_Path_visualization.poses.push_back(pose_msg);
  Travel_Path_publisher_visual.publish(Travel_Path_visualization);

  nav_msgs::Odometry pose_msg_odom;
  header.stamp = ros::Time::now();
  header.frame_id = "world";

  pose_msg_odom.header = msg->header;
  pose_msg_odom.pose.pose.position = msg->pose.pose.position;
  
  double cur_base_roll, cur_base_pitch, cur_base_yaw; //定义存储r\p\y的容器

  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
  // tf::Matrix3x3(quat).getRPY(cur_base_pitch, cur_base_roll, 
  //                            cur_base_yaw); // plt_base
  tf::Matrix3x3(quat).getRPY(cur_base_roll, cur_base_pitch, 
                             cur_base_yaw); // landshaker
  cur_base_pitch = -cur_base_pitch;

  geometry_msgs::Quaternion geo_quat = tf::createQuaternionMsgFromRollPitchYaw(cur_base_roll, cur_base_pitch, 
                             cur_base_yaw);

  pose_msg_odom.pose.pose.orientation = geo_quat;

  Odometry_publisher_visual.publish(pose_msg_odom);

}

void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg) {
  octomap::AbstractOcTree *read_tree = octomap_msgs::msgToMap(*msg);
#if def_octo_generator
  _octree = dynamic_cast<ColorOcTree *>(read_tree);
#else
  _octree = dynamic_cast<OcTree *>(read_tree);
#endif

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl_cloud->header.frame_id = "world";
  // pcl_cloud->header.stamp = 

#if def_octo_generator
  for (octomap::ColorOcTree::iterator it = _octree->begin(16), end = _octree->end(); it != end; ++it){
#else
  for (octomap::OcTree::iterator it = _octree->begin(16), end = _octree->end(); it != end; ++it){
#endif
    if (_octree->isNodeOccupied(*it)){
      pcl::PointXYZ point;
      point.x = it.getX();
      point.y = it.getY();
      point.z = it.getZ();

      (*pcl_cloud).push_back(point);
    }
    else{
      point3d query = it.getCoordinate();
      OcTreeNode *node = _octree->search(query, 16);
      if(node != NULL && it.getZ() < 1){
        pcl::PointXYZ point;
        point.x = it.getX();
        point.y = it.getY();
        point.z = it.getZ();

        (*pcl_cloud).push_back(point);
      }
    }
  }

  // std::cerr << "Got " << pcl_cloud->size() << " data points in frame "
  //           << pcl_cloud->header.frame_id
  //           << " with the following fields: " << pcl::getFieldsList(*pcl_cloud)
  //           << std::endl;

  // debug
  sensor_msgs::PointCloud2 out_cloud_msg;
  out_cloud_msg.header.frame_id = "world";
	// out_cloud_msg.header.stamp=ros::Time::now();
  pcl::toROSMsg(*pcl_cloud, out_cloud_msg);
  //out_cloud_msg.header = cloud_ptr->header;
  pub_cloud_.publish(out_cloud_msg);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "octomap_planner");
  ros::NodeHandle n;
  // planner planner_object;
  // ros::Subscriber orbpose_sub = n.subscribe<geometry_msgs::PoseStamped>(
  //     "/orb_slam2_rgbd/pose", 1, orbpose_callback);
  ros::Subscriber cam_state_sub = n.subscribe<nav_msgs::Odometry>(
      "/ground_truth/camera_state", 1, CameraState_Cb);

  Travel_Path_publisher_visual = n.advertise<nav_msgs::Path> ("/Traveled_path_bag", 1);
  Odometry_publisher_visual = n.advertise<nav_msgs::Odometry> ("/Odometry_visual", 1);
  
#if def_octo_generator
  ros::Subscriber octree_sub = n.subscribe<octomap_msgs::Octomap>(
      "/octomap_full_semantic", 1, octomapCallback);
#else
  ros::Subscriber octree_sub = n.subscribe<octomap_msgs::Octomap>(
      "/origin_octomap", 1, octomapCallback);
#endif

  pub_cloud_ = n.advertise<sensor_msgs::PointCloud2>("/debug_points", 2);

  // 显示探索范围的设定边界
  ros::Publisher Polygon_pub =
      n.advertise<geometry_msgs::PolygonStamped>("polygonpublisher", 1);

  n.getParam("explore_range_blx", explore_range_blx);
  n.getParam("explore_range_bly", explore_range_bly);
  n.getParam("explore_range_urx", explore_range_urx);
  n.getParam("explore_range_ury", explore_range_ury);
  ROS_INFO("param explore_range_blx: %f", explore_range_blx);
  ROS_INFO("param explore_range_bly: %f", explore_range_bly);
  ROS_INFO("param explore_range_urx: %f", explore_range_urx);
  ROS_INFO("param explore_range_ury: %f", explore_range_ury);

  ros::Rate loop_rate(10);

  myPolygon.header.frame_id = "world";
  myPolygon.polygon.points.clear();
  geometry_msgs::Point32 Polygon_point;

  // 显示局部搜索边界点的范围
  Polygon_point.x = explore_range_urx;
  Polygon_point.y = explore_range_bly;
  Polygon_point.z = 1;
  myPolygon.polygon.points.push_back(Polygon_point);

  Polygon_point.x = explore_range_blx;
  Polygon_point.y = explore_range_bly;
  Polygon_point.z = 1;
  myPolygon.polygon.points.push_back(Polygon_point);

  Polygon_point.x = explore_range_blx;
  Polygon_point.y = explore_range_ury;
  Polygon_point.z = 1;
  myPolygon.polygon.points.push_back(Polygon_point);

  Polygon_point.x = explore_range_urx;
  Polygon_point.y = explore_range_ury;
  Polygon_point.z = 1;
  myPolygon.polygon.points.push_back(Polygon_point);

  while (ros::ok()) {
    // ROS_INFO("This is an info statement, and should print");
    // 定义多边形对象


    // 发布边界
    // Polygon_pub.publish(myPolygon);

    // from_des_to_cmdvel();

    ros::spinOnce();

    loop_rate.sleep();
  }
  // std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

  // ros::spin();

  return 0;
}
