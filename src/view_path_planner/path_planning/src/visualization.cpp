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

#include "../include/visualization.h"


using namespace std;
using namespace octomap;

typedef octomap::point3d point3d;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> Color_PointCloud;

ros::Publisher pcl_marker_pub;
ros::Publisher point_sample_pub;
ros::Publisher task_view_pub;
ros::Publisher terrain_norm_vec_pub;
ros::Publisher rrtpath_pub;
ros::Publisher track_rrtpath_pub;
ros::Publisher goal_pub_;


void PlanningVisualization(ros::NodeHandle &nh){
  
  rrtpath_pub =
      nh.advertise<nav_msgs::Path>("/rrt_path", 1, true);  // For visual
  track_rrtpath_pub =
      nh.advertise<nav_msgs::Path>("/track_rrt_path", 1, true);  // For tracking
  point_sample_pub =
      nh.advertise<visualization_msgs::MarkerArray>("point_sample", 1, true);  //
  task_view_pub =
      nh.advertise<visualization_msgs::MarkerArray>("task_views", 1, true);  //
  pcl_marker_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_marker", 1);     
  terrain_norm_vec_pub = 
      nh.advertise<visualization_msgs::MarkerArray>("norm_vectors", 1, true);  //
  goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

}

void visualize_points(const vector<Eigen::Vector3d> points_list,
                      const vector<float> points_gain) {
  if(points_list.empty()){
    return;
  }
  
  visualization_msgs::MarkerArray ma;

  visualization_msgs::Marker fp_points;
  visualization_msgs::Marker text_marker;
  // ma.markers.clear();
  // text_marker.header.frame_id = fp_points.header.frame_id = "/world";
  // text_marker.action = fp_points.action = visualization_msgs::Marker::DELETEALL;
  // ma.markers.push_back(fp_points);
  // ma.markers.push_back(text_marker);
  // task_view_pub.publish(ma);

  // ma.markers.clear();
  text_marker.header.frame_id = fp_points.header.frame_id = "/world";
  text_marker.header.stamp = fp_points.header.stamp = ros::Time::now();
  text_marker.ns = fp_points.ns = "points_and_lines";
  text_marker.action = fp_points.action = visualization_msgs::Marker::ADD;
  text_marker.pose.orientation.w = fp_points.pose.orientation.w = 1.0;
  // fp_points.type = visualization_msgs::Marker::POINTS;
  fp_points.type = visualization_msgs::Marker::ARROW;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  fp_points.scale.x = 3;
  fp_points.scale.y = 0.2;
  fp_points.scale.z = 0.2;

  text_marker.scale.z = 0.8;

  // Points are green
  fp_points.color.r = 1.0f;
  fp_points.color.g = 0.0f;
  fp_points.color.b = 1.0f;
  fp_points.color.a = 1.0;

  text_marker.color.r = 0.0f;
  text_marker.color.g = 1.0f;
  text_marker.color.b = 0.0f;
  text_marker.color.a = 1.0;

  int i = 2;
  // Create the vertices for the points and lines
  for (auto fp : points_list) {
    fp_points.pose.position.x = fp.x();
    fp_points.pose.position.y = fp.y();

    tf::Quaternion quat = tf::createQuaternionFromYaw(fp.z());

    geometry_msgs::Quaternion quat_geo;
    quaternionTFToMsg(quat, quat_geo);
    fp_points.pose.orientation = quat_geo;

    text_marker.pose.position.x = fp.x() + 0.6;
    text_marker.pose.position.y = fp.y();
    text_marker.pose.position.z = 0.5;

    text_marker.text = std::to_string(int(points_gain[i - 2]));
    // text_marker.text = "Imnnn";
    // ostringstream str;
    // str<<i;
    // text_marker.text=str.str();

    text_marker.id = 200 + i;
    fp_points.id = i;

    ma.markers.push_back(fp_points);
    ma.markers.push_back(text_marker);
    i++;
  }

  point_sample_pub.publish(ma);
}


void visualize_points_pcl(const vector<Eigen::Vector3d> points_list) {
  if(points_list.empty()){
    return;
  }
  pcl::PointCloud<pcl::PointXYZ> cloud; 
  sensor_msgs::PointCloud2 output; 
  // Fill in the cloud data 

  cloud.points.resize(points_list.size()); 
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {  
    cloud.points[i].x = points_list[i].x(); 
    cloud.points[i].y = points_list[i].y(); 
    cloud.points[i].z = points_list[i].z(); 
  } 
  //Convert the cloud to ROS message 
  pcl::toROSMsg(cloud, output); 
  output.header.frame_id = "world"; 

  pcl_marker_pub.publish(output);
}

void visualize_points(const vector<Eigen::Vector3d> points_list) {
  if(points_list.empty()){
    return;
  }
  
  visualization_msgs::MarkerArray ma;

  visualization_msgs::Marker fp_points;
  // ma.markers.clear();
  // text_marker.header.frame_id = fp_points.header.frame_id = "/world";
  // text_marker.action = fp_points.action = visualization_msgs::Marker::DELETEALL;
  // ma.markers.push_back(fp_points);
  // ma.markers.push_back(text_marker);
  // task_view_pub.publish(ma);

  // ma.markers.clear();
  fp_points.header.frame_id = "/world";
  fp_points.header.stamp = ros::Time::now();
  fp_points.ns = "points_and_lines";
  fp_points.action = visualization_msgs::Marker::ADD;
  fp_points.pose.orientation.w = 1.0;
  // fp_points.type = visualization_msgs::Marker::POINTS;
  fp_points.type = visualization_msgs::Marker::CUBE;

  fp_points.scale.x = 0.5;
  fp_points.scale.y = 0.5;
  fp_points.scale.z = 0.5;

  // Points are green
  fp_points.color.r = 0.0f;
  fp_points.color.g = 1.0f;
  fp_points.color.b = 1.0f;
  fp_points.color.a = 1.0;

  int i = 2;
  // Create the vertices for the points and lines
  for (auto fp : points_list) {
    fp_points.pose.position.x = fp.x();
    fp_points.pose.position.y = fp.y();
    fp_points.pose.position.z = fp.z();

    fp_points.id = i;

    ma.markers.push_back(fp_points);
    i++;
  }

  point_sample_pub.publish(ma);
}

void visualize_points(const vector<Eigen::Vector3d> points_list,
                      const vector<Eigen::Vector3d> orientation_list,
                      const vector<float> points_gain) {
  if(points_list.empty()){
    return;
  }
  
  visualization_msgs::MarkerArray ma;

  visualization_msgs::Marker fp_points;
  visualization_msgs::Marker text_marker;
  // ma.markers.clear();
  // text_marker.header.frame_id = fp_points.header.frame_id = "/world";
  // text_marker.action = fp_points.action = visualization_msgs::Marker::DELETEALL;
  // ma.markers.push_back(fp_points);
  // ma.markers.push_back(text_marker);
  // task_view_pub.publish(ma);

  // ma.markers.clear();
  text_marker.header.frame_id = fp_points.header.frame_id = "/world";
  text_marker.header.stamp = fp_points.header.stamp = ros::Time::now();
  text_marker.ns = fp_points.ns = "points_and_lines";
  text_marker.action = fp_points.action = visualization_msgs::Marker::ADD;
  text_marker.pose.orientation.w = fp_points.pose.orientation.w = 1.0;
  // fp_points.type = visualization_msgs::Marker::POINTS;
  fp_points.type = visualization_msgs::Marker::ARROW;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  fp_points.scale.x = 3.5;
  fp_points.scale.y = 0.2;
  fp_points.scale.z = 0.2;

  text_marker.scale.z = 0.8;

  // Points are green
  fp_points.color.r = 1.0f;
  fp_points.color.g = 0.0f;
  fp_points.color.b = 1.0f;
  fp_points.color.a = 1.0;

  text_marker.color.r = 0.0f;
  text_marker.color.g = 1.0f;
  text_marker.color.b = 0.0f;
  text_marker.color.a = 1.0;

  ROS_WARN_STREAM("POINT_LIST size: " << points_list.size());
  // Create the vertices for the points and lines
  for (int i = 0; i < points_list.size(); ++i) {
    fp_points.pose.position.x = points_list[i].x();
    fp_points.pose.position.y = points_list[i].y();
    fp_points.pose.position.z = points_list[i].z();

    // tf::Quaternion quat = tf::createQuaternionFromYaw(orientation_list[i].z());
    // geometry_msgs::Quaternion quat_geo;
    // quaternionTFToMsg(quat, quat_geo);
    // fp_points.pose.orientation = quat_geo;

    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(orientation_list[i].x(), orientation_list[i].y(), orientation_list[i].z());
    fp_points.pose.orientation = quat;

    text_marker.pose.position.x = points_list[i].x() + 0.6;
    text_marker.pose.position.y = points_list[i].y();
    text_marker.pose.position.z = 0.5;

    text_marker.text = std::to_string(int(points_gain[i]));
    // text_marker.text = "Imnnn";
    // ostringstream str;
    // str<<i;
    // text_marker.text=str.str();

    text_marker.id = 200 + i;
    fp_points.id = i;

    ma.markers.push_back(fp_points);
    ma.markers.push_back(text_marker);
  }

  point_sample_pub.publish(ma);
}

visualization_msgs::MarkerArray ma_taskview;
visualization_msgs::Marker fp_points_taskview;
visualization_msgs::Marker text_marker_taskview;

void visualize_task_view(const vector<Eigen::Vector3d> points_list,
                      const vector<Eigen::Vector3d> orientation_list,
                      const vector<float> points_gain) {
  if(points_list.empty()){
    return;
  }

  // rviz_visual_tools::RvizVisualTools rviz_interface("world","/task_views");
  // rviz_interface.deleteAllMarkers();
  ma_taskview.markers.clear();
  text_marker_taskview.header.frame_id = fp_points_taskview.header.frame_id = "/world";
  text_marker_taskview.action = fp_points_taskview.action = visualization_msgs::Marker::DELETEALL;
  ma_taskview.markers.push_back(fp_points_taskview);
  ma_taskview.markers.push_back(text_marker_taskview);
  task_view_pub.publish(ma_taskview);

  ma_taskview.markers.clear();
  text_marker_taskview.header.frame_id = fp_points_taskview.header.frame_id = "/world";
  text_marker_taskview.header.stamp = fp_points_taskview.header.stamp = ros::Time::now();
  text_marker_taskview.ns = fp_points_taskview.ns = "points_and_lines";
  text_marker_taskview.action = fp_points_taskview.action = visualization_msgs::Marker::ADD;
  text_marker_taskview.pose.orientation.w = fp_points_taskview.pose.orientation.w = 1.0;
  // fp_points_taskview.type = visualization_msgs::Marker::POINTS;
  fp_points_taskview.type = visualization_msgs::Marker::ARROW;
  text_marker_taskview.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  fp_points_taskview.scale.x = 3.5;
  fp_points_taskview.scale.y = 0.2;
  fp_points_taskview.scale.z = 0.2;

  text_marker_taskview.scale.z = 0.8;

  // Points are green
  fp_points_taskview.color.r = 1.0f;
  fp_points_taskview.color.g = 0.0f;
  fp_points_taskview.color.b = 1.0f;
  fp_points_taskview.color.a = 1.0;

  text_marker_taskview.color.r = 0.0f;
  text_marker_taskview.color.g = 1.0f;
  text_marker_taskview.color.b = 0.0f;
  text_marker_taskview.color.a = 1.0;

  ROS_WARN_STREAM("POINT_LIST size: " << points_list.size());
  // Create the vertices for the points and lines
  for (int i = 0; i < points_list.size(); ++i) {
    fp_points_taskview.pose.position.x = points_list[i].x();
    fp_points_taskview.pose.position.y = points_list[i].y();
    fp_points_taskview.pose.position.z = points_list[i].z();

    // tf::Quaternion quat = tf::createQuaternionFromYaw(orientation_list[i].z());
    // geometry_msgs::Quaternion quat_geo;
    // quaternionTFToMsg(quat, quat_geo);
    // fp_points_taskview.pose.orientation = quat_geo;

    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(orientation_list[i].x(), orientation_list[i].y(), orientation_list[i].z());
    fp_points_taskview.pose.orientation = quat;

    text_marker_taskview.pose.position.x = points_list[i].x() + 0.6;
    text_marker_taskview.pose.position.y = points_list[i].y();
    text_marker_taskview.pose.position.z = 0.5;

    text_marker_taskview.text = std::to_string(int(points_gain[i]));
    // text_marker_taskview.text = "Imnnn";
    // ostringstream str;
    // str<<i;
    // text_marker_taskview.text=str.str();

    text_marker_taskview.id = 200 + i;
    fp_points_taskview.id = i;

    ma_taskview.markers.push_back(fp_points_taskview);
    ma_taskview.markers.push_back(text_marker_taskview);
  }

  task_view_pub.publish(ma_taskview);
}


void visualize_terrain_norm(const vector<Eigen::Vector3d> points_list,
                      const vector<Eigen::Vector3d> orientation_list) {
  if(points_list.empty()){
    return;
  }
  
  visualization_msgs::MarkerArray ma;

  visualization_msgs::Marker fp_points;

  fp_points.header.frame_id = "/world";
  fp_points.header.stamp = ros::Time::now();
  fp_points.ns = "points_and_lines";
  fp_points.action = visualization_msgs::Marker::ADD;
  fp_points.pose.orientation.w = 1.0;
  // fp_points.type = visualization_msgs::Marker::POINTS;
  fp_points.type = visualization_msgs::Marker::ARROW;

  fp_points.scale.x = 0.3;
  fp_points.scale.y = 0.5;
  // fp_points.scale.z = 0.2;


  // Points are green
  fp_points.color.r = 1.0f;
  fp_points.color.g = 0.0f;
  fp_points.color.b = 1.0f;
  fp_points.color.a = 1.0;

  // Create the vertices for the points and lines
  for (int i = 0; i < points_list.size(); ++i) {
    // fp_points.pose.position.x = points_list[i].x();
    // fp_points.pose.position.y = points_list[i].y();
    // fp_points.pose.position.z = points_list[i].z();
    fp_points.points.clear();
    geometry_msgs::Point start_point;
    start_point.x = points_list[i].x();
    start_point.y = points_list[i].y();
    start_point.z = points_list[i].z();
    fp_points.points.push_back(start_point);

    geometry_msgs::Point end_point;
    end_point.x = points_list[i].x() + orientation_list[i].x()*3;
    end_point.y = points_list[i].y() + orientation_list[i].y()*3;
    end_point.z = points_list[i].z() + orientation_list[i].z()*3;
    fp_points.points.push_back(end_point);

    fp_points.id = i;

    ma.markers.push_back(fp_points);
  }

  terrain_norm_vec_pub.publish(ma);
}


void Convert2Navpath() {
  cout << "Publish Nav Path..." << endl;
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "world";

  cout << "path.size: " << path_.size() << endl;
  for (auto pt : path_) {
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = pt->position.x();
    this_pose_stamped.pose.position.y = pt->position.y();

    geometry_msgs::Quaternion goal_quat =
        tf::createQuaternionMsgFromYaw(pt->position.z());
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;

    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.header.frame_id = "world";
    path.poses.push_back(this_pose_stamped);
  }

  rrtpath_pub.publish(path);
}

void Convert2Navpath_3D(vector<rrt_3d::Node_3d *> path_) {
  cout << "Publish Nav Path_3D..." << endl;
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "world";

  cout << "path.size: " << path_.size() << endl;
  for (int i = path_.size() - 1; i >= 0; --i) {
    // cout << "path_3d node" << pt->position.transpose() << " " << pt->orientation.transpose() << endl;
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = path_[i]->position.x();
    this_pose_stamped.pose.position.y = path_[i]->position.y();
    this_pose_stamped.pose.position.z = path_[i]->position.z();
    // this_pose_stamped.pose.position.z = get_mapz(path_[i]->position.segment(0,2));

    geometry_msgs::Quaternion goal_quat =
        tf::createQuaternionMsgFromRollPitchYaw(path_[i]->orientation.x(), path_[i]->orientation.y(), path_[i]->orientation.z());
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;

    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.header.frame_id = "world";
    path.poses.push_back(this_pose_stamped);
  }

  rrtpath_pub.publish(path);
}

void Convert2Navpath_track(vector<Eigen::VectorXd>& path_) {
  cout << "Publish Track Path_3D..." << endl;
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "world";

  cout << "path.size: " << path_.size() << endl;
  for (int i = 0; i < path_.size(); ++i) {
    cout << "path_3d node" << path_[i].transpose() << endl;
  }

#if 1
  geometry_msgs::PoseStamped this_pose_stamped;
  this_pose_stamped.pose.position.x = path_[path_.size() - 1](0);
  this_pose_stamped.pose.position.y = path_[path_.size() - 1](1);
  this_pose_stamped.pose.position.z = path_[path_.size() - 1](2);
  // this_pose_stamped.pose.position.z = get_mapz(path_[path_.size() - 1].segment(0,2));

  // cout << "Convert2Navpath - euler angle: " << path_[i](3) << " " << path_[i](4) << " " << path_[i](5) << endl;

  geometry_msgs::Quaternion goal_quat =
      tf::createQuaternionMsgFromRollPitchYaw(path_[path_.size() - 1](3), path_[path_.size() - 1](4), path_[path_.size() - 1](5));
  cout << "Convert2Navpath - quat: " << goal_quat.x << " " << goal_quat.y << " " << goal_quat.z << " " << goal_quat.w << endl;

  this_pose_stamped.pose.orientation.x = goal_quat.x;
  this_pose_stamped.pose.orientation.y = goal_quat.y;
  this_pose_stamped.pose.orientation.z = goal_quat.z;
  this_pose_stamped.pose.orientation.w = goal_quat.w;

  this_pose_stamped.header.stamp = ros::Time::now();
  this_pose_stamped.header.frame_id = "world";
  goal_pub_.publish(this_pose_stamped);
#else
  for (int i = 0; i < path_.size(); ++i) {
    cout << "path_3d node" << path_[i].transpose() << endl;
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = path_[i](0);
    this_pose_stamped.pose.position.y = path_[i](1);
    this_pose_stamped.pose.position.z = path_[i](2);

    // cout << "Convert2Navpath - euler angle: " << path_[i](3) << " " << path_[i](4) << " " << path_[i](5) << endl;

    geometry_msgs::Quaternion goal_quat =
        tf::createQuaternionMsgFromRollPitchYaw(path_[i](3), path_[i](4), path_[i](5));
    cout << "Convert2Navpath - quat: " << goal_quat.x << " " << goal_quat.y << " " << goal_quat.z << " " << goal_quat.w << endl;

    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;

    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.header.frame_id = "world";
    path.poses.push_back(this_pose_stamped);
  }
  track_rrtpath_pub.publish(path);
#endif

}
