#ifndef VISUALIZATION_
#define VISUALIZATION_

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

// #include "../include/online_map_merge.hpp"
// #include "../include/project2d.hpp"
#include "gridmap_rrt.h"
#include "gridmap_rrt_3d.h"
// #include "../include/terrain_map.h"
// #include <nodehash.h>


using namespace std;
using namespace octomap;

typedef octomap::point3d point3d;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> Color_PointCloud;

void PlanningVisualization(ros::NodeHandle &nh);

void visualize_points(const vector<Eigen::Vector3d> points_list,
                      const vector<float> points_gain) ;


void visualize_points_pcl(const vector<Eigen::Vector3d> points_list);

void visualize_points(const vector<Eigen::Vector3d> points_list);

void visualize_points(const vector<Eigen::Vector3d> points_list,
                      const vector<Eigen::Vector3d> orientation_list,
                      const vector<float> points_gain);

void visualize_task_view(const vector<Eigen::Vector3d> points_list,
                      const vector<Eigen::Vector3d> orientation_list,
                      const vector<float> points_gain);


void visualize_terrain_norm(const vector<Eigen::Vector3d> points_list,
                      const vector<Eigen::Vector3d> orientation_list);


void Convert2Navpath();

void Convert2Navpath_3D(vector<rrt_3d::Node_3d *> path_);

void Convert2Navpath_track(vector<Eigen::VectorXd>& path_);

#endif