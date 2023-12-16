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

#include "../include/online_map_merge.hpp"
#include "../include/project2d.hpp"
#include "../include/gridmap_rrt.h"
#include "../include/gridmap_rrt_3d.h"
#include "../include/terrain_map.h"
#include "../include/visualization.h"
#include <nodehash.h>


using namespace std;
using namespace octomap;

typedef octomap::point3d point3d;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> Color_PointCloud;

int Fail_tracked_count = 0;

// ROS publishers
ros::Publisher best_view_point_pub;
ros::Publisher Travel_Path_publisher_visual;
nav_msgs::Path Travel_Path_visualization;

ros::Publisher pub_terrain_map_1;
ros::Publisher pub_terrain_map_tracking;
ros::Publisher pub_terrain_map_2;
ros::Publisher pub_terrain_map_3;

ros::Timer timer;

bool is_init_track_path = false;
bool f_goal_reached = false;

geometry_msgs::PolygonStamped myPolygon;

Eigen::Vector3d Cur_position_SLAMfb_last(0, 0, 0);
Eigen::Vector3d Cur_position_SLAMfb(0, 0, 0);
Eigen::Vector3d Cur_orientation_SLAMfb(0,0,0);   // x, y, z -> roll, pitch, yaw
Eigen::Vector3d Cur_orientation_SLAMfb_last(0,0,0);   // x, y, z -> roll, pitch, yaw

Eigen::Vector3d Last_track_end_position(0,0,0);
Eigen::Vector3d Last_track_end_orientation(0,0,0);

Eigen::Vector3d start_position(0, 0, 0);
Eigen::Vector3d end_position(10, -0, 0);
Eigen::Vector3d end_orientation(0, -0, 0);

int max_iterator_path_planner = 200;
float step_size_path_planner =
    0.8;  //分辨率为0.2，机器人步长为0.3m，所以每一次要走1.5步

// Position best_view_point;
bool rrt_done = true;

float Arrive_distance = 200;  // 0.8

bool orbpose_cb_entered = false;
bool octomap_cb_entered = false;

Eigen::Vector2d cur_goal_nbvp(0, 0);

void goalreachCallback(const std_msgs::Int16& msg){
  cout << "goalreachCallback - msg.data: " << msg.data << endl;
  if(msg.data == 1){
    f_goal_reached = true;
  }
  else{
    f_goal_reached = false;
  }
}

void insertFeatureCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
  // ROS_WARN_STREAM("insertFeatureCloudCallback");
  tf::StampedTransform sensorToWorldTf;
  tf::TransformListener tf_listener;

  tf::TransformListener tf_listener_;
  tf::StampedTransform Camera_World_Tf;

  // struct timeval tv;
  // gettimeofday(&tv, NULL);
  // printf("insertFeatureCloudCallback second:%ld \n", tv.tv_sec);      //秒
  // printf("millisecond:%ld \n", tv.tv_sec * 1000 + tv.tv_usec / 1000); //毫秒

  PointCloud *cloud(new PointCloud);
  PointCloud *cloud_local(new PointCloud);
  // 中间转换
  // pcl::PCLPointCloud2 cloud2;
  // pcl_conversions::toPCL(*cloud_msg, cloud2);
  // pcl::fromPCLPointCloud2(cloud2,*cloud_local);

  // 直接转换
  pcl::fromROSMsg(*cloud_msg, *cloud);  //关键的一句数据的转换

  // octomap::Pointcloud hits;

  // pcl_ros::transformPointCloud(*cloud_local, *cloud, sensorToWorldTf);
  // ROS_WARN_STREAM("Frame_id: 0." << 3);

  auto start_dm = std::chrono::system_clock::now();

  density_map.clear();

  for (auto p : cloud->points) {
    // cur_tree->updateNode(octomap::point3d(p.x, p.y, p.z), true);
    if (std::abs(p.x) < 0.05 && std::abs(p.y) < 0.05) {
      // ROS_ERROR_STREAM("The origin point : " << p.x << " " << p.y);
      continue;
    }
    if ((std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z))) {
      continue;
    } else {
      // hits.push_back(p.x, p.y, p.z);
      // ROS_INFO_STREAM("The point : " << p.x << " " << p.y << " " << p.z);

      // 建立密度地图，这里的索引是整数，需要乘以分辨率才能得到与oc_tree一样的坐标
      // Eigen::Vector3d v_p(p.x, p.y, p.z);
      // if((v_p - Cur_position_SLAMfb).norm() <= 10)    // 增加这个范围限制，加大了计算量，t*2
      {
        int index_x, index_y, index_z;
        if (p.x > 0) {
          index_x = floor((p.x + Octomap_Resolution_) / Octomap_Resolution_);
        } else {
          index_x = floor(p.x / Octomap_Resolution_);
        }

        if (p.y > 0) {
          index_y = floor((p.y + Octomap_Resolution_) / Octomap_Resolution_);
        } else {
          index_y = floor(p.y / Octomap_Resolution_);
        }

        if (p.z > 0) {
          index_z = floor((p.z + Octomap_Resolution_) / Octomap_Resolution_);
        } else {
          index_z = floor(p.z / Octomap_Resolution_);
        }

        // cout << "index: " << index_x << " " << index_y << " " << index_z << "
        // " << endl;
        auto search = density_map.find(Node_hash(index_x, index_y, index_z));
        if (search != density_map.end()) {
          search->second += 1;
          if (search->second >= 1000) {
            search->second = 1000;
          }
        } else {
          density_map[Node_hash(index_x, index_y, index_z)] = 1;
          // hits.push_back(p.x, p.y, p.z);
        }
      }
    }
  }
  cout << "density_map.size(): " << density_map.size() << endl;

  auto end_dm = std::chrono::system_clock::now();
  auto elapsed_seconds_dm =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_dm - start_dm);
  ROS_WARN_STREAM("Build density map takes: " << elapsed_seconds_dm.count()
                                              << " ms");

  if (density_map.size() <= 0) {
    cout << "map not initialized..." << endl;
    return;
  }

  delete cloud;
  delete cloud_local;
}

bool Reach_Goal_func(){
  float _reach_pos_eps = 0.6;
  float _reach_rot_eps = 0.2;
  int _trackpath3d_length = track_path_3d.size();

  if(((Cur_position_SLAMfb.segment(0,2) - track_path_3d[_trackpath3d_length - 1].segment(0,2)).norm() < _reach_pos_eps && ((Cur_orientation_SLAMfb.segment(1,2) - track_path_3d[_trackpath3d_length - 1].segment(4,2)).norm() < _reach_rot_eps || abs((Cur_orientation_SLAMfb.segment(1,2) - track_path_3d[_trackpath3d_length - 1].segment(4,2)).norm() - 2*M_PI) < _reach_rot_eps)) ){
    //  || ((Cur_position_SLAMfb.segment(0,2) - track_path_3d[_trackpath3d_length - 2].segment(0,2)).norm() < _reach_pos_eps && (((Cur_orientation_SLAMfb - track_path_3d[_trackpath3d_length - 2].segment(3,3)).norm() < _reach_rot_eps) || ((Cur_orientation_SLAMfb - track_path_3d[_trackpath3d_length - 2].segment(3,3)).norm() - 2*M_PI) < _reach_rot_eps))

    cout << "reach distance pos 1:" << (Cur_position_SLAMfb - track_path_3d[_trackpath3d_length - 1].segment(0,3)).norm() << endl;
    cout << "reach distance rot 1:" << (Cur_orientation_SLAMfb - track_path_3d[_trackpath3d_length - 1].segment(3,3)).norm() << endl;
    cout << "reach distance pos 2:" << (Cur_position_SLAMfb - track_path_3d[_trackpath3d_length - 2].segment(0,3)).norm() << endl;
    cout << "reach distance rot 2:" << (Cur_orientation_SLAMfb - track_path_3d[_trackpath3d_length - 2].segment(3,3)).norm() << endl;
    return true;
  }
  else{

    cout << "unreach distance pos 1:" << (Cur_position_SLAMfb - track_path_3d[_trackpath3d_length - 1].segment(0,3)).norm() << endl;
    cout << "unreach distance rot 1:" << (Cur_orientation_SLAMfb - track_path_3d[_trackpath3d_length - 1].segment(3,3)).norm() << endl;
    cout << "unreach distance pos 2:" << (Cur_position_SLAMfb - track_path_3d[_trackpath3d_length - 2].segment(0,3)).norm() << endl;
    cout << "unreach distance rot 2:" << (Cur_orientation_SLAMfb - track_path_3d[_trackpath3d_length - 2].segment(3,3)).norm() << endl;
    return false;
  }
}


void timerCallback(const ros::TimerEvent &e) {
  if (!(octomap_cb_entered && orbpose_cb_entered)) {
    cout << "octomap callback or orbpose callback not ready" << endl;
    return;
  }

  // if(Explore_Done_2d && Explore_Done_3d){
  //   ROS_ERROR_STREAM("2D and 3D Exploration End~~~~~~~~");
  //   return;
  // }

  int _trackpath3d_length = track_path_3d.size();
  float _stay_eps = 0.05;    // 若处于静止状态，才开始进行计数
  float _stay_rot_eps = 0.05;    // 若处于静止状态，才开始进行计数

#if 0
  // 未到达则返回
  if(EnsureTrackingAcc && (_trackpath3d_length > 1) && !Reach_Goal_func()){
    
    // 静止了但检测到没达到
    cout << "pos diff: " << (Cur_position_SLAMfb_last - Cur_position_SLAMfb).segment(0,2).norm() << endl;
    if((Cur_position_SLAMfb_last - Cur_position_SLAMfb).segment(0,2).norm() < _stay_eps && (Cur_orientation_SLAMfb_last - Cur_orientation_SLAMfb).norm() < _stay_rot_eps){
      ROS_WARN_STREAM("Not reach path end: COUNTING");
      Fail_tracked_count++;
    }
    else{   // 运动过程中，检测到未达到
      Fail_tracked_count = 0;
    }
    cout << "Path end: " << track_path_3d[_trackpath3d_length - 1].transpose() << endl;

    // 若长时间未到达，则进入recover mode 20
    if(Fail_tracked_count > 50){
      ROS_ERROR_STREAM("Not reach path end......Enter Recover Mode......");

      rrt_3d RRT_3D(Cur_position_SLAMfb, Cur_orientation_SLAMfb, end_position, end_orientation, max_iterator_path_planner, step_size_path_planner);
      g_Height_deviation_map = RRT_3D.Height_deviation_map;

      RRT_3D.Enter_recover_mode();

      if(!track_path_3d.empty()){
        cout << "rrt track_path_3d size: " << track_path_3d.size() << endl;
        Convert2Navpath_track(track_path_3d);
      }    

      visualize_task_view(RRT_3D.g_BestPositionSet_aroundFrontier, RRT_3D.g_BestOrinientSet_aroundFrontier, RRT_3D.g_sampleSet_fpNum, task_view_pub);

      RRT_3D.g_BestPositionSet_aroundFrontier.clear();
      RRT_3D.g_BestOrinientSet_aroundFrontier.clear();
      RRT_3D.g_sampleSet_fpNum.clear();
      
      Fail_tracked_count = 0;
    }
    cout << "RRT_3D.Fail_tracked_count: " << Fail_tracked_count << endl;

    Cur_position_SLAMfb_last = Cur_position_SLAMfb;
    return;
  }
#endif

  Fail_tracked_count = 0;

  ros::Time t1 = ros::Time::now();
  double t_cur = t1.toSec();  //获取的是自1970年一月一日到现在时刻的秒数
  printf("Enter Timer Callback. The time is: %16f\n",
         t_cur);  //打印，%16f表示的是16位宽度的float类型的数字
  
  // 
  rrt_3d RRT_3D(Cur_position_SLAMfb, Cur_orientation_SLAMfb, end_position, end_orientation, max_iterator_path_planner, step_size_path_planner);
  g_Height_deviation_map = RRT_3D.Height_deviation_map;

  auto start_dm = std::chrono::system_clock::now();

  Eigen::Vector2d Pos_maxMI;

  cout << "Explore_Done_2d: " << Explore_Done_2d << endl;
  cout << "Explore_Done_3d: " << Explore_Done_3d << endl;
  if(!Explore_Done_2d){
    Eigen::Vector2d pos(Cur_position_SLAMfb.x(), Cur_position_SLAMfb.y());
    Eigen::Vector2i node_index = ConvertWorld2GridIndex(pos);

    Eigen::Vector2i Idx_maxMI;
    cur_goal = mutual_information_compute(_gridmap, frontier2d_vector, node_index);

    Pos_maxMI = ConvertGridIndex2World(cur_goal);
    cout << "Frontier Pos_maxMI: " << Pos_maxMI.transpose() << endl;

    end_position.x() = Pos_maxMI.x();
    end_position.y() = Pos_maxMI.y();
    end_position.z() = Cur_position_SLAMfb.z();    //! 这里可能存在bug，应该要根据目标点x,y的地形来确定
    // end_position.z() = get_mapz(end_position.segment(0,2));    // 这里可能存在bug，应该要根据目标点x,y的地形来确定
    
    Eigen::Vector3d _unit_vec(1,0,0);
    // cout << "theta_best_mi: " << theta_best << endl;
    Eigen::AngleAxisd angle_axis1(-theta_best, Eigen::Vector3d(0, 0, 1));
    Eigen::Vector3d _direction = angle_axis1.matrix().inverse()*_unit_vec;
    // cout << "(1, 0, 0) theta_best:" << endl << _direction.transpose() << endl;

    _direction = _direction.normalized();
    float retreat_step = 0.0;
    Eigen::Vector3d _query_end_pos = end_position;
    Eigen::Vector3d _adjust_end_pos = end_position;
    
    // && !isInfeatureless(_query_end_pos)  RRT_3D.isInKnown_Feature_Area(_query_end_pos)
    // (_gridmap.data[(ConvertWorld2GridIndex(_query_end_pos.segment(0,2)).x() + ConvertWorld2GridIndex(_query_end_pos.segment(0,2)).y() * _gridmap.info.width)] >= 0)
    while(!isInfeatureless(_query_end_pos) && retreat_step < 2.5){  
      if(!IsInLimitarea(ConvertWorld2GridIndex(_query_end_pos.segment(0,2)))){
        break;
      }
      retreat_step += 0.2;
      _query_end_pos = end_position - retreat_step * _direction;
      // ROS_ERROR_STREAM("end Adjusting Goal......." << _query_end_pos.transpose());
    }

    end_position = _query_end_pos;
    theta_best = find_best_theta(ConvertWorld2GridIndex(end_position.segment(0,2)).x(), ConvertWorld2GridIndex(end_position.segment(0,2)).y(), g_param_2dfsmi_range, 0, false);

    end_orientation.x() = 0;    // 
    end_orientation.y() = 0;    // 
    end_orientation.z() = theta_best;

    ros::Time current_time;
    current_time = ros::Time::now();
    geometry_msgs::PoseStamped best_viewPoint_stamped;
    best_viewPoint_stamped.header.stamp = current_time;
    best_viewPoint_stamped.header.frame_id = "world";
    best_viewPoint_stamped.pose.position.x = end_position.x();
    best_viewPoint_stamped.pose.position.y = end_position.y();
    best_viewPoint_stamped.pose.position.z = end_position.z();    //! 这里可能存在bug，应该要根据目标点x,y的地形来确定
    // best_viewPoint_stamped.pose.position.z = get_mapz(end_position.segment(0,2));

    ROS_WARN_STREAM("Theta_best: " << theta_best);

    Eigen::Vector3d ea(theta_best, 0, 0);  // Z-Y-X
    Eigen::Quaterniond quaternion3;
    quaternion3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());

    best_viewPoint_stamped.pose.orientation.w = quaternion3.w();
    best_viewPoint_stamped.pose.orientation.x = quaternion3.x();
    best_viewPoint_stamped.pose.orientation.y = quaternion3.y();
    best_viewPoint_stamped.pose.orientation.z = quaternion3.z();

    best_view_point_pub.publish(best_viewPoint_stamped);

  }
  else if(Explore_Done_2d && !Explore_Done_3d){
    ROS_ERROR_STREAM("2D Exploration End=========Doing 3D Exploration++++++++++");
    Frontier3D_black_list.clear();
    Frontier3d_Extraction(Cur_position_SLAMfb);
    clusterFrontierAndPublish();

    Eigen::VectorXd Target_pose_frontier3d = RRT_3D.Extract_BestObserve_Frontier3d();

    end_position = Target_pose_frontier3d.segment(0,3);
    end_orientation = Target_pose_frontier3d.segment(3,3);

    ros::Time current_time;
    current_time = ros::Time::now();
    geometry_msgs::PoseStamped best_viewPoint_stamped;
    best_viewPoint_stamped.header.stamp = current_time;
    best_viewPoint_stamped.header.frame_id = "world";
    best_viewPoint_stamped.pose.position.x = end_position.x();
    best_viewPoint_stamped.pose.position.y = end_position.y();
    best_viewPoint_stamped.pose.position.z = end_position.z();

    geometry_msgs::Quaternion goal_quat =
        tf::createQuaternionMsgFromRollPitchYaw(end_orientation(0), end_orientation(1), end_orientation(2));
    cout << "Doing 3D Exploration - quat: " << goal_quat.x << " " << goal_quat.y << " " << goal_quat.z << " " << goal_quat.w << endl;

    best_viewPoint_stamped.pose.orientation.x = goal_quat.x;
    best_viewPoint_stamped.pose.orientation.y = goal_quat.y;
    best_viewPoint_stamped.pose.orientation.z = goal_quat.z;
    best_viewPoint_stamped.pose.orientation.w = goal_quat.w;

    best_view_point_pub.publish(best_viewPoint_stamped);
  }
  else{
    Frontier3D_black_list.clear();
    Frontier3d_Extraction(Cur_position_SLAMfb);
    clusterFrontierAndPublish();

    cout << "Exploration3D Check: " << g_Frontiers_Cluster_pos.size() << endl;
    if(!g_Frontiers_Cluster_pos.empty()){
      Explore_Done_3d = false;
      ROS_ERROR_STREAM("2D Exploration End=========ReDetecting 3D Exploration++++++++++");
      return;
    }

    end_position = Cur_position_SLAMfb;
    end_orientation = Cur_orientation_SLAMfb;
    
    ros::Time current_time;
    current_time = ros::Time::now();
    geometry_msgs::PoseStamped best_viewPoint_stamped;
    best_viewPoint_stamped.header.stamp = current_time;
    best_viewPoint_stamped.header.frame_id = "world";
    best_viewPoint_stamped.pose.position.x = end_position.x();
    best_viewPoint_stamped.pose.position.y = end_position.y();
    best_viewPoint_stamped.pose.position.z = end_position.z();

    geometry_msgs::Quaternion goal_quat =
        tf::createQuaternionMsgFromRollPitchYaw(end_orientation(0), end_orientation(1), end_orientation(2));
    cout << "Doing 3D Exploration - quat: " << goal_quat.x << " " << goal_quat.y << " " << goal_quat.z << " " << goal_quat.w << endl;

    best_viewPoint_stamped.pose.orientation.x = goal_quat.x;
    best_viewPoint_stamped.pose.orientation.y = goal_quat.y;
    best_viewPoint_stamped.pose.orientation.z = goal_quat.z;
    best_viewPoint_stamped.pose.orientation.w = goal_quat.w;

    best_view_point_pub.publish(best_viewPoint_stamped);

    ROS_ERROR_STREAM("2D and 3D Exploration End~~~~~~~~");
    return;
  }

  auto end_dm = std::chrono::system_clock::now();
  auto elapsed_seconds_dm =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_dm - start_dm);
  ROS_WARN_STREAM("Find 2d exploration goal takes: " << elapsed_seconds_dm.count()
                                              << " ms");

  /**********3D RRT planning************/
  // rrt_3d RRT_3D(Cur_position_SLAMfb, Cur_orientation_SLAMfb, end_position, end_orientation, max_iterator_path_planner, step_size_path_planner);

  RRT_3D.end_position_ = end_position;
  RRT_3D.end_orientation_ = end_orientation;
  
  RRT_3D.rrt3d_intersection_point_visualization.clear();
  RRT_3D.rrt3d_sample_position.clear();
  RRT_3D.rrt3d_sample_orientation.clear();
  RRT_3D.rrt3d_sample_point_gain.clear();
  RRT_3D.path_3d.clear();
  bool rrt3d_result = RRT_3D.run_w_perception_layered();    // RRT Planning
  
  if (rrt3d_result) {
    if(!RRT_3D.path_3d.empty()){
      cout << "rrt path_3d size: " << RRT_3D.path_3d.size() << endl;
      Convert2Navpath_3D(RRT_3D.path_3d);   // 显示最终的rrt树
    }
    else{
      cout << "rrt path_3d empty..." << endl;
    }

    if(!track_path_3d.empty()){
      cout << "rrt track_path_3d size: " << track_path_3d.size() << endl;
      Convert2Navpath_track(track_path_3d);
    }    
    else{
      cout << "rrt track_path_3d empty..." << endl;
    }

    cout << "rrt3d sample point size: " << RRT_3D.rrt3d_sample_position.size() << endl;

    visualize_task_view(RRT_3D.g_BestPositionSet_aroundFrontier, RRT_3D.g_BestOrinientSet_aroundFrontier, RRT_3D.g_sampleSet_fpNum);

    RRT_3D.g_BestPositionSet_aroundFrontier.clear();
    RRT_3D.g_BestOrinientSet_aroundFrontier.clear();
    RRT_3D.g_sampleSet_fpNum.clear();
    // visualize_points(RRT_3D.rrt3d_intersection_point);
    // RRT_3D.rrt3d_intersection_point.clear();

    if(RRT_3D.Flag_visualize_intersact_points){
      // visualize_points(RRT_3D.rrt3d_intersection_point_visualization);
      visualize_points_pcl(RRT_3D.rrt3d_intersection_point_visualization);
    }
    else{
      // visualize_points(RRT_3D.rrt3d_sample_position, RRT_3D.rrt3d_sample_orientation, RRT_3D.rrt3d_sample_point_gain);
      visualize_points_pcl(RRT_3D.rrt3d_sample_position);
    }

    Fail_grow_count = 0;
  }
  else{
    visualize_points_pcl(RRT_3D.rrt3d_sample_position);
    if(Fail_grow_count > 8){
      RRT_3D.Enter_recover_mode();
      cout << "RRT_3D.Enter_recover_mode for RRT Cannot Grow...... " << endl;

      if(!track_path_3d.empty()){
        cout << "rrt track_path_3d size: " << track_path_3d.size() << endl;
        Convert2Navpath_track(track_path_3d);
      }    

      visualize_task_view(RRT_3D.g_BestPositionSet_aroundFrontier, RRT_3D.g_BestOrinientSet_aroundFrontier, RRT_3D.g_sampleSet_fpNum);

      RRT_3D.g_BestPositionSet_aroundFrontier.clear();
      RRT_3D.g_BestOrinientSet_aroundFrontier.clear();
      RRT_3D.g_sampleSet_fpNum.clear();
      
      Fail_grow_count = 0;
    }
    cout << "RRT_3D.Fail_grow_count: " << Fail_grow_count << endl;
  }

  // RRT_3D.Height_map = terrain_map.Get_Height_map();
  pub_terrain_map_1.publish(RRT_3D.Get_Height_map());
  pub_terrain_map_2.publish(g_Height_deviation_map);
  // visualize_terrain_norm(RRT_3D.terrain_norm_center_list, RRT_3D.terrain_norm_list);
  // visualize_terrain_norm(g_Frontiers3D_pos, g_globalFrontierNormVec);

  Cur_position_SLAMfb_last = Cur_position_SLAMfb;
}

geometry_msgs::PoseStamped best_viewPoint_stamped;

static int visualization_count = 1;
void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  printf("orbpose_callback second:%ld \n", tv.tv_sec);                //秒
  printf("millisecond:%ld \n", tv.tv_sec * 1000 + tv.tv_usec / 1000); //毫秒

  std::cout << "entered octomapCallback..." << std::endl;

  if (msg->data.empty()) {
    return;
  }

  AbstractOcTree *read_tree = octomap_msgs::msgToMap(*msg);
#if def_octo_generator
  _octree = dynamic_cast<ColorOcTree *>(read_tree);
#else
  _octree = dynamic_cast<OcTree *>(read_tree);
#endif

  auto start_dm = std::chrono::system_clock::now();
#if def_w_uncertainty
  // Perception-aware
  build_realtime_info_semantic_mapping(Cur_position_SLAMfb);
  
  auto end_merge = std::chrono::system_clock::now();
  auto elapsed_seconds_merge =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_merge - start_dm);
  ROS_WARN_STREAM("Adaptive information extract takes: " << elapsed_seconds_merge.count() << " ms");
#endif

  // gridmap for exploration 2d
  publish2Dmap(msg->header, -3, 5.0, Cur_position_SLAMfb);    
  // cout << "frontier2d_vector.size()" << frontier2d_vector.size() << endl;

  Frontier3d_Extraction(Cur_position_SLAMfb);   // 

  Viewpoint_Check_Frontier(Cur_position_SLAMfb, Cur_orientation_SLAMfb);
  publishglobalFrontierCells(g_Frontiers3D_pos);

  clusterFrontierAndPublish();
  auto end_dm = std::chrono::system_clock::now();
  auto elapsed_seconds_dm =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_dm - start_dm);
  ROS_WARN_STREAM("Frontier3d_Extraction takes: " << elapsed_seconds_dm.count()
                                              << " ms");

  // ! for mpc tracking high frequence
  // terrain_mapping _terrain_mapping(Cur_position_SLAMfb, false);   // no smooth
  // pub_terrain_map_tracking.publish(_terrain_mapping.Get_Height_map());

  octomap_cb_entered = true;
}


Eigen::Vector3d Body_dir_init(1, 0, 0);
Eigen::Vector3d Body_dir(1, 0, 0);
std::list<Eigen::Vector3d> Dir_queue(5, Body_dir);

static int count_imu = 0;
Eigen::Matrix3d R3_world_robot_init;

void body_imu_callback(const sensor_msgs::Imu::ConstPtr &msg) {
  // struct timeval tv;
  // gettimeofday(&tv, NULL);
  // printf("second:%ld \n", tv.tv_sec);                                 //秒
  // printf("millisecond:%ld \n", tv.tv_sec * 1000 + tv.tv_usec / 1000); //毫秒

  Eigen::Quaterniond q2(msg->orientation.w, msg->orientation.x,
                        msg->orientation.y, msg->orientation.z);

  // cout << "ORbSLAM pose callback G_T_wc: \n" << G_T_wc << endl;
  if (count_imu <= 10) {
    R3_world_robot_init = q2.matrix();
    count_imu++;
    cout << "count_imu: " << count_imu << endl;
    cout << "R3_world_robot_init: " << R3_world_robot_init << endl;
  } else {
    // R3_world_robot_curr = quat.matrix();
    // cout << "R3_world_robot_curr: " << R3_world_robot_curr << endl;
    // cout << "count_imu: " << count_imu << endl;

    Body_dir = R3_world_robot_init.inverse() * q2.matrix() * Body_dir_init;

    Body_dir.z() = 0;
    Body_dir = Body_dir.normalized();

    //
    Dir_queue.pop_front();
    Dir_queue.push_back(Body_dir);

    // cout << "Dir_queue.size(): " << Dir_queue.size() << endl;

    Eigen::Vector3d Body_dir_sum(0, 0, 0);
    for (auto it = Dir_queue.begin(); it != Dir_queue.end(); it++) {
      Body_dir_sum.x() += (*it).x();
      Body_dir_sum.y() += (*it).y();
      Body_dir_sum.z() += (*it).z();
    }

    Body_dir_avg.x() = Body_dir_sum.x() / 5;
    Body_dir_avg.y() = Body_dir_sum.y() / 5;
    // cout << "Body_dir_avg: " << Body_dir_avg.transpose() << endl;
  }
}

geometry_msgs::Pose m_uavCurrentPose;

void orbpose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  // std::cout << "Entered Orbslam pose Callback..." << std::endl;
  // struct timeval tv;
  // gettimeofday(&tv, NULL);
  // printf("orbpose_callback second:%ld \n", tv.tv_sec);                //秒
  // printf("millisecond:%ld \n", tv.tv_sec * 1000 + tv.tv_usec / 1000); //毫秒

  // m_uavCurrentPose = msg->pose;
  double current_roll = 0;
  double current_pitch = 0;
  double current_yaw = 0;
  
  Cur_orientation_SLAMfb_last = Cur_orientation_SLAMfb;

  tf::Quaternion quat_tf;
  tf::quaternionMsgToTF(msg->pose.orientation, quat_tf);
  tf::Matrix3x3(quat_tf).getRPY(current_roll, current_pitch, current_yaw);
  Cur_orientation_SLAMfb = Eigen::Vector3d(current_roll, current_pitch, current_yaw);
  cout << "ORBSLAM - Cur_orientation_SLAMfb: " << Eigen::Vector3d(current_roll, current_pitch, current_yaw).transpose() << endl;

  Eigen::Quaterniond quat;
  quat.w() = msg->pose.orientation.w;
  quat.x() = msg->pose.orientation.x;
  quat.y() = msg->pose.orientation.y;
  quat.z() = msg->pose.orientation.z;
  g_T_w_c_Fisherinfo.block<3, 3>(0, 0) = quat.toRotationMatrix();

  Eigen::Vector3d translate;
  translate[0] = msg->pose.position.x;
  translate[1] = msg->pose.position.y;
  translate[2] = msg->pose.position.z;
  Cur_position_SLAMfb = translate;
  cout << "ORBSLAM - Cur_position_SLAMfb: " << Cur_position_SLAMfb.transpose() << endl;

  g_T_w_c_Fisherinfo.block<3, 1>(0, 3) = translate;
  // g_Twc_3DMapper.block<3, 1>(0, 3) = translate;
  // cout << "g_Twc_3DMapper: " << g_Twc_3DMapper << endl;

  Eigen::Vector3d Motion_dir_3d;
  Motion_dir_3d = quat.matrix() * Body_dir_init;
  // Motion_dir_3d = Cur_position_SLAMfb - Cur_position_SLAMfb_last;
  Motion_dir_3d.z() = 0;
  Motion_dir_3d = Motion_dir_3d.normalized();
  Motion_dir.x() = Motion_dir_3d.x();
  Motion_dir.y() = Motion_dir_3d.y();

  // ROS_WARN_STREAM("Motion direction SLAM: " << quat.matrix() * Body_dir_init);
  // ROS_WARN_STREAM("Motion direction: " << Motion_dir);
  // ROS_WARN_STREAM("Body direction: " << Body_dir_avg);

  geometry_msgs::PoseStamped pose_msg;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "world";

  Travel_Path_visualization.header = msg->header;
  // orb_path_.header.frame_id = map_frame_id_param_;
  pose_msg.pose.position = msg->pose.position;

  Travel_Path_visualization.poses.push_back(pose_msg);
  Travel_Path_publisher_visual.publish(Travel_Path_visualization);

  orbpose_cb_entered = true;
}

// pitch角与ORBSLAM 的pitch角
void GroundTruth_Cb(const nav_msgs::Odometry::ConstPtr &msg) {
  double current_roll = 0;
  double current_pitch = 0;
  double current_yaw = 0;

  Cur_orientation_SLAMfb_last = Cur_orientation_SLAMfb;

  tf::Quaternion quat_tf;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quat_tf);
  tf::Matrix3x3(quat_tf).getRPY(current_roll, current_pitch, current_yaw);
  Cur_orientation_SLAMfb = Eigen::Vector3d(current_roll, -current_pitch, current_yaw);
  cout << "GroundTruth - Cur_orientation_SLAMfb: " << Cur_orientation_SLAMfb.transpose() << endl;

  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(current_roll,-current_pitch,current_yaw);
  m_uavCurrentPose.orientation = q;
  m_uavCurrentPose.position = msg->pose.pose.position;
  // cout << "m_uavCurrentPose: " << m_uavCurrentPose << endl;

  Eigen::Quaterniond quat;
  quat.w() = m_uavCurrentPose.orientation.w;
  quat.x() = m_uavCurrentPose.orientation.x;
  quat.y() = m_uavCurrentPose.orientation.y;
  quat.z() = m_uavCurrentPose.orientation.z;
  g_T_w_c_Fisherinfo.block<3, 3>(0, 0) = quat.toRotationMatrix();

  Eigen::Vector3d translate;
  translate[0] = msg->pose.pose.position.x;
  translate[1] = msg->pose.pose.position.y;
  translate[2] = msg->pose.pose.position.z;
  Cur_position_SLAMfb = translate;
  cout << "GroundTruth - Cur_position_SLAMfb: " << Cur_position_SLAMfb.transpose() << endl;
  // Cur_position_SLAMfb_last = Cur_position_SLAMfb;

  g_T_w_c_Fisherinfo.block<3, 1>(0, 3) = translate;

  Eigen::Vector3d Motion_dir_3d;
  Motion_dir_3d = quat.matrix() * Body_dir_init;
  // Motion_dir_3d = Cur_position_SLAMfb - Cur_position_SLAMfb_last;
  Motion_dir_3d.z() = 0;
  Motion_dir_3d = Motion_dir_3d.normalized();
  Motion_dir.x() = Motion_dir_3d.x();
  Motion_dir.y() = Motion_dir_3d.y();

  geometry_msgs::PoseStamped pose_msg;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "world";

  Travel_Path_visualization.header = msg->header;
  // orb_path_.header.frame_id = map_frame_id_param_;
  pose_msg.pose.position = msg->pose.pose.position;

  Travel_Path_visualization.poses.push_back(pose_msg);
  Travel_Path_publisher_visual.publish(Travel_Path_visualization);

  orbpose_cb_entered = true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "octomap_planner");
  ros::NodeHandle n;
  // planner planner_object;

#if def_USING_ORBSLAM
  ros::Subscriber orbpose_sub = n.subscribe<geometry_msgs::PoseStamped>(
      "/orb_slam2_rgbd/pose", 1, orbpose_callback);
  ros::Subscriber sub_orbslam_mappoints = n.subscribe<sensor_msgs::PointCloud2>(
      "/orb_slam2_rgbd/map_points", 1, &insertFeatureCloudCallback);
#else
  ros::Subscriber odom_sub =
      n.subscribe<nav_msgs::Odometry>("/ground_truth/camera_state", 1, GroundTruth_Cb);
#endif

  Travel_Path_publisher_visual = n.advertise<nav_msgs::Path> ("/Traveled_path", 1);

#if def_octo_generator
  ros::Subscriber octree_sub = n.subscribe<octomap_msgs::Octomap>(
      "/octomap_full_semantic", 1, octomapCallback);
#else
  ros::Subscriber octree_sub = n.subscribe<octomap_msgs::Octomap>(
      "/octomap_full", 1, octomapCallback);
#endif
  
  ros::Timer timer = n.createTimer(ros::Duration(0.5), timerCallback);   // 周期为t s

  // body_imu用于获取body朝向
  ros::Subscriber sub_view_robot_camera =
  n.subscribe<sensor_msgs::Imu>("/body_imu", 1, body_imu_callback);

  ros::Publisher pub_map_2d =
      n.advertise<nav_msgs::OccupancyGrid>("projected_map2d", 1, true);
  
  pub_terrain_map_1 =
      n.advertise<nav_msgs::OccupancyGrid>("Height_map", 1, true);
  pub_terrain_map_tracking =
      n.advertise<nav_msgs::OccupancyGrid>("Height_map_tracking", 1, true);
  pub_terrain_map_2 =
      n.advertise<nav_msgs::OccupancyGrid>("Height_deviation_map", 1, true);
  pub_terrain_map_3 =
      n.advertise<nav_msgs::OccupancyGrid>("Curvature_map", 1, true);
  
  m_markerAdaptSemanticOctreePub = n.advertise<sensor_msgs::PointCloud2>(
      "AdaptSemantic_pcl", 1);

  // 显示探索范围的设定边界
  ros::Publisher Polygon_pub =
      n.advertise<geometry_msgs::PolygonStamped>("polygonpublisher", 1);

  // mutual information map publisher
  mi_map_pub = n.advertise<nav_msgs::OccupancyGrid>("mi_map", 1, true);

  frontier_pub = n.advertise<visualization_msgs::MarkerArray>(
      "visualization_frontier", 10);


  best_view_point_pub =
      n.advertise<geometry_msgs::PoseStamped>("best_view_point", 1, true);

  PlanningVisualization(n);

  m_binaryMapPub =
      n.advertise<octomap_msgs::Octomap>("octomap_binary", 1, false);
  m_markerOccPub = n.advertise<visualization_msgs::MarkerArray>(
      "occupied_cells_vis_array", 1, false);
  m_markerFreePub = n.advertise<visualization_msgs::MarkerArray>(
      "free_cells_vis_array", 1, false);

  m_markerFrontierPub = n.advertise<visualization_msgs::MarkerArray>(
      "frontier_cells_vis_array", 1, false);
  m_markerClusteredFrontierPub = n.advertise<visualization_msgs::MarkerArray>(
      "frontier_cluster_vis_array", 1, false);

  ros::Subscriber goalreached_sub_ = n.subscribe("/goal_reached", 1, goalreachCallback);

  n.getParam("explore_range_blx", explore_range_blx);
  n.getParam("explore_range_bly", explore_range_bly);
  n.getParam("explore_range_urx", explore_range_urx);
  n.getParam("explore_range_ury", explore_range_ury);
  ROS_INFO("param explore_range_blx: %f", explore_range_blx);
  ROS_INFO("param explore_range_bly: %f", explore_range_bly);
  ROS_INFO("param explore_range_urx: %f", explore_range_urx);
  ROS_INFO("param explore_range_ury: %f", explore_range_ury);

  if (!n.getParam("map_2d/fsmi_range", g_param_2dfsmi_range)) {
    ROS_WARN_STREAM("No g_param_2dfsmi_range specified.");
  }
  ROS_WARN_STREAM("g_param_2dfsmi_range specified : " << g_param_2dfsmi_range);

  if (!n.getParam("map_2d/weight_direction", g_param_weight_direction)) {
    ROS_WARN_STREAM("No g_param_weight_direction specified.");
  }
  ROS_WARN_STREAM("g_param_weight_direction specified : " << g_param_weight_direction);

  if (!n.getParam("map_2d/weight_distance", g_param_weight_distance)) {
    ROS_WARN_STREAM("No g_param_weight_distance specified.");
  }
  ROS_WARN_STREAM("g_param_weight_distance specified : " << g_param_weight_distance);
  
  if (!n.getParam("map_2d/visualize_fpsize", g_param_visualize_fpsize)) {
    ROS_WARN_STREAM("No g_param_visualize_fpsize specified.");
  }
  ROS_WARN_STREAM("g_param_visualize_fpsize specified : " << g_param_visualize_fpsize);
  
  if (!n.getParam("map_2d/thresh_toexplore", g_param_thresh_toexplore)) {
    ROS_WARN_STREAM("No g_param_thresh_toexplore specified.");
  }
  ROS_WARN_STREAM("g_param_thresh_toexplore specified : " << g_param_thresh_toexplore);
  
  if (!n.getParam("rrt_3d/fsmi_range", g_param_3dfsmi_range)) {
    ROS_WARN_STREAM("No g_param_3dfsmi_range specified.");
  }
  ROS_WARN_STREAM("g_param_3dfsmi_range specified : " << g_param_3dfsmi_range);

  if (!n.getParam("buffer_strategy", g_param_buffer_strategy)) {
    ROS_WARN_STREAM("No g_param_buffer_strategy specified.");
  }
  ROS_WARN_STREAM("g_param_buffer_strategy specified : " << g_param_buffer_strategy);

  if (!n.getParam("maze_mode", def_maze_mode)) {
    ROS_WARN_STREAM("No def_maze_mode specified.");
  }
  ROS_WARN_STREAM("def_maze_mode specified : " << def_maze_mode);
  
  if (!n.getParam("terrain_mode", def_terrain_mode)) {
    ROS_WARN_STREAM("No def_terrain_mode specified.");
  }
  ROS_WARN_STREAM("def_terrain_mode specified : " << def_terrain_mode);

  ros::Rate loop_rate(10);

  myPolygon.header.frame_id = "world";
  myPolygon.polygon.points.clear();
  geometry_msgs::Point32 Polygon_point;

  // 显示局部搜索边界点的范围
  Polygon_point.x = explore_range_urx;
  Polygon_point.y = explore_range_bly;
  Polygon_point.z = 0;
  myPolygon.polygon.points.push_back(Polygon_point);

  Polygon_point.x = explore_range_blx;
  Polygon_point.y = explore_range_bly;
  Polygon_point.z = 0;
  myPolygon.polygon.points.push_back(Polygon_point);

  Polygon_point.x = explore_range_blx;
  Polygon_point.y = explore_range_ury;
  Polygon_point.z = 0;
  myPolygon.polygon.points.push_back(Polygon_point);

  Polygon_point.x = explore_range_urx;
  Polygon_point.y = explore_range_ury;
  Polygon_point.z = 0;
  myPolygon.polygon.points.push_back(Polygon_point);

  while (ros::ok()) {
    // ROS_INFO("This is an info statement, and should print");
    // 定义多边形对象

    // 发布边界
    Polygon_pub.publish(myPolygon);

    pub_map_2d.publish(_gridmap);

    // from_des_to_cmdvel();

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
