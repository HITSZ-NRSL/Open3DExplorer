//
// Created by dango on 2021/11/3.
//

#include <vector>
#include <chrono>

// #include "MPC.hpp"
#include "math.h"
#include "mpc.h"
#include "uniform_bspline.h"

////ros include
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/JointState.h>

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

#define PI 3.1415926
// #define N 10

// !下列三个中必须有一个为1
#define def_Predefined_traj_TEST 1    // 通过movebase_goal设定终点的跟踪
#define def_Insert_Traj 0   // 对rrt规划结果进行跟踪
#define SetValue_TEST 0   // 数值仿真测试

#define def_USING_ORBSLAM 1

#define Pub_traveled_traj 1

using namespace Eigen;
using namespace std;

const unsigned int Predict_steps_set = 20;
unsigned int Predict_steps = 10;

vector<Eigen::Vector2d> preset_waypoints;

bool f_goal_reached = false;
// ego_planner::UniformBspline B_spline_trajectory;

ros::Publisher predefine_trajectory_pub;
ros::Publisher pub_terrain_map_tracking;

ros::Publisher trajectory_pub;
ros::Publisher trajectory_predict_pub;
ros::Publisher vel_cmd_pub;
ros::Publisher g_yaw_cmd_pub;
ros::Publisher g_pitch_cmd_pub;
ros::Publisher pub_reach_goal;
ros::Publisher pub_pitch;
ros::Publisher pub_cur_goal;
ros::Publisher pub_terrain_map_1;
ros::Publisher NormVec_publisher;
ros::Publisher Travel_Path_publisher;
nav_msgs::Path Travel_Path_visualization;
nav_msgs::Path Poly_fitting_curve;

nav_msgs::Path predefine_trajectory;
nav_msgs::Path plan_trajectory;
nav_msgs::Path trajectory_predict;

nav_msgs::Path rrt_recieved_path;
vector<Eigen::VectorXd> rrt_recieved_vector;
MPC mpc;

vector<Eigen::VectorXd> Waypoints_rrt;
vector<Eigen::VectorXd> Waypoints;

std::vector<double> delta_yaw_rrt;
std::vector<double> yaw_rrt;
std::vector<double> yaw_recieved_rrt;
std::vector<double> yaw_execute;

Vector3d g_goal_pos;
Vector3d g_goal_orient;
bool is_recieved_goal = false;
int pit_ctrl_count = 0;

int flag_knownmap_track = 0;
void pcdoctomapCallback(const octomap_msgs::Octomap::ConstPtr &msg) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  printf("orbpose_callback second:%ld \n", tv.tv_sec);                //秒
  printf("millisecond:%ld \n", tv.tv_sec * 1000 + tv.tv_usec / 1000); //毫秒
  
  std::cout << "entered KnownoctomapCallback..." << std::endl;
  if (msg->data.empty()) {
    return;
  }

  if(flag_knownmap_track == 1){
    // pub_terrain_map_tracking.publish(Height_map);
  }
  else{
    flag_knownmap_track++;
    octomap::OcTree *_octree_known;

    octomap::AbstractOcTree *read_tree = octomap_msgs::msgToMap(*msg);
    _octree_known = dynamic_cast<octomap::OcTree *>(read_tree);

    Height_map.header.frame_id = "/world";
    Height_map.header.stamp = ros::Time::now();
    
    float Map_range = 40;
    const int Height_map_Scale = 25;    // -127~127

    Height_map.info.width = int(Map_range*2/_octree_known->getResolution()) + 1;
    Height_map.info.height = int(Map_range*2/_octree_known->getResolution()) + 1;
    // ROS_WARN_STREAM("Height_map.info.width: " << Height_map.info.width << " Height_map.info.height: " << Height_map.info.height);

    // might not exactly be min / max of octree:
    Height_map.info.resolution = _octree_known->getResolution();
    Height_map.info.origin.position.x = -Map_range;
    Height_map.info.origin.position.y = -Map_range;
    // ROS_WARN_STREAM("Rebuilding complete 2D map");
    // ROS_WARN_STREAM("Height_map.origin: " << Height_map.info.origin.position.x << " " << Height_map.info.origin.position.y);

    // gridmap_l.data.clear();
    Height_map.data.resize(Height_map.info.width * Height_map.info.height, int(-50));

    vector<Eigen::Vector3d> terrain_cell_set;

    double x_search_min =  -Map_range;
    double x_search_max = Map_range;
    double y_search_min = -Map_range;
    double y_search_max = Map_range;

    for (double x_search = x_search_max; x_search >= x_search_min; x_search -= Height_map.info.resolution)
    {
      for (double y_search = y_search_max; y_search >= y_search_min; y_search -= Height_map.info.resolution)
      {
        double z_min = -2;
        double z_max = 5;
        double cell_z_max = z_min;

        bool _occupied_flag = false;
        for(double z_search = z_max; z_search >= z_min; z_search -= Height_map.info.resolution){
          octomap::point3d temp(x_search, y_search, z_search);
          octomap::OcTreeNode *node = _octree_known->search(temp);        
          if (node != NULL && _octree_known->isNodeOccupied(node))
          {
            // cout << "z_search: " << z_search << endl;
            if(z_search > cell_z_max){
              cell_z_max = z_search;
              // cout << "cell_z_max: " << cell_z_max << endl;
            }
            _occupied_flag = true;
            // _init_z = cell_z_max;
          }
        }
        
        // cout << "cell_z_max given: " << cell_z_max << endl;

        if(_occupied_flag){
          Eigen::Vector3d cell_xyz(x_search, y_search, cell_z_max);
          // ROS_WARN_STREAM("cell_xyz: " << cell_xyz);
          terrain_cell_set.push_back(cell_xyz);
        }
        else{
          Eigen::Vector3d cell_xyz(x_search, y_search, -2);
          // ROS_WARN_STREAM("cell_xyz: " << cell_xyz);
          terrain_cell_set.push_back(cell_xyz);
        }
      }
    }

    for(auto cell:terrain_cell_set){
      Eigen::Vector2d cell_xy(cell.x(), cell.y());
      Eigen::Vector2i node_index;
      node_index.x() = std::ceil((cell.x() - Height_map.info.origin.position.x) / Height_map.info.resolution);
      node_index.y() = std::ceil((cell.y() - Height_map.info.origin.position.y) / Height_map.info.resolution);
      // cout << "terrain_cell_set: " << cell.transpose() << endl;

      int Idx = Height_map.info.width * (node_index.y() - 1) + node_index.x() - 1;
      // int Idx = mapIdx(node_index.x() - 1, node_index.y() - 1);

      // cout << "cell_z_norm: " << cell_z_norm << endl;
      // Height_map.data[Idx] = floor((cell_z_norm));    // else is the initual value 0
      // double _init_z = Initual_map_value/Height_map_Scale;
      if(abs(cell.z() - (-2)) < 5e-2){
        Height_map.data[Idx] = floor(((-2)*Height_map_Scale));    // else is the initual value 0
        // cout << "origin height:" << _init_z << " int: " << floor(Height_map.data[Idx]) << endl;
      }
      else{
        Height_map.data[Idx] = floor((cell.z()*Height_map_Scale));    // else is the initual value 0
        // cout << "origin height:" << cell.z() << " int: " << floor(Height_map.data[Idx]) << endl;
      }
    }
  }
}

double cur_cam_yaw_last;
bool b_received_pose = false;
void orbpose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  ROS_INFO("Entered Orbslam pose Callback...");
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(cur_cam_roll, cur_cam_pitch,
                             cur_cam_yaw); //进行转换
  cur_cam_pitch = -cur_cam_pitch;

  cout << "FEEDBACK - cur_cam_orient: " << cur_cam_roll << " " << cur_cam_pitch << " "
       << cur_cam_yaw << endl;

  g_orientation_fb.x() = cur_cam_roll;
  g_orientation_fb.y() = cur_cam_pitch;
  g_orientation_fb.z() = cur_cam_yaw;

  // t_w_c.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(3, 3);
  g_position_fb.x() = msg->pose.position.x;
  g_position_fb.y() = msg->pose.position.y;
  g_position_fb.z() = msg->pose.position.z;
  // cout << "g_position_fb: " << g_position_fb.x() << " " << g_position_fb.y() <<
  // " " << g_position_fb.z() << endl;
  cout << "FEEDBACK - cur_cam_position: " << g_position_fb.transpose() << endl;

  geometry_msgs::PoseStamped pose_msg;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "world";

  Travel_Path_visualization.header = msg->header;
  // orb_path_.header.frame_id = map_frame_id_param_;
  pose_msg.pose.position = msg->pose.position;

  Travel_Path_visualization.poses.push_back(pose_msg);
  // Travel_Path_publisher.publish(Travel_Path_visualization);

  b_received_pose = true;
}

// Not Used
void body_imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
  // struct timeval tv;
  // gettimeofday(&tv, NULL);
  // printf("second:%ld \n", tv.tv_sec);                                 //秒
  // printf("millisecond:%ld \n", tv.tv_sec * 1000 + tv.tv_usec / 1000); //毫秒
  // printf("microsecond:%ld \n",tv.tv_sec*1000000 + tv.tv_usec); //微秒

  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->orientation, quat);
  tf::Matrix3x3(quat).getRPY(cur_base_roll, cur_base_pitch,
                             cur_base_yaw);   // 朝上为正
  cout << "cur_base_orient from imu: " << cur_base_roll << " " << cur_base_pitch << " "
       << cur_base_yaw << endl;
  
}

tf::Quaternion quat_camera_cb;
void CameraState_Cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  // cout << "msg->pose.pose.orientation: " << msg->pose.pose.orientation << endl;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quat_camera_cb);

  cout << "quat_camera_cb: " << quat_camera_cb.x() << " " << quat_camera_cb.y() << " " << quat_camera_cb.z() << " " << quat_camera_cb.w() << endl;

  tf::Matrix3x3(quat_camera_cb).getRPY(cur_cam_roll, cur_cam_pitch,
                                       cur_cam_yaw); //进行转换
  // cur_cam_pitch = -cur_cam_pitch;

  cout << "FEEDBACK - cur_cam_orient: " << cur_cam_roll << " " << cur_cam_pitch << " "
       << cur_cam_yaw << endl;

  g_position_fb.x() = msg->pose.pose.position.x;
  g_position_fb.y() = msg->pose.pose.position.y;
  g_position_fb.z() = msg->pose.pose.position.z;
  cout << "FEEDBACK - cur_cam_position: " << g_position_fb.transpose() << endl;

  g_orientation_fb.x() = cur_cam_roll;
  g_orientation_fb.y() = cur_cam_pitch;
  g_orientation_fb.z() = cur_cam_yaw;
  
  Poly_fitting_curve.poses.clear();
  Poly_fitting_curve.header = msg->header;
  for(int i = 0; i < test_pos_visual.size(); ++i){
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = test_pos_visual[i].x();
    pose_msg.pose.position.y = test_pos_visual[i].y();
    pose_msg.pose.position.z = test_pos_visual[i].z();

    geometry_msgs::Quaternion goal_quat =
        tf::createQuaternionMsgFromRollPitchYaw(test_rot_visual[i].x(), test_rot_visual[i].y(),test_rot_visual[i].z());
    pose_msg.pose.orientation.x = goal_quat.x;
    pose_msg.pose.orientation.y = goal_quat.y;
    pose_msg.pose.orientation.z = goal_quat.z;
    pose_msg.pose.orientation.w = goal_quat.w;

    Poly_fitting_curve.poses.push_back(pose_msg);
  }
  NormVec_publisher.publish(Poly_fitting_curve);

  geometry_msgs::PoseStamped pose_msg;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "world";
  Travel_Path_visualization.header = msg->header;
  // orb_path_.header.frame_id = map_frame_id_param_;
  pose_msg.pose.position = msg->pose.pose.position;
  Travel_Path_visualization.poses.push_back(pose_msg);
#if Pub_traveled_traj
  Travel_Path_publisher.publish(Travel_Path_visualization);
#endif

  std_msgs::Float64 pub_cur_pitch;
  pub_cur_pitch.data = cur_cam_pitch;
  pub_pitch.publish(pub_cur_pitch);

  b_received_pose = true;
}

double pitch_base2cam, yaw_base2cam;
void JointState_Cb(const sensor_msgs::JointState::ConstPtr &msg){
  pitch_base2cam = -msg->position[0];
  yaw_base2cam = -msg->position[1];
  cout << "joint_state pitch: " << pitch_base2cam << " yaw_state: " << yaw_base2cam << endl;
}

bool enter_basestate_cb = false;
void BaseState_Cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
  // tf::Matrix3x3(quat).getRPY(cur_base_pitch, cur_base_roll, 
  //                            cur_base_yaw); // plt_base
  tf::Matrix3x3(quat).getRPY(cur_base_roll, cur_base_pitch, 
                             cur_base_yaw); // landshaker
  cout << "FEEDBACK - origin cur_base_quat: " << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << endl;
  cout << "FEEDBACK - origin cur_base_orient: " << cur_base_roll << " " << cur_base_pitch << " " << cur_base_yaw << endl;

  cur_base_pitch = -cur_base_pitch;
  cur_base_roll = -cur_base_roll;
  cur_base_yaw_origin = cur_base_yaw;

  cout << "FEEDBACK - cur_base_orient: " << cur_base_roll << " " << cur_base_pitch << " " << cur_base_yaw << endl;
  
  cout << "_ref_base_yaw: " << _ref_base_yaw << endl;
  if (abs(_ref_base_yaw - cur_base_yaw) > M_PI)
  {
    cout << "Before cur_base_yaw feedback: " << cur_base_yaw << endl;

    cur_base_yaw = (_ref_base_yaw - cur_base_yaw > 0) ? (cur_base_yaw + 2 * M_PI) : (cur_base_yaw - 2 * M_PI);

    cout << "After cur_base_yaw feedback: " << cur_base_yaw << endl;
  }

  enter_basestate_cb = true;
}

bool heightmap_initialed = false;
void HeightMapCb(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  Height_map.header = msg->header;
  Height_map.info = msg->info;
  ROS_INFO("Got map %d %d", Height_map.info.width, Height_map.info.height);
  cout << "Got map resolution: " << Height_map.info.resolution << endl;
  Height_map.data.resize(Height_map.info.width * Height_map.info.height, 0);

  for (unsigned int x = 0; x < Height_map.info.height; x++)
    for (unsigned int y = 0; y < Height_map.info.width; y++)
      Height_map.data[x + Height_map.info.width * y] =
          ((msg->data[x + Height_map.info.width * y]));

  pub_terrain_map_1.publish(Height_map);

  ROS_INFO("pub map %d %d", Height_map.info.width, Height_map.info.height);
  heightmap_initialed = true;
}

// bool update_rrt_ctrl = true;
bool rrt_update = false;
bool rrt_initialed = false;
void rrtpath_callback(const nav_msgs::Path::ConstPtr &msg)
{
  ROS_INFO("Enter rrt path callback.");
  if (!b_received_pose)
  {
    cout << "Not enter orbpose callback..." << endl;
    return;
  }

  if (msg->poses.size() <= 1)
  {
    cout << "msg.size <= 1" << endl;
    return;
  }
  else
  {
    rrt_initialed = true;
  }

  cout << "msg.size : " << msg->poses.size() << endl;

  int rrtpath_size = msg->poses.size();

  if (rrtpath_size < Predict_steps_set)
  {
    Predict_steps = msg->poses.size();
  }
  else
  {
    Predict_steps = Predict_steps_set;
  }

  rrt_recieved_path.poses.clear();
  rrt_recieved_vector.clear();
  tf::Quaternion quat;
  double roll, pitch, yaw; //定义存储r\p\y的容器
  yaw_recieved_rrt.clear();
  delta_yaw_rrt.clear();
  for (int i = 0; i < Predict_steps; ++i)
  {
    Eigen::VectorXd _node_vec(6); // x,y,z,r,p,y

    _node_vec(0) = msg->poses[i].pose.position.x;
    _node_vec(1) = msg->poses[i].pose.position.y;
    _node_vec(2) = msg->poses[i].pose.position.z;

    tf::quaternionMsgToTF(msg->poses[i].pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //进行转换

    _node_vec(3) = roll;
    _node_vec(4) = pitch;
    _node_vec(5) = yaw;

    rrt_recieved_vector.push_back(_node_vec);
    cout << "rrt node vec: " << _node_vec.transpose() << endl;
    // yaw_rrt.push_back(yaw);

    geometry_msgs::PoseStamped _node_pose;
    _node_pose.pose.position = msg->poses[i].pose.position;
    _node_pose.pose.orientation = msg->poses[i].pose.orientation;

    _node_pose.header.frame_id = "/world";
    rrt_recieved_path.poses.push_back(_node_pose);
    // cout << "rrt_recieved_path.poses[i]: " << rrt_recieved_path.poses[i].pose.orientation << endl;

    tf::quaternionMsgToTF(rrt_recieved_path.poses[i].pose.orientation,
                          quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //进行转换
    cout << "rrt_recieved_path[i].yaw: " << yaw << endl;

#if (def_Insert_Traj == 0)
    yaw_rrt.push_back(yaw);
#endif
  }

#if (def_Insert_Traj == 1)
  for (int i = 0; i < rrt_recieved_vector.size() - 1; ++i)
  {
    if (abs(rrt_recieved_vector[i + 1][5] - rrt_recieved_vector[i][5]) > M_PI)
    {
      cout << "Before yaw: " << rrt_recieved_vector[i + 1][5] << endl;

      rrt_recieved_vector[i + 1][5] = (rrt_recieved_vector[i + 1][5] - rrt_recieved_vector[i][5] > 0) ? (rrt_recieved_vector[i + 1][5] - 2 * M_PI) : (2 * M_PI + rrt_recieved_vector[i + 1][5]);

      cout << "Afetr yaw: " << rrt_recieved_vector[i + 1][5] << endl;
    }
  }

  for (int i = 0; i < rrt_recieved_vector.size() - 1; ++i)
  {
    float _total_distance = (rrt_recieved_vector[i + 1].segment(0, 2) - rrt_recieved_vector[i].segment(0, 2)).norm();

    Eigen::ArrayXXd _delta_rotation_abs = (rrt_recieved_vector[i + 1].segment(3, 3) - rrt_recieved_vector[i].segment(3, 3)).array().abs();
    float _total_rotation_distance = _delta_rotation_abs.maxCoeff();
    cout << "delta rotation: " << (rrt_recieved_vector[i + 1].segment(3, 3) - rrt_recieved_vector[i].segment(3, 3)).transpose() << endl;
    cout << "max_delta: " << _total_rotation_distance << endl;

    float _step_size = dt * max_v;              // step_size与cost数量级有关，因此cost系数需要修改
    float _step_rotation_size = dt * max_w_inw; // step_size与cost数量级有关，因此cost系数需要修改

    int _insert_steps = max(ceil(_total_distance / _step_size), ceil(_total_rotation_distance / _step_rotation_size));

    cout << "i: " << i << endl;
    cout << "rrt_recieved_vector.size: " << rrt_recieved_vector.size() << endl;
    cout << "rrt node: " << rrt_recieved_vector[i].transpose() << endl;
    yaw_recieved_rrt.push_back(rrt_recieved_vector[i][5]);

    Eigen::VectorXd _node_i1 = rrt_recieved_vector[i + 1];
    Eigen::VectorXd _node_i = rrt_recieved_vector[i];
    int j;
    for (j = 1; j < _insert_steps; ++j)
    {
      Eigen::VectorXd _node_vec(6); // x,y,z,r,p,y
      cout << "j: " << j << endl;

      _node_vec = j * (_node_i1 - _node_i) / _insert_steps + _node_i;

      cout << "insert node: " << _node_vec.transpose() << endl;
      rrt_recieved_vector.insert(rrt_recieved_vector.begin() + i + j, _node_vec);

      yaw_recieved_rrt.push_back(_node_vec[5]);
      // i++;
    }

    i += (j - 1);
  }
  yaw_recieved_rrt.push_back(rrt_recieved_vector.back()[5]);
  
  cout << "rrt_recieved_vector.size: " << rrt_recieved_vector.size() << endl;
  cout << "yaw_rrt size:" << yaw_recieved_rrt.size() << endl;

  Predict_steps = rrt_recieved_vector.size();
#endif

  rrt_update = true;
}

Eigen::Quaternion<double> goal_quat_eigen;
std_msgs::Int16 _reached;
static int goal_cb_count = 0;
void goalposeCallback(const geometry_msgs::PoseStampedConstPtr &waypoint)
{
  if(!f_goal_reached && goal_cb_count!=0)
    return;
  
  goal_cb_count = goal_cb_count > 1000 ? 1 : (++goal_cb_count);

  //成功接收到了点的坐标信息
  ROS_INFO("pose show start!");
  ROS_INFO("stamp is %f", waypoint->header.stamp.toSec());
  ROS_INFO("pose x is %f", waypoint->pose.position.x);
  ROS_INFO("pose y is %f", waypoint->pose.position.y);
  ROS_INFO("pose z is %f", waypoint->pose.position.z);

  g_goal_pos.x() = waypoint->pose.position.x;
  g_goal_pos.y() = waypoint->pose.position.y;
  g_goal_pos.z() = g_position_fb.z();

  tf::Quaternion quat;
  tf::quaternionMsgToTF(waypoint->pose.orientation,
                        quat);
  double roll, pitch, yaw;                      //定义存储r\p\y的容器
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //进行转换
  pitch = -pitch;
  ROS_INFO("goal roll is %f", roll);
  ROS_INFO("goal pitch is %f", pitch);
  ROS_INFO("goal yaw is %f", yaw);

  g_goal_orient.x() = g_orientation_fb.x();
  g_goal_orient.y() = pitch;
  g_goal_orient.z() = yaw;

  is_recieved_goal = true;

  pub_cur_goal.publish(waypoint);
}

bool Reach_Goal_func(){
  float _reach_pos_eps = 4;
  // float _reach_pos_eps = 1;
  float _reach_rot_eps = 0.3;
  
  // float _reach_pos_eps = 0.8;
  // float _reach_rot_eps = 0.2;
  
  int _trackpath3d_length = Waypoints_rrt.size();
  cout << "_trackpath3d_length: " << _trackpath3d_length << endl;

  if(_trackpath3d_length < 1){
    return true;
  }
  Eigen::Vector2d _orient_fb(g_orientation_fb(1),g_orientation_fb(2));

  if(((g_position_fb.segment(0,2) - Waypoints_rrt.back().segment(0,2)).norm() < _reach_pos_eps && ((_orient_fb - Waypoints_rrt.back().segment(4,2)).norm() < _reach_rot_eps || abs((_orient_fb - Waypoints_rrt.back().segment(4,2)).norm() - 2*M_PI) < _reach_rot_eps)) ){
    //  || ((g_position_fb.segment(0,2) - track_path_3d[_trackpath3d_length - 2].segment(0,2)).norm() < _reach_pos_eps && (((g_orientation_fb - track_path_3d[_trackpath3d_length - 2].segment(3,3)).norm() < _reach_rot_eps) || ((g_orientation_fb - track_path_3d[_trackpath3d_length - 2].segment(3,3)).norm() - 2*M_PI) < _reach_rot_eps))

    cout << "reach distance pos 1:" << (g_position_fb.segment(0,2) - Waypoints_rrt.back().segment(0,2)).norm() << endl;
    cout << "reach distance rot 1:" << (_orient_fb - Waypoints_rrt.back().segment(4,2)).transpose() << endl;
    return true;
  }
  else{
    cout << "unreach distance pos 1:" << (g_position_fb.segment(0,2) - Waypoints_rrt.back().segment(0,2)).norm() << endl;
    cout << "unreach distance rot 1:" << (_orient_fb - Waypoints_rrt.back().segment(4,2)).transpose() << endl;
    return false;
  }
}

void Predefined_Trajectory()
{

  Waypoints_rrt.clear();

  Eigen::VectorXd _waypoint(6);
  _waypoint << g_position_fb.x(), g_position_fb.y(), g_position_fb.z(), g_orientation_fb.x(), g_orientation_fb.y(), cur_cam_yaw;
  cout << "Predefined_Trajectory - Waypoints_rrt [0] : " << _waypoint.transpose() << endl;
  Waypoints_rrt.push_back(_waypoint);

  cout << "yaw j: " << cur_cam_yaw << " yaw j+1: " << g_goal_orient.z() << endl;
  if (abs(g_goal_orient.z() - cur_cam_yaw) > M_PI)
  {
    g_goal_orient.z() = (cur_cam_yaw > 0) ? (M_PI + M_PI + g_goal_orient.z()) : (-M_PI - (M_PI - g_goal_orient.z()));
    cout << "After yaw j: " << cur_cam_yaw << " yaw j+1: " << g_goal_orient.z() << endl;
  }

  cout << "Predefined_Trajectory - g_goal_orient : " << g_goal_orient.transpose() << endl;

#if (def_USING_ORBSLAM == 1)
  float _total_distance = (g_goal_pos.segment(0, 2) - g_position_fb.segment(0, 2)).norm();
  cout << "max_delta_distance: " << _total_distance << endl;
  Eigen::ArrayXXd _delta_rotation_abs = (g_goal_orient - g_orientation_fb).array().abs();
  float _total_rotation_distance = _delta_rotation_abs.maxCoeff();
  cout << "delta rotation: " << (g_goal_orient - g_orientation_fb).transpose() << endl;
  cout << "max_delta_rotation: " << _total_rotation_distance << endl;

  float _step_size = dt * max_v;              // step_size与cost数量级有关，因此cost系数需要修改
  float _step_rotation_size = dt * max_w_inw * 0.8; // step_size与cost数量级有关，因此cost系数需要修改

  int _insert_steps = max(ceil(_total_distance / _step_size), ceil(_total_rotation_distance / _step_rotation_size));

  Predict_steps = _insert_steps;
#else
  float _total_distance = (g_goal_pos - g_position_fb).norm();
  cout << "_total_distance: " << _total_distance << endl;

  float _step_size = dt * max_v;    // step_size与cost数量级有关，因此cost系数需要修改
  Predict_steps = ceil(_total_distance / _step_size);
#endif

  cout << "Predict_steps: " << Predict_steps << endl;


  double _x,_y,_z,_r,_p,_ya;
  // waypoint 0 为当前位置
  for (int i = 1; i < Predict_steps; ++i)
  {
    _x = i * (g_goal_pos.x() - g_position_fb.x()) / Predict_steps + g_position_fb.x();
    _y = i * (g_goal_pos.y() - g_position_fb.y()) / Predict_steps + g_position_fb.y();
    _z = i * (g_goal_pos.z() - g_position_fb.z()) / Predict_steps + g_position_fb.z();

    _r = i * (g_goal_orient.x() - g_orientation_fb.x()) / Predict_steps + g_orientation_fb.x();   // roll
    _p = i * (g_goal_orient.y() - g_orientation_fb.y()) / Predict_steps + g_orientation_fb.y(); // pitch
    _ya = i * (g_goal_orient.z() - cur_cam_yaw) / Predict_steps + cur_cam_yaw;     // yaw
    
    _p = 0;
    _ya = g_goal_orient.z();
    // _p = g_goal_orient.y();    // 不进行插值

    Eigen::VectorXd _waypoint_(6);
    _waypoint_ << _x,_y,_z,_r,_p,_ya;

    Waypoints_rrt.push_back(_waypoint_);

    cout << "Waypoints_rrt: " << Waypoints_rrt[i].transpose() << endl;
  }
  Eigen::VectorXd _waypoint_(6);
  _waypoint_ << g_goal_pos.x(),g_goal_pos.y(),g_goal_pos.z(),g_goal_orient.x(),g_goal_orient.y(),g_goal_orient.z();
  Waypoints_rrt.push_back(_waypoint_);

  // Waypoints_rrt.push_back(Waypoints_rrt.back());
  Predict_steps++;

  rrt_initialed = true;
  heightmap_initialed = true;
  rrt_update = false;
}

int excute_idx = 0;
bool init_predefine_traj = false;
static int goal_set_count = 0;
void timerCallback(const ros::TimerEvent &e)
{
  ros::Time t1 = ros::Time::now();
  double t_cur = t1.toSec(); //获取的是自1970年一月一日到现在时刻的秒数
  printf("The time is: %16f\n",
         t_cur); //打印，%16f表示的是16位宽度的float类型的数字

  if(goal_cb_count == 0){
    g_goal_pos = g_position_fb;
    g_goal_orient = g_orientation_fb;
  }

  f_goal_reached = Reach_Goal_func();

  if(f_goal_reached){
    ROS_WARN_STREAM("Reached Goal~~~~~~~~");
    _reached.data = 1;
    is_recieved_goal = true;
    goal_set_count++;
  }
  else{
    ROS_WARN_STREAM("Not Reach Goal~~~~~~~~");
    _reached.data = 0;
  }
  cout << "msg.data: " << _reached.data << endl;
  
  if (!init_predefine_traj && enter_basestate_cb && b_received_pose && is_recieved_goal)
  {
    Predefined_Trajectory();
    is_recieved_goal = false;
    // init_predefine_traj = true;
    excute_idx = 0;
  }

  cout << "rrt_initialed: " << rrt_initialed << endl;
  if (!rrt_initialed)
  {
    cout << "rrt not initialed..." << endl;
    return;
  }

  if (!rrt_update)
  {
    ros::Time t1 = ros::Time::now();
    double t_cur = t1.toSec();//获取的是自1970年一月一日到现在时刻的秒数
    
    Waypoints.clear();
    // Waypoints.resize(Predict_steps);

    cout << "rrt not update..." << endl;

    // 用于RRT没更新时的路径跟踪。在未执行的路点中寻找与当前位置最近的路点的下一个路点及后续的路点，作为新的跟踪路径
    // 存在的问题：当节点的位置在一起，但旋转都不同时，会直接导致路径中只存在最后一个路点
    float _min_distance_pos = 1e5;
    float _min_distance_orient = 1e5;
    int _min_idx = 0;
    cout << "Waypoints_rrt.size: " << Waypoints_rrt.size() << endl;
    ROS_INFO_STREAM("excute_idx: " << excute_idx);
    for (int i = excute_idx; i < Waypoints_rrt.size(); ++i)
    {
      float _distance_to_curpos = (g_position_fb.segment(0, 2) - Waypoints_rrt[i].segment(0, 2)).norm();
      ROS_INFO_STREAM("_DIS_pos: " << _distance_to_curpos);
      float _distance_to_orient = pow((cur_cam_yaw - Waypoints_rrt[i][5]), 2);
      ROS_INFO_STREAM("_DIS_rot: " << _distance_to_orient);
      float _step_dis = (g_position_fb.segment(0, 2) - Waypoints_rrt.back().segment(0, 2)).norm()/(dt * max_v);
      float _step_rot = (g_orientation_fb.segment(0, 2) - Waypoints_rrt.back().segment(3, 2)).norm()/(dt * max_w_inw);

      if(_step_dis >= _step_rot)
      // if((g_position_fb.segment(0, 2) - Waypoints_rrt.back().segment(0, 2)).norm() > 1)
      {
        cout << "goal distance: " << (g_position_fb.segment(0, 2) - Waypoints_rrt.back().segment(0, 2)).norm() << endl;
        if (_distance_to_curpos < _min_distance_pos)
        {
          ROS_WARN_STREAM("i: " << i << "_min_distance_pos: " << _min_distance_pos);
          _min_distance_pos = _distance_to_curpos;
          _min_idx = i;
        }
      }
      else{
        if (_distance_to_orient < _min_distance_orient)
        {
          ROS_WARN_STREAM("i: " << i << "_distance_to_orient: " << _distance_to_orient);
          _min_distance_orient = _distance_to_orient;
          _min_idx = i;
        }
      }
    }
    excute_idx = _min_idx;
    ROS_INFO_STREAM("min_DIS: " << _min_distance_pos);
    ROS_INFO_STREAM("_min_distance_orient: " << _min_distance_orient);
    ROS_INFO_STREAM("_min_idx: " << _min_idx);

    Eigen::VectorXd _waypoint(6);
    _waypoint << g_position_fb.x(), g_position_fb.y(), g_position_fb.z(), g_orientation_fb.x(), g_orientation_fb.y(), cur_cam_yaw;
    Waypoints.push_back(_waypoint);

    cout << "Waypoint[" << 0 << "]: " << Waypoints[0].transpose() << endl;

#if 0 //! 目标为终点位姿
    cout << "yaw j: " << cur_cam_yaw << " yaw j+1: " << g_goal_orient.z() << endl;
    if (abs(g_goal_orient.z() - cur_cam_yaw) > M_PI)
    {
      g_goal_orient.z() = (cur_cam_yaw > 0) ? (M_PI + M_PI + g_goal_orient.z()) : (-M_PI - (M_PI - g_goal_orient.z()));
      cout << "After yaw j: " << cur_cam_yaw << " yaw j+1: " << g_goal_orient.z() << endl;
    }

    double _x,_y,_z,_r,_p,_ya;
    // waypoint 0 为当前位置
    Predict_steps = Waypoints_rrt.size() - _min_idx;
    for (int i = 1; i < Predict_steps; ++i)
    {
      _x = i * (g_goal_pos.x() - Waypoints[0][0]) / Predict_steps + Waypoints[0][0];
      _y = i * (g_goal_pos.y() - Waypoints[0][1]) / Predict_steps + Waypoints[0][1];
      _z = i * (g_goal_pos.z() - Waypoints[0][2]) / Predict_steps + Waypoints[0][2];


      _r = i * (g_goal_orient.x() - Waypoints[0][3]) / Predict_steps + Waypoints[0][3];   // roll
      _p = i * (g_goal_orient.y() - Waypoints[0][4]) / Predict_steps + Waypoints[0][4]; // pitch
      // _r = 0;
      // _p = 0;
      _ya = i * (g_goal_orient.z() - Waypoints[0][5]) / Predict_steps + Waypoints[0][5];     // yaw

      Eigen::VectorXd _waypoint_(6);
      _waypoint_ << _x,_y,_z,_r,_p,_ya;

      Waypoints.push_back(_waypoint_);

      cout << "Waypoint[" << i << "]: " << Waypoints[i].transpose() << endl;
    }

    Eigen::VectorXd _waypoint_(6);
    _waypoint_ << g_goal_pos.x(),g_goal_pos.y(),g_goal_pos.z(),g_goal_orient.x(),g_goal_orient.y(),g_goal_orient.z();
    Waypoints.push_back(_waypoint_);
    Predict_steps++;
    cout << "Waypoint[" << Predict_steps-1 << "]: " << Waypoints[Predict_steps-1].transpose() << endl;

#else   // !目标参考轨迹为一系列值, 可以避免过冲
      // vector<double> _origin_Waypoints_rrt_5;
      // for(int i = 0; i < Waypoints_rrt.size(); ++i){
      //   _origin_Waypoints_rrt_5.push_back(Waypoints_rrt[i][5]);
      // }
      int j = 1;
      for (int i = _min_idx + 1; i < Waypoints_rrt.size(); ++i)
      {
        cout << "yaw j: " << Waypoints[j - 1][5] << " yaw j+1: " << Waypoints_rrt[i][5] << endl;
        if (abs(Waypoints_rrt[i][5] - Waypoints[j - 1][5]) > M_PI)
        {
          Waypoints_rrt[i][5] = (Waypoints[j - 1][5] > 0) ? (M_PI + M_PI + Waypoints_rrt[i][5]) : (-M_PI - (M_PI - Waypoints_rrt[i][5]));
          cout << "After yaw j: " << Waypoints[j - 1][5] << " yaw j+1: " << Waypoints_rrt[i][5] << endl;
        }

        double des_roll = 0;    // g_orientation_fb.x()
        Eigen::VectorXd _waypoint(6);
        _waypoint << Waypoints_rrt[i][0], Waypoints_rrt[i][1], Waypoints_rrt[i][2], des_roll, Waypoints_rrt[i][4], Waypoints_rrt[i][5];
        Waypoints.push_back(_waypoint);
        cout << "Waypoint[" << j << "]: " << Waypoints[j].transpose() << endl;
        
        j++;
      }
      
      // for(int i = 0; i < Waypoints_rrt.size(); ++i){
      //   Waypoints_rrt[i][5] = _origin_Waypoints_rrt_5[i];
      // }
      Predict_steps = j;
      if (abs(g_goal_orient.z() - cur_cam_yaw) > M_PI)
      {
        g_goal_orient.z() = (cur_cam_yaw > 0) ? (M_PI + M_PI + g_goal_orient.z()) : (-M_PI - (M_PI - g_goal_orient.z()));
        cout << "After yaw j: " << cur_cam_yaw << " yaw j+1: " << g_goal_orient.z() << endl;
      }
      // Eigen::VectorXd _waypoint_(6);
      // _waypoint_ << g_goal_pos.x(),g_goal_pos.y(),g_goal_pos.z(),g_orientation_fb.x(),g_goal_orient.y(),g_goal_orient.z();
      // cout << "final _waypoint: " << _waypoint_.transpose() << endl;
      // Waypoints.push_back(_waypoint_);
      // Predict_steps++;
      cout << "Waypoint[" << Predict_steps-1 << "]: " << Waypoints[Predict_steps-1].transpose() << endl;
#endif

    // 首尾两点xy坐标向量
    if(Predict_steps >= 3)
    {
      Eigen::Vector2d prev_pos(Waypoints[0][0], Waypoints[0][1]);
      Eigen::Vector2d last_pos(Waypoints[Predict_steps - 1][0], Waypoints[Predict_steps - 1][1]);
      cout << "prev_pos: " << prev_pos.transpose() << endl;
      cout << "last_pos: " << last_pos.transpose() << endl;

      Eigen::Vector2d _dir = last_pos - prev_pos;
      _ref_base_yaw = atan2(_dir.y(), _dir.x());
      double _ref_base_yaw_before = _ref_base_yaw;

      cout << "cur_base_yaw: " << cur_base_yaw << "     _ref_base_yaw: " << _ref_base_yaw << endl;
      // _ref_base_yaw = abs(_ref_base_yaw - _ref_base_yaw_last) > M_PI ? (_ref_base_yaw - M_PI) : _ref_base_yaw;    // 机器人可能倒着走
      if (abs(_ref_base_yaw - _ref_base_yaw_last) > M_PI)
      {
        cout << "Before _ref_base_yaw: " << _ref_base_yaw << endl;

        _ref_base_yaw = (_ref_base_yaw - _ref_base_yaw_last > 0) ? (_ref_base_yaw - 2 * M_PI) : (2 * M_PI + _ref_base_yaw);

        cout << "After _ref_base_yaw: " << _ref_base_yaw << endl;
      }

      if (abs(_ref_base_yaw - cur_base_yaw) > M_PI*0.5)
      {
        cout << "Before _ref_base_yaw with cur: " << _ref_base_yaw << endl;

        _ref_base_yaw = (_ref_base_yaw - cur_base_yaw > 0) ? (_ref_base_yaw - M_PI) : (M_PI + _ref_base_yaw);

        cout << "After _ref_base_yaw with cur: " << _ref_base_yaw << endl;
      }

      _ref_base_yaw_last = _ref_base_yaw_before;
    }
    else{
      _ref_base_yaw = cur_base_yaw;
    }
    
  }
  else
  {
    Waypoints.clear();
    Waypoints.resize(Predict_steps);

    for (int i = 0; i < Predict_steps; ++i)
    {
      Waypoints[i] = Waypoints_rrt[i];
      cout << "Waypoint from rrt" << Waypoints[i].transpose() << endl;
    }
  }

  cout << "Predict_steps: " << Predict_steps << endl;
  // Waypoints.erase(Waypoints.begin() + Predict_steps - 1, Waypoints.begin() + _Waypoints_length - 1);

  // generate_Waypoints();    // slope
  // Predict_steps = 10;
  
  predefine_trajectory.poses.clear();
  for (int i = 0; i < Predict_steps; ++i)
  {
    cout << "i: " << i << endl;
    geometry_msgs::PoseStamped _node_pose;
    _node_pose.pose.position.x = Waypoints[i][0];
    _node_pose.pose.position.y = Waypoints[i][1];
    _node_pose.pose.position.z = Waypoints[i][2];

    // _node_pose.pose.position.z = Waypoints[i][5];
    // tf::Quaternion quat = tf::createQuaternionFromYaw(Waypoints[i][5]);
    tf::Quaternion quat =
        tf::createQuaternionFromRPY(Waypoints[i][3], -Waypoints[i][4], Waypoints[i][5]);
    geometry_msgs::Quaternion quat_geo;
    quaternionTFToMsg(quat, quat_geo);
    _node_pose.pose.orientation = quat_geo;

    _node_pose.header.frame_id = "/world";
    predefine_trajectory.poses.push_back(_node_pose);
    cout << "Waypoint in predefine_trajectory: " << Waypoints[i].transpose() << endl;
  }

  rrt_update = false;

  ros::Time t_start = ros::Time::now();

  Eigen::VectorXd state(11);    // cur_cam_yaw pitch_base2cam  g_orientation_fb.y()
  state << g_position_fb.x(), g_position_fb.y(), cur_base_yaw, cur_cam_yaw,
      cur_base_pitch, pitch_base2cam, cur_base_roll, 0, 0, 0, 0;

  cout << "yaw_bc from joint state: " << yaw_base2cam << endl;
  cout << "yaw_bc from camera: " << cur_cam_yaw - cur_base_yaw << endl;

  cout << "current state: " << state.transpose() << endl;

  Eigen::VectorXd ref_wp(4 * Predict_steps);
  for (int i = 0; i < Predict_steps; ++i)
  {
    ref_wp[i] = (Waypoints[i](0));
  }
  for (int i = 0; i < Predict_steps; ++i)
  {
    ref_wp[Predict_steps + i] = (Waypoints[i](1));
  }
  for (int i = 0; i < Predict_steps; ++i)
  {
    ref_wp[2 * Predict_steps + i] = (Waypoints[i](5));    // yaw
  }
  for (int i = 0; i < Predict_steps; ++i)
  {
    ref_wp[3 * Predict_steps + i] = (Waypoints[i](4));    // pitch
  }

  MatrixXd u_k;
  // solve mpc for state and reference trajectory
  // returns [steering_angle, acceleration]
  u_k = mpc.Solve(state, ref_wp, Predict_steps);

  // if(u_k.col(0).norm() < 0.2 && Predict_steps >= 2){
  //   cout << "control is too small!!!!!!!!" << endl;
  //   return;
  // }
  // cout << "u_k: " << u_k.transpose() << endl;

  // publish cmd_vel
  geometry_msgs::Twist cmd;
  cmd.linear.x = u_k.col(0)(0);
  cmd.angular.z = u_k.col(0)(1);
  vel_cmd_pub.publish(cmd);

  std_msgs::Float64 yaw_desire;
  // yaw_desire.data = abs(cur_cam_yaw) > 1.5708 ? u_k.col(0)(2) : -u_k.col(0)(2);
  yaw_desire.data = -u_k.col(0)(2);
  g_yaw_cmd_pub.publish(yaw_desire);

  std_msgs::Float64 pitch_desire;
  pitch_desire.data = -u_k.col(0)(3); // pitch控制：负值朝上运动
  // pitch_desire.data = 0;
  g_pitch_cmd_pub.publish(pitch_desire);

  // Visualization of predict path
  trajectory_predict.poses.clear();
  // Vector3d X_k_1;
  // Eigen::Vector4d X_k;
  MatrixXd X_k = MatrixXd::Zero(5, 1);
  MatrixXd X_k_1 = MatrixXd::Zero(5, 1);
  geometry_msgs::PoseStamped temp_pose;

  X_k(0) = g_position_fb.x();
  X_k(1) = g_position_fb.y();
  X_k(2) = cur_base_yaw;  // theta_basese<std_msgs::Int16>(
  //     "/goal_reached", 1);
  X_k(3) = cur_cam_yaw;   // theta_camera
  X_k(4) = g_orientation_fb.y(); // pitch_camera
  temp_pose.pose.position.x = X_k(0);
  temp_pose.pose.position.y = X_k(1);
  temp_pose.pose.position.z = Waypoints[0][2];

  // tf::Quaternion quat = tf::createQuaternionFromYaw(X_k(3));
  tf::Quaternion quat = tf::createQuaternionFromRPY(0, -X_k(4), X_k(3)); // 显示，pitch需要加负号
  // tf::Quaternion quat = tf::createQuaternionFromRPY(0, -X_k(4), cur_base_yaw);
  geometry_msgs::Quaternion quat_geo;
  quaternionTFToMsg(quat, quat_geo);
  temp_pose.pose.orientation = quat_geo;
  // trajectory_predict.poses.push_back(temp_pose);

  // X_k(0) = mpc.x_pred_vals[0];
  // X_k(1) = mpc.y_pred_vals[1];
  // X_k(2) = mpc.theta_b_pred_vals[2];     // theta_base
  // X_k(3) = mpc.theta_c_pred_vals[2];     // theta_camera
  for (int i = 0; i < Predict_steps - 1; i++)
  {
    // 不完整约束的运动学模型
    // X_k_1(0) = X_k(0) + u_k.col(i)(0) * cos(X_k(2)) * dt;
    // X_k_1(1) = X_k(1) + u_k.col(i)(0) * sin(X_k(2)) * dt;
    // X_k_1(2) = X_k(2) + u_k.col(i)(1) * dt;

    cout << "u_k " << i << " : " << u_k.col(i).transpose() << endl;

    // 带云台的运动学模型
    X_k_1(0) = X_k(0) + u_k.col(i)(0) * cos(X_k(2)) * dt;
    X_k_1(1) = X_k(1) + u_k.col(i)(0) * sin(X_k(2)) * dt;
    X_k_1(2) = X_k(2) + u_k.col(i)(1) * dt;
    X_k_1(3) = X_k(3) + (u_k.col(i)(1) + u_k.col(i)(2)) * dt;
    X_k_1(4) = X_k(4) + (u_k.col(i)(3)) * dt;

    X_k = X_k_1;

    // cout << "X_k_output: " << X_k_1.transpose() << endl;

    // temp_pose.pose.position.x = X_k_1(0);
    // temp_pose.pose.position.y = X_k_1(1);

    temp_pose.pose.position.x = mpc.x_pred_vals[i];
    temp_pose.pose.position.y = mpc.y_pred_vals[i];
    temp_pose.pose.position.z = Waypoints[i+1][2];

    geometry_msgs::Quaternion quat_geo;

#if def_Using_Quaternion
    quat_geo.x = mpc.quat_pred[i].x();
    quat_geo.y = mpc.quat_pred[i].y();
    quat_geo.z = mpc.quat_pred[i].z();
    quat_geo.w = mpc.quat_pred[i].w();
    temp_pose.pose.orientation = quat_geo;
#else
    tf::Quaternion quat = tf::createQuaternionFromRPY(0, -mpc.pitch_c_pred_vals[i], mpc.theta_c_pred_vals[i]);
    quaternionTFToMsg(quat, quat_geo);
#endif
    // temp_pose.pose.position.x = mpc.x_pred_vals[i];
    // temp_pose.pose.position.y = mpc.y_pred_vals[i];
    // temp_pose.pose.position.z = mpc.theta_c_pred_vals[i];
    trajectory_predict.poses.push_back(temp_pose);
  }

  ros::Time t_end = ros::Time::now();
  ROS_INFO("control total time : %5.3f ms", (t_end - t_start).toSec() * 1000);
  
  // trajectory_pub.publish(plan_trajectory);
  trajectory_predict_pub.publish(trajectory_predict);

  // f_goal_reached = Reach_Goal_func();
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpc_test_node");
  ros::NodeHandle n("mpc_test");
  ros::Rate loop_rate(500);

  ros::Timer timer = n.createTimer(ros::Duration(dt), timerCallback);

#if (def_Predefined_traj_TEST != 1)
  ros::Subscriber rrtpath_sub = n.subscribe<nav_msgs::Path>("/track_rrt_path", 1,
                                                            rrtpath_callback);
  // ros::Subscriber rrtpath_sub = n.subscribe<nav_msgs::Path>("/rrt_path", 1,
  // rrtpath_callback);

#endif

#if (def_USING_ORBSLAM == 1)
  ros::Subscriber orbpose_sub =
      n.subscribe<geometry_msgs::PoseStamped>("/orb_slam2_rgbd/pose", 1,
                                              orbpose_callback);
#else
  ros::Subscriber cam_state_sub = n.subscribe<nav_msgs::Odometry>(
      "/ground_truth/camera_state", 1, CameraState_Cb);

#endif
  
  ros::Subscriber joint_state_sub = n.subscribe<sensor_msgs::JointState>(
      "/joint_states", 1, JointState_Cb);

  NormVec_publisher = n.advertise<nav_msgs::Path> ("/visual_norm_vec", 1);
  Travel_Path_publisher = n.advertise<nav_msgs::Path> ("/Traveled_path", 1);

  // body_imu用于获取body朝向 ros::Subscriber
  //  sub_view_robot_camera = n.subscribe<sensor_msgs::Imu>("/body_imu", 1,
  //  body_imu_callback);

  ros::Subscriber base_state_sub = n.subscribe<nav_msgs::Odometry>(
      "/ground_truth/base_state", 1, BaseState_Cb);

  ros::Subscriber HeightMap_sub =
  n.subscribe<nav_msgs::OccupancyGrid>("/Height_map_tracking", 1, HeightMapCb);

  // ros::Subscriber octree_sub_known = n.subscribe<octomap_msgs::Octomap>(
  //     "/octomap_full", 1, pcdoctomapCallback);    // 加载已知地形地图

  predefine_trajectory_pub =
      n.advertise<nav_msgs::Path>("/predefine_trajectory", 1);

  trajectory_pub = n.advertise<nav_msgs::Path>("/plan_trajectory", 1);
  trajectory_predict_pub =
      n.advertise<nav_msgs::Path>("/trajectory_predict", 1);
  // B_spline_trajectory.setUniformBspline(control_points, 3, ts);
  // B_spline_trajectory.getTimeSpan(t, tmp);    // 获取时间跨度

  // vel_cmd_pub =
  //     n.advertise<geometry_msgs::Twist>("/plt_velocity_controller/cmd_vel", 1);
  vel_cmd_pub =
      n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  g_yaw_cmd_pub = n.advertise<std_msgs::Float64>(
      "/joint11_velocity_controller/command", 1);
  g_pitch_cmd_pub = n.advertise<std_msgs::Float64>(
      "/joint10_velocity_controller/command", 1);

  pub_reach_goal = n.advertise<std_msgs::Int16>(
      "/goal_reached", 1);

  pub_pitch = n.advertise<std_msgs::Float64>(
      "/cur_cam_pitch", 1);

  pub_cur_goal = n.advertise<geometry_msgs::PoseStamped>("/cur_goal", 1);

  pub_terrain_map_1 =
      n.advertise<nav_msgs::OccupancyGrid>("Height_map_mpc", 5, true);

  ros::Subscriber goalpoint_sub_ = n.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goalposeCallback);
  // ros::Subscriber goalpoint_sub_ = n.subscribe<geometry_msgs::PoseStamped>("/aeplanner/setpoint_position/local", 1, goalposeCallback);

  plan_trajectory.header.frame_id = "/world";
  trajectory_predict.header.frame_id = "/world";
  predefine_trajectory.header.frame_id = "/world";

  Eigen::Vector2d wp(0,0);
  preset_waypoints.push_back(wp);
  wp.x() = 5; wp.y() = 5;
  preset_waypoints.push_back(wp);
  wp.x() = 7.5; wp.y() = 7.5;
  preset_waypoints.push_back(wp);
  wp.x() = 10; wp.y() = 10;
  preset_waypoints.push_back(wp);
  wp.x() = 12.5; wp.y() = 12.5;
  preset_waypoints.push_back(wp);
  wp.x() = 15; wp.y() = 15;
  preset_waypoints.push_back(wp);

  while (ros::ok())
  {
    predefine_trajectory_pub.publish(predefine_trajectory);
    pub_reach_goal.publish(_reached);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}