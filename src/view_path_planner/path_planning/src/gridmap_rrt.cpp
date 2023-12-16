//
//  octomap_rrt.cpp
//  OctomapRTT
//
//  Created by 王大鑫 on 2018/7/17.
//  Copyright © 2018 Daxin Wang. All rights reserved.
//
#include <ros/ros.h>
#include <ros/console.h>
#include <log4cxx/logger.h>
#include <nav_msgs/OccupancyGrid.h>

#include "../include/gridmap_rrt.h"
#include <chrono>
#include <fstream>

#define horizontal_fov 69.4
#define vertical_fov 50

#define drg2rad (M_PI/180)

// 待探索区域的长宽
const int AreaRange_X = 200;
const int AreaRange_Y = 200;

const int Max_range = 10;

Eigen::Vector3d start_position_;
Eigen::Vector3d end_position_;

vector<Node *> rrt2d_nodes_;
vector<Node *> path_;
Node *root_;
Node *lastNode_;

double bestGain_;
double bestGain_notarrive;
Node *bestNode_;
Node *bestNode_notarrive;
// Map *map_;

int max_iter_;
//step_size: 1 unit based
float step_size_;
float radius;   //range to choose new parent node


vector<Node*> Near;//没有进行初始化,并且为什么要用*
vector<double> cost;//起点到近邻点的距离
vector<double> dist;//近邻点到新节点的距离
double minCost;

vector<Eigen::Vector3d> rrt2d_sample_point;

vector<float> rrt2d_sample_point_gain;

/**
 * @brief initialize base information of this planning
 * @param start_position
 * @param end_position
 * @param map
 * @param max_iter
 * @param step_size
 */
void RRT2D_init(Eigen::Vector3d start_position, Eigen::Vector3d end_position, int max_iter, float step_size, short radius)
{
  rrt2d_nodes_.clear();
  start_position_ = start_position;
  end_position_ = end_position;
  // gain_ = gain;
  // map_ = map;
  root_ = new Node;
  root_->parent = NULL;
  root_->position = start_position_;
  root_->gain = 0;
  root_->total_gain = 0;
  rrt2d_nodes_.push_back(root_);
  lastNode_ = root_;
  max_iter_ = max_iter;
  step_size_ = step_size;
  bestGain_ = -1000;
  radius = radius;
  //std::cout << "step_size = " << step_size_ << std::endl;
}

Eigen::Vector3d getBBXMax(){
  double maxX, maxY, maxZ;
  _octree->getMetricMax(maxX, maxY, maxZ);

  Eigen::Vector3d max_bound;
  max_bound.x() = maxX + 5;
  max_bound.y() = maxY + 5;
  max_bound.z() = maxZ + 5;
  return max_bound;
}

Eigen::Vector3d getBBXMin(){
  double minX, minY, minZ;
  _octree->getMetricMin(minX, minY, minZ);

  Eigen::Vector3d min_bound;
  min_bound.x() = minX - 5;
  min_bound.y() = minY - 5;
  min_bound.z() = minZ - 5;
  return min_bound;
}


void draw_line(
    unsigned int height,
    unsigned int width,
    double x,
    double y,
    double theta,
    unsigned int range,
    unsigned int *const line,
    double *const widths,
    unsigned int &num_cells)
{

  // Pre-compute sin and cos
  double sin = std::sin(theta);
  double cos = std::cos(theta);

  // Reset the line
  num_cells = 0;

  // Initialize the cell indices
  int floor_x = std::floor(x);
  int floor_y = std::floor(y);

  // Initialize the distances along the line to the
  // next horizontal cell boundary (dx) or vertical
  // cell boundary (dy). The maximum distance for
  // either is dx/y_unit.
  double dx_unit = 1 / std::abs(cos);
  double dy_unit = 1 / std::abs(sin);
  double dx = dx_unit * ((cos > 0) ? floor_x + 1 - x : x - floor_x);
  double dy = dy_unit * ((sin > 0) ? floor_y + 1 - y : y - floor_y);

  // Compute the sign of the steps taken
  int x_step = (cos > 0) ? 1 : -1;
  int y_step = (sin > 0) ? 1 : -1;

  // While we are inside the map
  // while (
  //     floor_x >= 0 and
  //     floor_y >= 0 and
  //     floor_x < (int) (x + range) and
  //     floor_y < (int) (y + range)) {
  while (
      floor_x >= (int)(x - range) and
      floor_y >= (int)(y - range) and
      floor_x < (int)(x + range) and
      floor_y < (int)(y + range))
  {
    // std::cout << "floor_x: " << floor_x << "  floor_y:" << floor_y << std::endl;
    // Add the cell index
    line[num_cells] = floor_y * width + floor_x;

    // The width is the minimum distance to either cell
    widths[num_cells] = std::min(dx, dy);

    // Subtract the width from the line boundaries
    // and the distance to the boundary
    dx -= widths[num_cells];
    dy -= widths[num_cells];

    // Replenish if necessary
    if (dx <= 0)
    {
      dx = dx_unit;
      floor_x += x_step;
    }
    if (dy <= 0)
    {
      dy = dy_unit;
      floor_y += y_step;
    }

    // Increment the cell number
    num_cells++;
  }
}

Eigen::Vector2i Convert2pos(int index){
  Eigen::Vector2i pos;

  pos.x() = index % _gridmap.info.width;
  pos.y() = index / _gridmap.info.width;

  return pos;
}

// 统计某视角下点是否大部分在无效定位区
bool isViewInfeatureless(Eigen::Vector3d point_position){
  Eigen::Vector2d node_position(point_position.x(), point_position.y());

  // cout << "node position===: " << node_position.transpose() << endl;
  // cout << "node yaw: " << point_position.z() << endl;

  Eigen::Vector2i node_index = ConvertWorld2GridIndex(node_position);

  int range = 6/_gridmap.info.resolution;
  std::vector<unsigned int> line(2 * _gridmap.info.width);
  std::vector<double> widths(2 * _gridmap.info.width);
  unsigned int num_cells;

  draw_line(_gridmap.info.width, _gridmap.info.width,
        node_index.x(), node_index.y(), point_position.z(), range,
        line.data(),
        widths.data(),
        num_cells);

  int cnt_line = 0;
  for (unsigned int i = 0; i < num_cells; i++)
  {
    // cout << "line node: " << ConvertGridIndex2World(Convert2pos(line[i])).transpose() << endl;
    // cout << "grid data: " << int(_gridmap.data[line[i]]) << endl;
    if(_gridmap.data[line[i]] < 0){
      cnt_line++;
    }
  }

  // yaw角直线上超过0.3比例的点在无效定位区，则该视角无效。
  if(cnt_line > num_cells*0.3){
    return true;
  }
  else{
    return false;
  }
}   

bool isInfeatureless(Eigen::Vector3d point_position){
  
  Eigen::Vector2d node_position(point_position.x(), point_position.y());

  Eigen::Vector2i node_index = ConvertWorld2GridIndex(node_position);

  int Idx = (node_index.x() + node_index.y() * _gridmap.info.width);

  // if((_gridmap.data[Idx] < 0) || (_gridmap.data[Idx - 2] < 0) || (_gridmap.data[Idx + 2] < 0) || (_gridmap.data[Idx - 6] < 0) || (_gridmap.data[Idx + 6] < 0) || (_gridmap.data[Idx - 2 * _gridmap.info.width] < 0) || (_gridmap.data[Idx + 2 * _gridmap.info.width] < 0) || (_gridmap.data[Idx - 6 * _gridmap.info.width] < 0) || (_gridmap.data[Idx + 6 * _gridmap.info.width] < 0))
  if((_gridmap.data[Idx] < 0) || (_gridmap.data[Idx - 1] < 0) || (_gridmap.data[Idx + 1] < 0) || (_gridmap.data[Idx - 1 * _gridmap.info.width] < 0) || (_gridmap.data[Idx + 1 * _gridmap.info.width] < 0))
  {
    return true;  
  }
  else if(isViewInfeatureless(point_position)){
    return true;
  }
  else{
    return false;
  }  
}

bool OutofRange(Eigen::Vector3d point_position){
  
  Eigen::Vector2d node_position(point_position.x(), point_position.y());

  if(node_position.x() < -AreaRange_X/2 || node_position.x() > AreaRange_X/2 || node_position.y() < -AreaRange_Y/2 || node_position.y() > AreaRange_Y/2){
    // cout << "Is out of Limit area!!!" << endl;
    return false;
  }
  else{
    // cout << "In Limit area......" << endl;
    return true;
  }
}

void deleteNodes(Node *root)
{
  for (auto node : root->children)
  {
    deleteNodes(node);
  }
  delete root;
}

Node *getRandomNotObstacleNode()
{
  float prob = 0.5;
  float random_f;
  int N = 999;
  random_f = rand() % (N + 1) / (float)(N + 1); //生成（0,1）的随机数
  if (random_f >= prob)
  { // 大于一定概率，则进行随机采样；否则向着目标点采样
    Eigen::Vector3d rand_point;
    short x_max = getBBXMax().x() / _gridmap.info.resolution;
    short y_max = getBBXMax().y() / _gridmap.info.resolution;
    float z_max = 3.14;

    short x_min = getBBXMin().x() / _gridmap.info.resolution;
    short y_min = getBBXMin().y() / _gridmap.info.resolution;
    float z_min = -3.14;

    do
    {
      rand_point = Eigen::Vector3d(((rand() % (x_max - x_min)) + x_min) * _gridmap.info.resolution,
                                    ((rand() % (y_max - y_min)) + y_min) * _gridmap.info.resolution,
                                    ((rand() % (N + 1) / (float)(N + 1))*2 - 1) * z_max);
    } while (isInfeatureless(rand_point));

    Node *rand_node = new Node;
    rand_node->position = rand_point;
    return rand_node;
  }
  else
  {
    Node *rand_node = new Node;
    rand_node->position = end_position_;

    return rand_node;
  }
}

Node *findNearestNode(Eigen::Vector3d current_position)
{
  double min_distance = 1e5;
  Node *closest_node = NULL;
  for (auto node : rrt2d_nodes_)
  {
    double distance = (current_position - node->position).norm();
    if (distance < min_distance)
    {
      min_distance = distance;
      closest_node = node;
    }
  }

  //ROS_INFO("closest_node: %f %f %f", closest_node->position.x(), closest_node->position.y(), closest_node->position.z());
  return closest_node;
}
/**
 * @brief 从树中离采样点最近的节点开始，向采样点延伸stepsize，获得q_new,但是另外两个参数没啥用啊
 * @param q_rand
 * @param q_nearest
 * @param direction
 * @return
 */
Node *getNewNode(Node *q_rand, Node *q_nearest, Eigen::Vector3d direction)
{
  Eigen::Vector3d result;

  // ROS_INFO("q_nearest: %f %f %f", q_nearest->position.x(), q_nearest->position.y(), q_nearest->position.z());
  // ROS_INFO("map_->getResolution(): %f", map_->getResolution());
  // ROS_INFO("step_size_: %d", step_size_);
  result.x() = q_nearest->position.x() + step_size_ * direction.x();
  result.y() = q_nearest->position.y() + step_size_ * direction.y();
  result.z() = q_nearest->position.z() + step_size_ * direction.z();
  Node *q_new = new Node;
  q_new->position = result;
  return q_new;
}
/**
 * @brief 检测q_new是否位于不可检测区域，分多段检测，就像之前写的demo一样，如果只检测q_new位置出有没有障碍
 * 是不合适的，因为从q_nearest到q_new这段距离，如果中间遇到障碍物也是不可以的，这样处理也可以适应不规则障碍物
 * @param q_new
 * @param q_nearest
 * @param direction
 * @return
 */
bool isNewNodeCollision(Eigen::Vector3d q_new, Eigen::Vector3d q_nearest, Eigen::Vector3d direction)
{
  int test_step = 4;
  bool ret = false;
  for (int i = 1; i <= test_step; i++)
  {
    float step = 1.0 * i / test_step * step_size_;
    Eigen::Vector3d test_point;
    test_point.x() = q_nearest.x() + step * direction.x();
    test_point.y() = q_nearest.y() + step * direction.y();
    test_point.z() = q_nearest.z() + step * direction.z();
    // cout << "test point:" << test_point << endl;

    // test_point.x() = 9.82179;
    // test_point.y() = -6.72599;
    // test_point.z() = 0.11962;
    if (isInfeatureless(test_point))
    {
      // cout << "test point:" << test_point << endl;

      // cout << "is In featureless!!!" << endl;
      ret = true;
      break;
    }
  }
  return ret;
}

// double border_gain(Node *q_new)
// {
//   double gain;

//   double y_s = q_new->position.y();
//   for (int i = 0; i <= 5; ++i)
//   {
//     double y_hight = i * 0.2 + 0.1;

//     q_new->position.y() = y_hight;
//     octomap::ColorOcTreeNode *node = _octree->search(q_new->position);

//     if (node != NULL && node->getOccupancy() > 0.5)
//     {
//       //ROS_WARN("searched node position: %f %f %f", q_new->position.x(), q_new->position.y(), q_new->position.z());
//       //ROS_WARN("q_new_gain_color: %d %d %d", node->getColor().r,node->getColor().g, node->getColor().b);

//       if ((node->getColor().r == 255) && (node->getColor().g == 255) && (node->getColor().b == 255))
//       {
//         continue;
//       }

//       if ((node->getColor().r <= (61 + 5)) && (node->getColor().r >= (61 - 5)) && (node->getColor().g <= (230 + 5)) && (node->getColor().g >= (230 - 5)) && (node->getColor().b <= (250 + 5)) && (node->getColor().b >= (250 - 5)))
//       {
//         ////// 在无效定位区域内，则增益为1000.因为最后最优增益是求最小值
//         //ROS_ERROR("Invalid area!!!");
//         gain = -100;
//         q_new->position.y() = y_s;
//         return gain;
//       }
//       else
//       {
//         ///// todo: 不在无效定位区域内，则计算距离
//         gain = 50;
//         q_new->position.y() = y_s;
//         return gain;
//       }

//       // if(node->getColor())
//       break;
//     }
//   }

//   /***可用来表征未建图区域**********************************************/
//   q_new->position.y() = y_s;
//   return 0; // 若在未知区域，返回0
// }


const int N_S_rrt = 8;  // 8邻域

void get_neighbours_rrt(Eigen::Vector2i n_array[], Eigen::Vector2i position) {
	n_array[0].x() = position.x() - 1;
  n_array[0].y() = position.y() - 1;

	n_array[1].x() = position.x() - 1; 
  n_array[1].y() = position.y(); 

	n_array[2].x() = position.x() - 1; 
  n_array[2].y() = position.y() + 1; 

  n_array[3].x() = position.x(); 
	n_array[3].y() = position.y() - 1;

  n_array[4].x() = position.x(); 
	n_array[4].y() = position.y() + 1;

	n_array[5].x() = position.x() + 1;
  n_array[5].y() = position.y() - 1;

	n_array[6].x() = position.x() + 1;
  n_array[6].y() = position.y();

	n_array[7].x() = position.x() + 1;
  n_array[7].y() = position.y() + 1;
}

int gain_freeVoxels(Node *q_new){
  Eigen::Vector2d q_new_pos;
  q_new_pos.x() = q_new->position.x();
  q_new_pos.y() = q_new->position.y();

  GridIndex q_new_idx = ConvertWorld2GridIndex(q_new_pos);

  Eigen::Vector2i locations[N_S_rrt]; 
	get_neighbours_rrt(locations, q_new_idx);

  int found = 0;
  for(int i = 0; i < N_S_rrt; ++i){
    int Idx = (locations[i].x() + locations[i].y()*_gridmap.info.width);

    if(abs(_gridmap.data[Idx]) != 100){
      found++;
    }
  }

  return found;
}

float gain_exploration_2d(Node *q_new){

  Eigen::Vector2d pos(q_new->position.x(), q_new->position.y());
  GridIndex q_new_idx = ConvertWorld2GridIndex(pos);

  // float fsmi_data_node = compute_fsmi_point(q_new_idx.x(), q_new_idx.y());
  float fsmi_data_node;
  fsmi_data_node = compute_fsmi_point(q_new_idx.x(), q_new_idx.y(),5, true);

  return fsmi_data_node;
}

// octomap::point3d seed_fp_infov{0,0,0};


bool fpInFOV(Eigen::Vector3d node_pos, Eigen::Vector3d frontierpoint){

  Eigen::Vector3d FP_eigen{frontierpoint.x(), frontierpoint.y(), frontierpoint.z()};
  // pitch in fov

  // yaw in fov
  Eigen::Vector3d dir = FP_eigen - node_pos;

  double dir_yaw = atan2(dir.y(), dir.x());

  // cout << "node_pos: " << node_pos.transpose() << "frontierpoint: " << frontierpoint << "dir_yaw: " << dir_yaw << endl;

  if((abs(dir_yaw - node_pos.z()) < horizontal_fov*drg2rad*0.5) && sqrt(dir.x()*dir.x() + dir.y()*dir.y()) < Max_range){
    // cout << "yaw error: " << abs(dir_yaw - node_pos.z()) << endl;
    // cout << "horizon thresh: " << horizontal_fov*drg2rad*0.5 << endl;
    // seed_fp_infov = frontierpoint;
    return true;
  }
  else{
    // cout << "yaw error: " << abs(dir_yaw - node_pos.z()) << endl;
    // cout << "horizon thresh: " << horizontal_fov*drg2rad*0.5 << endl;
    return false;
  }
}

float gain_perception(Node *q_new){
  // 统计某个视角内栅格总的信息量，from color of octree
  float gain_perc = 0;

  Eigen::Vector3f Unit_vec{1,0,0};
#if 1

  
#endif
  

}


double test_yaw(Eigen::Vector3d node_pos, octomap::point3d frontierpoint){
  Eigen::Vector3d FP_eigen{frontierpoint.x(), frontierpoint.y(), frontierpoint.z()};
  // pitch in fov

  // yaw in fov
  Eigen::Vector3d dir = FP_eigen - node_pos;

  double dir_yaw = atan2(dir.y(), dir.x());

  return dir_yaw;
}

int gain_construction(Node *q_new){

  int count_fp = 0;

  for(auto it:g_Frontiers3D_pos){
    if(fpInFOV(q_new->position, it)){
      count_fp++;
    }
  }
 
  return count_fp;
}



// double compute_gain(Node *q_new)
// {
//   double gain = 0;
//   if (q_new->parent == NULL)
//   {
//     return gain;
//   }

//   cout << "q_new->parent: " << q_new->parent->position.x() << " " << q_new->parent->position.y() << " " << q_new->parent->position.z() << endl;

//   // gain分为与目标点的距离+与边界的距离

//   double gain_border_distance = border_gain(q_new);
//   //ROS_INFO("gain_border_distance: %f", gain_border_distance);

//   double parent_distance;
//   parent_distance = (q_new->parent->position - end_position_).norm();

//   double current_distance;
//   current_distance = (q_new->position - end_position_).norm();

//   double gain_goal_distance = current_distance - parent_distance;
//   //ROS_INFO("gain_goal_distance: %f", gain_goal_distance);

//   // gain = q_new->parent->gain - 10*gain_goal_distance - gain_border_distance;

//   if (q_new->parent != NULL)
//   {
//     gain = 5 * gain_goal_distance - gain_border_distance;
//     ROS_INFO("node_gain: %f", gain);
//   }
//   else
//   {
//     gain = 5 * gain_goal_distance - gain_border_distance;
//     ROS_INFO("root_node_gain: %f", gain);
//   }

//   return gain;
// }
/**
 * @brief 如果q_new合适的话，就把它填入路径
 * @param q_nearest
 * @param q_new
 */
void addNewNode(Node *q_nearest, Node *q_new)
{
  q_new->parent = q_nearest;
  q_nearest->children.push_back(q_new);
  rrt2d_nodes_.push_back(q_new);
  lastNode_ = q_new;
  //    std::cout<<"Get a new Node"<<std::endl;
  //    std::cout<<"Now nodes have:"<<nodes_.size()<<std::endl;
  //    for(auto node:nodes_){
  //        std::cout<<node->position<<"\n";
  //    }
  //    std::cout<<"\n";
}
/**
 * @brief 判断是否到达目的地
 * @return
 */
bool isArrived(Node *node)
{
  //    if((lastNode_->position - end_position_).norm() < 2.2*map_->getResolution())
  //        return true;

  if ((node->position - end_position_).norm() < 2 * _gridmap.info.resolution)
    return true;
  return false;
}


float lamda = 1;
float success_distance = 3;
bool run_w_perception()
{
  
  srand(static_cast<ushort>(time(NULL)));
  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();
  bool arrive_flag = false;

  // cout << "node_size: " << rrt2d_nodes_.size() << endl;
  for (int i = 0; i < max_iter_; i++)
  {
    // Eigen::Vector3d cur_pos_test{0,0,0};
    // octomap::point3d frontierpoint_test{1, -1*float(i*10)/max_iter_, 0};
    // double test_y = test_yaw(cur_pos_test, frontierpoint_test);
    // cout << "frontierpoint_test: " << frontierpoint_test << "test_yaw: " << test_y << endl;

    // std::cout<<"i="<<i<<std::endl;
    Node *q_rand = getRandomNotObstacleNode();
    // ROS_INFO("q_rand: %f %f %f", q_rand->position.x(), q_rand->position.y(), q_rand->position.z());
    
    // 寻找离采样点最近的节点
    Node *q_nearest = findNearestNode(q_rand->position);

    Eigen::Vector3d direction = q_rand->position - q_nearest->position;
    // cout << "rand direction norm: " << direction.norm() << endl;

    bool grow_flag = true;
    Node *q_new;
    if (direction.norm() > step_size_)
    {
      direction = direction.normalized();
      q_new = getNewNode(q_rand, q_nearest, direction);
    }
    else
    {
      q_new = q_rand;
    }

    Eigen::Vector3d position_tmp(q_new->position.x(), q_new->position.y(), q_new->position.z());
    if(isInfeatureless(position_tmp)){
      grow_flag = false;
    }

    // cout << "is grow?  " << grow_flag << endl;
    // Eigen::Vector3d position_tmp(q_new->position.x(), q_new->position.y(), 0);
    // if(!OutofRange(position_tmp)){
    //   grow_flag = false;
    // }

    if(grow_flag == true){
      float distance_ = (q_new->position - q_nearest->position).norm();
            
      q_new->gain = (-1) * gain_exploration_2d(q_new) * exp(lamda * distance_);
      q_new->total_gain = q_nearest->total_gain + q_new->gain;

      // 重建增益
      // cout << "gain construction: " << gain_construction(q_new) << endl;
      // q_new->total_gain = gain_construction(q_new);

      // cout << "gain_exploration_2d(q_new): " << gain_exploration_2d(q_new) << endl;
      // cout << "distance_: " << distance_ << endl;

      // q_new->gain = (-1) * gain_exploration_2d(q_new) * exp(-lamda * distance_);

      // cout << "q_new->gain: " << q_new->gain << endl;
      addNewNode(q_nearest, q_new);

      // 到达终点
      if(isArrived(q_new)){
        // cout << "rrt Arrived end_position!" << endl;
        if(q_new->total_gain > bestGain_){
          bestGain_ = q_new->total_gain;
          bestNode_ = q_new;
          // cout << "bestGain_: " << bestGain_ << endl;
          // cout << "bestNode Gain_: " << bestNode_->gain << endl;
        }
        arrive_flag = true;
      }
      else{
        Node *q_nearest = findNearestNode(end_position_);

        bestGain_notarrive = q_nearest->total_gain;
        bestNode_notarrive = q_nearest;
      }
      
      rrt2d_sample_point.push_back(q_new->position);
      rrt2d_sample_point_gain.push_back(q_new->total_gain);
    }
    else{
      i--;
      // std::cout<<"i--="<<i<<std::endl;
    }
  }

  if(arrive_flag)
  {
    Node *q;
    // q = bestNode_;
    // q=lastNode_;
    // q = findNearestNode(end_position_);
    q = bestNode_;

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    //std::cout<<"calculate_time = "<<elapsed_seconds.count()<<"s\n";
    while (q != NULL)
    {
      path_.push_back(q);
      // rrt2d_sample_point.push_back(q->position);
      // rrt2d_sample_point_gain.push_back(q->gain);
      q = q->parent;
    }

    // 这里应该return false的，为了显示暂时设置为return true
    return true;
  }
  else{
    Node *q;
    // q = bestNode_;
    // q=lastNode_;
    // q = findNearestNode(end_position_);
    q = bestNode_notarrive;

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    //std::cout<<"calculate_time = "<<elapsed_seconds.count()<<"s\n";
    while (q != NULL)
    {
      path_.push_back(q);
      // rrt2d_sample_point.push_back(q->position);
      // rrt2d_sample_point_gain.push_back(q->gain);
      q = q->parent;
    }

    if((bestNode_notarrive->position - end_position_).norm() < success_distance){
      return true;
    }
    else{
      return false;
    }

    // 这里应该return false的，为了显示暂时设置为return true
    // return true;
  }
}

Node *chooseParent(Node *&q)
{
  Node *potentialParent = NULL;
  Node *curNode = NULL;
  for (auto node : rrt2d_nodes_)
  { //遍历所有节点，找到位于新节点radius范围内的所有节点，并添加至Near中
    double distance = (node->position - q->position).norm();
    // if (distance < radius * map_->getResolution())
    {
      dist.push_back(distance); //记录新节点和近邻之间的距离
      Near.push_back(node);     //获得了新节点的近邻
    }
  }
  for (int i = 0; i < (int)Near.size(); i++)
  { //依据从起点到近邻点距离代价最小原则选择新的父节点
    curNode = Near[i];
    double c = dist[i];
    // 回溯
    while ((curNode->parent) != NULL)
    {
      c += (curNode->position - curNode->parent->position).norm();
      //记录从起点到达每一个近邻节点的路径代价
      curNode = curNode->parent;
    }
    cost.push_back(c);
    if (c < minCost)
    {
      minCost = c;
      potentialParent = Near[i];
    }
  }
  return potentialParent;
}

void rewire(Node *&q)
{ //基于到每个节点的路径代价最小，判断是否要将Near中的节点的父节点设置为新节点
  for (int i = 0; i < (int)Near.size(); i++)
  {
    if (minCost + dist[i] < cost[i])
    {
      Near[i]->parent = q;
    }
  }
}
/**
 * @brief visualize path by writing path into map module
 */
void writeMap()
{
  for (auto node : path_)
  {
    std::cout << node->position << std::endl;
    // map_->mixPathMap(node->position, true);
  }
}

void writeInfo2File(std::string output_name)
{
  double distance = 0;
  double tmp_distance = 0;
  ofstream fout;
  fout.open(output_name);
  //write basic infomation to file
  fout << "step_size = " << step_size_ << std::endl;
  fout << "max_iter = " << max_iter_ << std::endl;
  fout << "start_position = " << start_position_ << "\tend_position = " << end_position_ << std::endl;
  //write position of path-node and distance between two nodes
  fout << "START_POSITION\t\t\t"
       << "END_POSITION\t\t\t"
       << "DISTANCE\t"
       << "TOTAL_DISTANCE\n";
  for (int i = path_.size() - 1; i > 0; i--)
  {
    tmp_distance = (path_[i]->position - path_[i - 1]->position).norm();
    fout << path_[i]->position << "\t" << path_[i - 1]->position << "\t" << tmp_distance << "\t";
    distance += tmp_distance;
    fout << distance << std::endl;
  }
  //write distance between last_node_position and end_position
  fout << "LAST_NODE_POSITION\t\t\t"
       << "FINAL_POSITION\t\t\t"
       << "DISTANCE\t"
       << "TOTAL_DISTANCE\n";
  tmp_distance = (end_position_ - path_[0]->position).norm();
  fout << path_[0]->position << "\t" << end_position_ << "\t" << tmp_distance << "\t";
  distance += tmp_distance;
  fout << distance << std::endl;

  std::cout << "distance = " << distance << std::endl;
  fout << flush;
  fout.close();
}

/**
 * @brief Delete all nodes using DFS technique.
 * @param root
 */
void deleteAllNodes(Node *root)
{
  for (int i = 0; i < (int)root->children.size(); i++)
  {
    deleteNodes(root->children[i]);
  }
  delete root;
}

void deleteNode(Node *node)
{
  Eigen::Vector3d position_tmp = node->position;
  Node *parent_node = node->parent;
  ROS_WARN("Children.size: %d", parent_node->children.size());

  if (parent_node->children.size() == 1)
  {
    ROS_WARN("delete children: %f %f %f", node->position.x(), node->position.y(), node->position.z());
    parent_node->children.clear();
    ROS_WARN("Children.size: %d", parent_node->children.size());
  }
  else
  {
    vector<Node *>::iterator it;

    for (it = parent_node->children.begin(); it != parent_node->children.end(); ++it)
    {
      ROS_WARN("delete children: %f %f %f", (*it)->position.x(), (*it)->position.y(), (*it)->position.z());
      if (abs(position_tmp.x() - (*it)->position.x()) < 0.0005 && abs(position_tmp.y() - (*it)->position.y()) < 0.0005 && abs(position_tmp.z() - (*it)->position.z()) < 0.0005)
      {
        ROS_WARN("delete children: %f %f %f", (*it)->position.x(), (*it)->position.y(), (*it)->position.z());
        parent_node->children.erase(it);
        ROS_WARN("Children.size: %d", parent_node->children.size());
        break;
      }
    }

    for (it = parent_node->children.begin(); it != parent_node->children.end(); it++)
    {
      ROS_WARN("delete children: %f %f %f", (*it)->position.x(), (*it)->position.y(), (*it)->position.z());
    }
  }
}