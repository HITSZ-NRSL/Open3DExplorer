#include <ros/ros.h>

#include <chrono>
#include <numeric>
#include <queue>
#include <nav_msgs/OccupancyGrid.h>
// #include <range_mi/MIGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include "../include/fsmi_cal.h"

// #include "range_mi/grid_mi.hpp"
// #include "range_mi/grid_line.hpp"

// Map data
// nav_msgs::MapMetaData map_info;
// std_msgs::Header map_header;
// std::vector<double> vacancy;
bool def_maze_mode;
bool def_terrain_mode;

float explore_range_blx;
float explore_range_bly;
float explore_range_urx;
float explore_range_ury;

double g_param_2dfsmi_range;
double g_param_3dfsmi_range;
bool g_param_buffer_strategy;
double g_param_weight_direction;
double g_param_weight_distance;
double g_param_visualize_fpsize;
double g_param_thresh_toexplore;

ros::Publisher mi_map_pub;
ros::Publisher frontier_pub;

// Initialize a place for mutual information
// std::vector<double> fsmi_data;

// typedef Eigen::Vector2i GridIndex;
//从世界坐标系转换到栅格坐标系
// GridIndex ConvertWorld2GridIndex(Eigen::Vector2d pos);
// Eigen::Vector2d ConvertGridIndex2World(Eigen::Vector2i index);

#if def_octo_generator
octomap::ColorOcTree *_octree;
#else
octomap::OcTree *_octree;
#endif
// octomap::OcTree* frontier3d_octree{NULL};
// octomap::ColorOcTree* frontier3d_octree{NULL};
nav_msgs::OccupancyGrid _gridmap;
nav_msgs::OccupancyGrid g_Height_deviation_map;

nav_msgs::MapMetaData map_info;
std_msgs::Header map_header;
std::vector<double> vacancy;

std::vector<double> fsmi_data;

std::vector<Eigen::Vector3d> g_Frontiers3D_pos;
std::vector<Eigen::Vector3d> g_Frontiers_Cluster_pos;
std::vector<Eigen::Vector2i> g_Frontier2D_Cluster_pos;

vector<Eigen::Vector3d> Frontier3D_black_list;
vector<Eigen::Vector2i> frontier2d_vector;
Eigen::Vector3d m_Cur_position(0, 0, 0);

bool visualize = true;
bool All_FP_inarea = false;

unsigned int range = 20; // 边界点互信息的计算范围，8m

double fsmi_point = 0;
const unsigned int num_beams = 20;
vector<double> fsmi_beam_vector(2 * num_beams);

double fsmi_beam = 0;

Eigen::Vector2i zero_vec(0, 0);
Eigen::Vector2i cur_goal(0, 0);

// std::vector<float> top_mi_frontier;
// std::vector<Eigen::Vector2i> top_mi_frontier_index;

// int threshold_near_nofeature = 2;

Eigen::Vector2d Body_dir_avg(1, 0); // 与运动方向的一致性作为代价
Eigen::Vector2d Motion_dir(1, 0); // 与运动方向的一致性作为代价

bool Explore_Done_2d = false;
float frontier_range = 16; // 统计局部范围内的边界点
float minimal_observe_range = 2;

double theta_best;

// nav_msgs::OccupancyGrid _gridmap;
// typedef Eigen::Vector2i GridIndex;
//从世界坐标系转换到栅格坐标系
GridIndex ConvertWorld2GridIndex(Eigen::Vector2d pos)
{

  GridIndex index;
  // ceil返回大于等于该数的整数值，与floor相反
  index.x() = std::floor((pos.x() - _gridmap.info.origin.position.x) / _gridmap.info.resolution);
  index.y() = std::floor((pos.y() - _gridmap.info.origin.position.y) / _gridmap.info.resolution);

  return index;
}

GridIndex ConvertWorld2GridIndex(double pos_x, double pos_y)
{

  GridIndex index;
  // ceil返回大于等于该数的整数值，与floor相反
  index.x() = std::floor((pos_x - _gridmap.info.origin.position.x) / _gridmap.info.resolution);
  index.y() = std::floor((pos_y - _gridmap.info.origin.position.y) / _gridmap.info.resolution);

  return index;
}

//从栅格坐标系转换到世界坐标系
Eigen::Vector2d ConvertGridIndex2World(Eigen::Vector2i index)
{

  Eigen::Vector2d pos;

  pos.x() = index.x() * _gridmap.info.resolution + _gridmap.info.origin.position.x;
  pos.y() = index.y() * _gridmap.info.resolution + _gridmap.info.origin.position.y;

  return pos;
}

//从栅格坐标系转换到世界坐标系
Eigen::Vector2d ConvertGridIndex2World(int index_x, int index_y)
{

  Eigen::Vector2d pos;

  pos.x() = index_x * _gridmap.info.resolution + _gridmap.info.origin.position.x;
  pos.y() = index_y * _gridmap.info.resolution + _gridmap.info.origin.position.y;

  return pos;
}

bool IsInLimitArea(const Eigen::Vector2d& _query_point)
{
  if (_query_point.x() < explore_range_blx || _query_point.x() > explore_range_urx ||
      _query_point.y() > explore_range_bly || _query_point.y() < explore_range_ury)
  {
    // cout << "Is out of Limit area!!!" << endl;
    return false;
  }
  else
  {
    // cout << "In Limit area......" << endl;
    return true;
  }
}

double get_mapz(const Eigen::Vector2d _pos){
  
  double z_min = -3;
  double z_max = 8;
  double cell_z_max = z_min;

  bool _occupied_flag = false;
  for(double z_search = z_max; z_search >= z_min; z_search -= _octree->getNodeSize(16)){
    octomap::point3d temp(_pos.x(), _pos.y(), z_search);
    octomap::OcTreeNode *node = _octree->search(temp);        
    if (node != NULL && _octree->isNodeOccupied(node))
    {
      return z_search;
    }
  }
  return z_min;
}

// bool visualize = true;

void draw(
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

double func(double delta, double vacancy)
{
  vacancy = std::max(vacancy, 0.00000001);
  vacancy = std::min(vacancy, 0.99999999);
  double r = (1 - vacancy) / vacancy;
  return std::log((r + 1) / (r + 1 / delta)) - std::log(delta) / (r * delta + 1);
}


void visualize_frontier(const vector<Eigen::Vector2i> frontier2d_vector){
  visualization_msgs::MarkerArray ma;
  
  visualization_msgs::Marker fp_points;
  visualization_msgs::Marker text_marker;

  fp_points.header.frame_id = "/world";
  fp_points.header.stamp = ros::Time::now();
  fp_points.ns = "points_and_lines";
  fp_points.action = visualization_msgs::Marker::ADD;
  fp_points.pose.orientation.w = 1.0;
  // fp_points.type = visualization_msgs::Marker::POINTS;
  fp_points.type = visualization_msgs::Marker::CUBE;
  
  text_marker.header.frame_id = "/world";
  text_marker.header.stamp = ros::Time::now();
  text_marker.ns = "points_and_lines";
  text_marker.action = visualization_msgs::Marker::ADD;
  text_marker.pose.orientation.w = 1;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  fp_points.scale.x = g_param_visualize_fpsize;
  fp_points.scale.y = g_param_visualize_fpsize;
  fp_points.scale.z = 0.001;
  
  text_marker.scale.z = g_param_visualize_fpsize;

  // Points are green
  fp_points.color.r = 0.0f;
  fp_points.color.g = 1.0f;
  fp_points.color.b = 0.0f;
  fp_points.color.a = 1.0;
  
  text_marker.color.r = 0.0f;
  text_marker.color.g = 1.0f;
  text_marker.color.b = 0.0f;
  text_marker.color.a = 1.0;

  // double mi_max = *std::max_element(fsmi_data.begin(), fsmi_data.end());

  int i = 2;
  // Create the vertices for the points and lines
  for (auto fp:frontier2d_vector)
  {
    // if(i % 2 == 0)
    {
      Eigen::Vector2d fp_pos = ConvertGridIndex2World(fp);

      fp_points.pose.position.x = fp_pos.x();
      fp_points.pose.position.y = fp_pos.y();
      fp_points.pose.position.z = 0;

      fp_points.id = i;

      // if(fsmi_data[fp.x() + fp.y() * map_info.width] == 100){
      //   fp_points.color.a = 0.4;
      // }
      // else{
      //   fp_points.color.a = (1 - fsmi_data[fp.x() + fp.y() * map_info.width] / mi_max) > 1 ? 1 : (1 - fsmi_data[fp.x() + fp.y() * map_info.width] / mi_max);
      // }
      
      // fp_points.color.a = 1;

      text_marker.pose.position.x = fp_pos.x() + 0.6;
      text_marker.pose.position.y = fp_pos.y();
      text_marker.pose.position.z = 0.5;

      text_marker.text = std::to_string(int(fsmi_data[fp.x() + fp.y() * _gridmap.info.width]));
      text_marker.id = 200 + i;

      ma.markers.push_back(fp_points);
      ma.markers.push_back(text_marker);

    }    
    i++;
  }

  frontier_pub.publish(ma);
}

bool in_range(Eigen::Vector2d query_pos){
  // cout << "in range query_pos:" << query_pos.transpose() << endl;
  if(query_pos.x() > explore_range_blx && query_pos.x() < explore_range_urx && query_pos.y() < explore_range_bly && query_pos.y() > explore_range_ury){
    // cout << "in range!" << endl;
    return true;
  }
  else{
    // cout << "out range!" << endl;
    return false;
  }
}


float beam_gain(const unsigned int num_cells_, const vector<unsigned int>& line_, bool _area_limit){
  float _grid_gain = 0.0;
  float _beam_gain = 0.0;
  float data_scale = 1;
  float _scale_outrange = 1;


  bool _unknown_flag = false;
  for (unsigned int i = 0; i < num_cells_; i++){
    if(line_[i] > map_info.height * map_info.width){
      continue;
    }

    if(abs(vacancy[line_[i]] - 1) < 0.02) {    // 统计到无效定位区则停止
      break;
    }
    if(_area_limit && !in_range(ConvertGridIndex2World(convert_1d_2d(line_[i])))){
      _scale_outrange = 0.5;
    }

    if(abs(vacancy[line_[i]] - 0.5) <= 0.02){
      _grid_gain = (-data_scale*2*_scale_outrange);
      _unknown_flag = true;
    }
    else{
      _grid_gain = (-data_scale*_scale_outrange);
    }
    _beam_gain += _grid_gain;
  }
  
  // ROS_INFO_STREAM("Beam gain: " << _beam_gain);
  
  if(!_unknown_flag){
    _beam_gain = 0;
  }
  // _beam_gain = (_beam_gain == 0) ? 10 : _beam_gain;

  // ROS_INFO_STREAM("Beam gain: " << _beam_gain);
  return _beam_gain;
}

void draw_fsmi_map()
{
  // Convert to an occupancy map for visualization
  nav_msgs::OccupancyGrid mi_map_msg;
  mi_map_msg.header = map_header;
  mi_map_msg.info = map_info;
  mi_map_msg.data = std::vector<int8_t>(fsmi_data.size());
  double mi_max = *std::max_element(fsmi_data.begin(), fsmi_data.end());
  // std::cout << "mi_max:" << mi_max << std::endl;
  for (size_t i = 0; i < fsmi_data.size(); i++)
  {
    // Normalize between 0 and 1
    double normalized = fsmi_data[i] / mi_max;
    // std::cout << "mi_max normalized:" << normalized << std::endl;
    // Change to 100%

    mi_map_msg.data[i] = 100 * (1 - normalized);
    // mi_map_msg.data[i] = vacancy[i]*100;
  }
  mi_map_pub.publish(mi_map_msg);
}

// 最佳观测角度,0代表朝上，逆时针
// 这里的x，y是index
/**
 * @brief calculate the fsmi of point
 * 
 * @param x 
 * @param y 
 * @param _range 
 * @param _area_limit if true, the raycast only count the grid in limit range
 * @return ** double 
 */
double compute_fsmi_point(unsigned int x, unsigned int y, double _range, bool _area_limit)
{
  range = int(_range / map_info.resolution); // 互信息统计范围
  // cout << "range: " << range << endl;
  unsigned int max_side_length = map_info.width;

  // FSMI parameters
  double delta_occ = 1.5;
  double delta_emp = 1 / delta_occ;

  // Initialize data
  std::vector<unsigned int> line(2 * max_side_length);
  std::vector<double> widths(2 * max_side_length);

  unsigned int side_length = map_info.width;
  // std::cout << "side_length: " << side_length << std::endl;

  double theta;
  unsigned int num_cells;
  double p_previous_empty;
  double p_i_first_non_empty;
  double info_gain, info_loss;

  fsmi_point = 0;
  // std::cout << "x: " << x << " y: " << y << " vacancy[x,y]: " << vacancy[x + y * side_length] << std::endl;
  for (unsigned int b = 0; b < num_beams; b++)
  {
    fsmi_beam = 0;
    // std::cout << "b: " << b << std::endl;
    theta = float(b) * (2 * M_PI) / num_beams;
    // std::cout << "theta: " << theta << std::endl;
    // std::cout << "======================beam : " << b << " ======================" << std::endl;

    // Compute the intersections of the line with the grid
    draw(
        side_length, side_length,
        x, y, theta, range,
        line.data(),
        widths.data(),
        num_cells);

    fsmi_point += beam_gain(num_cells, line, _area_limit);
  }

  return fsmi_point;
}

void test_mi()
{
  double p_i_first_non_empty;
  double info_gain, info_loss;
  double delta_occ = 1.5;

  for (float prob = 0; prob <= 1; prob += 0.02)
  {
    p_i_first_non_empty = abs(0.5 - prob);

    info_gain = func(delta_occ, prob);

    // float weight = (prob > 0.5) ? (1 - prob) : ((-1) * prob);
    // float weight = 1;
    float weight = (prob > 0.5) ? 0.5 : -0.5;

    cout << setiosflags(ios::fixed); //保证setprecision()是设置小数点后的位数。

    cout << setprecision(2) << prob << "   " << setprecision(4) << (weight - (prob - 0.5)) << std::endl;
    // cout << setprecision(2) << prob << "   "  << setprecision(4) << weight * p_i_first_non_empty * info_gain * 100 << std::endl;
  }
}

// sub_beam为0，则是通过单线；
float find_best_theta(unsigned int x, unsigned int y, float _range, int _sub_beam, bool _area_limit)
{
  range = int(_range / map_info.resolution); // 互信息统计范围

  float _theta_best = 0;
  unsigned int max_side_length = map_info.width;

  // FSMI parameters
  double delta_occ = 1.5;
  double delta_emp = 1 / delta_occ;

  // Initialize data
  std::vector<unsigned int> line(2 * max_side_length);
  std::vector<double> widths(2 * max_side_length);

  unsigned int side_length = map_info.width;
  // std::cout << "side_length: " << side_length << std::endl;

  double theta;
  unsigned int num_cells;
  double p_previous_empty;
  double p_i_first_non_empty;
  double info_gain, info_loss;

  fsmi_point = 0;
  // std::cout << "x: " << x << " y: " << y << " vacancy[x,y]: " << vacancy[x + y * side_length] << std::endl;
  cout << "position: " << ConvertGridIndex2World(x,y).transpose() << endl;

  for (unsigned int b = 0; b < num_beams; b++)
  {
    fsmi_beam = 0;
    theta = float(b) * (2 * M_PI) / num_beams;
    // std::cout << "===========theta: " << theta << std::endl;
    // std::cout << "======================beam : " << b << " ======================" << std::endl;

    // Compute the intersections of the line with the grid
    draw(
        side_length, side_length,
        x, y, theta, range,
        line.data(),
        widths.data(),
        num_cells);

    fsmi_beam = beam_gain(num_cells, line, _area_limit);
    fsmi_beam_vector[b] = fsmi_beam;
    fsmi_beam_vector[b + num_beams] = fsmi_beam;
    fsmi_point += fsmi_beam;
  }

  double min_sum = 1e4;
  unsigned int min_beam_index = 0;
  // 寻找fsmi_beam_vector数组中连续n个数相加的最小值
  for (unsigned int i = 0; i < num_beams; ++i)
  {
    // cout << "beam: " << float(i)*(2 * M_PI)/num_beams << endl;
    double beam_sum = accumulate(fsmi_beam_vector.begin() + i, fsmi_beam_vector.begin() + i + 1 + _sub_beam, 0.0);
    // cout << "beam information sum: " << beam_sum << endl;
    
    // double beam_sum = fsmi_beam_vector[i];

    if (beam_sum < min_sum)
    {
      min_sum = beam_sum;
      min_beam_index = i + floor(_sub_beam / 2);
      // cout << "min_sum: " << min_sum << endl;
      // cout << "min beam idx: " << min_beam_index << endl;
    }
  }

  // cout << "minmum beam idx: " << min_beam_index << endl;
  // 世界坐标系下的旋转角
  _theta_best = float(min_beam_index) * (2 * M_PI) / num_beams;

  // 0~6.28 -> -3.14~3.14
  if(_theta_best > M_PI){
    _theta_best = _theta_best - 2 * M_PI;
  }

  return _theta_best;
}

float find_best_theta(unsigned int _frontier_x, unsigned int _frontier_y, Eigen::Vector2d _query_pos, float _range, int _sub_beam, bool _area_limit)
{
  range = int(_range / map_info.resolution); // 互信息统计范围

  float _theta_best = 0;
  unsigned int max_side_length = map_info.width;

  // FSMI parameters
  double delta_occ = 1.5;
  double delta_emp = 1 / delta_occ;

  // Initialize data
  std::vector<unsigned int> line(2 * max_side_length);
  std::vector<double> widths(2 * max_side_length);

  unsigned int side_length = map_info.width;
  // std::cout << "side_length: " << side_length << std::endl;

  double theta;
  unsigned int num_cells;
  double p_previous_empty;
  double p_i_first_non_empty;
  double info_gain, info_loss;

  fsmi_point = 0;
  // std::cout << "x: " << x << " y: " << y << " vacancy[x,y]: " << vacancy[x + y * side_length] << std::endl;
  cout << "position: " << ConvertGridIndex2World(_frontier_x,_frontier_y).transpose() << endl;

  for (unsigned int b = 0; b < num_beams; b++)
  {
    fsmi_beam = 0;
    // std::cout << "b: " << b << std::endl;
    theta = float(b) * (2 * M_PI) / num_beams;
    // std::cout << "===========theta: " << theta << std::endl;
    // std::cout << "======================beam : " << b << " ======================" << std::endl;

    // Compute the intersections of the line with the grid
    draw(
        side_length, side_length,
        _frontier_x, _frontier_y, theta, range,
        line.data(),
        widths.data(),
        num_cells);

    fsmi_beam = beam_gain(num_cells, line, _area_limit);
    fsmi_beam_vector[b] = fsmi_beam;
    fsmi_beam_vector[b + num_beams] = fsmi_beam;
    fsmi_point += fsmi_beam;
  }

  double min_sum = 1e4;
  unsigned int min_beam_index = 0;
  // 寻找fsmi_beam_vector数组中连续n个数相加的最小值
  for (unsigned int i = 0; i < num_beams; ++i)
  {
    // cout << "beam: " << float(i)*(2 * M_PI)/num_beams << endl;
    double beam_sum = accumulate(fsmi_beam_vector.begin() + i, fsmi_beam_vector.begin() + i + 1 + _sub_beam, 0.0);
    // cout << "beam information sum: " << beam_sum << endl;

    if (beam_sum < min_sum)
    {
      min_sum = beam_sum;
      min_beam_index = i + floor(_sub_beam / 2);
    }
  }

  // 世界坐标系下的旋转角
  _theta_best = float(min_beam_index) * (2 * M_PI) / num_beams;

  // 0~6.28 -> -3.14~3.14
  if(_theta_best > M_PI){
    _theta_best = _theta_best - 2 * M_PI;
  }

  /** 后处理，考虑机器人与边界点之间的方向 **/
  Eigen::Vector3d _unit_vec(1,0,0);
  // cout << "theta_best_mi: " << _theta_best << endl;
  Eigen::AngleAxisd angle_axis1(-_theta_best, Eigen::Vector3d(0, 0, 1));
  Eigen::Vector3d rotated_v1 = angle_axis1.matrix().inverse()*_unit_vec;
  // cout << "(1, 0, 0) theta_best:" << endl << rotated_v1.transpose() << endl;

  // 确定最佳朝向
  // theta_best = find_best_theta(Idx_maxMI.x(), Idx_maxMI.y(), 5);

  Eigen::Vector2d _goal_pos = ConvertGridIndex2World(_frontier_x, _frontier_y);
  Eigen::Vector2d _direction = _goal_pos - _query_pos;

  // theta_best = atan2(_direction.y(), _direction.x());
  double theta_best_direct = atan2(_direction.y(), _direction.x());
  // cout << "theta_best_direct: " << theta_best_direct << endl;
  Eigen::AngleAxisd angle_axis2(-theta_best_direct, Eigen::Vector3d(0, 0, 1));
  Eigen::Vector3d rotated_v2 = angle_axis2.matrix().inverse()*_unit_vec;
  // cout << "(1, 0, 0) theta_best_direct:" << endl << rotated_v2.transpose() << endl;

  _theta_best = atan2((rotated_v1 + rotated_v2).y(), (rotated_v1 + rotated_v2).x());
  // cout << "_theta_best: " << _theta_best << endl;
  
  return _theta_best;
}

Eigen::Vector2i convert_1d_2d(int one_index)
{
  Eigen::Vector2i two_index;

  two_index.x() = one_index % map_info.width;
  two_index.y() = int(one_index / map_info.width);

  return two_index;
}

// Eigen::Vector2i zero_vec(0, 0);
// Eigen::Vector2i cur_goal(0, 0);
// int top_num = 5;
// std::vector<float> top_mi_frontier;
// std::vector<Eigen::Vector2i> top_mi_frontier_index;

// int threshold_near_nofeature = 2;

// Eigen::Vector2d Body_dir_avg(1, 0); // 与运动方向的一致性作为代价
// bool Explore_Done_2d = false;
// float frontier_range = 16; // 统计局部范围内的边界点
// 实时计算过程中使用，统计每个点的信息
Eigen::Vector2i compute_mi_map_point(vector<Eigen::Vector2i> frontier2d_vector, Eigen::Vector2i current_pos_idx)
{
  if (frontier2d_vector.empty() || Explore_Done_2d)
  {
    Explore_Done_2d = true;
    ROS_ERROR_STREAM("End exploration 2d~~~~~");
    return current_pos_idx;
  }
  int top_num = 5;
  unsigned int max_side_length = map_info.width;
  unsigned int max_area = map_info.height * map_info.width;

  fsmi_data.clear();
  fsmi_data.resize(max_area, 1e4);
  // std::cout << "fsmi_data.size():" << fsmi_data.size() << std::endl;

  unsigned int side_length = map_info.width;

  Eigen::Vector2i Idx_maxMI;
  auto start = std::chrono::high_resolution_clock::now();

  // test_mi();

  typedef pair<double, int> mi_node;
  std::priority_queue<mi_node, vector<mi_node>, greater<mi_node> > top_mi_queue; // 信息增益小的在队列前面
  bool local_exist_frontier = false;
  double cost_fp = 0;
  float frontier_nosafe_distance = 0;
  double omega_1 = 0;   // 300
  double omega_2 = 0.0;

  cout << "frontier size after delete: " << frontier2d_vector.size() << endl;
  for (int i = 0; i < frontier2d_vector.size(); i++)
  {
    unsigned int x = frontier2d_vector[i].x();
    unsigned int y = frontier2d_vector[i].y();

    // 计算局部范围内边界点的信息
    if ((current_pos_idx - frontier2d_vector[i]).norm() <= (frontier_range / map_info.resolution))
    {
      Eigen::Vector2d _cur_position = ConvertGridIndex2World(current_pos_idx);
      Eigen::Vector2d _frontier2d_position = ConvertGridIndex2World(frontier2d_vector[i]);
      // 判断frontier及邻域是否高程差很大
      // 高程图的边界
      // if((abs(_frontier2d_position.x() - _cur_position.x()) <= 8.0) && (abs(_frontier2d_position.y() - _cur_position.y()) <= 8.0)){
      //   // Eigen::Vector2d node_position(point_position.x(), point_position.y());
      //   // Eigen::Vector2i node_index = World2GridIndex(node_position);
      //   GridIndex node_index;

      //   // ceil返回大于等于该数的整数值，与floor相反
      //   node_index.x() = std::ceil((_frontier2d_position.x() - g_Height_deviation_map.info.origin.position.x) /
      //                         g_Height_deviation_map.info.resolution);
      //   node_index.y() = std::ceil((_frontier2d_position.y() - g_Height_deviation_map.info.origin.position.y) /
      //                         g_Height_deviation_map.info.resolution);

      //   int Idx = (node_index.x() - 1 + (node_index.y() - 1)*g_Height_deviation_map.info.width);
      //   cout << "compute_mi_map_point - node_index: " << Idx << endl;
      //   cout << "compute_mi_map_point - node_index limit: " << g_Height_deviation_map.info.height * g_Height_deviation_map.info.width << endl;

      //   if((g_Height_deviation_map.data[Idx] > 0) || (g_Height_deviation_map.data[Idx - 1] > 0) || (g_Height_deviation_map.data[Idx + 1] > 0) || (g_Height_deviation_map.data[Idx - 1 * g_Height_deviation_map.info.width] > 0) || (g_Height_deviation_map.data[Idx + 1 * g_Height_deviation_map.info.width] > 0))  
      //   {
      //     ROS_INFO_STREAM("frontier2d is NearWall...");
      //     fsmi_data[x + y * side_length] = 1e3;
      //     continue;
      //   }
      // }
      
      // 距离过近的边界点，视野范围之内的边界点，不考虑
      // if((current_pos_idx - frontier2d_vector[i]).norm() <= (minimal_observe_range / map_info.resolution)){
      //   fsmi_data[x + y * side_length] = 1.5e3;
      // }

      // else if((frontier2d_vector[i]).norm() <= (minimal_observe_range / map_info.resolution)){
      //   fsmi_data[x + y * side_length] = 1e4;
      // }

      // else
      {
        fsmi_data[x + y * side_length] = compute_fsmi_point(x, y, g_param_2dfsmi_range, true);

        // 信息太少的点，不能作为目标点
        if(fsmi_data[x + y * side_length] > g_param_thresh_toexplore){
          frontier2d_vector.erase(frontier2d_vector.begin()+i);
          --i;
          continue;
        }

        // cout << "frontier position:" << ConvertGridIndex2World(frontier2d_vector[i]).transpose() << endl;
        // cout << "Local frontier mi:" << fsmi_data[x + y * side_length] << endl;

        // cout << "current_pos: " << ConvertGridIndex2World(current_pos_idx).transpose() << endl;

        Eigen::Vector2d Vec_robot_fp = (ConvertGridIndex2World(frontier2d_vector[i]) - ConvertGridIndex2World(current_pos_idx));
        Eigen::Vector2d Vec_robot_fp_norm = Vec_robot_fp.normalized();

        double cost_Angle_body_fp = Vec_robot_fp_norm.dot(Motion_dir); //角度cos值, 1~夹角为0
        // cout << "Angle_cost: " << acos(cost_Angle_body_fp) << endl;
        // cout << "Vec_robot_fp_norm: " << Vec_robot_fp.norm() << endl;

        // cout << "fsmi_data near the forntier: " << fsmi_data[x + y * side_length] << endl;

        {
          // cost_fp = fsmi_data[x + y * side_length] + omega_1 * (sqrt(1 - cost_Angle_body_fp*cost_Angle_body_fp) * M_PI + acos(cost_Angle_body_fp));// + omega_2 * abs(Vec_robot_fp.norm());// + omega_2 * abs(Vec_robot_fp.norm() - 10);
          // cost_fp = fsmi_data[x + y * side_length] + omega_1 * (acos(cost_Angle_body_fp)) + omega_2 * abs(Vec_robot_fp.norm());// + omega_2 * abs(Vec_robot_fp.norm() - 10);
          cost_fp = fsmi_data[x + y * side_length] * exp(-g_param_weight_distance * Vec_robot_fp.norm()) * exp(-g_param_weight_direction * cost_Angle_body_fp);
          // cout << "fsmi: " << fsmi_data[x + y * side_length] << endl;
          // cout << "cost_fp: " << cost_fp << endl;
        }

        top_mi_queue.push(std::pair<double, int>(cost_fp, x + y * side_length));
      }
    }
    else
    {
      fsmi_data[x + y * side_length] = 1e4;
    }
  }

  
  if(!g_param_buffer_strategy)
    local_exist_frontier = true;
  else{
    //+++++++++++++++++++++++++++++++++++++++++++++//
    // 如果局部边界点的最小信息值都大于阈值，则搜索全局的边界点
    if ((!top_mi_queue.empty())){
      Eigen::Vector2i _top_mi_queue_index = convert_1d_2d(top_mi_queue.top().second);
      if((compute_fsmi_point(_top_mi_queue_index.x(), _top_mi_queue_index.y(), g_param_2dfsmi_range, false) < g_param_thresh_toexplore)){
        local_exist_frontier = true;
      }
    }
  }

  //+++++++++++++++++++++++++++++++++++++++++++++//
  // 如果局部区域内不存在边界点了，则遍历全局边界点，选取
  if (local_exist_frontier == false)
  {
    cout << "there is no frontier in the local area..." << endl;

    while (!top_mi_queue.empty())
    {
      top_mi_queue.pop();
    }

    for (int i = 0; i < frontier2d_vector.size(); i++)
    {
      unsigned int x = frontier2d_vector[i].x();
      unsigned int y = frontier2d_vector[i].y();
      cout << "frontier2d_vector[" << i << "]: " << endl;

      // if((current_pos_idx - frontier2d_vector[i]).norm() <= (minimal_observe_range / map_info.resolution)){
      //   fsmi_data[x + y * side_length] = 1.5e3;
      // }

      // else if((frontier2d_vector[i]).norm() <= (minimal_observe_range / map_info.resolution)){
      //   fsmi_data[x + y * side_length] = 1e4;
      // }
      // else
      {
        fsmi_data[x + y * side_length] = compute_fsmi_point(x, y, g_param_2dfsmi_range, false);
        // top_mi_queue.push(std::pair<double, int>(fsmi_data[x + y * side_length], x + y * side_length));
        
        if(fsmi_data[x + y * side_length] > g_param_thresh_toexplore){
          frontier2d_vector.erase(frontier2d_vector.begin()+i);
          --i;
          continue;
        }

        Eigen::Vector2d Vec_robot_fp = (ConvertGridIndex2World(frontier2d_vector[i]) - ConvertGridIndex2World(current_pos_idx));
        Eigen::Vector2d Vec_robot_fp_norm = Vec_robot_fp.normalized();
        // cout << "Vec_robot_fp_norm: " << Vec_robot_fp_norm.transpose() << endl;

        double cost_Angle_body_fp = Vec_robot_fp_norm.dot(Motion_dir); //角度cos值, 1~夹角为0

        // Start Mode
        // if (ConvertGridIndex2World(current_pos_idx).norm() < 0)
        // {
        //   cost_fp = fsmi_data[x + y * side_length];
        // }
        // else
        {
          // cost_fp = fsmi_data[x + y * side_length] + omega_1 * (1 - cost_Angle_body_fp*cost_Angle_body_fp);// + omega_2 * abs(Vec_robot_fp.norm());// + omega_2 * abs(Vec_robot_fp.norm() - 10);
          
          // cost_fp = fsmi_data[x + y * side_length] + omega_1 * (acos(cost_Angle_body_fp)) + omega_2 * abs(Vec_robot_fp.norm());
          cost_fp = fsmi_data[x + y * side_length] * exp(-g_param_weight_distance * Vec_robot_fp.norm()) * exp(-g_param_weight_direction * cost_Angle_body_fp);
          // cout << "cost_fp: " << cost_fp << endl;
          // cout << "fsmi: " << fsmi_data[x + y * side_length] << endl;
        }
        top_mi_queue.push(std::pair<double, int>(cost_fp, x + y * side_length));
      }
    }
  }
  cout << "top_mi_queue.size: " << top_mi_queue.size() << endl;

  // 判断信息最小的点的值是否大于0，若是则终止探索
  if (top_mi_queue.empty() || ((!top_mi_queue.empty()) && (*min_element(fsmi_data.begin(), fsmi_data.end()) > g_param_thresh_toexplore)))
  {
    // cout << "top1 mi: " << top_mi_queue.top().first << endl;
    ROS_ERROR_STREAM("End exploration 2d~~~~~");
    Explore_Done_2d = true;
    // 当前位置作为终点
    return (current_pos_idx);
  }

  std::vector<std::pair<double, int> > top_mi_vector;

  // top_num = top_num < top_mi_queue.size()?top_num:top_mi_queue.size();
  top_num = std::min(top_num, (int)(top_mi_queue.size()));
  // 提取前top_num个点建立缓冲区
  for (int i = 0; i < top_num; ++i)
  {
    if (top_mi_queue.empty())
    {
      break;
    }
    top_mi_vector.push_back(top_mi_queue.top());

    // std::cout << "Vacancy[i]:" << vacancy[top_mi_queue.top().second] << std::endl;

    // cout << "top " << i << ": mi " << top_mi_queue.top().first << " index " << top_mi_queue.top().second << endl;
    top_mi_queue.pop();
  }

  bool is_in_top = false;
  for (int i = 0; i < top_mi_vector.size(); ++i)
  {
    // 若当前目标点在top中，则不改变目标点；
    if (top_mi_vector[i].second == (cur_goal.x() + cur_goal.y() * side_length))
    {
      // none return
      is_in_top = true;
      break;
    }
  }
  
  if(!g_param_buffer_strategy)
    is_in_top = false;

  // 若在top中，则不改变目标点
  if (is_in_top == true || top_mi_vector.empty())
  {
    cout << "Current goal in the top...." << endl;

    // cout << "Best index: " << cur_goal.x() + cur_goal.y() * side_length << endl;

    // 确定最佳朝向
    // theta_best = find_best_theta(cur_goal.x(), cur_goal.y(), m_Cur_position.segment(0,2), 5);
    theta_best = find_best_theta(cur_goal.x(), cur_goal.y(), g_param_2dfsmi_range, 5, false);
    
    // if (visualize)
    //   draw_fsmi_map();
    
    visualize_frontier(frontier2d_vector);
    // visualize_frontier(g_Frontier2D_Cluster_pos);

    auto end = std::chrono::high_resolution_clock::now();
    auto total = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // std::cout << "take: " << total.count() << " ms" << std::endl;
    ROS_WARN_STREAM("Calculate Point Mutual Info takes: " << total.count() << " ms");

    return cur_goal;
  }
  else
  { // 若不在top中，则在top中选取离cur_goal最近的点作为目标点；
    cout << "Current goal not in the top...." << endl;

    float min_distance = 1000000;

    for (int i = 0; i < top_num; ++i)
    {
      // 一维索引转二维索引
      Eigen::Vector2i top_index = convert_1d_2d(top_mi_vector[i].second);

      if ((top_index - cur_goal).norm() < min_distance)
      {
        // none return
        min_distance = (top_index - cur_goal).norm();

        Idx_maxMI = top_index;
      }
    }

    cout << "Best fp position: " << ConvertGridIndex2World(Idx_maxMI) << endl;

    // 确定最佳朝向
    
    // theta_best = find_best_theta(Idx_maxMI.x(), Idx_maxMI.y(), m_Cur_position.segment(0,2), 5);
    theta_best = find_best_theta(Idx_maxMI.x(), Idx_maxMI.y(), g_param_2dfsmi_range, 5, false);
    
    // if (visualize)
    //   draw_fsmi_map();

    visualize_frontier(frontier2d_vector);
    // visualize_frontier(g_Frontier2D_Cluster_pos);

    auto end = std::chrono::high_resolution_clock::now();
    auto total = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // std::cout << "take: " << total.count() << " ms" << std::endl;
    ROS_WARN_STREAM("Calculate Point Mutual Info takes: " << total.count() << " ms");

    return Idx_maxMI;
  }
}


// 用于整张图的可视化
int compute_fsmi_map()
{
  range = int(10 / map_info.resolution); // 互信息统计范围

  unsigned int num_beams = 20;
  unsigned int max_side_length = map_info.width;
  unsigned int max_area = map_info.height * map_info.width;
  std::cout << "map_info.height: " << map_info.height << std::endl;
  std::cout << "map_info.width: " << map_info.width << std::endl;

  // FSMI parameters
  double delta_occ = 1.5;
  double delta_emp = 1 / delta_occ;

  // Initialize a place for mutual information
  // std::vector<double> fsmi_data(max_area);
  fsmi_data.resize(max_area, 0.5);
  std::cout << "fsmi_data.size():" << fsmi_data.size() << std::endl;

  // Initialize data
  std::vector<unsigned int> line(2 * max_side_length);
  std::vector<double> widths(2 * max_side_length);

  // unsigned int* line = new unsigned int[20];
  // double* widths = new double[20];

  unsigned int side_length = map_info.width;
  std::cout << "side_length: " << side_length << std::endl;

  double theta;
  unsigned int num_cells;
  double p_previous_empty;
  double p_i_first_non_empty;
  double info_gain, info_loss;

  auto start = std::chrono::high_resolution_clock::now();
  for (unsigned int x = 0; x < side_length; x++)
  {
    for (unsigned int y = 0; y < side_length; y++)
    {
      fsmi_data[x + y * side_length] = 0;
      // std::cout << "x: " << x << " y: " << y << " vacancy[x,y]: " << vacancy[x + y * side_length] << std::endl;
      for (unsigned int b = 0; b < num_beams; b++)
      {
        // std::cout << "b: " << b << std::endl;
        theta = float(b) * (2 * M_PI) / num_beams;
        // std::cout << "theta: " << theta << std::endl;

        // Compute the intersections of
        // the line with the grid
        draw(
            side_length, side_length,
            x, y, theta, range,
            line.data(),
            widths.data(),
            num_cells);

        fsmi_data[x + y * side_length] = beam_gain(num_cells, line, true);
        // Compute the mutual information
        // p_previous_empty = 1;
        // info_loss = 0;
        // // fsmi_data[x + y * side_length] = 0;
        // // double mi_single_beam = 0;
        // for (unsigned int i = 0; i < num_cells; i++)
        // {
        //   // std::cout << "num_cells i: " << i << std::endl;
        //   // std::cout << "======line[i]: " << line[i] << std::endl;
        //   // std::cout << "======vacancy[line[i]]: " << vacancy[line[i]] << std::endl;

        //   p_i_first_non_empty = p_previous_empty * (1 - vacancy[line[i]]);
        //   // std::cout << "p_i_first_non_empty: " << p_i_first_non_empty << std::endl;

        //   p_previous_empty *= vacancy[line[i]];

        //   // std::cout << "func occ: " << func(delta_occ, vacancy[line[i]]) << std::endl;
        //   info_gain = info_loss + func(delta_occ, vacancy[line[i]]);
        //   // std::cout << "info_gain: " << info_gain << std::endl;
        //   info_loss += func(delta_emp, vacancy[line[i]]);
        //   // std::cout << "func emp: " << func(delta_emp, vacancy[line[i]]) << std::endl;

        //   // Assume the noise width is zero to be fair,
        //   // so no inner loop
        //   // fsmi_data[x + y * side_length] += p_i_first_non_empty * info_gain * ((vacancy[line[i]] - 0.5)>0?1:-1);
        //   fsmi_data[x + y * side_length] += p_i_first_non_empty * info_gain;
        //   // mi_single_beam += p_i_first_non_empty * info_gain;
        // }
        // std::cout << "mi_single_beam: " << mi_single_beam << std::endl;
      }
      // std::cout << "x: " << x << " y: " << y << " fsmi_data: " << fsmi_data[x + y * side_length] << std::endl;
    }
  }
  auto end = std::chrono::high_resolution_clock::now();
  auto total = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

  if (visualize)
    draw_fsmi_map();
  std::cout << "take: " << total.count() << " ms" << std::endl;

  // publish_fsmi();
}

Eigen::Vector2i mutual_information_compute(const nav_msgs::OccupancyGrid &map_msg, vector<Eigen::Vector2i> frontier2d_vector, Eigen::Vector2i current_pos_idx)
{

  // Store map information
  map_info = map_msg.info;
  map_info.height = map_info.width;
  map_header = map_msg.header;

  // Convert to probability
  vacancy = std::vector<double>(map_info.height * map_info.width);
  for (unsigned int i = 0; i < vacancy.size(); i++)
  {
    // vacancy[i] = 1 - map_msg.data[i]/99.;    // demo
    // 将二维栅格值转化成概率，1~0,1：无特征区域；0：有特征区域，黑色区域
    // -100 -> 1; 100 -> 0    100对应rich
    vacancy[i] = 1 - ((int)(map_msg.data[i]) / 2 + 50) * 1.0 / 100;

    // if (vacancy[i] <= 0.01 or vacancy[i] >= 0.99){
    //   std::cout << "map_msg.data[i]:" << int(map_msg.data[i]) << std::endl;
    //   std::cout << "Vacancy[i]:" << vacancy[i] << std::endl;
    // }

    if (vacancy[i] < 0 or vacancy[i] > 1)
    {
      std::cout << "map_msg.data[i]: " << (int)(map_msg.data[i]) << std::endl;
      std::cout << "Vacancy out of bounds! " << vacancy[i] << std::endl;
      vacancy[i] = 0;
    }
  }

  // 用于可视化整个图的互信息。 白色：信息量高；黑色：信息量少
  // compute_fsmi_map();

  // 计算边界点的互信息，用于实时运行
  return compute_mi_map_point(frontier2d_vector, current_pos_idx);
  // return compute_mi_map_point(g_Frontier2D_Cluster_pos, current_pos_idx);
}

void vacancy_construction(const nav_msgs::OccupancyGrid &map_msg)
{

  // Store map information
  map_info = map_msg.info;
  map_info.height = map_info.width;
  map_header = map_msg.header;

  // Convert to probability
  vacancy = std::vector<double>(map_info.height * map_info.width);
  for (unsigned int i = 0; i < vacancy.size(); i++)
  {

    // vacancy[i] = 1 - map_msg.data[i]/99.;    // demo
    // 将二维栅格值转化成概率，1~0,1：无特征区域；0：有特征区域，黑色区域
    // -100 -> 1; 100 -> 0    100对应rich
    vacancy[i] = 1 - ((int)(map_msg.data[i]) / 2 + 50) * 1.0 / 100;

    // if (vacancy[i] <= 0.01 or vacancy[i] >= 0.99){
    //   std::cout << "map_msg.data[i]:" << int(map_msg.data[i]) << std::endl;
    //   std::cout << "Vacancy[i]:" << vacancy[i] << std::endl;
    // }

    if (vacancy[i] < 0 or vacancy[i] > 1)
    {
      std::cout << "map_msg.data[i]: " << (int)(map_msg.data[i]) << std::endl;
      std::cout << "Vacancy out of bounds! " << vacancy[i] << std::endl;
      vacancy[i] = 0;
    }
  }

  // 用于可视化整个图的互信息。 白色：信息量高；黑色：信息量少
  // compute_fsmi_map();

  // 计算边界点的互信息，用于实时运行
  // return compute_mi_map_point(frontier2d_vector, current_pos_idx);
}