#include "../include/gridmap_rrt_3d.h"

#include <log4cxx/logger.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <algorithm>
#include <chrono>
#include <fstream>

// #include "../include/frontier_detection_3d.h"

#define horizontal_fov 69.4
#define vertical_fov 50

#define drg2rad (M_PI / 180)

bool EnsureTrackingAcc = false;

// 待探索区域的长宽
const int AreaRange_X = 200;
const int AreaRange_Y = 200;

const int Max_range = 10;
int Fail_grow_count = 0;

const float Range_Roll_Sampling = 0;
const float Range_Pitch_Sampling = 0;
const float Range_Yaw_Sampling = M_PI;

bool Explore_Done_3d = false;

vector<Eigen::VectorXd> track_path_3d; // 顺序，[0]对应的是起点

octomap::point3d Convert_EigenVec2Octomap3d(Eigen::Vector3d query_point)
{
  octomap::point3d ret_point;

  ret_point.x() = query_point.x();
  ret_point.y() = query_point.y();
  ret_point.z() = query_point.z();

  return ret_point;
}

Eigen::Vector3d Convert_Octomap3d2EigenVec(octomap::point3d query_point)
{
  Eigen::Vector3d ret_point;

  ret_point.x() = query_point.x();
  ret_point.y() = query_point.y();
  ret_point.z() = query_point.z();

  return ret_point;
}

// 用于统计FOV内有效栅格个数
// search the first occupied voxel along the intersect_line_dir
bool get_intersect_point(Eigen::Vector3d node_position, Eigen::Vector3d intersect_line_dir, octomap::point3d& _Intersected_point){

  float search_step_ = _gridmap.info.resolution*0.5;
  Eigen::Vector3d intersect_line_dir_norm = intersect_line_dir.normalized();

  // Max_range范围内靠语义栅格地图来提供感知信息
  for(float scale = search_step_; scale < Max_range; scale += search_step_){
    Eigen::Vector3d search_point;
    search_point = intersect_line_dir_norm*scale + node_position;

    // rrt3d_intersection_point.push_back(search_point);    // For visualization

    octomap::point3d search_pt_(search_point.x(), search_point.y(), search_point.z());

    octomap::OcTreeKey key;
    _octree->coordToKeyChecked(search_pt_, key);
    octomap::point3d query = _octree->keyToCoord(key);
    // Eigen::Vector3d search_query{query.x(), query.y(), query.z()};

#if def_octo_generator
    octomap::ColorOcTreeNode* node = _octree->search(search_pt_, m_Frontier3D_depth);
#else
    octomap::OcTreeNode* node = _octree->search(search_pt_, m_Frontier3D_depth);
#endif
    if(node && _octree->isNodeOccupied(node)){
      // cout << "node occupied" << query << endl;
      _Intersected_point = query;
      return true;
    }
    else{
      // unknown_num++;
      // return false;
      // cout << "node unknown..." << search_point.transpose() << endl;
    }
  }

  // Max_range范围之外靠density map中的特征密度

  // cout << "!!!can not find the intersect point..." << endl;
  return false;
}

// 用于统计FOV内有效栅格个数
// search the first occupied voxel along the intersect_line_dir
bool get_intersect_point(Eigen::Vector3d node_position, Eigen::Vector3d intersect_line_dir, octomap::point3d& _Intersected_point, int& unknown_num){

  float search_step_ = _gridmap.info.resolution*0.5;
  Eigen::Vector3d intersect_line_dir_norm = intersect_line_dir.normalized();

  // Max_range范围内靠语义栅格地图来提供感知信息
  for(float scale = search_step_; scale < Max_range; scale += search_step_){
    Eigen::Vector3d search_point;
    search_point = intersect_line_dir_norm*scale + node_position;

    // rrt3d_intersection_point.push_back(search_point);    // For visualization

    octomap::point3d search_pt_(search_point.x(), search_point.y(), search_point.z());

    octomap::OcTreeKey key;
    _octree->coordToKeyChecked(search_pt_, key);
    octomap::point3d query = _octree->keyToCoord(key);
    // Eigen::Vector3d search_query{query.x(), query.y(), query.z()};

#if def_octo_generator
    octomap::ColorOcTreeNode* node = _octree->search(search_pt_, m_Frontier3D_depth);
#else
    octomap::OcTreeNode* node = _octree->search(search_pt_, m_Frontier3D_depth);
#endif
    if(node){
      // cout << "node occupied" << query << endl;
      if(_octree->isNodeOccupied(node)){
        _Intersected_point = query;
        return true;
      }      
    }
    else{
      unknown_num++;
      // return false;
      // cout << "node unknown..." << search_point.transpose() << endl;
    }
  }

  // Max_range范围之外靠density map中的特征密度

  // cout << "!!!can not find the intersect point..." << endl;
  return false;
}

int observe_times = 0;
vector<Eigen::Vector3d> Frontier3D_black_list_check;
vector<int> Frontier3D_black_list_check_time;
void Viewpoint_Check_Frontier(Eigen::Vector3d _cur_position,
                              Eigen::Vector3d _cur_orientation)
{
  _cur_orientation.y() = -_cur_orientation.y();
  for (int i = 0; i < g_Frontiers3D_pos.size(); ++i)
  {
    // cout << "Viewpoint_Check_Frontier - i: " << i << endl;
    bool pitch_inFOV = false, yaw_inFOV = false, no_occlued = false;
    // Eigen::Vector3d FP_eigen{g_Frontiers3D_pos[i].x(),
    // g_Frontiers3D_pos[i].y(), g_Frontiers3D_pos[i].z()};
    Eigen::Vector3d dir = g_Frontiers3D_pos[i] - _cur_position;
    // cout << "_cur_orientation: " << _cur_orientation.transpose() << endl;
    // cout << "Viewpoint_Check_Frontier - g_Frontiers3D_pos[i]: " << g_Frontiers3D_pos[i].transpose() << endl;

    // pitch in fov
    double xy_norm = sqrt(dir.x() * dir.x() + dir.y() * dir.y());
    // cout << "xy_norm: " << xy_norm << endl;
    double dir_pitch = atan2(dir.z(), xy_norm);
    // cout << "dir_pitch: " << dir_pitch << endl;

    if ((abs(dir_pitch - _cur_orientation.y()) <
         vertical_fov * drg2rad * 0.5) &&
        sqrt(xy_norm * xy_norm + dir.z() * dir.z()) < Max_range)
    {
      // cout << "pitch error: " << abs(dir_pitch - q_node->orientation.y()) <<
      // endl; cout << "horizon thresh: " << vertical_fov*drg2rad*0.5 << endl;
      pitch_inFOV = true;
    }
    else
    {
      // cout << "pitch error: " << abs(dir_pitch - q_node->orientation.y()) <<
      // endl; cout << "horizon thresh: " << vertical_fov*drg2rad*0.5 << endl;
      pitch_inFOV = false;
    }

    // if(pitch_inFOV)
    //   cout << "pitch in FOV: " << pitch_inFOV << endl;

    // yaw in fov
    double dir_yaw = atan2(dir.y(), dir.x());
    // cout << "dir_yaw: " << dir_yaw << endl;

    if ((abs(dir_yaw - _cur_orientation.z()) <
         horizontal_fov * drg2rad * 0.5) &&
        sqrt(xy_norm * xy_norm + dir.z() * dir.z()) < Max_range)
    {
      // cout << "yaw error: " << abs(dir_yaw - node_pos.z()) << endl;
      // cout << "horizon thresh: " << horizontal_fov*drg2rad*0.5 << endl;
      yaw_inFOV = true;
    }
    else
    {
      // cout << "yaw error: " << abs(dir_yaw - node_pos.z()) << endl;
      // cout << "horizon thresh: " << horizontal_fov*drg2rad*0.5 << endl;
      yaw_inFOV = false;
    }

    // if(yaw_inFOV)
    //   cout << "yaw in FOV: " << yaw_inFOV << endl;

    octomap::point3d _Intersected_point;
    bool Is_hit_occupied = get_intersect_point(_cur_position, dir, _Intersected_point);

    // bool Is_hit_occupied = _octree->castRay(
    //     Convert_EigenVec2Octomap3d(_cur_position),
    //     Convert_EigenVec2Octomap3d(dir), _Intersected_point, true, Max_range);

    // if(Is_hit_occupied){
    //   cout << "Is_hit_occupied: " << Is_hit_occupied << endl;
    // }

    // const int m_Frontier3D_depth = 16; // !!!与project2d.hpp中一致
    // 相交的点是否为边界点
    if (Is_hit_occupied &&
        ((Convert_Octomap3d2EigenVec(_Intersected_point) - g_Frontiers3D_pos[i])
             .norm() < _octree->begin_leafs(m_Frontier3D_depth).getSize()*1.1))
    {
      // cout << "Viewpoint_Check_Frontier - true _Intersected_point: " <<
      // Convert_Octomap3d2EigenVec(_Intersected_point).transpose() << endl;
      no_occlued = true;
    }
    else
    {
      // cout << "Viewpoint_Check_Frontier - false _Intersected_point: " <<
      // Convert_Octomap3d2EigenVec(_Intersected_point).transpose() << endl;
      no_occlued = false;
    }
    // if(no_occlued)
    //   cout << "no_occlued: " << no_occlued << endl;

    // Eigen::Vector3d _Intersected_point =
    // get_intersect_point(q_node->position, dir); Eigen::Vector3d
    // frontierpoint_vec3d(frontierpoint.x(), frontierpoint.y(),
    // frontierpoint.z()); if((_Intersected_point - frontierpoint_vec3d).norm()
    // < 0.1){
    //   no_occlued = true;
    // }
    // else{
    //   no_occlued = false;
    // }

    if (pitch_inFOV && yaw_inFOV && no_occlued)
    {
      cout << "Viewpoint_Check_Frontier - blacklist_check size: " << Frontier3D_black_list_check.size() << "    blacklist_time size: " << Frontier3D_black_list_check_time.size() << endl;
      vector<Eigen::Vector3d>::iterator _iter_fp_blacklist = find(Frontier3D_black_list_check.begin(), Frontier3D_black_list_check.end(), g_Frontiers3D_pos[i]);
      if((_iter_fp_blacklist == Frontier3D_black_list_check.end())){    // 不在列表中，则添加
        Frontier3D_black_list_check.push_back(g_Frontiers3D_pos[i]);
        Frontier3D_black_list_check_time.push_back(1);
        ROS_WARN_STREAM("Viewpoint_Check_Frontier - first observe at " << g_Frontiers3D_pos[i].transpose());
      }
      else{
        int _idx = distance(Frontier3D_black_list_check.begin(), _iter_fp_blacklist);
        cout << "Viewpoint_Check_Frontier - idx: " << _idx << endl;
        cout << "Viewpoint_Check_Frontier - multiple observe at " << Frontier3D_black_list_check[_idx].transpose() << endl;
        Frontier3D_black_list_check_time[_idx]++;
        ROS_WARN_STREAM("Viewpoint_Check_Frontier - check time " << Frontier3D_black_list_check_time[_idx]);

        if(Frontier3D_black_list_check_time[_idx] > 50){
          ROS_WARN_STREAM("Viewpoint_Check_Frontier - the frontier is occlued: " << g_Frontiers3D_pos[i].transpose());
          Frontier3D_black_list.push_back(g_Frontiers3D_pos[i]);
          g_Frontiers3D_pos.erase(g_Frontiers3D_pos.begin() + i);
          i--;
          cout << "Viewpoint_Check_Frontier - blacklist size: " << Frontier3D_black_list.size() << endl;
        }
      }
    }
    // else{
    //   return false;
    // }
  }
  // cout << "After View Check fpnum: " << g_Frontiers3D_pos.size() << endl;
}

// vector<Eigen::Vector3d> rrt3d_sample_position;

// vector<float> rrt3d_sample_point_gain;

/**
 * @brief initialize base information of this planning
 * @param start_position
 * @param end_position
 * @param map
 * @param max_iter
 * @param step_size
 */
rrt_3d::rrt_3d(Eigen::Vector3d start_position,
               Eigen::Vector3d start_orientation, Eigen::Vector3d end_position,
               Eigen::Vector3d end_orientation, int max_iter, float step_size)
    : terrain_mapping(start_position, true)
{
  rrt2d_nodes_.clear();
  start_position_ = start_position;
  start_orientation_ = start_orientation;
  end_position_ = end_position;
  end_orientation_ = end_orientation;
  root_ = new Node_3d;
  root_->parent = NULL;
  root_->position = start_position_;
  root_->orientation = start_orientation_;
  root_->gain = 0;
  root_->total_gain = 0;
  root_->cost_goal = 0;
  root_->total_cost_goal = 0;
  root_->cost_motion = 0;
  root_->total_cost_motion = 0;
  root_->is_adjust_construct = false;
  root_->is_adjust_explore = false;
  rrt2d_nodes_.push_back(root_);
  lastNode_ = root_;
  max_iter_ = max_iter;
  step_size_position = step_size;
  m_choose_radius = step_size_position * 1.2;
  bestGain_ = 1e5;
  std::cout << "rrt_3d inited start_position: " << start_position_.transpose()
            << start_orientation_.transpose() << std::endl;
  std::cout << "rrt_3d inited end_position: " << end_position_.transpose()
            << " " << end_orientation_.transpose() << std::endl;
}

rrt_3d::~rrt_3d() {}

// limit the sample range
Eigen::Vector3d rrt_3d::getBBXMax()
{
  double maxX, maxY, maxZ;
  _octree->getMetricMax(maxX, maxY, maxZ);

  Eigen::Vector3d max_bound;
  max_bound.x() = maxX + 5;
  max_bound.y() = maxY + 5;
  max_bound.z() = maxZ + 5;
  return max_bound;
}

Eigen::Vector3d rrt_3d::getBBXMin()
{
  double minX, minY, minZ;
  _octree->getMetricMin(minX, minY, minZ);

  Eigen::Vector3d min_bound;
  min_bound.x() = minX - 5;
  min_bound.y() = minY - 5;
  min_bound.z() = minZ - 5;
  return min_bound;
}

// bool isInfeatureless(Eigen::Vector3d point_position, Eigen::Vector3d
// point_orientation);

void rrt_3d::draw_line(unsigned int height, unsigned int width, double x,
                       double y, double theta, unsigned int range,
                       unsigned int *const line, double *const widths,
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
  while (floor_x >= (int)(x - range) and floor_y >= (int)(y - range) and
         floor_x < (int)(x + range) and floor_y < (int)(y + range))
  {
    // std::cout << "floor_x: " << floor_x << "  floor_y:" << floor_y <<
    // std::endl; Add the cell index
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

Eigen::Vector2i rrt_3d::Convert2pos(int index)
{
  Eigen::Vector2i pos;

  pos.x() = index % _gridmap.info.width;
  pos.y() = index / _gridmap.info.width;

  return pos;
}

// 统计某视角下点是否大部分在无效定位区
bool rrt_3d::isViewInfeatureless(Eigen::Vector3d point_position,
                                 Eigen::Vector3d point_orientation)
{
  Eigen::Vector2d node_position(point_position.x(), point_position.y());

  // cout << "node position===: " << node_position.transpose() << endl;
  // cout << "node yaw: " << point_position.z() << endl;

  Eigen::Vector2i node_index = ConvertWorld2GridIndex(node_position);

  int range = 6 / _gridmap.info.resolution;
  std::vector<unsigned int> line(2 * _gridmap.info.width);
  std::vector<double> widths(2 * _gridmap.info.width);
  unsigned int num_cells;

  draw_line(_gridmap.info.width, _gridmap.info.width, node_index.x(),
            node_index.y(), point_orientation.z(), range, line.data(),
            widths.data(), num_cells);

  int cnt_line = 0;
  for (unsigned int i = 0; i < num_cells; i++)
  {
    // cout << "line node: " <<
    // ConvertGridIndex2World(Convert2pos(line[i])).transpose() << endl; cout <<
    // "grid data: " << int(_gridmap.data[line[i]]) << endl;
    if (_gridmap.data[line[i]] < 0)
    {
      cnt_line++;
    }
  }

  // yaw角直线上超过0.3比例的点在无效定位区，则该视角无效。
  if (cnt_line > num_cells * 0.5)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool rrt_3d::isNearWall(Eigen::Vector3d point_position)
{
  // cout << "isNearWall position x_delta: " << abs(point_position.x() -
  // start_position_.x()) << endl; cout << "isNearWall position y_delta: " <<
  // abs(point_position.y() - start_position_.y()) << endl; 边界

  if ((abs(point_position.x() - start_position_.x()) > 8.0) ||
      (abs(point_position.y() - start_position_.y()) > 8.0))
  {
    return false;
  }

  Eigen::Vector2d node_position(point_position.x(), point_position.y());
  Eigen::Vector2i node_index = World2GridIndex(node_position);

  int Idx = mapIdx(node_index.x() - 1, node_index.y() - 1);
  int _extend_grids;
  if(def_maze_mode){
    _extend_grids = 1;
  }
  else{
    _extend_grids = 2;
  }  // cout << "node_index: " << Idx << endl;
  // cout << "node_index limit: " << Height_deviation_map.info.height *
  // Height_deviation_map.info.width << endl;
  if ((Height_deviation_map.data[Idx] > 0) ||
      (Height_deviation_map.data[Idx - _extend_grids] > 0) ||
      (Height_deviation_map.data[Idx + _extend_grids] > 0) ||
      (Height_deviation_map.data[Idx - _extend_grids * Height_deviation_map.info.width] >
       0) ||
      (Height_deviation_map.data[Idx + _extend_grids * Height_deviation_map.info.width] >
       0))
  {
    // ROS_INFO_STREAM("Node is NearWall...");
    return true;
  }
  else
  {
    return false;
  }
}

bool rrt_3d::isInfeatureless(Eigen::Vector3d point_position,
                             Eigen::Vector3d point_orientation)
{
  Eigen::Vector2d node_position(point_position.x(), point_position.y());

  Eigen::Vector2i node_index = ConvertWorld2GridIndex(node_position);

  int Idx = (node_index.x() + node_index.y() * _gridmap.info.width);
  int _extend_grids;
  if(def_maze_mode){
    _extend_grids = 1;
  }
  else{
    _extend_grids = 2;
  }
  // if((_gridmap.data[Idx] < 0) || (_gridmap.data[Idx - 2] < 0) ||
  // (_gridmap.data[Idx + 2] < 0) || (_gridmap.data[Idx - 6] < 0) ||
  // (_gridmap.data[Idx + 6] < 0) || (_gridmap.data[Idx - 2 *
  // _gridmap.info.width] < 0) || (_gridmap.data[Idx + 2 * _gridmap.info.width]
  // < 0) || (_gridmap.data[Idx - 6 * _gridmap.info.width] < 0) ||
  // (_gridmap.data[Idx + 6 * _gridmap.info.width] < 0))
  if ((_gridmap.data[Idx] < 0) || (_gridmap.data[Idx - _extend_grids] < 0) ||
      (_gridmap.data[Idx + _extend_grids] < 0) ||
      (_gridmap.data[Idx - _extend_grids * _gridmap.info.width] < 0) ||
      (_gridmap.data[Idx + _extend_grids * _gridmap.info.width] < 0)) // is in featureless
  {
    // ROS_INFO_STREAM("Node in featureless...");
    return true;
  }
  // else if(isViewInfeatureless(point_position, point_orientation)){
  //   return true;
  // }
  else
  {
    return false;
  }
}

bool rrt_3d::isInfeatureless(Eigen::Vector3d point_position)
{
  Eigen::Vector2d node_position(point_position.x(), point_position.y());

  Eigen::Vector2i node_index = ConvertWorld2GridIndex(node_position);

  int Idx = (node_index.x() + node_index.y() * _gridmap.info.width);

  // if((_gridmap.data[Idx] < 0) || (_gridmap.data[Idx - 2] < 0) ||
  // (_gridmap.data[Idx + 2] < 0) || (_gridmap.data[Idx - 6] < 0) ||
  // (_gridmap.data[Idx + 6] < 0) || (_gridmap.data[Idx - 2 *
  // _gridmap.info.width] < 0) || (_gridmap.data[Idx + 2 * _gridmap.info.width]
  // < 0) || (_gridmap.data[Idx - 6 * _gridmap.info.width] < 0) ||
  // (_gridmap.data[Idx + 6 * _gridmap.info.width] < 0))
  if ((_gridmap.data[Idx] < 0) || (_gridmap.data[Idx - 2] < 0) ||
      (_gridmap.data[Idx + 2] < 0) ||
      (_gridmap.data[Idx - 2 * _gridmap.info.width] < 0) ||
      (_gridmap.data[Idx + 2 * _gridmap.info.width] < 0)) // is in featureless
  {
    // ROS_INFO_STREAM("node in featureless...");
    return true;
  }
  // else if(isViewInfeatureless(point_position, point_orientation)){
  //   return true;
  // }
  else
  {
    return false;
  }
}

// Eigen::Vector3d g_origin_(3, -5, 0.835);
Eigen::Vector3d g_origin_(1, -5, 0.835);

bool rrt_3d::isInKnown_Feature_Area(Eigen::Vector3d point_position)
{
  int Featurerich_griddata = 100;
  Eigen::Vector2d node_position(point_position.x(), point_position.y());

  Eigen::Vector2i node_index = ConvertWorld2GridIndex(node_position);

  int Idx = (node_index.x() + node_index.y() * _gridmap.info.width);

  // if(def_terrain_mode)
  {
    g_origin_.x() = 0;
    g_origin_.y() = 0;
  }
// &&
//       (_gridmap.data[Idx - 1] == Featurerich_griddata) &&
//       (_gridmap.data[Idx + 1] == Featurerich_griddata) &&
//       (_gridmap.data[Idx - 1 * _gridmap.info.width] == Featurerich_griddata) &&
//       (_gridmap.data[Idx + 1 * _gridmap.info.width] ==
//        Featurerich_griddata)
  // cout << "_gridmap.data:" << int(_gridmap.data[Idx]) << endl;
  if ((_gridmap.data[Idx] == Featurerich_griddata) || (start_position_ - g_origin_).norm() < 1 ) 
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool rrt_3d::OutofRange(Eigen::Vector3d point_position)
{
  Eigen::Vector2d node_position(point_position.x(), point_position.y());

  if (node_position.x() < explore_range_blx || node_position.x() > explore_range_urx ||
      node_position.y() > explore_range_bly || node_position.y() < explore_range_ury)
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

void rrt_3d::deleteNodes(Node_3d *root)
{
  for (auto node : root->children)
  {
    deleteNodes(node);
  }
  delete root;
}

rrt_3d::Node_3d *rrt_3d::getRandomNotObstacleNode_3d()
{
  float prob = 0.0;
  float random_f;
  int N = 999;
  random_f = rand() % (N + 1) / (float)(N + 1); //生成（0,1）的随机数
  if (random_f >= prob)
  { // 大于一定概率，则进行随机采样；否则向着目标点采样
    Eigen::Vector3d rand_point;
    Eigen::Vector3d rand_orientation;

    short x_max = (start_position_.x() + 10) / _gridmap.info.resolution;
    short y_max = (start_position_.y() + 10) / _gridmap.info.resolution;

    short x_min = (start_position_.x() - 10) / _gridmap.info.resolution;
    short y_min = (start_position_.y() - 10) / _gridmap.info.resolution;

    rand_point = Eigen::Vector3d(
        ((rand() % (x_max - x_min)) + x_min) * _gridmap.info.resolution,
        ((rand() % (y_max - y_min)) + y_min) * _gridmap.info.resolution,
        start_position_.z());
    rand_orientation = Eigen::Vector3d(
        start_orientation_.x(),
        ((rand() % (N + 1) / (float)(N + 1)) * 2 - 1) * Range_Pitch_Sampling,
        ((rand() % (N + 1) / (float)(N + 1)) * 2 - 1) * Range_Yaw_Sampling);

    // do
    // {
    //   rand_point = Eigen::Vector3d(((rand() % (x_max - x_min)) + x_min) *
    //   _gridmap.info.resolution,
    //                                 ((rand() % (y_max - y_min)) + y_min) *
    //                                 _gridmap.info.resolution,
    //                                 start_position_.z());
    //   rand_orientation = Eigen::Vector3d(start_orientation_.x(),
    //                                 ((rand() % (N + 1) / (float)(N + 1))*2 -
    //                                 1) * Range_Pitch_Sampling,
    //                                 ((rand() % (N + 1) / (float)(N + 1))*2 -
    //                                 1) * Range_Yaw_Sampling);
    // } while (isInfeatureless(rand_point, rand_orientation));

    Node_3d *rand_node = new Node_3d;
    rand_node->position = rand_point;
    rand_node->orientation = rand_orientation;
    cout << "sample orientation from random sample: "
         << rand_node->orientation << endl;
    return rand_node;
  }
  else
  {
    Node_3d *rand_node = new Node_3d;
    rand_node->position = end_position_;
    rand_node->orientation = end_orientation_;

    cout << "sample orientation from end_point: " << rand_node->orientation
         << endl;

    return rand_node;
  }
}

// bool rrt_3d::In_Circle_Range(Eigen::Vector3d rand_position){
//   cout << "rand_position: " << rand_position.transpose() << endl;

//   Eigen::Vector3d _Circle_Center((end_position_ - start_position_)*0.5);
//   cout << "_Circle_Center: " << _Circle_Center.transpose() << endl;

//   float _scale = 1.2;
//   float _radius_sample = (end_position_ - start_position_).norm()*_scale*0.5;
//   cout << "_radius_sample: " << _radius_sample << endl;

//   cout << "distance" << (rand_position - _Circle_Center).norm() << endl;
//   if((rand_position - _Circle_Center).norm() < _radius_sample){
//     return true;
//   }
//   else{
//     return false;
//   }
// }

Eigen::Vector3d rrt_3d::getRandom_aroundFrontier(Eigen::Vector3d _Position)
{
  int N = 999;

  Eigen::Vector3d rand_point;
  Eigen::Vector3d rand_orientation;

  int _max_iteration = 200;
  int _iter = 0;

  float _radius_sample = 2;   //
  float _radius_bias;
  if(!def_maze_mode)
    _radius_bias = 5; // _radius_bias+-_radius_sample,形成的环形区域
  else
    _radius_bias = 1; // _radius_bias+-_radius_sample,形成的环形区域

  do
  {
    _iter++;
    float _dist_sample =
        ((rand() % (N + 1) / (float)(N + 1)) * 2 - 1) * _radius_sample +
        _radius_bias;
    float _theta_sample = ((rand() % (N + 1) / (float)(N + 1)) * 2 - 1) * M_PI;
    // cout << "_dist_sample: " << _dist_sample << endl;
    // cout << "_theta_sample: " << _theta_sample << endl;

    float _x_sample = _dist_sample * cos(_theta_sample);
    float _y_sample = _dist_sample * sin(_theta_sample);
    // cout << "_x_sample: " << _x_sample << endl;
    // cout << "_y_sample: " << _y_sample << endl;

    Eigen::Vector3d _Decrease_vector(_x_sample, _y_sample, 0);
    rand_point = _Position + _Decrease_vector;

    // cout << "getRandom_aroundFrontier - rand_position: " << rand_point.transpose() << endl;

    if(_iter > _max_iteration){
      ROS_ERROR_STREAM("getRandom_aroundFrontier - Cannot Sample Valid point");
      break;
    }
  } while (!isInKnown_Feature_Area(rand_point) || isNearWall(rand_point) || !IsInLimitArea(rand_point.segment(0,2)));
  // }while(false);

  rand_point.z() = start_position_.z();
  // rand_point.z() = get_mapz(rand_point.segment(0,2));
  return rand_point;
}

rrt_3d::Node_3d *rrt_3d::getRandomNotObstacleNode_3d_inRange()
{
  float prob = 0.0;
  float random_f;
  int N = 999;
  random_f = rand() % (N + 1) / (float)(N + 1); //生成（0,1）的随机数
  if (random_f >= prob)
  { // 大于一定概率，则进行随机采样；否则向着目标点采样
    // 采样范围为起点与终点的中点为圆心，以一定的半径的圆范围内采样
    Eigen::Vector3d rand_point;
    Eigen::Vector3d rand_orientation;

    // cout << "end_position_: " << end_position_.transpose() << endl;
    // cout << "start_position_: " << start_position_.transpose() << endl;
    Eigen::Vector3d _Circle_Center((end_position_ + start_position_) * 0.5);
    // cout << "_Circle_Center: " << _Circle_Center.transpose() << endl;

    float _scale = 2;
    double _radius_sample =
        (end_position_ - start_position_).norm() * _scale * 0.5;
    // cout << "_radius_sample: " << _radius_sample << endl;

    // _radius_sample = (_radius_sample < 8) ? 8 : _radius_sample;
    _radius_sample = std::max(_radius_sample, 12.0);

    float _dist_sample = (rand() % (N + 1) / (float)(N + 1)) * _radius_sample;
    float _theta_sample = ((rand() % (N + 1) / (float)(N + 1)) * 2 - 1) * M_PI;
    // cout << "_dist_sample: " << _dist_sample << endl;
    // cout << "_theta_sample: " << _theta_sample << endl;

    float _x_sample = _dist_sample * cos(_theta_sample);
    float _y_sample = _dist_sample * sin(_theta_sample);
    // cout << "_x_sample: " << _x_sample << endl;
    // cout << "_y_sample: " << _y_sample << endl;

    Eigen::Vector3d _Decrease_vector(_x_sample, _y_sample, 0);
    rand_point = _Circle_Center + _Decrease_vector;
    // cout << "rand_position: " << rand_point.transpose() << endl;

    rand_orientation = Eigen::Vector3d(
        start_orientation_.x(),
        ((rand() % (N + 1) / (float)(N + 1)) * 2 - 1) * Range_Pitch_Sampling,
        end_orientation_.z());

    Node_3d *rand_node = new Node_3d;
    rand_node->position = rand_point;
    rand_node->orientation = rand_orientation;
    // cout << "sample orientation from random sample: " <<
    // rand_node->orientation << endl;
    return rand_node;
  }
  else
  {
    Node_3d *rand_node = new Node_3d;
    rand_node->position = end_position_;
    rand_node->orientation = end_orientation_;

    // cout << "sample orientation from end_point: " << rand_node->orientation
    // << endl;
    return rand_node;
  }
}

rrt_3d::Node_3d *rrt_3d::findNearestNode(Eigen::Vector3d current_position)
{
  double min_distance = 1e5;
  Node_3d *closest_node = NULL;
  for (auto node : rrt2d_nodes_)
  {
    double distance = (current_position - node->position).norm();
    if (distance < min_distance)
    {
      min_distance = distance;
      closest_node = node;
    }
  }

  // ROS_INFO("closest_node: %f %f %f", closest_node->position.x(),
  // closest_node->position.y(), closest_node->position.z());
  return closest_node;
}

rrt_3d::Node_3d *rrt_3d::findNearestNode(Node_3d *q_node)
{
  double min_distance = 1e5;
  Node_3d *closest_node = NULL;
  for (auto node : rrt2d_nodes_)
  {
    double distance_position = (q_node->position - node->position).norm();
    double distance_orientation =
        (q_node->orientation - node->orientation).norm();
    if ((distance_position + distance_orientation) < min_distance)
    {
      min_distance = (distance_position + distance_orientation);
      closest_node = node;
    }
  }

  // ROS_INFO("closest_node: %f %f %f", closest_node->position.x(),
  // closest_node->position.y(), closest_node->position.z());
  return closest_node;
}

/**
 * @brief
 * 从树中离采样点最近的节点开始，向采样点延伸stepsize，获得q_new,但是另外两个参数没啥用啊
 * @param q_rand
 * @param q_nearest
 * @param direction
 * @return
 */
rrt_3d::Node_3d *rrt_3d::getNewNode_3d(Node_3d *q_rand, Node_3d *q_nearest,
                                       Eigen::Vector3d direction)
{
  Eigen::Vector3d result;

  // ROS_INFO("q_nearest: %f %f %f", q_nearest->position.x(),
  // q_nearest->position.y(), q_nearest->position.z());
  // ROS_INFO("map_->getResolution(): %f", map_->getResolution());
  // ROS_INFO("step_size_position: %d", step_size_position);
  result.x() = q_nearest->position.x() + step_size_position * direction.x();
  result.y() = q_nearest->position.y() + step_size_position * direction.y();
  result.z() = q_nearest->position.z() + step_size_position * direction.z();

  Node_3d *q_new = new Node_3d;
  q_new->position = result;
  q_new->orientation = q_nearest->orientation;

  return q_new;
}

rrt_3d::Node_3d *rrt_3d::getNewNode_3d(Node_3d *q_rand, Node_3d *q_nearest,
                                       Eigen::Vector3d direction_position,
                                       Eigen::Vector3d direction_orientation)
{
  Eigen::VectorXd result(6);

  // ROS_INFO("q_nearest: %f %f %f", q_nearest->position.x(),
  // q_nearest->position.y(), q_nearest->position.z());
  // ROS_INFO("map_->getResolution(): %f", map_->getResolution());
  // ROS_INFO("step_size_position: %d", step_size_position);
  result(0) =
      q_nearest->position.x() + step_size_position * direction_position(0);
  result(1) =
      q_nearest->position.y() + step_size_position * direction_position(1);
  result(2) =
      q_nearest->position.z() + step_size_position * direction_position(2);

  result(3) = q_nearest->orientation.x() +
              step_size_orientation * direction_orientation(0);
  result(4) = q_nearest->orientation.y() +
              step_size_orientation * direction_orientation(1);
  result(5) = q_nearest->orientation.z() +
              step_size_orientation * direction_orientation(2);

  Node_3d *q_new = new Node_3d;
  q_new->position << result(0), result(1), result(2);
  q_new->orientation << result(3), result(4), result(5);

  return q_new;
}

/**
 * @brief 如果q_new合适的话，就把它填入路径
 * @param q_nearest
 * @param q_new
 */
void rrt_3d::addNewNode(Node_3d *q_nearest, Node_3d *q_new)
{
  q_new->parent = q_nearest;
  q_nearest->children.push_back(q_new);
  rrt2d_nodes_.push_back(q_new);
  lastNode_ = q_new;
  //    std::cout<<"Get a new Node_3d"<<std::endl;
  //    std::cout<<"Now nodes have:"<<nodes_.size()<<std::endl;
  //    for(auto node:nodes_){
  //        std::cout<<node->position<<"\n";
  //    }
  //    std::cout<<"\n";
}

bool rrt_3d::addNewNode_rrtstar(Node_3d *q_new)
{
  Near.clear();
  dist.clear();
  cost.clear();
  m_gain_vec.clear();
  m_totalgain_vec.clear();
  minCost = 10000;
  maxGain = -10000;
  idx_maxCost = -1;

  Node_3d *q_nearest_maxgain = chooseParent(q_new);

  // Node_3d *q_nearest_maxgain = chooseParent_gain(q_new);
  // cout << "q_new parent position: " <<
  // q_nearest_maxgain->position.transpose() << endl;

  if(q_nearest_maxgain == NULL){
    return false;
  }

  q_new->parent = q_nearest_maxgain;
  q_new->cost_motion = minCost;

  // cout << "q_new position: " << q_new->position.transpose() << endl;
  // cout << "q_new parent position: " << q_new->parent->position.transpose() <<
  // endl;

  // q_new->total_gain = maxGain;
  // q_new->total_cost_goal = q_new->parent->total_cost_goal + q_new->cost_goal;
  // rewire_gain(q_new);
  rewire(q_new);

  q_nearest_maxgain->children.push_back(q_new);
  rrt2d_nodes_.push_back(q_new);
  return true;
}

/**
 * @brief 判断是否到达目的地
 * @return
 */
bool rrt_3d::isArrived(Node_3d *node)
{
  //    if((lastNode_->position - end_position_).norm()
  //    < 2.2*map_->getResolution())
  //        return true;

  if ((node->position - end_position_).norm() < 2 * _gridmap.info.resolution && Line_NoCollision(node->position, end_position_))
    return true;
  return false;
}

rrt_3d::Node_3d *rrt_3d::chooseParent(Node_3d *&q)
{
  Node_3d *potentialParent = NULL;
  Node_3d *curNode = NULL;
  for (
      auto node :
      rrt2d_nodes_)
  { //遍历所有节点，找到位于新节点radius范围内的所有节点，并添加至Near中
    double distance = (node->position - q->position).norm();
    if (distance < m_choose_radius)
    {
      dist.push_back(distance); //记录新节点和近邻之间的距离
      Near.push_back(node);     //获得了新节点的近邻
    }
  }
  for (int i = 0; i < (int)Near.size();
       i++)
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
    //  && Line_NoCollision(curNode->position, q->position)
    if (c < minCost)
    {
      minCost = c;
      potentialParent = Near[i];
    }
  }
  return potentialParent;
}

void rrt_3d::rewire(
    Node_3d *&
        q)
{ //基于到每个节点的路径代价最小，判断是否要将Near中的节点的父节点设置为新节点
  for (int i = 0; i < (int)Near.size(); i++)
  {
    if (minCost + dist[i] < cost[i])
    {
      Near[i]->parent = q;
      Near[i]->cost_motion = q->cost_motion + dist[i];
    }
  }
}

rrt_3d::Node_3d *rrt_3d::chooseParent_gain(Node_3d *&q)
{
  Node_3d *potentialParent = NULL;
  Node_3d *curNode = NULL;
  for (
      auto node :
      rrt2d_nodes_)
  { //遍历所有节点，找到位于新节点radius范围内的所有节点，并添加至Near中
    double distance = (node->position - q->position).norm();
    if (distance < m_choose_radius)
    {
      m_gain_vec.push_back(node->gain);            //记录新节点和近邻之间的距离
      m_totalgain_vec.push_back(node->total_gain); //记录新节点和近邻之间的距离
      Near.push_back(node);                        //获得了新节点的近邻
    }
  }

  // cout << "Near.size(): " << Near.size() << endl;

  for (int i = 0; i < (int)Near.size();
       i++)
  { //依据从起点到近邻点距离代价最小原则选择新的父节点
    curNode = Near[i];
    double c = m_totalgain_vec[i];

    c += q->gain;
    cost.push_back(c);
    if (c > maxGain)
    {
      maxGain = c;
      idx_maxCost = i;
      potentialParent = Near[i];
    }
  }
  return potentialParent;
}

void rrt_3d::rewire_gain(Node_3d *&q)
{
  //基于到每个节点的路径代价最小，判断是否要将Near中的节点的父节点设置为新节点
  for (int i = 0; i < (int)Near.size(); i++)
  {
    if (i == idx_maxCost)
    {
      continue;
    }

    if (maxGain + m_gain_vec[i] > Near[i]->total_gain)
    {
      Near[i]->parent = q;
      Near[i]->total_gain = maxGain + m_gain_vec[i];
    }
  }
}

/**
 * @brief visualize path by writing path into map module
 */
void rrt_3d::writeMap()
{
  for (auto node : path_3d)
  {
    std::cout << node->position << std::endl;
    // map_->mixPathMap(node->position, true);
  }
}

void rrt_3d::writeInfo2File(std::string output_name)
{
  double distance = 0;
  double tmp_distance = 0;
  ofstream fout;
  fout.open(output_name);
  // write basic infomation to file
  fout << "step_size = " << step_size_orientation << std::endl;
  fout << "max_iter = " << max_iter_ << std::endl;
  fout << "start_position = " << start_position_
       << "\tend_position = " << end_position_ << std::endl;
  // write position of path-node and distance between two nodes
  fout << "START_POSITION\t\t\t"
       << "END_POSITION\t\t\t"
       << "DISTANCE\t"
       << "TOTAL_DISTANCE\n";
  for (int i = path_3d.size() - 1; i > 0; i--)
  {
    tmp_distance = (path_3d[i]->position - path_3d[i - 1]->position).norm();
    fout << path_3d[i]->position << "\t" << path_3d[i - 1]->position << "\t"
         << tmp_distance << "\t";
    distance += tmp_distance;
    fout << distance << std::endl;
  }
  // write distance between last_node_position and end_position
  fout << "LAST_NODE_POSITION\t\t\t"
       << "FINAL_POSITION\t\t\t"
       << "DISTANCE\t"
       << "TOTAL_DISTANCE\n";
  tmp_distance = (end_position_ - path_3d[0]->position).norm();
  fout << path_3d[0]->position << "\t" << end_position_ << "\t" << tmp_distance
       << "\t";
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
void rrt_3d::deleteAllNodes(Node_3d *root)
{
  for (int i = 0; i < (int)root->children.size(); i++)
  {
    deleteNodes(root->children[i]);
  }
  delete root;
}

void rrt_3d::deleteNode(Node_3d *node)
{
  Eigen::Vector3d position_tmp = node->position;
  Node_3d *parent_node = node->parent;
  ROS_WARN("Children.size: %d", parent_node->children.size());

  if (parent_node->children.size() == 1)
  {
    ROS_WARN("delete children: %f %f %f", node->position.x(),
             node->position.y(), node->position.z());
    parent_node->children.clear();
    ROS_WARN("Children.size: %d", parent_node->children.size());
  }
  else
  {
    vector<Node_3d *>::iterator it;

    for (it = parent_node->children.begin(); it != parent_node->children.end();
         ++it)
    {
      ROS_WARN("delete children: %f %f %f", (*it)->position.x(),
               (*it)->position.y(), (*it)->position.z());
      if (abs(position_tmp.x() - (*it)->position.x()) < 0.0005 &&
          abs(position_tmp.y() - (*it)->position.y()) < 0.0005 &&
          abs(position_tmp.z() - (*it)->position.z()) < 0.0005)
      {
        ROS_WARN("delete children: %f %f %f", (*it)->position.x(),
                 (*it)->position.y(), (*it)->position.z());
        parent_node->children.erase(it);
        ROS_WARN("Children.size: %d", parent_node->children.size());
        break;
      }
    }

    for (it = parent_node->children.begin(); it != parent_node->children.end();
         it++)
    {
      ROS_WARN("delete children: %f %f %f", (*it)->position.x(),
               (*it)->position.y(), (*it)->position.z());
    }
  }
}

float rrt_3d::gain_exploration_2d(Node_3d *q_new)
{
  Eigen::Vector2d pos(q_new->position.x(), q_new->position.y());
  GridIndex q_new_idx = ConvertWorld2GridIndex(pos);

  // float fsmi_data_node = compute_fsmi_point(q_new_idx.x(), q_new_idx.y());
  float fsmi_data_node;
  fsmi_data_node = compute_fsmi_point(q_new_idx.x(), q_new_idx.y(),
                                      5, false);

  return ((-1) * fsmi_data_node);
}

// pitch 为正，则相机朝下； yaw 为正，则相机逆时针旋转
float rrt_3d::gain_perception(Node_3d *q_new)
{
  // 统计某个视角内栅格总的信息量，from color of octree
  Eigen::Vector3d Unit_vec{1, 0, 0};

  const float angle_resolution = 0.1; // 0.1 about 6 degree

  // cout << "position: " << q_new->position.transpose() << endl;

  float pitch_max = q_new->orientation[1] + vertical_fov * drg2rad * 0.5;
  float pitch_min = q_new->orientation[1] - vertical_fov * drg2rad * 0.5;

  float yaw_max = q_new->orientation[2] + horizontal_fov * drg2rad * 0.5;
  float yaw_min = q_new->orientation[2] - horizontal_fov * drg2rad * 0.5;

  rrt3d_intersection_point.clear();
  // int line_count = 0;
  // 117 line
  for (float pitch_iter = pitch_min; pitch_iter < pitch_max;
       pitch_iter += angle_resolution)
  {
    for (float yaw_iter = yaw_min; yaw_iter < yaw_max;
         yaw_iter += angle_resolution)
    {
      // line_count ++ ;
      // cout << "rpy = " << q_new->orientation[0] << " " << pitch_iter << " "
      // << yaw_iter << endl;
      Eigen::Matrix3d rot_mat;
      rot_mat =
          Eigen::AngleAxisd(q_new->orientation[0], Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(pitch_iter, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(yaw_iter, Eigen::Vector3d::UnitZ());
      // cout << "rotation matrix3 =\n" << rot_mat << endl;

      Eigen::Vector3d intersect_line_dir = rot_mat * Unit_vec;

      // cout << "vec after rotation: " << intersect_line_dir.transpose() <<
      // endl;

      Eigen::Vector3d intersect_point;
      // intersect_point = get_intersect_point(q_new->position,
      // intersect_line_dir); cout << "intersect_point: " <<
      // intersect_point.transpose() << endl;

      octomap::point3d _Intersected_point;
      bool Is_hit_occupied =
          _octree->castRay(Convert_EigenVec2Octomap3d(q_new->position),
                           Convert_EigenVec2Octomap3d(intersect_line_dir),
                           _Intersected_point, true, Max_range);
      intersect_point = Convert_Octomap3d2EigenVec(_Intersected_point);

      // can not get the occupied voxel
      if (Is_hit_occupied)
      {
        // cout << "intersect_point: " << intersect_point.transpose() << endl;

        // 避免重复插入
        if (std::find(rrt3d_intersection_point.begin(),
                      rrt3d_intersection_point.end(),
                      intersect_point) == rrt3d_intersection_point.end())
        {
          rrt3d_intersection_point.push_back(intersect_point);
        }
      }
    }
  }
  // if(m_gain_construction > 0)
  //   cout << "RRT3D::Construct gain: " << m_gain_construction << endl;

  // cout << "line_count: " << line_count << endl;

  int featureRich_counter = 1;
  for (auto observed_voxel : rrt3d_intersection_point)
  {
    // cout << "intersect_point: " << observed_voxel.transpose() << endl;

    octomap::point3d observed_voxel_pt_(observed_voxel.x(), observed_voxel.y(),
                                        observed_voxel.z());

#if def_octo_generator
    octomap::ColorOcTreeNode *node = _octree->search(observed_voxel_pt_, m_Frontier3D_depth);
#else
    octomap::OcTreeNode *node = _octree->search(observed_voxel_pt_, m_Frontier3D_depth);
#endif

    // if color is green
#if def_w_uncertainty
    if ((node->getColor().r <= (4 + 8)) && (node->getColor().r >= (4 - 4)) &&
        (node->getColor().g <= (250 + 5)) &&
        (node->getColor().g >= (250 - 5)) && (node->getColor().b <= (7 + 7)) &&
        (node->getColor().b >= (7 - 7)))
    {
      featureRich_counter++;
    }
#endif
    // light blue
    // if ((r <= (61 + 5)) && (r >= (61 - 5)) && (g <= (230 + 5)) && (g >= (230
    // - 5)) && (b <= (250 + 5)) && (b >= (250 - 5))) else
    // {
    //   return false;
    // }
  }

  // cout << "gain perception = " << featureRich_counter << endl;
  return featureRich_counter;
}

float m_min_perception_range = 2.0;   // 根据1/2 pitch计算得到
float m_max_perception_range = 10.0;
float rrt_3d::gain_perception(Eigen::Vector3d position, Eigen::Vector3d orientation)
{
  // 统计某个视角内栅格总的信息量，from color of octree
  int featureLess_counter = 0;

  float _query_step = _gridmap.info.resolution;
  
  for(float _query_range = m_min_perception_range; _query_range < m_max_perception_range; _query_range += _query_step ){
    float _delta_x = cos(orientation[2]) * _query_range;
    float _delta_y = sin(orientation[2]) * _query_range;

    Eigen::Vector2d _query_pos;
    _query_pos.x() = position.x() + _delta_x;
    _query_pos.y() = position.y() + _delta_y;

    Eigen::Vector2i _query_idx = ConvertWorld2GridIndex(_query_pos);
    int _query_map_idx = _gridmap.info.width * _query_idx.y() + _query_idx.x();
    
    Eigen::Vector3d observed_voxel(_query_pos.x(), _query_pos.y(), 0);

    if(_gridmap.data[_query_map_idx] == 100){
      // rrt3d_intersection_point_visualization.push_back(observed_voxel);
    }
    else if(_gridmap.data[_query_map_idx] == -100){
      featureLess_counter++;
      rrt3d_intersection_point_visualization.push_back(observed_voxel);
    }
  }

  // cout << "gain perception = " << featureRich_counter << endl;
  return featureLess_counter;
}

// ! 十分耗时
// float rrt_3d::gain_perception(Eigen::Vector3d position, Eigen::Vector3d orientation)
// {
//   // 统计某个视角内栅格总的信息量，from color of octree
//   Eigen::Vector3d Unit_vec{1, 0, 0};

//   const float angle_resolution = 0.1; // 0.1 about 6 degree

//   // cout << "position: " << position.transpose() << endl;

//   float pitch_max = orientation[1] + vertical_fov * drg2rad * 0.5;
//   float pitch_min = orientation[1] - vertical_fov * drg2rad * 0.5;

//   float yaw_max = orientation[2] + horizontal_fov * drg2rad * 0.5;
//   float yaw_min = orientation[2] - horizontal_fov * drg2rad * 0.5;

//   rrt3d_intersection_point.clear();
//   // int line_count = 0;
//   // 117 line
//   for (float pitch_iter = pitch_min; pitch_iter < pitch_max;
//        pitch_iter += angle_resolution)
//   {
//     for (float yaw_iter = yaw_min; yaw_iter < yaw_max;
//          yaw_iter += angle_resolution)
//     {
//       // line_count ++ ;
//       // cout << "rpy = " << orientation[0] << " " << pitch_iter << " "
//       // << yaw_iter << endl;
//       Eigen::Matrix3d rot_mat;
//       rot_mat =
//           Eigen::AngleAxisd(orientation[0], Eigen::Vector3d::UnitX()) *
//           Eigen::AngleAxisd(pitch_iter, Eigen::Vector3d::UnitY()) *
//           Eigen::AngleAxisd(yaw_iter, Eigen::Vector3d::UnitZ());
//       // cout << "rotation matrix3 =\n" << rot_mat << endl;

//       Eigen::Vector3d intersect_line_dir = rot_mat * Unit_vec;

//       // cout << "vec after rotation: " << intersect_line_dir.transpose() <<
//       // endl;

//       Eigen::Vector3d intersect_point;
//       // intersect_point = get_intersect_point(position,
//       // intersect_line_dir); cout << "intersect_point: " <<
//       // intersect_point.transpose() << endl;

//       octomap::point3d _Intersected_point;
//       bool Is_hit_occupied =
//           _octree->castRay(Convert_EigenVec2Octomap3d(position),
//                            Convert_EigenVec2Octomap3d(intersect_line_dir),
//                            _Intersected_point, true, Max_range);
//       intersect_point = Convert_Octomap3d2EigenVec(_Intersected_point);

//       // can not get the occupied voxel
//       if (Is_hit_occupied)
//       {
//         // cout << "intersect_point: " << intersect_point.transpose() << endl;

//         // 避免重复插入
//         if (std::find(rrt3d_intersection_point.begin(),
//                       rrt3d_intersection_point.end(),
//                       intersect_point) == rrt3d_intersection_point.end())
//         {
//           rrt3d_intersection_point.push_back(intersect_point);
//         }
//       }
//     }
//   }
//   // if(m_gain_construction > 0)
//   //   cout << "RRT3D::Construct gain: " << m_gain_construction << endl;

//   // cout << "line_count: " << line_count << endl;

//   int featureRich_counter = 1;
//   for (auto observed_voxel : rrt3d_intersection_point)
//   {
//     // cout << "intersect_point: " << observed_voxel.transpose() << endl;

//     octomap::point3d observed_voxel_pt_(observed_voxel.x(), observed_voxel.y(),
//                                         observed_voxel.z());

//     octomap::ColorOcTreeNode *node = _octree->search(observed_voxel_pt_, 16);

//     // if node is occupied and color is green
//     if (((node->getColor().r <= (4 + 8)) && (node->getColor().r >= (4 - 4)) &&
//         (node->getColor().g <= (250 + 5)) &&
//         (node->getColor().g >= (250 - 5)) && (node->getColor().b <= (7 + 7)) &&
//         (node->getColor().b >= (7 - 7))))
//     {
//       featureRich_counter++;
//       // rrt3d_intersection_point_visualization.push_back(observed_voxel);
//     }
//     else{
//       rrt3d_intersection_point_visualization.push_back(observed_voxel);
//     }
//     // light blue
//     // if ((r <= (61 + 5)) && (r >= (61 - 5)) && (g <= (230 + 5)) && (g >= (230
//     // - 5)) && (b <= (250 + 5)) && (b >= (250 - 5))) else
//     // {
//     //   return false;
//     // }
//   }

//   // cout << "gain perception = " << featureRich_counter << endl;
//   return featureRich_counter;
// }

// 可视化FOV内的栅格
void rrt_3d::intersection_point_visualization(Node_3d *q_new)
{
  // 统计某个视角内栅格总的信息量，from color of octree

  Eigen::Vector3d Unit_vec{1, 0, 0};

  const float angle_resolution = 0.1; // 0.1 about 6 degree

  // cout << "position: " << q_new->position.transpose() << endl;

  float pitch_max = q_new->orientation[1] + vertical_fov * drg2rad * 0.5;
  float pitch_min = q_new->orientation[1] - vertical_fov * drg2rad * 0.5;

  float yaw_max = q_new->orientation[2] + horizontal_fov * drg2rad * 0.5;
  float yaw_min = q_new->orientation[2] - horizontal_fov * drg2rad * 0.5;

  // int line_count = 0;
  // 117 line
  for (float pitch_iter = pitch_min; pitch_iter < pitch_max;
       pitch_iter += angle_resolution)
  {
    for (float yaw_iter = yaw_min; yaw_iter < yaw_max;
         yaw_iter += angle_resolution)
    {
      // line_count ++ ;
      // cout << "rpy = " << q_new->orientation[0] << " " << pitch_iter << " "
      // << yaw_iter << endl;
      Eigen::Matrix3d rot_mat;
      rot_mat =
          Eigen::AngleAxisd(q_new->orientation[0], Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(pitch_iter, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(yaw_iter, Eigen::Vector3d::UnitZ());
      // cout << "rotation matrix3 =\n" << rot_mat << endl;

      Eigen::Vector3d intersect_line_dir = rot_mat * Unit_vec;

      // cout << "vec after rotation: " << intersect_line_dir.transpose() <<
      // endl;

      Eigen::Vector3d intersect_point;
      // intersect_point = get_intersect_point(q_new->position,
      // intersect_line_dir); cout << "intersect_point: " <<
      // intersect_point.transpose() << endl;

      octomap::point3d _Intersected_point;
      bool Is_hit_occupied =
          _octree->castRay(Convert_EigenVec2Octomap3d(q_new->position),
                           Convert_EigenVec2Octomap3d(intersect_line_dir),
                           _Intersected_point, true, Max_range);
      intersect_point = Convert_Octomap3d2EigenVec(_Intersected_point);

      if (Is_hit_occupied)
      {
        // 避免重复插入
        if (std::find(rrt3d_intersection_point_visualization.begin(),
                      rrt3d_intersection_point_visualization.end(),
                      intersect_point) ==
            rrt3d_intersection_point_visualization.end())
        {
          rrt3d_intersection_point_visualization.push_back(intersect_point);
        }
      }
    }
  }
  // cout << "line_count: " << line_count << endl;
}

Eigen::Vector3d rrt_3d::get_OrinientFromVector(Eigen::Vector3d _origin_point,
                                               Eigen::Vector3d _target_point)
{
  Eigen::Vector3d _ret_Orinient;

  // cout << "get_OrinientFromVector - _origin_point: " <<
  // _origin_point.transpose() << endl; cout << "get_OrinientFromVector -
  // _target_point: " << _target_point.transpose() << endl;
  Eigen::Vector3d _dir_view = _target_point - _origin_point;

  double _dir_view_xy_norm =
      sqrt(_dir_view.x() * _dir_view.x() + _dir_view.y() * _dir_view.y());
  // cout << "_dir_view_xy_norm: " << _dir_view_xy_norm << endl;
  double _dir_view_pitch = atan2(_dir_view.z(), _dir_view_xy_norm);
  // cout << "_dir_view_pitch: " << _dir_view_pitch << endl;
  double _dir_view_yaw = atan2(_dir_view.y(), _dir_view.x());
  // cout << "get_OrinientFromVector - _dir_view_pitch: " << _dir_view_pitch <<
  // endl; cout << "get_OrinientFromVector - _dir_view_yaw: " << _dir_view_yaw
  // << endl;

  _ret_Orinient.x() = 0;                // roll
  _ret_Orinient.y() = -_dir_view_pitch; // pitch
  _ret_Orinient.z() = _dir_view_yaw;    // yaw

  return _ret_Orinient;
}

bool rrt_3d::fpInFOV(Eigen::Vector3d _sample_around_Frontier,
                     Eigen::Vector3d _frontierCluster,
                     Eigen::Vector3d _frontierpoint,
                     int& _unknow_num)
{
  bool _DEBUG_F = false;
  float _Max_range = 10;
  bool pitch_inFOV = false, yaw_inFOV = false, no_occlued = false;

  Eigen::Vector3d _dir_view = _frontierCluster - _sample_around_Frontier;
  Eigen::Vector3d _dir_fp = _frontierpoint - _sample_around_Frontier;
  
  if(_DEBUG_F){
    cout << "_sample_around_Frontier: " << _sample_around_Frontier.transpose()
    << endl; 
    cout << "_frontierCluster: " << _frontierCluster.transpose() <<
    endl; 
    cout << "_frontierpoint: " << _frontierpoint.transpose() << endl;
    cout << "_dir_view: " << _dir_view.transpose() << endl;
    cout << "_dir_fp: " << _dir_fp.transpose() << endl;
  }
  
  // pitch in fov
  double _dir_view_xy_norm =
      sqrt(_dir_view.x() * _dir_view.x() + _dir_view.y() * _dir_view.y());
  // cout << "_dir_view_xy_norm: " << _dir_view_xy_norm << endl;
  double _dir_view_pitch = atan2(_dir_view.z(), _dir_view_xy_norm);
  // cout << "_dir_view_pitch: " << _dir_view_pitch << endl;

  double _dir_fp_xy_norm =
      sqrt(_dir_fp.x() * _dir_fp.x() + _dir_fp.y() * _dir_fp.y());
  // cout << "_dir_fp_xy_norm: " << _dir_fp_xy_norm << endl;
  double _dir_fp_pitch = atan2(_dir_fp.z(), _dir_fp_xy_norm);
  // cout << "_dir_fp_pitch: " << _dir_fp_pitch << endl;

  if ((abs(_dir_view_pitch - _dir_fp_pitch) < vertical_fov * drg2rad * 0.5) &&
      sqrt(_dir_fp_xy_norm * _dir_fp_xy_norm + _dir_fp.z() * _dir_fp.z()) <
          Max_range)
  {
    // cout << "true pitch error: " << abs(_dir_view_pitch - _dir_fp_pitch) <<
    // endl; cout << "true horizon thresh: " << vertical_fov*drg2rad*0.5 <<
    // endl;
    pitch_inFOV = true;
  }
  else
  {
    // cout << "false pitch error: " << abs(_dir_view_pitch - _dir_fp_pitch) <<
    // endl; cout << "false horizon thresh: " << vertical_fov*drg2rad*0.5 <<
    // endl;
    pitch_inFOV = false;
    return false;
  }
  if(_DEBUG_F && pitch_inFOV)
    cout << "pitch in FOV: " << pitch_inFOV << endl;

  // yaw in fov
  double _dir_view_yaw = atan2(_dir_view.y(), _dir_view.x());
  double _dir_fp_yaw = atan2(_dir_fp.y(), _dir_fp.x());
  // cout << "_dir_view_yaw: " << _dir_view_yaw << endl;
  // cout << "_dir_fp_yaw: " << _dir_fp_yaw << endl;

  // cout << "node_pos: " << node_pos.transpose() << "frontierpoint: " <<
  // frontierpoint << "dir_yaw: " << dir_yaw << endl;

  if ((abs(_dir_view_yaw - _dir_fp_yaw) < horizontal_fov * drg2rad * 0.5) &&
      sqrt(_dir_fp_xy_norm * _dir_fp_xy_norm + _dir_fp.z() * _dir_fp.z()) <
          Max_range)
  {
    // cout << "true yaw error: " << abs(_dir_view_yaw - _dir_fp_yaw) << endl;
    // cout << "true horizon thresh: " << horizontal_fov*drg2rad*0.5 << endl;
    yaw_inFOV = true;
  }
  else
  {
    // cout << "false yaw error: " << abs(_dir_view_yaw - _dir_fp_yaw) << endl;
    // cout << "false horizon thresh: " << horizontal_fov*drg2rad*0.5 << endl;
    yaw_inFOV = false;
    return false;
  }

  if(_DEBUG_F && yaw_inFOV)
    cout << "yaw in FOV: " << yaw_inFOV << endl;

  octomap::point3d _Intersected_point;
  bool Is_hit_occupied =
  get_intersect_point(_sample_around_Frontier, _dir_fp, _Intersected_point, _unknow_num);

  // bool Is_hit_occupied = _octree->castRay(
  //     Convert_EigenVec2Octomap3d(_sample_around_Frontier),
  //     Convert_EigenVec2Octomap3d(_dir_fp), _Intersected_point, true, Max_range);
  if(_DEBUG_F && Is_hit_occupied){
    cout << "Is_hit_occupied: " << Is_hit_occupied << endl;
  }

  // const int m_Frontier3D_depth = 16; // !!!与project2d.hpp中一致
  // 相交的点是否为边界点
  if (Is_hit_occupied &&
      ((Convert_Octomap3d2EigenVec(_Intersected_point) - _frontierpoint)
           .norm() < _octree->begin_leafs(m_Frontier3D_depth).getSize()))
  {
    // cout << "true _Intersected_point: " <<
    // Convert_Octomap3d2EigenVec(_Intersected_point).transpose() << endl;
    no_occlued = true;
  }
  else
  {
    // cout << "false _Intersected_point: " <<
    // Convert_Octomap3d2EigenVec(_Intersected_point).transpose() << endl;
    no_occlued = false;
    return false;
  }
  if(_DEBUG_F && no_occlued)
    cout << "no_occlued: " << no_occlued << endl;

  // 2d探索结束后，剔除被墙遮挡的点
  if(Explore_Done_2d && !Explore_Done_3d && pitch_inFOV && yaw_inFOV && !no_occlued){
    cout << "the frontier is occlued: " << _frontierpoint.transpose() << endl;
    Frontier3D_black_list.push_back(_frontierpoint);
  }

  if (pitch_inFOV && yaw_inFOV && no_occlued)
  {
    // seed_fp_infov = frontierpoint;
    return true;
  }
  else
  {
    return false;
  }
}

int rrt_3d::fpNumber_InFOV(Eigen::Vector3d _sample_around_Frontier,
                           Eigen::Vector3d _frontierCluster,
                           int& count_unknow)
{
  int count_fp = 0;
  float _distance_Thresh = 10;
  for (auto it : g_Frontiers3D_pos)
  {
    if ((_sample_around_Frontier - it).norm() <= _distance_Thresh &&
        fpInFOV(_sample_around_Frontier, _frontierCluster, it, count_unknow))
    {
      count_fp++;
    }
  }
  return count_fp;
}

double rrt_3d::test_yaw(Eigen::Vector3d node_pos,
                        Eigen::Vector3d frontierpoint)
{
  // Eigen::Vector3d FP_eigen{frontierpoint.x(), frontierpoint.y(),
  // frontierpoint.z()}; pitch in fov

  // yaw in fov
  Eigen::Vector3d dir = frontierpoint - node_pos;

  double dir_yaw = atan2(dir.y(), dir.x());

  return dir_yaw;
}


bool rrt_3d::Frontier3D_inRange(Node_3d *q_node)
{
  float _range_frontier_detect = 6;

  for (auto it : g_Frontiers3D_pos)
  {
    if ((q_node->position - it).norm() < _range_frontier_detect)
    {
      return true;
    }
  }
  return false;
}

void rrt_3d::Enter_recover_mode()
{
  path_3d.clear();
  track_path_3d.clear();

  double _min_dis = 1e5;
  Eigen::Vector3d _min_dis_sample;
  float _sample_radius = 5;
  float delta_theta = M_PI / 8;
  float _delta_x, _delta_y;
  bool _find_node = false;
  for(; _sample_radius > step_size_position; _sample_radius -= 1.0){
    for (float _theta_sample = 0; _theta_sample < 2 * M_PI;
       _theta_sample += delta_theta)
    {
      _delta_x = _sample_radius * cos(_theta_sample);
      _delta_y = _sample_radius * sin(_theta_sample);

      Eigen::Vector3d _Sample_Pos(start_position_.x() + _delta_x,
                                  start_position_.y() + _delta_y,
                                  start_position_.z());

      if (!isNearWall(_Sample_Pos) && !isInfeatureless(_Sample_Pos) &&
          Line_NoCollision(start_position_, _Sample_Pos) &&
          isInKnown_Feature_Area(_Sample_Pos))
      {
        double _sample_distance = (_Sample_Pos - end_position_).norm();
        if (_sample_distance < _min_dis)
        {
          _min_dis = _sample_distance;
          _min_dis_sample = _Sample_Pos;
          _find_node = true;
        }
      }
    }
    if(_find_node){
      cout << "Find the recover node with radius " << _sample_radius << endl;
      break;
    }
  }

  if(!_find_node){
    cout << "Not Find the recover node" << endl;
    return;
  }

  float _adjacent_dis = (_min_dis_sample - start_position_).norm();
  int _insert_num = floor(_adjacent_dis / step_size_position);

  for (int j = 0; j < _insert_num; ++j)
  {
    Node_3d *_q_insert = new Node_3d;
    _q_insert->is_adjust_construct = false;
    _q_insert->is_adjust_explore = false;

    double _step_scale = ((j + 1) * 1.0 / (_insert_num + 1));
    Eigen::Vector3d _position_step =
        (_min_dis_sample - start_position_) * _step_scale;
    // cout << "_position_step: " << _position_step.transpose() << endl;

    _q_insert->position = start_position_ + _position_step;
    cout << "Enter_recover_mode - _q_insert->position: " << _q_insert->position.transpose() << endl;

    _q_insert->orientation.x() = start_orientation_.x();
    _q_insert->orientation.y() = 0;
    _q_insert->orientation.z() = start_orientation_.z();
    // cout << "Enter_recover_mode - _q_insert->orientation: " << _q_insert->orientation.transpose() << endl;
    // cout << "_orientation_step: " << _orientation_step.transpose() << endl;

    path_3d.push_back(_q_insert);
    Eigen::VectorXd _node(6);
    _node << _q_insert->position, _q_insert->orientation;
    track_path_3d.push_back(_node);

    g_BestPositionSet_aroundFrontier.push_back(_min_dis_sample);
    g_BestOrinientSet_aroundFrontier.push_back(start_orientation_);
    g_sampleSet_fpNum.push_back(10000.0);
  }

  cout << "Enter_recover_mode - Done" << endl;
}

float lamda_C2goal = 0;
float minimum_size_position =
    0.3; // 相邻节点在minimum_size_position以内则不进行生长

bool rrt_3d::run_w_perception_layered()
{
  srand(static_cast<ushort>(time(NULL)));
  std::chrono::time_point<std::chrono::system_clock> start, end_rrtgrow, end_planning;
  start = std::chrono::system_clock::now();
  bool arrive_flag = false;

  cout << "node_size: " << rrt2d_nodes_.size() << endl;
  std::cout << "rrt_3d start_position: " << root_->position.transpose()
            << " " << root_->orientation.transpose() << std::endl;
  std::cout << "rrt_3d end_position: " << end_position_.transpose()
            << " " << end_orientation_.transpose() << std::endl;

  if((end_position_ - start_position_).norm() < 2){
    track_path_3d.clear();
    Eigen::VectorXd _node(6);
    _node << start_position_, start_orientation_;

    track_path_3d.push_back(_node);

    _node.segment(0,3) = end_position_;
    _node.segment(3,3) = end_orientation_;

    track_path_3d.push_back(_node);
    ROS_WARN("GOAL Too Near......");
    return true;
  }

  for (int i = 0; i < max_iter_; )
  {
    // Eigen::Vector3d cur_pos_test{0,0,0};
    // octomap::point3d frontierpoint_test{1, -1*float(i*10)/max_iter_, 0};
    // double test_y = test_yaw(cur_pos_test, frontierpoint_test);
    // cout << "frontierpoint_test: " << frontierpoint_test << "test_yaw: " <<
    // test_y << endl;

    // cout << "i=" << i << endl;
    Node_3d *q_rand = getRandomNotObstacleNode_3d_inRange();
    // ROS_INFO_STREAM("q_rand: " << q_rand->position.transpose() <<
    // q_rand->orientation.transpose());

    // 寻找离采样点最近的节点
    // Node_3d *q_nearest = findNearestNode(q_rand);
    Node_3d *q_nearest = findNearestNode(q_rand->position);
    // ROS_INFO_STREAM("q_nearest: " << q_nearest->position.transpose() <<
    // q_nearest->orientation.transpose());

    Eigen::Vector3d direction_position = q_rand->position - q_nearest->position;

    bool grow_flag = true;
    Node_3d *q_new = new Node_3d;
    // if (direction_position.norm() > (step_size_position))
    {
      // cout << "direction_position.normalized: " <<
      // direction_position.normalized().transpose() << endl;
      direction_position = direction_position.normalized();
      q_new->position =
          q_nearest->position + step_size_position * direction_position;
    }
    q_new->orientation = q_rand->orientation;
    // else
    // {
    //   // q_new->position = q_rand->position;
    //   grow_flag = false;
    // }

    // ROS_INFO_STREAM("q_new: " << q_new->position.transpose() << " "
    //                           << q_new->orientation.transpose());

    if (grow_flag == true &&
        (isInfeatureless(q_new->position, q_new->orientation) 
        || isNearWall(q_new->position) 
         || !isInKnown_Feature_Area(q_new->position)
         || !IsInLimitArea(q_new->position.segment(0,2))
        ))
    { 
      grow_flag = false;

    }

    if (grow_flag == true)
    {
      // RRT*
      if(!addNewNode_rrtstar(q_new)){
        continue;
      }

      // float node_gain;
      // node_gain = q_new->total_gain - lamda_C2goal * q_new->total_cost_goal;

      // 到达终点
      if (isArrived(q_new))
      {
        // cout << "rrt Arrived end_position!" << endl;
        if (q_new->cost_motion < bestGain_)
        {
          bestGain_ = q_new->cost_motion; // 取子节点增益最大的子树
          bestNode_ = q_new;
          // cout << "bestGain_: " << bestGain_ << endl;
          // cout << "bestNode Gain_: " << bestNode_->gain << endl;
        }
        arrive_flag = true;
      }
      else
      {
        // cout << "rrt not Arrived end_position!" << endl;
        Node_3d *q_nearest_end = findNearestNode(end_position_);

        bestGain_notarrive = q_nearest_end->cost_motion;
        bestNode_notarrive = q_nearest_end;

        // cout << "bestNode_notarrive: " <<
        // bestNode_notarrive->position.transpose() << endl;
      }

      // if(node_gain > bestGain_){
      //   bestGain_ = node_gain;    // 取子节点增益最大的子树
      //   bestNode_ = q_new;
      //   // cout << "bestGain_: " << bestGain_ << endl;
      //   // cout << "bestNode Gain_: " << bestNode_->gain << endl;
      //   arrive_flag = true;
      // }

      rrt3d_sample_position.push_back(q_new->position);
      rrt3d_sample_orientation.push_back(q_new->orientation);
      rrt3d_sample_point_gain.push_back(q_new->cost_motion);
    }
    else
    {
      // i--;
      // std::cout << "i--=" << i << std::endl;
    }
    i++;

    if (i < -5)
    {
      Fail_grow_count++;
      cout << "this rrt can not grow..." << endl;

      if (isInfeatureless(q_new->position, q_new->orientation))
      {
        ROS_ERROR_STREAM("Node in FeatureLess" << q_new->position.transpose());
      }
      if (isNearWall(q_new->position))
      {
        ROS_ERROR_STREAM("Node Near the Wall" << q_new->position.transpose());
      }
      // break;
      return false;
    }
  }

  if(root_->children.size() <= 1){
    ROS_ERROR_STREAM("ALL Node is Invalid...");
    return false;
  }
  cout << "end rrt grow - root_->children: " << root_->children.size() << endl;

  if (arrive_flag)
  {
    Node_3d *q;
    q = bestNode_;
    
    cout << "backtracking in arrive q: " << q->position.transpose() << endl;

    while (q != NULL)
    {
      q->is_adjust_construct = false;
      q->is_adjust_explore = false;
      path_3d.push_back(q);

      // if (Flag_visualize_intersact_points)
      // {
      //   intersection_point_visualization(q);
      // }

      q = q->parent;
    }
  }
  else
  {
    Node_3d *q;
    q = bestNode_notarrive;

    // if(!Line_NoCollision(q->position, end_position_)){
    //   cout << "Not arrived and collision " << endl;  
    //   return false;
    // }

    cout << "backtracking q: " << q->position.transpose() << endl;

    while (q != NULL)
    {
      // cout << "q.position: " << q->position.transpose() << endl;

      q->is_adjust_construct = false;
      q->is_adjust_explore = false;
      path_3d.push_back(q);

      // if (Flag_visualize_intersact_points)
      // {
      //   intersection_point_visualization(q);
      // }

      q = q->parent;
    }
  }

  end_rrtgrow = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end_rrtgrow - start;
  ROS_WARN_STREAM("rrt grow takes " << elapsed_seconds.count()*1000 << "ms");

  cout << "RRT3D - path_3d size: " << path_3d.size() << endl;
  auto start_dm = std::chrono::system_clock::now();
  cout << "g_Frontiers_pos size: " << g_Frontiers3D_pos.size() << endl;
  cout << "g_Frontiers_Cluster_pos size: " << g_Frontiers_Cluster_pos.size()
       << endl;
  
  bool Is_node_adjusted = false;

  if(g_Frontiers_Cluster_pos.empty() && Explore_Done_2d){
    Explore_Done_3d = true;
    ROS_ERROR_STREAM("Explore_Done_3d: " << Explore_Done_3d);
  }
  else{
    // 寻找观测三维边界点的最佳位置
    int _max_sample_iter = 10; // **采样点个数**
    float Adjust_distance_thresh = 5;
    const int _N = 1;
    int _distance_Thresh = 15;
    bool all_frontier3d_unseen = true;    // 若所有的3d边界点均不可见，则结束探索
    for (int i = 0; i < g_Frontiers_Cluster_pos.size(); ++i)
    {
      if ((start_position_ - g_Frontiers_Cluster_pos[i]).norm() >
          _distance_Thresh)
      {
        continue;
      }
      // 在边界点聚类附近采样，取其中可以看到边界点最多的采样点
      float _max_fpnum_InFOV = -1e5;
      Eigen::Vector3d _max_fpnum_sample_pos;
      Eigen::Vector3d _max_fpnum_sample_orinient;
      
      // cout << "g_Frontiers_Cluster_pos[i]: " << g_Frontiers_Cluster_pos[i].transpose() << endl;
      for (int j = 0; j < _max_sample_iter; ++j)
      {
        Eigen::Vector3d _sample_aroundFrontier;
        _sample_aroundFrontier =
            getRandom_aroundFrontier(g_Frontiers_Cluster_pos[i]);

        // cout << "_sample_aroundFrontier: " << _sample_aroundFrontier.transpose() << endl;
        int _unknown_voxel_InFOV = 0;
        int _frontier_num_InFOV =
            fpNumber_InFOV(_sample_aroundFrontier, g_Frontiers_Cluster_pos[i], _unknown_voxel_InFOV);
        // cout << "Exploration3D _frontier_num_InFOV: " << _frontier_num_InFOV << " _unknown_voxel_InFOV: " << _unknown_voxel_InFOV << endl;

        float _frontier_distance = abs(
            (_sample_aroundFrontier.segment(0, 2) -
            g_Frontiers_Cluster_pos[i].segment(0, 2))
                .norm() -
            Adjust_distance_thresh); // 距离Adjust_distance_thresh(5m)越近越好

        // gain_perception()
        // float _sample_gain = _frontier_num_InFOV + exp(-_frontier_distance);
        // float _sample_gain = _frontier_num_InFOV + exp(_unknown_voxel_InFOV/1000);
        float _sample_gain = _unknown_voxel_InFOV;

        if (_sample_gain > _max_fpnum_InFOV)
        {
          _max_fpnum_InFOV = _sample_gain;
          _max_fpnum_sample_pos = _sample_aroundFrontier;
          _max_fpnum_sample_orinient = get_OrinientFromVector(
              _sample_aroundFrontier, g_Frontiers_Cluster_pos[i]);
        }
        // g_BestPositionSet_aroundFrontier.push_back(_sample_aroundFrontier);
        // g_BestOrinientSet_aroundFrontier.push_back(get_OrinientFromVector(
        //       _sample_aroundFrontier, g_Frontiers_Cluster_pos[i]));
        // g_sampleSet_fpNum.push_back(_sample_gain);
      }

      // 如果采样点可见边界点数量都为0，可能会导致随便选取采样点，从而导致结果的不一致性
      if (_max_fpnum_InFOV >= 60)
      // if (_max_fpnum_InFOV >= 2)
      {
        // 在path_3d中选取距离 可看到最多边界点的采样点 最近的节点，并调整
        Node_3d *Adjust_q_nearest =
            findNearestNode_Inpath3d(_max_fpnum_sample_pos);

        // 并且clloision check满足条件
        if (Adjust_q_nearest != NULL &&
            (Adjust_q_nearest->position - _max_fpnum_sample_pos).norm() <
                Adjust_distance_thresh)   // 若最近点与采样点的距离小于5m，则
        {
          // !!! 注释掉就不关注3d重建了
#if 1
          Adjust_q_nearest->position = _max_fpnum_sample_pos;
          Adjust_q_nearest->orientation = _max_fpnum_sample_orinient;
          Adjust_q_nearest->is_adjust_construct = true;
          Is_node_adjusted = true;

          g_BestPositionSet_aroundFrontier.push_back(_max_fpnum_sample_pos);
          g_BestOrinientSet_aroundFrontier.push_back(_max_fpnum_sample_orinient);
          g_sampleSet_fpNum.push_back(_max_fpnum_InFOV);
#endif
        }

        all_frontier3d_unseen = false;
      }
      else
      {
        ROS_WARN_STREAM(
            "Not Find Best frontier observer - All see 0 frontier...");
      }
    }
    cout << "Adjust node towards Frontier3d Done......" << endl;
    if(all_frontier3d_unseen && Explore_Done_2d){
      Explore_Done_3d = true;
      ROS_ERROR_STREAM("Explore_Done_3d: " << Explore_Done_3d);
    }
  }
  
  auto end_dm = std::chrono::system_clock::now();
  auto elapsed_seconds_dm =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_dm - start_dm);
  ROS_WARN_STREAM("Find Best 3Dfrontier observer takes: "
                  << elapsed_seconds_dm.count() << " ms");

  // 调整不满足gain_perception（FOV中是否有足够的信息丰富特征-语义/特征分布）

  // !! 这里改为对construction节点附近进行插值，使节点与节点之间的距离相等，
  // ->可以把折线路径优化成直线路径
  if (Is_node_adjusted)
    Insert_path3d_node();
  cout << "Interpolation node Done......" << endl;

  // Find_Exploration2d_node(); // !!!可以对所有节点都进行一次theta_best的选取
  Find_Frontier2d_node();   // 只改变节点的orientation

  Smooth_node_path(); // rrt折线路径变为直线路径
  end_planning = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds_planning = end_planning - start;
  ROS_WARN_STREAM("rrt planning takes " << elapsed_seconds_planning.count()*1000 << "ms");

#if def_w_uncertainty
  AdjustNode_for_perception();
#endif

  Extract_Subtree_First_maxfp();

  return true;
}

const int track_path_length = 5;
void rrt_3d::Update_Track_path()
{
  cout << "Update_Track_path - path_3d.size: " << path_3d.size() << endl;
  track_path_3d.clear();
  if (path_3d.size() < track_path_length)
  {
    for (int i = path_3d.size() - 1; i >= 0; --i)
    {
      Eigen::VectorXd _node(6);
      _node << path_3d[i]->position, path_3d[i]->orientation;

      track_path_3d.push_back(_node);
    }
    cout << "Update_Track_path - track_path_3d.size: " << track_path_3d.size()
         << endl;
  }
  else
  {
    for (int i = path_3d.size() - 1; i >= path_3d.size() - track_path_length;
         --i)
    {
      Eigen::VectorXd _node(6);
      _node << path_3d[i]->position, path_3d[i]->orientation;

      track_path_3d.push_back(_node);
    }
    cout << "Update_Track_path - track_path_3d.size: " << track_path_3d.size()
         << endl;
  }
}

// 对整个path_3d进行插值调整，插入节点，使节点之间的距离相近
void rrt_3d::Insert_path3d_node()
{
  // 增加节点
  for (int i = 0; i < path_3d.size() - 1; ++i)
  {
    // cout << "path_3d " << i << " : " << path_3d[i]->position.transpose() << "
    // " << path_3d[i]->orientation.transpose() << endl; cout << "path_3d " <<
    // i+1 << " : " << path_3d[i+1]->position.transpose() << " " <<
    // path_3d[i+1]->orientation.transpose() << endl;

    float _adjacent_dis =
        (path_3d[i + 1]->position - path_3d[i]->position).norm();
    // cout << "_adjacent_dis: " << _adjacent_dis << endl;

    if (_adjacent_dis > step_size_position)
    {
      int _insert_num = floor(_adjacent_dis / step_size_position);
      // cout << "_insert_num: " << _insert_num << endl;

      Node_3d q_i1 = *path_3d[i + 1];
      Node_3d q_i = *path_3d[i];
      int _insert_idx = i + 1;
      for (int j = 0; j < _insert_num; ++j)
      {
        // cout << "j " << j << endl;
        Node_3d *_q_insert = new Node_3d;
        _q_insert->is_adjust_construct = false;
        _q_insert->is_adjust_explore = false;
        // _q_insert = path_3d[i];

        double _step_scale = ((j + 1) * 1.0 / (_insert_num + 1));
        Eigen::Vector3d _position_step =
            (q_i1.position - q_i.position) * _step_scale;
        // cout << "_position_step: " << _position_step.transpose() << endl;

        _q_insert->position = q_i.position + _position_step;
        // cout << "_q_insert->position: " << _q_insert->position.transpose() <<
        // endl;

        Eigen::Vector3d _orientation_step =
            (q_i1.orientation - q_i.orientation) * _step_scale;
        _q_insert->orientation = q_i.orientation + _orientation_step;
        // cout << "_q_insert->orientation: " <<
        // _q_insert->orientation.transpose() << endl; cout <<
        // "_orientation_step: " << _orientation_step.transpose() << endl;

        path_3d.insert(path_3d.begin() + _insert_idx + j, _q_insert);
      }
      i += _insert_num;
    }
  }
}

// 对整个path_3d进行插值调整，插入节点，使节点之间的距离相近
void rrt_3d::Insert_node_withorinient()
{
  // 对节点的orientation进行转换
  for (int i = path_3d.size() - 1; i > 0; --i)
  {

    // cout << "yaw j: " << path_3d[i]->orientation.z()
    //      << " yaw j-1:" << path_3d[i - 1]->orientation.z() << endl;
    if (abs(path_3d[i]->orientation.z() - path_3d[i - 1]->orientation.z()) >
        M_PI)
    {
      path_3d[i - 1]->orientation.z() =
          (path_3d[i]->orientation.z() > 0)
              ? (M_PI + M_PI + path_3d[i - 1]->orientation.z())
              : (-M_PI - (M_PI - path_3d[i - 1]->orientation.z()));
      
      // cout << "After yaw j: " << path_3d[i]->orientation.z()
      //      << " yaw j+1: " << path_3d[i - 1]->orientation.z() << endl;
    }

  }

  // 增加节点
  for (int i = 0; i < path_3d.size() - 1; ++i)
  {
    // cout << "path_3d " << i << " : " << path_3d[i]->position.transpose() << " "
    //      << path_3d[i]->orientation.transpose() << endl;
    // cout << "path_3d " << i + 1 << " : " << path_3d[i + 1]->position.transpose()
    //      << " " << path_3d[i + 1]->orientation.transpose() << endl;

    float _adjacent_dis =
        (path_3d[i + 1]->position - path_3d[i]->position).norm();
    // cout << "_adjacent_dis: " << _adjacent_dis << endl;

    float _adjacent_rot =
        (path_3d[i + 1]->orientation - path_3d[i]->orientation).norm();
    // cout << "_adjacent_dis: " << _adjacent_dis << endl;

    if (_adjacent_dis > step_size_position ||
        _adjacent_rot > step_size_orientation)
    {
      int _insert_num = max(floor(_adjacent_dis / step_size_position),
                            floor(_adjacent_rot / (step_size_orientation * 3)));
      // cout << "_insert_num: " << _insert_num << endl;

      Node_3d q_i1 = *path_3d[i + 1];
      Node_3d q_i = *path_3d[i];

      int _insert_idx = i + 1;
      for (int j = 0; j < _insert_num; ++j)
      {
        // cout << "j " << j << endl;
        Node_3d *_q_insert = new Node_3d;
        _q_insert->is_adjust_construct = false;
        _q_insert->is_adjust_explore = false;
        // _q_insert = path_3d[i];

        double _step_scale = ((j + 1) * 1.0 / (_insert_num + 1));
        Eigen::Vector3d _position_step =
            (q_i1.position - q_i.position) * _step_scale;
        // cout << "_position_step: " << _position_step.transpose() << endl;

        _q_insert->position = q_i.position + _position_step;
        // cout << "_q_insert->position: " << _q_insert->position.transpose() <<
        // endl;

        Eigen::Vector3d _orientation_step =
            (q_i1.orientation - q_i.orientation) * _step_scale;
        _q_insert->orientation = q_i.orientation + _orientation_step;
        
        _q_insert->orientation.y() = 0;

        // cout << "_q_insert->orientation: " << _q_insert->orientation.transpose() << endl;
        // cout << "_orientation_step: " << _orientation_step.transpose() <<
        // endl;

        path_3d.insert(path_3d.begin() + _insert_idx + j, _q_insert);
      }
      i += _insert_num;
    }
  }
}

rrt_3d::Node_3d *rrt_3d::findNearestNode_Inpath3d(
    Eigen::Vector3d current_position)
{
  double min_distance = 1e5;
  Node_3d *closest_node = NULL;
  for (int i = path_3d.size() - 2; i > 0; --i)
  {
    double distance = (current_position - path_3d[i]->position).norm();
    if (distance < min_distance)
    {
      min_distance = distance;
      closest_node = path_3d[i];
    }
  }

  // ROS_INFO("closest_node: %f %f %f", closest_node->position.x(),
  // closest_node->position.y(), closest_node->position.z());
  return closest_node;
}

void rrt_3d::Find_Exploration2d_node()
{
  // int _distance_Thresh = 15;

  // for (int i = 0; i < g_Frontier2D_Cluster_pos.size(); ++i)
  // {
  //   if ((start_position_ - ConvertGridIndex2World(g_Frontier2D_Cluster_pos[i])).norm() >
  //         _distance_Thresh)
  //   {
  //     continue;
  //   }


  // } 
}

// 遍历range内边界点的fsmi_data，取data最小的点作为目标点；如果该边界点的data少于200，说明处于外边界，则直接以该边界点的find_best_theta作为最佳朝向；如果data>200，说明是内边界且处于附近，则以该点与当前位置的朝向作为最优视角。
// 为了解决内边界点朝向内部，且机器人也位于该方向延长线上，而导致无法探索该区域的情形。
void rrt_3d::Find_Frontier2d_node()
{
  vector<Node_3d *> _important_node;
  vector<int> _important_node_idx;
  for (int i = path_3d.size() - 1; i >= 0; --i)
  { // || (path_3d[i]->is_adjust_construct)
    if (((i == path_3d.size() - 1) || (i == (int)(1 * path_3d.size() / 2))))
    {
      _important_node.push_back(path_3d[i]);
      _important_node_idx.push_back(i);
      cout << "Find_Exploration2d_node - i: " << i
           << " _important_node:" << path_3d[i]->position.transpose() << endl;
    }
  }

  float _min_node_info = 1.5e3;
  int _min_info_idx = 0;
  Eigen::Vector3d _best_vector(0, 0, theta_best);
  Eigen::Vector3d _adjust_end_pos;
  int j = 0;
  float _query_range = 5.0;
  // float _treshold_to_explore = ((M_PI * _query_range * _query_range) * 0.708/(_gridmap.info.resolution * _gridmap.info.resolution)) - 4 * 20;   // 20个栅格未知

  for (int i = path_3d.size() - 1; i >= 0; --i)
  {
    bool _Frontier_in_range = false;

    int _between_node_num =
        abs(_important_node_idx[j + 1] - _important_node_idx[j]);
    if (i < _important_node_idx[j] && i > _important_node_idx[j + 1])   // important node不进行调整
    {
      Eigen::Vector2d _node_pos(path_3d[i]->position.x(), path_3d[i]->position.y());
      Eigen::Vector3d _node_pos3d(path_3d[i]->position.x(), path_3d[i]->position.y(), start_position_.z());
      // cout << "Find_Exploration2d_node - query pos: " << _node_pos.transpose() << endl;
      Eigen::Vector2i _node_pos_grid = ConvertWorld2GridIndex(_node_pos);

      double fsmi_data_node = compute_fsmi_point(_node_pos_grid.x(), _node_pos_grid.y(), g_param_3dfsmi_range, true);

      if(fsmi_data_node < g_param_thresh_toexplore && fsmi_data_node <= _min_node_info){

        // _best_vector.z() = find_best_theta(frontier2d_vector[k].x(), frontier2d_vector[k].y(), _node_pos, 5.0);
        _best_vector.z() = find_best_theta(_node_pos_grid.x(), _node_pos_grid.y(),  g_param_3dfsmi_range, 0, true);
        // ROS_ERROR_STREAM("BEST THETA TO Adjust from find: " << _best_vector.z());    

        // if(abs(_best_vector.z() - end_orientation_.z()) < 0.2){
        //   ROS_WARN_STREAM("Adjust goal direction is near to goal");
        //   continue;
        // }

        Eigen::Vector3d _unit_vec(1,0,0);
        // cout << "theta_best_mi: " << _best_vector.z() << endl;
        Eigen::AngleAxisd angle_axis1(-_best_vector.z(), Eigen::Vector3d(0, 0, 1));
        Eigen::Vector3d _direction = angle_axis1.matrix().inverse()*_unit_vec;
        // cout << "(1, 0, 0) theta_best:" << endl << _direction.transpose() << endl;

        _direction = _direction.normalized();
        float retreat_step = 0.0;
        Eigen::Vector3d _query_end_pos = path_3d[i]->position;
        _adjust_end_pos = path_3d[i]->position;
        
        // && !isInfeatureless(_query_end_pos)
        while((isInKnown_Feature_Area(_query_end_pos)) && IsInLimitArea(_query_end_pos.segment(0,2)) && retreat_step < 5){  // 往后退5m
          retreat_step += 0.2;
          _query_end_pos = path_3d[i]->position + retreat_step * _direction;
          ROS_ERROR_STREAM("rrt3d detecting Goal......." << _query_end_pos.transpose());
        }

        if(retreat_step < 2.5){
          float _adjust_dis = 0.0;
          while(!isInfeatureless(_adjust_end_pos) && _adjust_dis < (2.5 - retreat_step)){
            _adjust_dis += 0.2;
            _adjust_end_pos = path_3d[i]->position - _adjust_dis * _direction;
            ROS_ERROR_STREAM("rrt3d <2 Adjusting Goal......." << _adjust_end_pos.transpose());
          }
        }
        else if(retreat_step <= 5.2 && (start_position_ - g_origin_).norm() > 1){
          float _adjust_dis = 0.0;
          while(!isInfeatureless(_adjust_end_pos) && _adjust_dis < (retreat_step - 2.5)){
            _adjust_dis += 0.2;
            
            _adjust_end_pos = path_3d[i]->position + _adjust_dis * _direction;
            ROS_ERROR_STREAM("rrt3d <5.2 Adjusting Goal......." << _adjust_end_pos.transpose());
          }
        }
        // ROS_ERROR_STREAM("rrt3d Adjusted Goal:" << _adjust_end_pos.transpose());

        _min_node_info = fsmi_data_node;
        _min_info_idx = i;

        i = _important_node_idx[j + 1];
      }
    }
    
    if (i <= _important_node_idx[j + 1])
    {
      if (_min_node_info < g_param_thresh_toexplore)   // 只要其中有一个边界点，_min_node_info就会小于1e5
      { // 检测到<200的点，即
        path_3d[_min_info_idx]->is_adjust_explore = true;
        path_3d[_min_info_idx]->orientation = _best_vector;
        path_3d[_min_info_idx]->position = _adjust_end_pos;

        g_BestPositionSet_aroundFrontier.push_back(
            path_3d[_min_info_idx]->position);
        g_BestOrinientSet_aroundFrontier.push_back(
            path_3d[_min_info_idx]->orientation);
        g_sampleSet_fpNum.push_back(_min_node_info);
        // g_sampleSet_fpNum.push_back(gain_perception(path_3d[_min_info_idx]->position, path_3d[_min_info_idx]->orientation));

      }
      _min_node_info = 1e5;

      j++;
    }

    if (j >= _important_node_idx.size() - 1)
    {
      break;
    }
  }
}

void rrt_3d::Smooth_node_path()
{
  // 平滑节点的视角变化
  vector<Node_3d *> _important_node;
  vector<int> _important_node_idx;
  for (int i = 0; i < path_3d.size(); ++i)
  {
    if ((i == 0) || (i == path_3d.size() - 1) ||
        (path_3d[i]->is_adjust_construct) || (path_3d[i]->is_adjust_explore))
    {
      _important_node.push_back(path_3d[i]);
      _important_node_idx.push_back(i);
      cout << "Smooth_node_path - i: " << i
           << " _important_node:" << path_3d[i]->position.transpose() << endl;
    }
  }

  // !! 增加直线连接
  vector<int> _erase_idx;
  int j = 0;
  int k = 0;
  Node_3d *_start_query_node = path_3d[_important_node_idx[j]];
  Node_3d *_end_query_node = path_3d[_important_node_idx[j + 1]];
  int _start_query_idx = _important_node_idx[j];
  int _end_query_idx = _important_node_idx[j + 1];
  for (; _start_query_idx < path_3d.size() - 1;)
  {
    _end_query_node = path_3d[_end_query_idx];
    _start_query_node = path_3d[_start_query_idx];

    if (Line_NoCollision(_start_query_node->position,
                         _end_query_node->position))
    {
      // assert(_end_query_idx >= _start_query_idx + 2);
      if (_end_query_idx > _start_query_idx + 2)
      { // 中间间隔1个点以上
        _erase_idx.push_back(_start_query_idx + 1);
        _erase_idx.push_back(_end_query_idx);
      }
      else if (_end_query_idx == _start_query_idx + 2)
      {
        _erase_idx.push_back(_start_query_idx + 1);
        _erase_idx.push_back(_start_query_idx + 1);
      }
      else
      {
        if (_end_query_idx <= _start_query_idx + 1)
        { // 起点终点重合了
          _start_query_idx = _end_query_idx;

          if (_end_query_idx == _important_node_idx[j + 1] &&
              j + 2 < _important_node_idx.size())
          {
            _end_query_idx = _important_node_idx[j + 2];
            j++;
          }
          else if (_end_query_idx == _important_node_idx[j + 1] &&
                   j + 2 >= _important_node_idx.size())
          {
            break;
          }
          else
          {
            _end_query_idx = _important_node_idx[j + 1];
          }
        }
        continue;
      }

      if (_end_query_idx == _important_node_idx[j + 1])
      {
        _start_query_idx = _important_node_idx[j + 1];
        _end_query_idx = _important_node_idx[j + 2];
        j++;
      }
      else
      {
        _start_query_idx = _end_query_idx;
        _end_query_idx = _important_node_idx[j + 1];
      }
    }
    else
    {
      if (_end_query_idx <= _start_query_idx + 1)
      { // 起点终点重合了
        _start_query_idx = _end_query_idx;
        if (_end_query_idx == _important_node_idx[j + 1] &&
            j + 2 < _important_node_idx.size())
        {
          _end_query_idx = _important_node_idx[j + 2];
          j++;
        }
        else if (_end_query_idx == _important_node_idx[j + 1] &&
                 j + 2 >= _important_node_idx.size())
        {
          break;
        }
        else
        {
          _end_query_idx = _important_node_idx[j + 1];
        }
      }
      else
      {
        _end_query_idx--;
      }
    }

    if (_start_query_idx == path_3d.size() - 2 &&
        _end_query_idx == path_3d.size() - 1)
    {
      break;
    }
  }

  for (int i = _erase_idx.size() - 1; i >= 0; i -= 2)
  {
    if (_erase_idx[i - 1] == _erase_idx[i])
    {
      path_3d.erase(path_3d.begin() + _erase_idx[i - 1]);
    }
    else
    {
      path_3d.erase(path_3d.begin() + _erase_idx[i - 1],
                    path_3d.begin() + _erase_idx[i]);
    }
  }

  // 插入间隔点
  Insert_node_withorinient();
}

bool rrt_3d::Line_NoCollision(Eigen::Vector3d _start, Eigen::Vector3d _end)
{
  Eigen::Vector3d _direction = _end - _start;
  _direction = _direction.normalized();

  float query_size_ = step_size_position * 0.25;   // 步长
  int _query_step = (_end - _start).norm() / query_size_;   // 步数

  for (int i = 1; i <= _query_step; ++i)
  {
    Eigen::Vector3d _query_pos = _start + _direction * query_size_ * i;
    if (isNearWall(_query_pos) ||
        isInfeatureless(
            _query_pos))
    { // !isInKnown_Feature_Area(_query_pos) ||
      // cout << "Line Collision - " << endl;
      return false;
    }
  }
  return true;
}

void rrt_3d::AdjustNode_for_perception(){

  // cout << "path node: " << path_3d[path_3d.size() - 1]->position.transpose() << endl;
  // ROS_ERROR_STREAM("Node perception gain:" << gain_perception(path_3d[path_3d.size() - 1]->position, path_3d[path_3d.size() - 1]->orientation));

  for(int i = 0; i < path_3d.size(); ++i){
    // cout << "path node " << i << " : " << path_3d[i]->position.transpose() << endl;

    bool perception_insert = false;

    if(isViewInfeatureless(path_3d[i]->position, path_3d[i]->orientation)){
      // ROS_ERROR_STREAM("Node perception gain:" << gain_perception(path_3d[i]->position, path_3d[i]->orientation));

      Eigen::Vector3d _node_new_pos;
      float retreat_step = 0.4;

      while(isViewInfeatureless(path_3d[i]->position, path_3d[i]->orientation))
      {
        float _delta_x = cos(path_3d[i]->orientation[2]) * retreat_step;
        float _delta_y = sin(path_3d[i]->orientation[2]) * retreat_step;

        path_3d[i]->position.x() -= _delta_x;
        path_3d[i]->position.y() -= _delta_y;
        
      }
      perception_insert = true;
    }
    
    if(perception_insert){
      int j = 0;
      // if(i != path_3d.size() - 1){
      //   Node_3d *_q_insert = new Node_3d;
      //   _q_insert->is_adjust_construct = false;
      //   _q_insert->is_adjust_explore = false;
      //   // _q_insert = path_3d[i];

      //   // cout << "_position_step: " << _position_step.transpose() << endl;
      //   _q_insert->position = (path_3d[i]->position + path_3d[i+1]->position)*0.5;
      //   // cout << "_q_insert->position: " << _q_insert->position.transpose() <<
      //   // endl;

      //   _q_insert->orientation = (path_3d[i]->orientation + path_3d[i+1]->orientation)*0.5;
      //   // cout << "_q_insert->orientation: " << _q_insert->orientation.transpose()
      //   //       << endl;

      //   path_3d.insert(path_3d.begin() + i + 1, _q_insert);
      //   j++;
      // }

      if(i != 0){
        Node_3d *_q_insert = new Node_3d;
        _q_insert->is_adjust_construct = false;
        _q_insert->is_adjust_explore = false;
        // _q_insert = path_3d[i];

        // cout << "_position_step: " << _position_step.transpose() << endl;
        _q_insert->position = (path_3d[i]->position + path_3d[i-1]->position)*0.5;
        // cout << "_q_insert->position: " << _q_insert->position.transpose() <<
        // endl;

        _q_insert->orientation = (path_3d[i]->orientation + path_3d[i-1]->orientation)*0.5;
        // cout << "_q_insert->orientation: " << _q_insert->orientation.transpose()
        //       << endl;

        path_3d.insert(path_3d.begin() + i, _q_insert);
        j++;
      }
      i+=j;
    }
  }
}

int m_min_node = 4;
void rrt_3d::Extract_Subtree_First_maxfp()
{
  cout << "Extract_Subtree_First_maxfp - path_3d.size: " << path_3d.size()
       << endl;
  track_path_3d.clear();

  for (int i = path_3d.size() - 1; i >= 0; --i)
  {
    
    Eigen::VectorXd _node(6);
    _node << path_3d[i]->position, path_3d[i]->orientation;

    track_path_3d.push_back(_node);

    if ((path_3d[i]->is_adjust_construct || path_3d[i]->is_adjust_explore))
    {
      EnsureTrackingAcc = true;
      break;
    }
  }

  if(track_path_3d.size() > m_min_node + 4){
    track_path_3d.erase(track_path_3d.begin() + m_min_node, track_path_3d.begin() + track_path_3d.size());

    // Eigen::Vector2d _node_pos(track_path_3d.back().x(), track_path_3d.back().y());

    // Eigen::Vector2i _node_pos_grid = ConvertWorld2GridIndex(_node_pos);

    // track_path_3d.back()[5] = find_best_theta(_node_pos_grid.x(), _node_pos_grid.y(),  g_param_3dfsmi_range);

    EnsureTrackingAcc = false;   // 为false则为实时单步规划，会产生震荡;但对环境的反应更快
    // EnsureTrackingAcc = true;
  }
  cout << "Update_Track_path - track_path_3d.size: " << track_path_3d.size()
       << endl;
}

// 用于main中，2D探索结束后，选取3D边界点作为目标
Eigen::VectorXd rrt_3d::Extract_BestObserve_Frontier3d()
{
  int _Adjust_distance_thresh = 5;
  int _max_sample_iter = 20;
  float _max_sample_gain = -1e5;
  Eigen::Vector3d _max_fpnum_sample_pos;
  Eigen::Vector3d _max_fpnum_sample_orinient;
  Eigen::Vector3d _nearest_frontiercluster;
  float _min_distance = 1e5;

  cout << "Extract_BestObserve_Frontier3d - g_Frontiers_Cluster_pos.size: " << g_Frontiers_Cluster_pos.size() << endl;
  if (g_Frontiers_Cluster_pos.empty())
  {
    Eigen::VectorXd ret_target_pose(6);
    ret_target_pose.segment(0, 3) = start_position_;
    ret_target_pose.segment(3, 3) = start_orientation_;
    Explore_Done_3d = true;

    return ret_target_pose;
  }

  for (int i = 0; i < g_Frontiers_Cluster_pos.size(); ++i)
  {
    if ((g_Frontiers_Cluster_pos[i] - start_position_).norm() < _min_distance)
    {
      _nearest_frontiercluster = g_Frontiers_Cluster_pos[i];
    }
  }

  for (int j = 0; j < _max_sample_iter; ++j)
  {
    Eigen::Vector3d _sample_aroundFrontier;
    _sample_aroundFrontier = getRandom_aroundFrontier(_nearest_frontiercluster);

    int _unknown_voxel_InFOV = 0;
    int _frontier_num_InFOV =
        fpNumber_InFOV(_sample_aroundFrontier, _nearest_frontiercluster, _unknown_voxel_InFOV);
    // cout << "Extract_BestObserve_Frontier3d - _frontier_num_InFOV: " << _frontier_num_InFOV << " _unknown_voxel_InFOV: " << _unknown_voxel_InFOV << endl;

    float _frontier_distance =
        abs((_sample_aroundFrontier.segment(0, 2) -
             _nearest_frontiercluster.segment(0, 2))
                .norm() -
            _Adjust_distance_thresh); // 距离Adjust_distance_thresh(5m)越近越好

    // float _sample_gain = _frontier_num_InFOV + exp(-_frontier_distance);
    float _sample_gain = _unknown_voxel_InFOV;    
    // cout << "Extract_BestObserve_Frontier3d - _sample_gain: " << _sample_gain << endl;

    if (_sample_gain > _max_sample_gain)
    {
      _max_sample_gain = _sample_gain;
      _max_fpnum_sample_pos = _sample_aroundFrontier;
      _max_fpnum_sample_orinient = get_OrinientFromVector(
          _sample_aroundFrontier, _nearest_frontiercluster);
    }
  }

  if (_max_sample_gain < 60)
  {
    Eigen::VectorXd ret_target_pose(6);
    ret_target_pose.segment(0, 3) = start_position_;
    ret_target_pose.segment(3, 3) = start_orientation_;
    Explore_Done_3d = true;

    // cout << "Extract_BestObserve_Frontier3d - _max_sample_gain < 60" << endl;
    return ret_target_pose;
  }

  // cout << "Extract_BestObserve_Frontier3d - max_sample_gain: " << _max_sample_gain << endl;

  Eigen::VectorXd ret_target_pose(6);
  ret_target_pose.segment(0, 3) = _max_fpnum_sample_pos;
  ret_target_pose.segment(3, 3) = _max_fpnum_sample_orinient;

  return ret_target_pose;
}
