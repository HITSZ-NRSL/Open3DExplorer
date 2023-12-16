#include <nav_msgs/OccupancyGrid.h>
#include <octomap/OcTreeKey.h>
#include <octomap/octomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <thread>
#include <vector>

#include "../include/ClusteringAlgorithm.h"
#include "KDTree.hpp"
#include "fsmi_cal.h"
#include "gaussian_process_regression/gaussian_process_regression.h"
#include <nodehash.h>

using namespace octomap;

// #include "../include/frontier_detection_3d.h"
ros::Publisher m_markerFrontierPub, m_markerClusteredFrontierPub;
ros::Publisher m_markerOccPub, m_markerFreePub, m_binaryMapPub;
// 待探索区域的长宽
const int LimitArea_X = 200;
const int LimitArea_Y = 200;

const int Featureless_griddata = -100; // NVBP: 100 PAE: -100
const int Featurerich_griddata = 100;

const int N_S = 8; // 8邻域

octomap::OcTreeKey _paddedMinKey;
double _occupancyMinZ;
double _occupancyMaxZ;

int map_extend_length = 10;
// const float max_predict_distance_ = 40;
const float max_predict_distance_ = 15;
const int project_2d_map_depth = 15;

bool IsInLimitarea(Eigen::Vector2i frontier_point_idx)
{
  // return true;
  Eigen::Vector2d fp_pos = ConvertGridIndex2World(frontier_point_idx);

  // if (fp_pos.x() < -LimitArea_X / 2 || fp_pos.x() > LimitArea_X / 2 ||
  //     fp_pos.y() < -LimitArea_Y / 2 || fp_pos.y() > LimitArea_Y / 2)
  // {
  //   // cout << "Is out of Limit area!!!" << endl;
  //   return false;
  // }
  // else
  // {
  //   // cout << "In Limit area......" << endl;
  //   return true;
  // }
  if (fp_pos.x() < explore_range_blx + _gridmap.info.resolution || fp_pos.x() > explore_range_urx - _gridmap.info.resolution ||
      fp_pos.y() > explore_range_bly - _gridmap.info.resolution || fp_pos.y() < explore_range_ury + _gridmap.info.resolution)
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

int mapIdx(int i, int j) { return _gridmap.info.width * j + i; }

int mapIdx(const octomap::OcTreeKey &key)
{
  // return mapIdx(key[0] - _paddedMinKey[0] + 1, key[1] - _paddedMinKey[1] + 1);
  return mapIdx(key[0] - _paddedMinKey[0], key[1] - _paddedMinKey[1]);
}

void handle_pre_simple(const std_msgs::Header &header)
{
  // init projected 2D map:
  _gridmap.header.frame_id = header.frame_id;
  _gridmap.header.stamp = header.stamp;
  
  _gridmap.info.width = int(100*2/_octree->getNodeSize(project_2d_map_depth));
  _gridmap.info.height = int(100*2/_octree->getNodeSize(project_2d_map_depth));
  ROS_WARN_STREAM("_gridmap.info.width: " << _gridmap.info.width
                                          << " _gridmap.info.height: "
                                          << _gridmap.info.height);

  _gridmap.info.resolution = _octree->getNodeSize(project_2d_map_depth);
  std::cout << "map.resolution: " << _gridmap.info.resolution << std::endl;

  _gridmap.info.origin.position.x = -100;
  _gridmap.info.origin.position.y = -100;
  std::cout << "map origin: " << _gridmap.info.origin.position << std::endl;

  _gridmap.data.resize(_gridmap.info.width * _gridmap.info.height, 0);
}

void handle_pre(const std_msgs::Header &header)
{
  // init projected 2D map:
  _gridmap.header.frame_id = header.frame_id;
  _gridmap.header.stamp = header.stamp;

  // TODO: move most of this stuff into c'tor and init map only once (adjust if
  // size changes)
  double minX, minY, minZ, maxX, maxY, maxZ;
  _octree->getMetricMin(minX, minY, minZ);
  _octree->getMetricMax(maxX, maxY, maxZ);

  // minX -= (map_extend_length+1);
  // maxX += (map_extend_length+1);
  // minY -= (map_extend_length+1);
  // maxY += (map_extend_length+1);

  minX = -100;
  maxX = 100;
  minY = -100;
  maxY = 100;

  octomap::point3d minPt(minX, minY, minZ);
  octomap::point3d maxPt(maxX, maxY, maxZ);
  octomap::OcTreeKey minKey = _octree->coordToKey(minPt, project_2d_map_depth);
  octomap::OcTreeKey maxKey = _octree->coordToKey(maxPt, project_2d_map_depth);

  ROS_WARN_STREAM("MinKey: " << minKey[0] << " " << minKey[1] << " "  << minKey[2] << " MaxKey: " << maxKey[0] << " "  << maxKey[1] << " "  << maxKey[2]);

  // add padding if requested (= new min/maxPts in x&y):
  // default: m_minSizeX(0.0), m_minSizeY(0.0)
  minX = std::min(minX, 0.0);
  maxX = std::max(maxX, 0.0);
  minY = std::min(minY, 0.0);
  maxY = std::max(maxY, 0.0);
  minPt = octomap::point3d(minX, minY, minZ);
  maxPt = octomap::point3d(maxX, maxY, maxZ);

  octomap::OcTreeKey paddedMaxKey;
  if (!_octree->coordToKeyChecked(minPt, project_2d_map_depth, _paddedMinKey))
  {
    ROS_ERROR("Could not create padded min OcTree key at %f %f %f", minPt.x(),
              minPt.y(), minPt.z());
    return;
  }
  if (!_octree->coordToKeyChecked(maxPt, project_2d_map_depth, paddedMaxKey))
  {
    ROS_ERROR("Could not create padded max OcTree key at %f %f %f", maxPt.x(),
              maxPt.y(), maxPt.z());
    return;
  }

  ROS_WARN_STREAM("Padded MinKey: "
                  << _paddedMinKey[0] << " " << _paddedMinKey[1] << " "
                  << _paddedMinKey[2] << " padded MaxKey: " << paddedMaxKey[0]
                  << " " << paddedMaxKey[1] << " " << paddedMaxKey[2]);
  assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);

  _gridmap.info.width = paddedMaxKey[0] - _paddedMinKey[0];
  _gridmap.info.height = paddedMaxKey[1] - _paddedMinKey[1];

  int mapOriginX = minKey[0] - _paddedMinKey[0];
  int mapOriginY = minKey[1] - _paddedMinKey[1];
  assert(mapOriginX >= 0 && mapOriginY >= 0);

  // might not exactly be min / max of octree:
  octomap::point3d origin = _octree->keyToCoord(_paddedMinKey, project_2d_map_depth);
  std::cout << "map origin " << origin << std::endl;
  double gridRes = _octree->getNodeSize(project_2d_map_depth);
  std::cout << "gridRes " << gridRes << std::endl;

  _gridmap.info.resolution = gridRes;
  _gridmap.info.origin.position.x = origin.x() - gridRes * 0.5;
  _gridmap.info.origin.position.y = origin.y() - gridRes * 0.5;
  // ROS_WARN_STREAM("Rebuilding complete 2D map");
  std::cout << "map origin after " << _gridmap.info.origin.position << std::endl;

  // _gridmap.data.clear();
  _gridmap.data.resize(_gridmap.info.width * _gridmap.info.height, 0);
  ROS_WARN_STREAM("_gridmap.info.width: " << _gridmap.info.width
                                          << " _gridmap.info.height: "
                                          << _gridmap.info.height);
}

bool isFeatureRich_project(const uint8_t r, const uint8_t g, const uint8_t b)
{
  if ((r <= (2 + 2)) && (r >= (2 - 2)) && (g <= (2 + 2)) && (g >= (2 - 2)) &&
      (b <= (253 + 2)) && (b >= (253 - 2)))
  {
    // cout << "isFeatureLess - rgb: " << int(r) << " " << int(g) << " " <<
    // int(b) << endl;
    return false;
  }
  else
  {
    // cout << "isFeatureRich - rgb: " << int(r) << " " << int(g) << " " <<
    // int(b) << endl;
    return true;
  }
}

bool isFeatureRich_AdaptiveSemantic(const uint8_t r, const uint8_t g, const uint8_t b)
{
  Eigen::Vector3i _query_color(r,g,b);
  if (ColorIsFeatureRich(_query_color))
  {
    // cout << "isFeatureLess - rgb: " << int(r) << " " << int(g) << " " <<
    // int(b) << endl;
    return true;
  }
  else if(ColorIsFeatureLess(_query_color)){
    // cout << "isFeatureLess - rgb: " << int(r) << " " << int(g) << " " <<
    // int(b) << endl;
    return false;
  }
  else
  {
    if(Color_density_map[Node_hash(r, g, b)] >= 20){
      // cout << "isFeatureRich - rgb: " << int(r) << " " << int(g) << " " <<
      // int(b) << endl;
      return true;  
    }
    else{
      return false;
    }
  }
}

void get_neighbours(Eigen::Vector2d n_array[], Eigen::Vector2d position)
{
  n_array[0].x() = position.x() - _gridmap.info.resolution;
  n_array[0].y() = position.y() - _gridmap.info.resolution;

  n_array[1].x() = position.x() - _gridmap.info.resolution;
  n_array[1].y() = position.y();

  n_array[2].x() = position.x() - _gridmap.info.resolution;
  n_array[2].y() = position.y() + _gridmap.info.resolution;

  n_array[3].x() = position.x();
  n_array[3].y() = position.y() - _gridmap.info.resolution;

  n_array[4].x() = position.x();
  n_array[4].y() = position.y() + _gridmap.info.resolution;

  n_array[5].x() = position.x() + _gridmap.info.resolution;
  n_array[5].y() = position.y() - _gridmap.info.resolution;

  n_array[6].x() = position.x() + _gridmap.info.resolution;
  n_array[6].y() = position.y();

  n_array[7].x() = position.x() + _gridmap.info.resolution;
  n_array[7].y() = position.y() + _gridmap.info.resolution;
}

bool isNearWall(double x, double y)
{
  // cout << "isNearWall position x_delta: " << abs(point_position.x() -
  // start_position_.x()) << endl; cout << "isNearWall position y_delta: " <<
  // abs(point_position.y() - start_position_.y()) << endl; 边界

  Eigen::Vector2d node_position(x, y);

  Eigen::Vector2d _neighbor_grid[N_S];
  get_neighbours(_neighbor_grid, node_position);

  double _query_center_height = get_mapz(node_position);
  // cout << "isNearWall _query_center_height: " << node_position.transpose() << " " << _query_center_height << endl;
  if(_query_center_height <= -3){
    return false;
  }

  int found = 0;
  for (int i = 0; i < N_S; ++i)
  {
    double _query_height = get_mapz(_neighbor_grid[i]);
    // cout << "isNearWall query_height: " << _neighbor_grid[i].transpose() << " " <<  _query_height << endl;
    if(_query_height <= -3){
      continue;
    }

    if (abs(_query_height - _query_center_height) > 3*_octree->getNodeSize(16))
    {
      // cout << "isNearWall... " << endl;

      found++;
      if (found >= 2)
      {
        return true;
      }
    }
  }
  // cout << "Not NearWall~~~~ " << endl;
  return false;
}

void get_neighbours(int n_array[], int index)
{
  n_array[0] = index - _gridmap.info.width;

  n_array[1] = index + _gridmap.info.width;

  n_array[2] = index - _gridmap.info.width - 1;

  n_array[3] = index - _gridmap.info.width + 1;

  n_array[4] = index - 1;

  n_array[5] = index + 1;

  n_array[6] = index + _gridmap.info.width + 1;

  n_array[7] = index + _gridmap.info.width - 1;
}

// 若邻域内超过半数栅格为featurerich, 则该栅格为featurerich
bool Neighbor_occupy(int Voxel_Idx)
{
  int locations[N_S];
  get_neighbours(locations, Voxel_Idx);

  int count = 0;
  for (int i = 0; i < N_S; ++i)
  {
    if (_gridmap.data[locations[i]] == Featurerich_griddata)
    {
      count++;
    }
  }

  if (count > (N_S / 2 + 2))
  {
    return true;
  }
  else
  {
    return false;
  }
}

#if def_octo_generator
void update2DMap(const octomap::ColorOcTree::iterator &it, bool occupied)
#else
void update2DMap(const octomap::OcTree::iterator &it, bool occupied)
#endif
{
  // update 2D map (occupied always overrides):
  if (it.getDepth() == project_2d_map_depth)
  {
    // int idx = mapIdx(it.getKey());
    int idx = mapIdx(ConvertWorld2GridIndex(it.getX(), it.getY()).x(), ConvertWorld2GridIndex(it.getX(), it.getY()).y());
    if (occupied)
    {
      // cout << "update2DMap - CellOccupied getCoordinate: " << it.getCoordinate() << endl;
      // cout << "update2DMap - CellOccupied getKey: " << it.getKey()[0] - _paddedMinKey[0] << " " << it.getKey()[1] - _paddedMinKey[1] << endl;
      // cout << "convert " << ConvertWorld2GridIndex(it.getCoordinate().x(), it.getCoordinate().y()).transpose() << endl;
      // if(_gridmap.data[idx] == 0)
      {
#if def_w_uncertainty
        if ((it.getZ() > 1) || (isFeatureRich_project(it->getColor().r, it->getColor().g, it->getColor().b) == false))
        {
          _gridmap.data[idx] = Featureless_griddata; // Featureless
        }
        else{
          if(isNearWall(it.getX(), it.getY())){
            _gridmap.data[idx] = Featureless_griddata;   // Featureless
          }
          else{
            _gridmap.data[idx] = Featurerich_griddata;   // Featurerich
          }            
        }
#else
          if((it.getZ() >= 3)){//m_Cur_position.z()    1
            _gridmap.data[idx] = Featureless_griddata;   // Featureless
          }
          else{
            if(isNearWall(it.getX(), it.getY())){
              _gridmap.data[idx] = Featureless_griddata;   // Featureless
            }
            else{
              _gridmap.data[idx] = Featurerich_griddata;   // Featurerich
            }            
            // _gridmap.data[idx] = Featurerich_griddata;   // Featurerich
          }
#endif
      }
    }
    else
    {
      // cout << "not occupied" << endl;
      // cout << "update2DMap - Cellfree: " << it.getCoordinate() << endl;
    }
  }
//   else
//   {
//     cout << "Depth not 15.............." << endl;

//     int intSize = 1 << (project_2d_map_depth - it.getDepth());
//     octomap::OcTreeKey minKey = it.getIndexKey();
//     for (int dx = 0; dx < intSize; dx++)
//     {
//       int i = minKey[0] + dx - _paddedMinKey[0];
//       for (int dy = 0; dy < intSize; dy++)
//       {
//         int idx = mapIdx(i, minKey[1] + dy - _paddedMinKey[1]);
//         // if(_gridmap.data[idx] == 0)
//         {
// #if def_w_uncertainty
//           if ((it.getZ() > 1) || (isFeatureRich_project(it->getColor().r, it->getColor().g, it->getColor().b) == false))
//           {
//             _gridmap.data[idx] = Featureless_griddata; // Featureless
//           }
//           else
//           {
//             if(isNearWall(it.getX(), it.getY())){
//               _gridmap.data[idx] = Featureless_griddata;   // Featureless
//             }
//             else{
//               _gridmap.data[idx] = Featurerich_griddata;   // Featurerich
//             }            
//           }
// #else
//           if((it.getZ() > 3)){//m_Cur_position.z()    1
//             _gridmap.data[idx] = Featureless_griddata;   // Featureless
//           }
//           else{
//             if(isNearWall(it.getX(), it.getY())){
//               _gridmap.data[idx] = Featureless_griddata;   // Featureless
//             }
//             else{
//               _gridmap.data[idx] = Featurerich_griddata;   // Featurerich
//             }            
//           }
// #endif
//         }
//       }
//     }
//   }
}


void Find_frontier_x_direction(Eigen::Vector2d cur_position)
{
  // x方向，找边界
  double minX, minY, minZ, maxX, maxY, maxZ;
  _octree->getMetricMin(minX, minY, minZ);
  _octree->getMetricMax(maxX, maxY, maxZ);
  minX -= 3*_gridmap.info.resolution;
  minY -= 3*_gridmap.info.resolution;
  maxX += 3*_gridmap.info.resolution;
  maxY += 3*_gridmap.info.resolution;

  // 间隔采样
  for (double x_left_extend = maxX; x_left_extend > minX;
       )
  {
    int min_index = 10000;
    int max_index = -10000;

    // 记录边界点用于提取frontier
    int min_frontierX_idx = 0;
    int min_frontierY_idx = 0;
    int max_frontierX_idx = 0;
    int max_frontierY_idx = 0;

    // 找到左右两侧端点，从两侧端点开始索引
    for (double y_search = maxY; y_search > minY;
         )
    {
      // 绝对坐标
      Eigen::Vector2d node_position(x_left_extend, y_search);
      Eigen::Vector2i node_index = ConvertWorld2GridIndex(node_position);

      // cout << "index: " << node_index.x() << " " << node_index.y() << " " <<
      // endl;
      int Idx = mapIdx(node_index.x(), node_index.y());

#if def_w_uncertainty
      // 剔除一些杂点
      if(_gridmap.data[Idx] == Featureless_griddata){
        // 若邻域内超过半数栅格为featurerich, 则该栅格为featurerich
        if(Neighbor_occupy(Idx)){
          _gridmap.data[Idx] = Featurerich_griddata;
        }
      }
#endif

      if (_gridmap.data[Idx] == Featurerich_griddata ||
          _gridmap.data[Idx] == Featureless_griddata)
      {
        if (node_index.y() < min_index)
        {
          min_index = node_index.y();
          min_frontierX_idx = node_index.x();
          min_frontierY_idx = node_index.y();
        }
        if (node_index.y() > max_index)
        {
          max_index = node_index.y();
          max_frontierX_idx = node_index.x();
          max_frontierY_idx = node_index.y();
        }
      }

      // 得到一些内部的边界点
      if ((abs(_gridmap.data[Idx]) == Featurerich_griddata) &&
          ((abs(_gridmap.data[Idx - _gridmap.info.width]) !=
            Featurerich_griddata) ||
           (abs(_gridmap.data[Idx + _gridmap.info.width]) !=
            Featurerich_griddata)))
      {
        Eigen::Vector2i frontier_idx(node_index.x(), node_index.y());
        // In limitation
        if (!(find(frontier2d_vector.begin(), frontier2d_vector.end(),
                   frontier_idx) != frontier2d_vector.end()) &&
            IsInLimitarea(frontier_idx))
        {
          frontier2d_vector.push_back(frontier_idx);
        }
      }

      if((y_search < m_Cur_position.y() + map_extend_length) && y_search > m_Cur_position.y() - map_extend_length){
        y_search -= _gridmap.info.resolution;
      }
      else{
        y_search -= 2 * _gridmap.info.resolution;
      }
    }

    if (min_index == 10000)
    {
      min_index = ConvertWorld2GridIndex(cur_position).y();
    }
    else
    {
      Eigen::Vector2i frontier_idx(min_frontierX_idx, min_frontierY_idx);
      if (!(find(frontier2d_vector.begin(), frontier2d_vector.end(),
                 frontier_idx) != frontier2d_vector.end()) &&
          IsInLimitarea(frontier_idx))
      {
        frontier2d_vector.push_back(frontier_idx);
      }
    }

    if (max_index == -10000)
    {
      max_index = ConvertWorld2GridIndex(cur_position).y();
    }
    else
    {
      Eigen::Vector2i frontier_idx(max_frontierX_idx, max_frontierY_idx);
      if (!(find(frontier2d_vector.begin(), frontier2d_vector.end(),
                 frontier_idx) != frontier2d_vector.end()) &&
          IsInLimitarea(frontier_idx))
      {
        frontier2d_vector.push_back(frontier_idx);
      }
    }

    // cout << "frontier2d_vector.size()" << frontier2d_vector.size() << endl;

    if((x_left_extend < m_Cur_position.x() + map_extend_length) && x_left_extend > m_Cur_position.x() - map_extend_length){
      x_left_extend -= _gridmap.info.resolution;
    }
    else{
      x_left_extend -= 2 * _gridmap.info.resolution;
    }
  }
}

void Find_frontier_y_direction(Eigen::Vector2d cur_position)
{
  // x方向，找边界
  double minX, minY, minZ, maxX, maxY, maxZ;
  _octree->getMetricMin(minX, minY, minZ);
  _octree->getMetricMax(maxX, maxY, maxZ);

  minX -= 3*_gridmap.info.resolution;
  minY -= 3*_gridmap.info.resolution;
  maxX += 3*_gridmap.info.resolution;
  maxY += 3*_gridmap.info.resolution;

  for (double y_search = maxY; y_search > minY;
       )
  {
    int min_index = 10000;
    int max_index = -10000;

    // 记录边界点用于提取frontier
    int min_frontierX_idx = 0;
    int min_frontierY_idx = 0;
    int max_frontierX_idx = 0;
    int max_frontierY_idx = 0;

    // 找到左右两侧端点，从两侧端点开始索引
    for (double x_left_extend = maxX; x_left_extend > minX;
         )
    {
      // 绝对坐标
      Eigen::Vector2d node_position(x_left_extend, y_search);
      Eigen::Vector2i node_index = ConvertWorld2GridIndex(node_position);

      int Idx = mapIdx(node_index.x(), node_index.y());
      
#if def_w_uncertainty
      // 剔除一些杂点
      if(_gridmap.data[Idx] == Featureless_griddata){
        // 若邻域内超过半数栅格为featurerich, 则该栅格为featurerich
        if(Neighbor_occupy(Idx)){
          _gridmap.data[Idx] = Featurerich_griddata;
        }
      }
#endif

      if (_gridmap.data[Idx] == Featurerich_griddata ||
          _gridmap.data[Idx] == Featureless_griddata)
      // if(_gridmap.data[Idx] != 0)
      {
        if (node_index.x() < min_index)
        {
          min_index = node_index.x();
          min_frontierX_idx = node_index.x();
          min_frontierY_idx = node_index.y();
        }
        if (node_index.x() > max_index)
        {
          max_index = node_index.x();
          max_frontierX_idx = node_index.x();
          max_frontierY_idx = node_index.y();
        }
      }

      if ((abs(_gridmap.data[Idx]) == Featurerich_griddata) &&
          ((abs(_gridmap.data[Idx - 1]) != Featurerich_griddata) ||
           (abs(_gridmap.data[Idx + 1]) != Featurerich_griddata)))
      {
        Eigen::Vector2i frontier_idx(node_index.x(), node_index.y());
        if (!(find(frontier2d_vector.begin(), frontier2d_vector.end(),
                   frontier_idx) != frontier2d_vector.end()) &&
            IsInLimitarea(frontier_idx))
        {
          frontier2d_vector.push_back(frontier_idx);
        }
      }

      if((x_left_extend < m_Cur_position.x() + map_extend_length) && x_left_extend > m_Cur_position.x() - map_extend_length){
        x_left_extend -= _gridmap.info.resolution;
      }
      else{
        x_left_extend -= 2 * _gridmap.info.resolution;
      }
    }

    if (min_index == 10000)
    {
      min_index = ConvertWorld2GridIndex(cur_position).x();
    }
    else
    {
      Eigen::Vector2i frontier_idx(min_frontierX_idx, min_frontierY_idx);
      if (!(find(frontier2d_vector.begin(), frontier2d_vector.end(),
                 frontier_idx) != frontier2d_vector.end()) &&
          IsInLimitarea(frontier_idx))
      {
        frontier2d_vector.push_back(frontier_idx);
      }
    }

    if (max_index == -10000)
    {
      max_index = ConvertWorld2GridIndex(cur_position).x();
    }
    else
    {
      Eigen::Vector2i frontier_idx(max_frontierX_idx, max_frontierY_idx);
      if (!(find(frontier2d_vector.begin(), frontier2d_vector.end(),
                 frontier_idx) != frontier2d_vector.end()) &&
          IsInLimitarea(frontier_idx))
      {
        frontier2d_vector.push_back(frontier_idx);
      }
    }

    // cout << "frontier2d_vector.size()" << frontier2d_vector.size() << endl;
    if((y_search < m_Cur_position.y() + map_extend_length) && y_search > m_Cur_position.y() - map_extend_length){
      y_search -= _gridmap.info.resolution;
    }
    else{
      y_search -= 2 * _gridmap.info.resolution;
    }
  }
}

void Find_frontier_x_direction_inrange(Eigen::Vector2d cur_position)
{
  // x方向，找边界
  double minX, minY, minZ, maxX, maxY, maxZ;
  _octree->getMetricMin(minX, minY, minZ);
  _octree->getMetricMax(maxX, maxY, maxZ);

  // minX = max(minX, m_Cur_position.x() - 10);
  // minY = max(minY, m_Cur_position.y() - 10);
  // maxX = min(maxX, m_Cur_position.x() + 10);
  // maxY = min(maxY, m_Cur_position.y() + 10);
  minX = (m_Cur_position.x() - 10);
  minY = (m_Cur_position.y() - 10);
  maxX = (m_Cur_position.x() + 10);
  maxY = (m_Cur_position.y() + 10);

  // minX -= 3*_gridmap.info.resolution;
  // minY -= 3*_gridmap.info.resolution;
  // maxX += 3*_gridmap.info.resolution;
  // maxY += 3*_gridmap.info.resolution;

  // 间隔采样
  for (double x_left_extend = maxX; x_left_extend >= (minX - _gridmap.info.resolution);
       )
  {
    int min_index = ConvertWorld2GridIndex(maxX, maxY).y();
    int max_index = ConvertWorld2GridIndex(minX, minY).y();
    // cout << "xd init min_index: " << min_index << "max_index: " << max_index << endl;

    // 记录边界点用于提取frontier
    int min_frontierX_idx = 0;
    int min_frontierY_idx = 0;
    int max_frontierX_idx = 0;
    int max_frontierY_idx = 0;

    // 找到左右两侧端点，从两侧端点开始索引
    for (double y_search = maxY; y_search >= (minY - _gridmap.info.resolution);
         )
    {
      // 绝对坐标
      Eigen::Vector2d node_position(x_left_extend, y_search);
      Eigen::Vector2i node_index = ConvertWorld2GridIndex(node_position);
      // cout << "xd search position: " << node_position.transpose() << endl;

      // cout << "xd search index: " << node_index.transpose() << endl;
      int Idx = mapIdx(node_index.x(), node_index.y());

#if def_w_uncertainty
      // 剔除一些杂点
      if(_gridmap.data[Idx] == Featureless_griddata){
        // 若邻域内超过半数栅格为featurerich, 则该栅格为featurerich
        if(Neighbor_occupy(Idx)){
          _gridmap.data[Idx] = Featurerich_griddata;
        }
      }
#endif

      if (_gridmap.data[Idx] == Featurerich_griddata ||
          _gridmap.data[Idx] == Featureless_griddata)
      {
        if (node_index.y() <= min_index)
        {
          min_index = node_index.y();
          min_frontierX_idx = node_index.x();
          min_frontierY_idx = node_index.y();
        }
        if (node_index.y() >= max_index)
        {
          max_index = node_index.y();
          max_frontierX_idx = node_index.x();
          max_frontierY_idx = node_index.y();
        }
      }
      // cout << "xd min_index: " << min_index << "max_index: " << max_index << endl;

      // 得到一些内部的边界点
      if ((abs(_gridmap.data[Idx]) == Featurerich_griddata) &&
          ((abs(_gridmap.data[Idx - _gridmap.info.width]) !=
            Featurerich_griddata) ||
           (abs(_gridmap.data[Idx + _gridmap.info.width]) !=
            Featurerich_griddata)))
      {
        Eigen::Vector2i frontier_idx(node_index.x(), node_index.y());
        // In limitation
        if (!(find(frontier2d_vector.begin(), frontier2d_vector.end(),
                   frontier_idx) != frontier2d_vector.end()) &&
            IsInLimitarea(frontier_idx))
        {
          frontier2d_vector.push_back(frontier_idx);
          // cout << "xd add frontier position: " << ConvertGridIndex2World(frontier_idx).transpose() << endl;
        }
      }

      y_search -= _gridmap.info.resolution;
      // if((y_search < m_Cur_position.y() + map_extend_length) && y_search > m_Cur_position.y() - map_extend_length){
      //   y_search -= _gridmap.info.resolution;
      // }
      // else{
      //   y_search -= 2 * _gridmap.info.resolution;
      // }
    }
    // cout << "xd end min_index: " << min_index << "max_index: " << max_index << endl;

    if (min_index <= ConvertWorld2GridIndex(minX, minY).y())
    {
      min_index = ConvertWorld2GridIndex(cur_position).y();
    }
    else
    {
      Eigen::Vector2i frontier_idx(min_frontierX_idx, min_frontierY_idx);
      if (!(find(frontier2d_vector.begin(), frontier2d_vector.end(),
                 frontier_idx) != frontier2d_vector.end()) &&
          IsInLimitarea(frontier_idx))
      {
        frontier2d_vector.push_back(frontier_idx);
        // cout << "xd add min_index frontier position: " << ConvertGridIndex2World(frontier_idx).transpose() << endl;
      }
    }

    if (max_index >= ConvertWorld2GridIndex(maxX, maxY).y())
    {
      max_index = ConvertWorld2GridIndex(cur_position).y();
    }
    else
    {
      Eigen::Vector2i frontier_idx(max_frontierX_idx, max_frontierY_idx);
      if (!(find(frontier2d_vector.begin(), frontier2d_vector.end(),
                 frontier_idx) != frontier2d_vector.end()) &&
          IsInLimitarea(frontier_idx))
      {
        frontier2d_vector.push_back(frontier_idx);
        // cout << "xd add max_index frontier position: " << ConvertGridIndex2World(frontier_idx).transpose() << endl;
      }
    }

    // cout << "frontier2d_vector.size()" << frontier2d_vector.size() << endl;
    x_left_extend -= _gridmap.info.resolution;

    // if((x_left_extend < m_Cur_position.x() + map_extend_length) && x_left_extend > m_Cur_position.x() - map_extend_length){
    //   x_left_extend -= _gridmap.info.resolution;
    // }
    // else{
    //   x_left_extend -= 2 * _gridmap.info.resolution;
    // }
  }
}

void Find_frontier_y_direction_inrange(Eigen::Vector2d cur_position)
{
  // x方向，找边界
  double minX, minY, minZ, maxX, maxY, maxZ;
  _octree->getMetricMin(minX, minY, minZ);
  _octree->getMetricMax(maxX, maxY, maxZ);

  // minX = max(minX, m_Cur_position.x() - 10);
  // minY = max(minY, m_Cur_position.y() - 10);
  // maxX = min(maxX, m_Cur_position.x() + 10);
  // maxY = min(maxY, m_Cur_position.y() + 10);
  minX = (m_Cur_position.x() - 10);
  minY = (m_Cur_position.y() - 10);
  maxX = (m_Cur_position.x() + 10);
  maxY = (m_Cur_position.y() + 10);

  // minX -= 3*_gridmap.info.resolution;
  // minY -= 3*_gridmap.info.resolution;
  // maxX += 3*_gridmap.info.resolution;
  // maxY += 3*_gridmap.info.resolution;

  for (double y_search = maxY; y_search >= (minY - _gridmap.info.resolution);
       )
  {
    int min_index = ConvertWorld2GridIndex(maxX, maxY).x();
    int max_index = ConvertWorld2GridIndex(minX, minY).x();

    // 记录边界点用于提取frontier
    int min_frontierX_idx = 0;
    int min_frontierY_idx = 0;
    int max_frontierX_idx = 0;
    int max_frontierY_idx = 0;

    // 找到左右两侧端点，从两侧端点开始索引
    for (double x_left_extend = maxX; x_left_extend >= (minX - _gridmap.info.resolution);
         )
    {
      // 绝对坐标
      Eigen::Vector2d node_position(x_left_extend, y_search);
      Eigen::Vector2i node_index = ConvertWorld2GridIndex(node_position);
      // cout << "yd search position: " << node_position.transpose() << endl;

      int Idx = mapIdx(node_index.x(), node_index.y());
      
#if def_w_uncertainty
      // 剔除一些杂点
      if(_gridmap.data[Idx] == Featureless_griddata){
        // 若邻域内超过半数栅格为featurerich, 则该栅格为featurerich
        if(Neighbor_occupy(Idx)){
          _gridmap.data[Idx] = Featurerich_griddata;
        }
      }
#endif

      if (_gridmap.data[Idx] == Featurerich_griddata ||
          _gridmap.data[Idx] == Featureless_griddata)
      // if(_gridmap.data[Idx] != 0)
      {
        if (node_index.x() < min_index)
        {
          min_index = node_index.x();
          min_frontierX_idx = node_index.x();
          min_frontierY_idx = node_index.y();
        }
        if (node_index.x() > max_index)
        {
          max_index = node_index.x();
          max_frontierX_idx = node_index.x();
          max_frontierY_idx = node_index.y();
        }
      }

      if ((abs(_gridmap.data[Idx]) == Featurerich_griddata) &&
          ((abs(_gridmap.data[Idx - 1]) != Featurerich_griddata) ||
           (abs(_gridmap.data[Idx + 1]) != Featurerich_griddata)))
      {
        Eigen::Vector2i frontier_idx(node_index.x(), node_index.y());
        if (!(find(frontier2d_vector.begin(), frontier2d_vector.end(),
                   frontier_idx) != frontier2d_vector.end()) &&
            IsInLimitarea(frontier_idx))
        {
          frontier2d_vector.push_back(frontier_idx);
          // cout << "yd add frontier position: " << ConvertGridIndex2World(frontier_idx).transpose() << endl;
        }
      }
      x_left_extend -= _gridmap.info.resolution;

      // if((x_left_extend < m_Cur_position.x() + map_extend_length) && x_left_extend > m_Cur_position.x() - map_extend_length){
      //   x_left_extend -= _gridmap.info.resolution;
      // }
      // else{
      //   x_left_extend -= 2 * _gridmap.info.resolution;
      // }
    }

    if (min_index <= ConvertWorld2GridIndex(minX, minY).x())
    {
      min_index = ConvertWorld2GridIndex(cur_position).x();
    }
    else
    {
      Eigen::Vector2i frontier_idx(min_frontierX_idx, min_frontierY_idx);
      if (!(find(frontier2d_vector.begin(), frontier2d_vector.end(),
                 frontier_idx) != frontier2d_vector.end()) &&
          IsInLimitarea(frontier_idx))
      {
        frontier2d_vector.push_back(frontier_idx);
        // cout << "yd add min_index frontier position: " << ConvertGridIndex2World(frontier_idx).transpose() << endl;
      }
    }

    if (max_index >= ConvertWorld2GridIndex(maxX, maxY).x())
    {
      max_index = ConvertWorld2GridIndex(cur_position).x();
    }
    else
    {
      Eigen::Vector2i frontier_idx(max_frontierX_idx, max_frontierY_idx);
      if (!(find(frontier2d_vector.begin(), frontier2d_vector.end(),
                 frontier_idx) != frontier2d_vector.end()) &&
          IsInLimitarea(frontier_idx))
      {
        frontier2d_vector.push_back(frontier_idx);
        // cout << "yd add max_index frontier position: " << ConvertGridIndex2World(frontier_idx).transpose() << endl;
      }
    }
    y_search -= _gridmap.info.resolution;

    // cout << "frontier2d_vector.size()" << frontier2d_vector.size() << endl;
    // if((y_search < m_Cur_position.y() + map_extend_length) && y_search > m_Cur_position.y() - map_extend_length){
    //   y_search -= _gridmap.info.resolution;
    // }
    // else{
    //   y_search -= 2 * _gridmap.info.resolution;
    // }
  }
}

void get_neighbours(Eigen::Vector2i n_array[], Eigen::Vector2i position)
{
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

const int MIN_FOUND = 1;
bool is_frontier_point(Eigen::Vector2i point)
{
  int Idx = mapIdx(point.x(), point.y());

  if (abs(_gridmap.data[Idx]) != Featurerich_griddata)
  {
    cout << "_gridmap.data[Idx]: " << _gridmap.data[Idx] << endl;
    return false;
  }

  Eigen::Vector2i locations[N_S];
  get_neighbours(locations, point);
  int found = 0;
  for (int i = 0; i < N_S; ++i)
  {
    int Idx = mapIdx(locations[i].x(), locations[i].y());

    if (abs(_gridmap.data[Idx]) != Featurerich_griddata)
    {
      found++;
      if (found == MIN_FOUND)
      {
        return true;
      }
    }
  }
  return false;
}

Eigen::Matrix2d Rot_90;
Eigen::Vector2d Dir_search_horizon;
void predict2DMap_distance_kd(Eigen::Vector2d cur_position,
                              float max_predict_distance)
{
  Rot_90 << 0, -1, 1, 0;
  Eigen::Vector2d Dir_search_horizon = Rot_90 * Body_dir_avg;

  auto start = std::chrono::high_resolution_clock::now();

  // frontier2d_vector.clear();
  
  for (vector<Eigen::Vector2i>::iterator it = frontier2d_vector.begin(); it != frontier2d_vector.end(); )
  {
    // cout << "earse frontier position: " << ConvertGridIndex2World(it->x(), it->y()).transpose() << endl;
    if (abs(ConvertGridIndex2World(it->x(), it->y()).x() - m_Cur_position.x()) <= 10 && abs(ConvertGridIndex2World(it->x(), it->y()).y() - m_Cur_position.y()) <= 10)
    {
      it = frontier2d_vector.erase(it);
    }
    else{
      ++it;
    }
  }

  // 提取边界
  // Find_frontier_x_direction(cur_position);
  // Find_frontier_y_direction(cur_position);
  Find_frontier_x_direction_inrange(cur_position);
  Find_frontier_y_direction_inrange(cur_position);

  // 剔除在无效定位区的边界点
  int threshold_near_nofeature = 1;
  for (vector<Eigen::Vector2i>::iterator it = frontier2d_vector.begin(); it != frontier2d_vector.end();)
  {
    if (_gridmap.data[it->x() + it->y() * _gridmap.info.width] <= -99)
    {
      // cout << "delete pos:" << ConvertGridIndex2World(it->x(),it->y()).transpose() << " _gridmap.data:" << int(_gridmap.data[it->x() + it->y() * _gridmap.info.width]) << endl;
      // cout << "frontier point is featureless..." << endl;
      it = frontier2d_vector.erase(it);
    }
    else
    {
      bool _neighbor_featureless = false;
      // cout << "vacancy:" << vacancy[it->x() + it->y() * map_info.width] << endl;
      // cout << "frontier point is feature-rich..." << endl;
      for (int dx = -threshold_near_nofeature; dx <= threshold_near_nofeature; dx++)
      {
        for (int dy = -threshold_near_nofeature; dy <= threshold_near_nofeature; dy++)
        {
          int query_idx = it->x() + dx + (it->y() + dy) * _gridmap.info.width;
          // 无效定位区1
          if (_gridmap.data[query_idx] <= -99)
          {
            // cout << "delete pos:" << ConvertGridIndex2World(it->x(),it->y()).transpose() << " _gridmap.data:" << int(_gridmap.data[query_idx]) << endl;

            it = frontier2d_vector.erase(it);
            _neighbor_featureless = true;
            // cout << "fsmi_data near the forntier: " << fsmi_data[x + y * side_length] << endl;
            break;
          }
        }
        if (_neighbor_featureless)
        {
          break;
        }
      }

      if (!_neighbor_featureless)
      {
        ++it;
      }
    }
  }

  auto stop = std::chrono::high_resolution_clock::now();
  auto timespan = std::chrono::duration<double>(stop - start);
  // std::cout << "it took " << timespan.count() << " seconds.";
  ROS_WARN_STREAM("Extract 2D Frontier takes: " << timespan.count() * 1000
                                             << " ms");

}

/*******************************************************************************************/
/*******************************************************************************************/
/*********************************Frotier 3D Extraction*************************************/
/*******************************************************************************************/
/*******************************************************************************************/

// const int m_Frontier3D_depth = 15;    // !!! gridmap_rrt_3d.cpp中的Viewpoint_Check_Frontier和fpInFOV也要修改
void get_neighbours(octomap::OcTreeKey start_key,
                    std::vector<octomap::point3d> &occupiedNeighbor)
{
  occupiedNeighbor.clear();
  point3d neighbor_position;

  int N_extend = 3;
  // cout << "N_extend: " << N_extend << endl;
  float _step = _octree->begin_leafs(m_Frontier3D_depth).getSize();
  // cout << "GetNeighbor: " << _step << endl;
  point3d query_center = _octree->keyToCoord(start_key);
  for (int i = -N_extend / 2; i <= N_extend / 2; ++i)
  {
    for (int j = -N_extend / 2; j <= N_extend / 2; ++j)
    {
      for (int k = -N_extend / 2; k <= N_extend / 2; ++k)
      {
        // cout << "i " << i << " j " << j << endl;
        if (i == 0 && j == 0 && k == 0)
        {
          continue;
        }
        else
        {
          neighbor_position.x() = query_center.x() + _step * i;
          neighbor_position.y() = query_center.y() + _step * j;
          neighbor_position.z() = query_center.z() + _step * k;
          // cout << "neigh_pos:  " << neighbor_position.transpose() << endl;
          occupiedNeighbor.push_back(neighbor_position);
        }
      }
    }
  }
}

KeySet m_clusteredCells, m_clusteredCellsUpdated;
void pointVectorToKey(vector<geometry_msgs::Point> &points,
                      vector<OcTreeKey> &clusterCellsKey)
{
  for (int i = 0; i < points.size(); i++)
  {
    point3d tempCellCoordinates;
    tempCellCoordinates.x() = points[i].x;
    tempCellCoordinates.y() = points[i].y;
    tempCellCoordinates.z() = points[i].z;
    // Transform from point to key
    OcTreeKey tempCellKey;
    if (!_octree->coordToKeyChecked(tempCellCoordinates, tempCellKey))
    {
      OCTOMAP_ERROR_STR("Error in search: [" << tempCellCoordinates
                                             << "] is out of OcTree bounds!");
      return;
    }
    clusterCellsKey.push_back(tempCellKey);
  }
}

void keyToPointVector(std::vector<Eigen::Vector3d> &frontierCells,
                      vector<geometry_msgs::Point> &originalPointsVector)
{
  for (vector<Eigen::Vector3d>::iterator iter = frontierCells.begin(),
                                         end = frontierCells.end();
       iter != end; ++iter)
  {
    geometry_msgs::Point tempCellPoint;
    tempCellPoint.x = (*iter).x();
    tempCellPoint.y = (*iter).y();
    tempCellPoint.z = (*iter).z();

    originalPointsVector.push_back(tempCellPoint);
  }
}

void publishglobalFrontierCells(
    const vector<Eigen::Vector3d> globalFrontierCells)
{
  cout << "publishFrontier" << endl;

  // init markers for free space:
  visualization_msgs::MarkerArray frontierNodesVis;
  visualization_msgs::Marker _delete_all;
  _delete_all.header.frame_id = "world";
  _delete_all.action = visualization_msgs::Marker::DELETEALL;
  frontierNodesVis.markers.push_back(_delete_all);
  m_markerFrontierPub.publish(frontierNodesVis);

  frontierNodesVis.markers.clear();
  // each array stores all cubes of a different size, one for each depth level:
  frontierNodesVis.markers.resize(globalFrontierCells.size());
  cout << "m_globalFrontierCellsUpdated.size(): " << globalFrontierCells.size()
       << endl;

  int counter{0};

  for (int iter = 0; iter < globalFrontierCells.size(); ++iter)
  {
    geometry_msgs::Point cubeCenter;
    cubeCenter.x = globalFrontierCells[iter].x();
    cubeCenter.y = globalFrontierCells[iter].y();
    cubeCenter.z = globalFrontierCells[iter].z();

    frontierNodesVis.markers[counter].points.push_back(cubeCenter);
    counter++;
  }

  cout << "frontierNodesVis.markers[i].points.size(): "
       << frontierNodesVis.markers.size() << endl;
  // finish MarkerArray:
  std_msgs::ColorRGBA colorFrontier;
  colorFrontier.r = 1.0;
  colorFrontier.g = 0.0;
  colorFrontier.b = 0.0;
  colorFrontier.a = 1.0;
  for (unsigned i = 0; i < frontierNodesVis.markers.size(); ++i)
  {
    double size = 0.5;

    frontierNodesVis.markers[i].header.frame_id = "world";
    frontierNodesVis.markers[i].header.stamp = ros::Time::now();
    frontierNodesVis.markers[i].ns = "red";
    frontierNodesVis.markers[i].id = i;
    frontierNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    frontierNodesVis.markers[i].scale.x = size;
    frontierNodesVis.markers[i].scale.y = size;
    frontierNodesVis.markers[i].scale.z = size;
    frontierNodesVis.markers[i].color = colorFrontier;

    if (frontierNodesVis.markers[i].points.size() > 0)
      frontierNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
    else
      frontierNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  m_markerFrontierPub.publish(frontierNodesVis);
}

void publishClusteredFrontier()
{
  cout << "clusterFrontierAndPublish - m_clusteredCellsUpdated.size: "
       << m_clusteredCellsUpdated.size() << endl;
  // init markers for free space:
  visualization_msgs::MarkerArray frontierNodesVis;
  // each array stores all cubes of a different size, one for each depth

  frontierNodesVis.markers.resize(m_clusteredCellsUpdated.size());
  int counter{0};

  for (KeySet::iterator iter = m_clusteredCellsUpdated.begin(),
                        end = m_clusteredCellsUpdated.end();
       iter != end; ++iter)
  {
    octomap::point3d fpoint;
    fpoint = _octree->keyToCoord(*iter);

    geometry_msgs::Point cubeCenter;
    cubeCenter.x = fpoint.x();
    cubeCenter.y = fpoint.y();
    cubeCenter.z = fpoint.z();

    frontierNodesVis.markers[counter].points.push_back(cubeCenter);
    counter++;
  }

  // finish MarkerArray:
  std_msgs::ColorRGBA colorClusteredFrontier;
  colorClusteredFrontier.r = 1.0;
  colorClusteredFrontier.g = 0.9;
  colorClusteredFrontier.b = 0.1;
  colorClusteredFrontier.a = 0.9;

  for (unsigned i = 0; i < frontierNodesVis.markers.size(); ++i)
  {
    double size = 0.6;

    frontierNodesVis.markers[i].header.frame_id = "world";
    frontierNodesVis.markers[i].header.stamp = ros::Time::now();
    frontierNodesVis.markers[i].ns = "red";
    frontierNodesVis.markers[i].id = i;
    frontierNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    frontierNodesVis.markers[i].scale.x = size;
    frontierNodesVis.markers[i].scale.y = size;
    frontierNodesVis.markers[i].scale.z = size;
    frontierNodesVis.markers[i].color = colorClusteredFrontier;

    if (frontierNodesVis.markers[i].points.size() > 0)
      frontierNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
    else
      frontierNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  m_markerClusteredFrontierPub.publish(frontierNodesVis);
}

void publishOccAndFree()
{
  float _m_occupancyMinZ = m_Cur_position.z(), _m_occupancyMaxZ = 5;
  // int _m_maxTreeDepth = 15, _m_treeDepth = 15;
  ros::WallTime startTime = ros::WallTime::now();
  size_t octomapSize = _octree->size();
  if (octomapSize <= 1)
  {
    ROS_WARN("Nothing to publish, octree is empty");
    return;
  }

  // Init markers for free space:
  visualization_msgs::MarkerArray freeNodesVis;
  // Each array stores all cubes of a different size, one for each depth level:
  freeNodesVis.markers.resize(m_Frontier3D_depth + 1);

  // Init markers:
  visualization_msgs::MarkerArray occupiedNodesVis;
  // Each array stores all cubes of a different size, one for each depth level:
  occupiedNodesVis.markers.resize(m_Frontier3D_depth + 1);

  // Traverse all leafs in the tree:
#if def_octo_generator
  for (ColorOcTree::iterator it = _octree->begin(m_Frontier3D_depth),
                             end = _octree->end();
       it != end; ++it)
#else
  for (OcTree::iterator it = _octree->begin(m_Frontier3D_depth),
                             end = _octree->end();
       it != end; ++it)
#endif
  {
    if (_octree->isNodeOccupied(*it))
    {
      double z = it.getZ();
      // cout << "publishOccAndFree - z: " << z << endl;
      if (z > _m_occupancyMinZ && z < _m_occupancyMaxZ)
      {
        double size = it.getSize();
        double x = it.getX();
        double y = it.getY();

        // Ignore speckles in the map:
        // if (m_filterSpeckles && (it.getDepth() == m_Frontier3D_depth + 1) &&
        //   isSpeckleNode(it.getKey()))
        // {
        //   ROS_DEBUG("Ignoring single speckle at (%f,%f,%f)", x, y, z);
        //   continue;
        // } // else: current octree node is no speckle, send it out

        // Create marker:
        unsigned idx = it.getDepth();
        assert(idx < occupiedNodesVis.markers.size());

        geometry_msgs::Point cubeCenter;
        cubeCenter.x = x;
        cubeCenter.y = y;
        cubeCenter.z = z;

        occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
      }
    }
    else
    {
      // Node not occupied => mark as free in 2D map if unknown so far
      double z = it.getZ();
      if (z > _m_occupancyMinZ && z < _m_occupancyMaxZ)
      {

        double x = it.getX();
        double y = it.getY();

        // Create marker for free space:
        unsigned idx = it.getDepth();
        assert(idx < freeNodesVis.markers.size());

        geometry_msgs::Point cubeCenter;
        cubeCenter.x = x;
        cubeCenter.y = y;
        cubeCenter.z = z;

        freeNodesVis.markers[idx].points.push_back(cubeCenter);
      }
    }
  }

  // Finish MarkerArray:
  std_msgs::ColorRGBA colorOcc, colorFree;
  colorOcc.r = 0.0;
  colorOcc.g = 0.0;
  colorOcc.b = 1.0;
  colorOcc.a = 1.0;

  colorFree.r = 0.0;
  colorFree.g = 1.0;
  colorFree.b = 0.0;
  colorFree.a = 1.0;
  for (unsigned i = 0; i < occupiedNodesVis.markers.size(); ++i)
  {
    double size = _octree->getNodeSize(i);

    occupiedNodesVis.markers[i].header.frame_id = "world";
    occupiedNodesVis.markers[i].header.stamp = ros::Time::now();
    occupiedNodesVis.markers[i].ns = "red";
    occupiedNodesVis.markers[i].id = i;
    occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    occupiedNodesVis.markers[i].scale.x = size;
    occupiedNodesVis.markers[i].scale.y = size;
    occupiedNodesVis.markers[i].scale.z = size;
    occupiedNodesVis.markers[i].color = colorOcc;

    if (occupiedNodesVis.markers[i].points.size() > 0)
      occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
    else
      occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  m_markerOccPub.publish(occupiedNodesVis);

  // Finish FreeMarkerArray:
  for (unsigned i = 0; i < freeNodesVis.markers.size(); ++i)
  {
    double size = _octree->getNodeSize(i);

    freeNodesVis.markers[i].header.frame_id = "world";
    freeNodesVis.markers[i].header.stamp = ros::Time::now();
    freeNodesVis.markers[i].ns = "red";
    freeNodesVis.markers[i].id = i;
    freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    freeNodesVis.markers[i].scale.x = size;
    freeNodesVis.markers[i].scale.y = size;
    freeNodesVis.markers[i].scale.z = size;
    freeNodesVis.markers[i].color = colorFree;

    if (freeNodesVis.markers[i].points.size() > 0)
      freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
    else
      freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  m_markerFreePub.publish(freeNodesVis);
}

void clusterFrontierAndPublish()
{
  // cout << "clusterFrontierAndPublish" << endl;
  ros::WallTime startTime_evaluation = ros::WallTime::now();
  m_clusteredCells.clear();

  // Preprocess put the frontier cells into a vector
  std::vector<geometry_msgs::Point> originalPointsVector{};
  std::vector<geometry_msgs::Point> clusteredPointsVector{};
  // cout << "clusterFrontierAndPublish - g_Frontiers3D_pos size(): "
  //      << g_Frontiers3D_pos.size() << endl;
  keyToPointVector(g_Frontiers3D_pos, originalPointsVector);
  // cout << "clusterFrontierAndPublish - originalPointsVector size(): "
  //      << originalPointsVector.size() << endl;

  double m_kernelBandwidth = 1;
  MSCluster *cluster = new MSCluster();
  cluster->getMeanShiftClusters(originalPointsVector, clusteredPointsVector,
                                m_kernelBandwidth);

  // cout << "clusterFrontierAndPublish - clusteredPointsVector size(): "
  //      << clusteredPointsVector.size() << endl;

  vector<OcTreeKey> clusterCellsKey{};
  pointVectorToKey(clusteredPointsVector, clusterCellsKey);

  // cout << "clusterFrontierAndPublish - clusterCellsKey size(): "
  //      << clusterCellsKey.size() << endl;

  for (std::vector<OcTreeKey>::iterator iter = clusterCellsKey.begin();
       iter != clusterCellsKey.end(); iter++)
  {
    m_clusteredCells.insert(*iter);
    // cout << "clusterFrontierAndPublish - m_clusteredCells position: "
    //      << _octree->keyToCoord(*iter) << endl;
  }
  delete cluster;
  // cout << "cluster_size: " << m_clusteredCells.size() << endl;
  // cout << "cluster_size: " << m_clusteredCells.size() << endl;
  double total_time_evaluation =
      (ros::WallTime::now() - startTime_evaluation).toSec();
  // cout << "clusterFrontier used total: " << total_time_evaluation << " sec"
  //      << endl;

  m_clusteredCellsUpdated.clear();
  g_Frontiers_Cluster_pos.clear();
  for (KeySet::iterator iter = m_clusteredCells.begin(),
                        end = m_clusteredCells.end();
       iter != end; ++iter)
  {
    // Get cell position
    m_clusteredCellsUpdated.insert(*iter);
    octomap::point3d query = _octree->keyToCoord(*iter);
    Eigen::Vector3d vec_pos(query.x(), query.y(), query.z());
    if(clusterCellsKey.size() >= 2){
      g_Frontiers_Cluster_pos.push_back(vec_pos);
    }
  }

  publishClusteredFrontier();
}

const int NeiborUnknowThresh = (6 + 6);
const int NeiborOccupiedThresh = 8; // 剔除在墙壁后面的边界点
const int NeiborFreeThresh = 5; 

void Frontier3d_Extraction(Eigen::Vector3d _cur_position)
{
  publishOccAndFree();

  g_Frontiers3D_pos.clear();

#if def_octo_generator
  for (octomap::ColorOcTree::iterator it = _octree->begin(m_Frontier3D_depth),
                                      end = _octree->end();
       it != end; ++it)
#else
  for (octomap::OcTree::iterator it = _octree->begin(m_Frontier3D_depth),
                                      end = _octree->end();
       it != end; ++it)
#endif
  {
    Eigen::Vector2d _dir_xy(it.getX() - _cur_position.x(), it.getY() - _cur_position.y());    // 只更新当前位置一定范围内的边界点
    if (_octree->isNodeOccupied(*it) && it.getZ() > _cur_position.z() && (Explore_Done_2d || _dir_xy.norm() <= 10)) // 2d exploration完成之前只考虑局部范围的3d点
    {
      // cout << "occupied pos:" << (it.getCoordinate()) << endl;
      // cout << "Frontier3d Extract - CellOccupied: " << it.getCoordinate() <<
      // endl;
      {
        bool unknownCellFlag = false;
        bool freeCellFlag = false;
        bool occpiedCellFlag = false;
        int occupied_counter{0};
        int unknown_counter{0};
        int free_counter{0};
        std::vector<octomap::point3d> changedCellNeighbor;

        get_neighbours(it.getKey(), changedCellNeighbor);
        for (std::vector<point3d>::iterator iter = changedCellNeighbor.begin();
             iter != changedCellNeighbor.end(); iter++)
        {
          // Check point state: unknown(null)/free
          // cout << "Frontier3d Extract - CellNeighbor: " << *iter << endl;

          OcTreeNode *node = _octree->search(*iter, m_Frontier3D_depth);
          if (node == NULL)
          {
            // cout << "Frontier3d Extract - Unknown: " << endl;
            unknownCellFlag = true;
            unknown_counter++;
          }
          else if (_octree->isNodeOccupied(node))
          {
            // cout << "Frontier3d Extract - Occupied: " << endl;
            occpiedCellFlag = true;
            occupied_counter++;
            // counter++;
          }
          else
          {
            // cout << "Frontier3d Extract - Free: " << endl;
            freeCellFlag = true;
            free_counter++;
          }
          // else if (!_octree->isNodeOccupied(node))
          //   freeCellFlag = true;
        }
        point3d query = it.getCoordinate();
        point3d query_up(query.x(), query.y(), query.z() + _octree->getResolution()); 
        point3d query_down(query.x(), query.y(), query.z() - _octree->getResolution()); 
        Eigen::Vector3d query_3d(query.x(), query.y(), query.z());

        OcTreeNode *node_up = _octree->search(query_up, m_Frontier3D_depth);
        OcTreeNode *node_down = _octree->search(query_down, m_Frontier3D_depth);
        /***** !崎岖地形下需要加上这一部分，防止任何一点都是frontier *****/
        if(def_terrain_mode && (node_up == NULL || node_down == NULL)){
          // ROS_ERROR_STREAM("Up or Down node of frontier is Unknown");
        }
        else
        {
          // cout << "Frontier3d Extract: " << unknown_counter << " " <<
          // occupied_counter << endl;
          // if (unknownCellFlag && free_counter > NeiborFreeThresh && 
          //     occupied_counter < NeiborOccupiedThresh && query_3d.z() < 4)
          if (unknownCellFlag && freeCellFlag && (unknown_counter > NeiborUnknowThresh || (unknown_counter > 8 && occupied_counter < NeiborOccupiedThresh))&& query_3d.z() < 4)  //  
          {
            if((find(Frontier3D_black_list.begin(), Frontier3D_black_list.end(), query_3d) != Frontier3D_black_list.end())){    // 在黑名单中，则不作为边界点
              // cout << "Frontier3d_Extraction - the frontier3d is in blacklist: " << query_3d.transpose() << endl;
            }
            else{   // 不在白名单中，则作为边界点
              g_Frontiers3D_pos.push_back(query_3d);
            }
          }
        }       
        
        // globalFrontierCells.insert(changedCellKey);
        // global_FrontierCell.insert(changedCellKey);
        // frontierSize++;
      }
    }
  }
}

static int initual_map = 0;
void publish2Dmap(const std_msgs::Header &header, double zmin, double zmax,
                  Eigen::Vector3d cur_position)
{
  // _octree = map->map_tree_;
  m_Cur_position = cur_position;

  // alloc memory stuff
  if (initual_map == 0)
  {
    // handle_pre(header);
    handle_pre_simple(header);
    initual_map = 1;
  }

  // 三维栅格投影到二维
  if (zmin >= zmax)
  {
    zmax = std::numeric_limits<double>::max();
    zmin = -zmax;
  }
  auto start_dm = std::chrono::system_clock::now();

  // now, traverse all leafs in the tree:
#if def_octo_generator
  for (octomap::ColorOcTree::iterator it = _octree->begin(project_2d_map_depth),
                                      end = _octree->end();
       it != end; ++it)
#else
  for (octomap::OcTree::iterator it = _octree->begin(project_2d_map_depth),
                                      end = _octree->end();
       it != end; ++it)
#endif
  {
    Eigen::Vector2d _dir_xy(it.getX() - m_Cur_position.x(), it.getY() - m_Cur_position.y());
    if ((_dir_xy.norm() <= 10)) // 只考虑视野范围内的点
    {
      if (_octree->isNodeOccupied(*it))
      {
        // cout << "occupied pos:" << (it.getCoordinate()) << endl;
        double z = it.getZ();
        if (z > zmin && z < zmax)
          update2DMap(it, true);
      }
      // else
      // { // node not occupied => mark as free in 2D map
      //   cout << "free pos:" << (it.getCoordinate()) << endl;
      //   double z = it.getZ();
      //   // if (z > zmin && z < zmax)
      //   update2DMap(it, false);
      // }
    }
    
  }
  auto end_dm = std::chrono::system_clock::now();
  auto elapsed_seconds_dm =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_dm - start_dm);
  ROS_WARN_STREAM("Projected 2d map takes: "
                  << elapsed_seconds_dm.count() << " ms");

  // 预测地图

  Eigen::Vector2d cur_position_2d(cur_position.x(), cur_position.y());

  // 根据xy方向上未知栅格与边界的距离，取两个方向上距离最近（即信任度最高的）的值为指标
  // predict2DMap_distance(cur_position_2d);

  // thread t(predict2DMap_GP, cur_position_2d);
  // t.detach();

  // 高斯过程预测未知栅格
  // predict2DMap_GP(cur_position_2d, max_predict_distance_);
  // predict2DMap_GP_frontier(cur_position_2d, max_predict_distance_);

  // 通过KD
  // tree构建边界，检索未知栅格与离得最近的边界的距离，以此距离作为衰减指标
  predict2DMap_distance_kd(cur_position_2d, max_predict_distance_);
}
