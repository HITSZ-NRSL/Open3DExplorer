#include <string>     

#include <ros/ros.h>
#include <ros/console.h>
#include <log4cxx/logger.h>

#include <vector>
#include <iostream>
#include <octomap/octomap.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
//#include <cmath>
#include <nav_msgs/OccupancyGrid.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <fstream>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <unordered_map>

#include <numeric>
#include <chrono>
#include <vector>

// #include <montecarlo.hpp>
#include "infomation_cal.hpp"
#include <nodehash.h>

// #include "map.hpp"
#include "../include/fsmi_cal.h"

using namespace std;
using namespace octomap;

typedef Eigen::Matrix<double, 3, 6> Matrix36;
typedef Eigen::Matrix<double, 3, 3> Matrix33;
typedef Eigen::Matrix<double, 3, 4> Matrix34;
typedef Eigen::Matrix<double, 3, 3> Matrix33;
typedef Eigen::Matrix<double, 6, 6> Matrix66;

vector<Eigen::Vector3d> sample_points_online_merge;
vector<int> sample_points_info;
vector<Eigen::Vector3i> sample_points_color;

vector<Eigen::Vector3d> sample_point_visualization_online;

ros::Publisher m_markerAdaptSemanticOctreePub;

int max_iteration_set = 200;
int sum_size_set = 10; // add top 10

struct myclass
{
  bool operator()(int i, int j) { return (i > j); }
} myobject;

int blue_info_avg, green_info_avg;

bool ColorIsFeatureRich(Eigen::Vector3i _query_color){
  // {4,250,7} -> grass
  if((_query_color.x() <= (4 + 8)) && (_query_color.x() >= (4 - 4)) && (_query_color.y() <= (250 + 5)) && (_query_color.y() >= (250 - 5)) && (_query_color.z() <= (7 + 7)) && (_query_color.z() >= (7 - 7))){
    return true;
  }
  // {4,200,3} -> tree
  else if((_query_color.x() <= (4 + 8)) && (_query_color.x() >= (4 - 4)) && (_query_color.y() <= (200 + 5)) && (_query_color.y() >= (200 - 5)) && (_query_color.z() <= (3 + 7)) && (_query_color.z() >= (3 - 3))){
    return true;
  }
}

bool ColorIsFeatureLess(Eigen::Vector3i _query_color){
  // {61,230,250} -> water
  if((_query_color.x() <= (61 + 6)) && (_query_color.x() >= (61 - 6)) && (_query_color.y() <= (230 + 5)) && (_query_color.y() >= (230 - 5)) && (_query_color.z() <= (250 + 5)) && (_query_color.z() >= (250 - 5))){
    return true;
  }
}

void publishAdaptiveOctree_Marker()
{
  int _AdaptiveSM_depth = 16;
  // int _m_maxTreeDepth = 15, _m_treeDepth = 15;
  ros::WallTime startTime = ros::WallTime::now();
  size_t octomapSize = _octree->size();
  if (octomapSize <= 1)
  {
    ROS_WARN("Nothing to publish, octree is empty");
    return;
  }

  // Init markers:
  visualization_msgs::MarkerArray occupiedNodesVis;
  // Each array stores all cubes of a different size, one for each depth level:
  // occupiedNodesVis.markers.resize(octomapSize);
  visualization_msgs::Marker cube_marker;
  
  cube_marker.header.frame_id = "/world";
  cube_marker.header.stamp = ros::Time::now();
  cube_marker.action = visualization_msgs::Marker::ADD;
  cube_marker.type = visualization_msgs::Marker::CUBE_LIST;
  cube_marker.ns = "ada_sm";

  double size = _octree->getNodeSize(_AdaptiveSM_depth);
  cout << "pub adaptive semantic size: " << size << endl;
  cube_marker.scale.x = size;
  cube_marker.scale.y = size;
  cube_marker.scale.z = size;

  int i = 0;
  // Traverse all leafs in the tree:
#if def_octo_generator
  for (ColorOcTree::iterator it = _octree->begin(_AdaptiveSM_depth),
                             end = _octree->end();
       it != end; ++it)
#else
  for (OcTree::iterator it = _octree->begin(_AdaptiveSM_depth),
                             end = _octree->end();
       it != end; ++it)
#endif
  {
    if (_octree->isNodeOccupied(*it))
    {
      // cout << "publishOccAndFree - z: " << z << endl;
      {
        
        cube_marker.pose.position.x = it.getX();
        cube_marker.pose.position.y = it.getY();
        cube_marker.pose.position.z = it.getZ();
        // cout << "pub adaptive semantic POS: " << it.getX() << " " << it.getY() << " " << it.getZ() << endl;

        cube_marker.id = i++;
        
        Eigen::Vector3i _query_color(it->getColor().r, it->getColor().g, it->getColor().b);

        // cout << "pub adaptive semantic color: " << _query_color.x() << " " << _query_color.y() << " " << _query_color.y() << endl;
        int heat = Color_density_map[Node_hash(_query_color.x(), _query_color.y(), _query_color.z())];
        // cout << "pub adaptive semantic heat: " << heat << endl;

        cube_marker.color.r = 255;
        cube_marker.color.g = heat;
        cube_marker.color.b = heat;
        cube_marker.color.a = 1;

        occupiedNodesVis.markers.push_back(cube_marker);
      }
    }
  }

  m_markerAdaptSemanticOctreePub.publish(occupiedNodesVis);
}

void publishAdaptiveOctree()
{
  int _AdaptiveSM_depth = 16;
  // int _m_maxTreeDepth = 15, _m_treeDepth = 15;
  size_t octomapSize = _octree->size();
  if (octomapSize <= 1)
  {
    ROS_WARN("Nothing to publish, octree is empty");
    return;
  }

  pcl::PointCloud<pcl::PointXYZRGB> cloud; 
  sensor_msgs::PointCloud2 output; 

  int i = 0;
  // Traverse all leafs in the tree:
#if def_octo_generator
  for (ColorOcTree::iterator it = _octree->begin(_AdaptiveSM_depth),
                             end = _octree->end();
       it != end; ++it)
#else
  for (OcTree::iterator it = _octree->begin(_AdaptiveSM_depth),
                             end = _octree->end();
       it != end; ++it)
#endif
  {
    if (_octree->isNodeOccupied(*it))
    {
      Eigen::Vector3i _query_color(it->getColor().r, it->getColor().g, it->getColor().b);

      uint8_t heat;
      if (ColorIsFeatureRich(_query_color))
      {
        heat = 128;
      }
      else if(ColorIsFeatureLess(_query_color)){
        heat = 0;
      }
      else{
        heat = Color_density_map[Node_hash(_query_color.x(), _query_color.y(), _query_color.z())]*4;
      }
      // cout << "pub adaptive semantic color: " << _query_color.x() << " " << _query_color.y() << " " << _query_color.y() << endl;
      
      // cout << "pub adaptive semantic heat: " << heat << endl;
      pcl::PointXYZRGB _point = pcl::PointXYZRGB(255, (255 - heat), (255 - heat));

      _point.x = it.getX();
      _point.y = it.getY();
      _point.z = it.getZ() - 0.2;

      cloud.points.push_back(_point);
    }
  }

  //Convert the cloud to ROS message 
  pcl::toROSMsg(cloud, output); 
  output.header.frame_id = "world"; 

  m_markerAdaptSemanticOctreePub.publish(output);
}

// random sample in the map point map, calculate the information of sample cells, return the new octomap with realtime information
void build_realtime_info_semantic_mapping(Eigen::Vector3d cur_position)
{
  if (density_map.size() <= 0)
  {
    ROS_WARN_STREAM("density_map not initialized...");
    return;
  }

  sample_points_color.clear();
  sample_points_info.clear();
  sample_points_online_merge.clear();

  float search_radius_ = 10;
  float search_height_ = 3;
  int sample_info = 0;
  Eigen::Vector3i sample_color;

  float dx, dy, dz;

  int sample_iteration_ = 0;

  int max_iteration = density_map.size() > max_iteration_set ? max_iteration_set : density_map.size();
  cout << "cur_position: " << cur_position.transpose() << endl;
  // sample 100 points around current position
  while (sample_iteration_ < max_iteration)
  {
    dx = ((double)rand() / (double)RAND_MAX) * (search_radius_ + search_radius_) - search_radius_;
    dy = ((double)rand() / (double)RAND_MAX) * (search_radius_ + search_radius_) - search_radius_;
    dz = ((double)rand() / (double)RAND_MAX) * (search_height_ + search_height_) - search_height_;

    // std::cout << "dx: " << dx << " " << dy << " " << dz << " "  << std::endl;

    Eigen::Vector3d point(cur_position.x() + dx, cur_position.y() + dy, cur_position.z() + dz);

    Eigen::Vector3i Index_Density = Get_DnesityMap_Index(point);

    auto res_search = density_map.find(Node_hash(Index_Density.x(), Index_Density.y(), Index_Density.z()));

    octomap::point3d point_position(point.x(), point.y(), point.z());

#if def_octo_generator
    octomap::ColorOcTreeNode *node = _octree->search(point_position);
#else
    octomap::OcTreeNode *node = _octree->search(point_position);
#endif
    if (node != NULL && node->getOccupancy() > 0.5)
    {
      // if(res_search != density_map.end()){
      // std::cout << "Adaptive semantic - sample position: " << point.x() << " " << point.y() << " " << point.z() << " "  << std::endl;
      // std::cout << "search density: " << res_search->second << std::endl;

      // save rgb in float
#if def_octo_generator
      sample_color.x() = node->getColor().r;
      sample_color.y() = node->getColor().g;
      sample_color.z() = node->getColor().b;
#endif
      sample_points_color.push_back(sample_color);
      // std::cout << "Adaptive semantic - sample color: " << sample_color.x() << " " << sample_color.y() << " " << sample_color.z() << " "  << std::endl;

      sample_points_online_merge.push_back(point);
      sample_iteration_++;

      // 以color voxel为主
      if (res_search != density_map.end())
      {
        sample_info = res_search->second;
        sample_points_info.push_back(sample_info);
      }
      else
      {
        sample_info = 0;
        sample_points_info.push_back(sample_info);
      }
      // std::cout << "Adaptive semantic - sample density: " << sample_info << std::endl;
    }
    else
    {
      continue;
    }
  }

  // cout << "sample_points_online_merge.size: " << sample_points_online_merge.size() << endl;
  // cout << "sample_info.size: " << sample_points_info.size() << endl;
  // cout << "sample_color.size: " << sample_points_color.size() << endl;

  vector<int> index_vector_blue;
  vector<int> index_vector_green;
  vector<Eigen::Vector3i> semantic_class_vec;
  vector<vector<int> > semantic_density_vvec;
  // deal with the two vector
  // sort the semantic color of sample points, based on the infomation of sample points
  for (int i = 0; i < max_iteration; ++i)
  {
    // cout << "sample_points_online_merge: " << sample_points_online_merge[i].transpose() << endl;
    // cout << "sample_info: " << sample_points_info[i] << endl;
    // cout << "sample_color: " << sample_points_color[i].transpose() << endl;

    if(!ColorIsFeatureRich(sample_points_color[i]) && !ColorIsFeatureLess(sample_points_color[i])){
      vector<Eigen::Vector3i>::iterator it = find(semantic_class_vec.begin(), semantic_class_vec.end(), sample_points_color[i]);
      if((it == semantic_class_vec.end())){   // not in the vec
        semantic_class_vec.push_back(sample_points_color[i]);
        vector<int> semantic_density_vec(1, sample_points_info[i]);
        semantic_density_vvec.push_back(semantic_density_vec);
      }
      else{   // in the vec
        int _insert_idx = std::distance(semantic_class_vec.begin(), it);
        semantic_density_vvec[_insert_idx].push_back(sample_points_info[i]);
      }
    }
  }

  for(int i = 0; i < semantic_class_vec.size(); ++i){
    if(semantic_density_vvec[i].size() < 5){    // 数量太少则认为是杂点，不进行统计
      semantic_density_vvec.erase(semantic_density_vvec.begin() + i);
      semantic_class_vec.erase(semantic_class_vec.begin() + i);
      i--;
    }
    else{
      sort(semantic_density_vvec[i].begin(), semantic_density_vvec[i].end(), myobject);

      int sum_size = semantic_density_vvec[i].size() > sum_size_set ? sum_size_set : semantic_density_vvec[i].size();
      int semantic_density_sum = accumulate(semantic_density_vvec[i].begin(),       semantic_density_vvec[i].begin() + sum_size, 0);
    
      auto search = Color_density_map.find(Node_hash(semantic_class_vec[i].x(), semantic_class_vec[i].y(), semantic_class_vec[i].z()));
      if (search != Color_density_map.end()) {    // previously exsit
        search->second = search->second * 0.8 + semantic_density_sum * 0.2;
        // cout << "add semantic_color: " << semantic_class_vec[i].transpose() << " semantic_density_sum " << i << " : " << semantic_density_sum << endl;
      } else {    // 
        Color_density_map[Node_hash(semantic_class_vec[i].x(), semantic_class_vec[i].y(), semantic_class_vec[i].z())] = semantic_density_sum;
        // cout << "init semantic_color: " << semantic_class_vec[i].transpose() << " semantic_density_sum " << i << " : " << semantic_density_sum << endl;
      }
    }
  }
  
  // todo 
  // 给不同的栅格赋予不同的颜色
  publishAdaptiveOctree();
}

//
bool isFeatureRich(const uint8_t r, const uint8_t g, const uint8_t b)
{

  // light blue
  if ((r <= (61 + 5)) && (r >= (61 - 5)) && (g <= (230 + 5)) && (g >= (230 - 5)) && (b <= (250 + 5)) && (b >= (250 - 5)))
  {
    return false;
  }
  // green
  else if ((r <= (4 + 8)) && (r >= (4 - 4)) && (g <= (250 + 5)) && (g >= (250 - 5)) && (b <= (7 + 7)) && (b >= (7 - 7)))
  {
    return true;
  }
}

auto make_vector(double beg, double step, double end)
{
  std::vector<double> vec;
  vec.reserve((end - beg) / step + 1);
  while (beg <= end)
  {
    vec.push_back(beg);
    beg += step;
  }
  return vec;
}

// void unordermap_2d_to_gridmap(){

//   // Convert to an occupancy map for visualization
//   nav_msgs::OccupancyGrid mi_map_msg;
//   mi_map_msg.header = map_header;
//   mi_map_msg.info = map_info;
//   mi_map_msg.data = std::vector<int8_t>(fsmi_data.size());
//   double mi_max = *std::max_element(fsmi_data.begin(), fsmi_data.end());
//   std::cout << "mi_max:" << mi_max << std::endl;
//   for (size_t i = 0; i < fsmi_data.size(); i++) {
//     // Normalize between 0 and 1
//     double normalized = fsmi_data[i]/mi_max;
//     std::cout << "mi_max normalized:" << normalized << std::endl;
//     // Change to 100%
//     mi_map_msg.data[i] = 100 * (1 - normalized);
//     // std::cout << "mi_map_msg.data:" << int(mi_map_msg.data[i]) << std::endl;
//   }
//   mi_map_pub.publish(mi_map_msg);


// }