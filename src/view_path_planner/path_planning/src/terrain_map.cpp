#include <ros/ros.h>
#include <ros/console.h>
#include <log4cxx/logger.h>
#include <nav_msgs/OccupancyGrid.h>
#include <algorithm>    // std::min_element, std::max_element

#include "../include/terrain_map.h"
#include <chrono>
#include <fstream>

#define drg2rad (M_PI/180)

// 局部地形建图区域范围
const int Map_range = 10;
// const int8_t Initual_map_value = -20;
const int8_t Initual_map_value = -50;   // 
const int N_S = 24;  // 8邻域

const int N_S_H_dev = 8;

const int Height_map_Scale = 25;    // -127~127

// vector<Eigen::Vector3d> rrt3d_sample_position;

// vector<float> rrt3d_sample_point_gain;

GridIndex terrain_mapping::World2GridIndex(Eigen::Vector2d pos)
{
  GridIndex index;
  // ceil返回大于等于该数的整数值，与floor相反
  index.x() = std::ceil((pos.x() - Height_map.info.origin.position.x) / Height_map.info.resolution);
  index.y() = std::ceil((pos.y() - Height_map.info.origin.position.y) / Height_map.info.resolution);

  return index;
}

terrain_mapping::terrain_mapping(Eigen::Vector3d _cur_position, bool smooth_flag)
{
  cout << "terrain_mapping constructed" << endl;
  terrain_cell_set.clear();
  Init_Height_map(_cur_position);

  Init_Height_deviation_map(_cur_position);
  Init_Slope_map(_cur_position);
  Init_Curvature_map(_cur_position);

  Extract_Height_data(_cur_position);

  if(smooth_flag){
    Update_Height_Maps(_cur_position);
  }
  else{
    Update_Height_Maps_track(_cur_position);
  }
  

  Update_Height_deviation_map();

  // Update_Slope_map();
}

terrain_mapping::~terrain_mapping(){

}


void terrain_mapping::Init_Height_deviation_map(Eigen::Vector3d cur_position_){
    // init projected 2D map:
  Height_deviation_map.header.frame_id = "/world";
  Height_deviation_map.header.stamp = ros::Time::now();

  Height_deviation_map.info.width = int(Map_range*2/_octree->getResolution()) + 1;
  Height_deviation_map.info.height = int(Map_range*2/_octree->getResolution()) + 1;
  // ROS_WARN_STREAM("Height_deviation_map.info.width: " << Height_deviation_map.info.width << " Height_deviation_map.info.height: " << Height_deviation_map.info.height);

  // might not exactly be min / max of octree:
  Height_deviation_map.info.resolution = _octree->getResolution();
  Height_deviation_map.info.origin.position.x = cur_position_.x() - Map_range;
  Height_deviation_map.info.origin.position.y = cur_position_.y() - Map_range;
  // ROS_WARN_STREAM("Rebuilding complete 2D map");
  // ROS_WARN_STREAM("Height_deviation_map.origin: " << Height_deviation_map.info.origin.position.x << " " << Height_deviation_map.info.origin.position.y);

  // gridmap_l.data.clear();
  Height_deviation_map.data.resize(Height_deviation_map.info.width * Height_deviation_map.info.height, Initual_map_value);

}

void terrain_mapping::Init_Height_map(Eigen::Vector3d cur_position_){
    // init projected 2D map:
  Height_map.header.frame_id = "/world";
  Height_map.header.stamp = ros::Time::now();

  Height_map.info.width = int(Map_range*2/_octree->getResolution()) + 1;
  Height_map.info.height = int(Map_range*2/_octree->getResolution()) + 1;
  // ROS_WARN_STREAM("Height_map.info.width: " << Height_map.info.width << " Height_map.info.height: " << Height_map.info.height);

  // might not exactly be min / max of octree:
  Height_map.info.resolution = _octree->getResolution();
  Height_map.info.origin.position.x = cur_position_.x() - Map_range;
  Height_map.info.origin.position.y = cur_position_.y() - Map_range;
  // ROS_WARN_STREAM("Rebuilding complete 2D map");
  // ROS_WARN_STREAM("Height_map.origin: " << Height_map.info.origin.position.x << " " << Height_map.info.origin.position.y);

  // gridmap_l.data.clear();
  Height_map.data.resize(Height_map.info.width * Height_map.info.height, Initual_map_value);

}


void terrain_mapping::Init_Slope_map(Eigen::Vector3d cur_position_){
    // init projected 2D map:
  Slope_map.header.frame_id = "/world";
  Slope_map.header.stamp = ros::Time::now();

  Slope_map.info.width = int(Map_range*2/_octree->getResolution()) + 1;
  Slope_map.info.height = int(Map_range*2/_octree->getResolution()) + 1;
  // ROS_WARN_STREAM("Slope_map.info.width: " << Slope_map.info.width << " Slope_map.info.height: " << Slope_map.info.height);

  // might not exactly be min / max of octree:
  Slope_map.info.resolution = _octree->getResolution();
  Slope_map.info.origin.position.x = cur_position_.x() - Map_range;
  Slope_map.info.origin.position.y = cur_position_.y() - Map_range;
  // ROS_WARN_STREAM("Rebuilding complete 2D map");
  // ROS_WARN_STREAM("Slope_map.origin: " << Slope_map.info.origin.position.x << " " << Slope_map.info.origin.position.y);

  // gridmap_l.data.clear();
  Slope_map.data.resize(Slope_map.info.width * Slope_map.info.height, Initual_map_value);

}

void terrain_mapping::Init_Curvature_map(Eigen::Vector3d cur_position_){
    // init projected 2D map:
  Curvature_map.header.frame_id = "/world";
  Curvature_map.header.stamp = ros::Time::now();

  Curvature_map.info.width = int(Map_range*2/_octree->getResolution()) + 1;
  Curvature_map.info.height = int(Map_range*2/_octree->getResolution()) + 1;
  // ROS_WARN_STREAM("Curvature_map.info.width: " << Curvature_map.info.width << " Curvature_map.info.height: " << Curvature_map.info.height);

  // might not exactly be min / max of octree:
  Curvature_map.info.resolution = _octree->getResolution();
  Curvature_map.info.origin.position.x = cur_position_.x() - Map_range;
  Curvature_map.info.origin.position.y = cur_position_.y() - Map_range;
  // ROS_WARN_STREAM("Rebuilding complete 2D map");
  // ROS_WARN_STREAM("Curvature_map.origin: " << Curvature_map.info.origin.position.x << " " << Curvature_map.info.origin.position.y);

  // gridmap_l.data.clear();
  Curvature_map.data.resize(Curvature_map.info.width * Curvature_map.info.height, Initual_map_value);

}

double _init_z = Initual_map_value/Height_map_Scale;
double m_min_z_local = 1e5;
void terrain_mapping::Extract_Height_data(Eigen::Vector3d cur_position){
  auto start = std::chrono::high_resolution_clock::now();

  // double x_search_min = cur_position.x() - Map_range + Height_map.info.resolution*0.5;
  // double x_search_max = cur_position.x() + Map_range - Height_map.info.resolution*0.5;
  // double y_search_min = cur_position.y() - Map_range + Height_map.info.resolution*0.5;
  // double y_search_max = cur_position.y() + Map_range - Height_map.info.resolution*0.5;
  double x_search_min = cur_position.x() - Map_range + Height_map.info.resolution*0;
  double x_search_max = cur_position.x() + Map_range - Height_map.info.resolution*0;
  double y_search_min = cur_position.y() - Map_range + Height_map.info.resolution*0;
  double y_search_max = cur_position.y() + Map_range - Height_map.info.resolution*0;

  m_min_z_local = 1e5;

  for (double x_search = x_search_max; x_search >= x_search_min; x_search -= Height_map.info.resolution)
  {
    for (double y_search = y_search_max; y_search >= y_search_min; y_search -= Height_map.info.resolution)
    {
      double z_min = cur_position.z() - 2;
      double z_max = cur_position.z() + 5;
      double cell_z_max = z_min;

      bool _occupied_flag = false;
      for(double z_search = z_max; z_search >= z_min; z_search -= Height_map.info.resolution){
        octomap::point3d temp(x_search, y_search, z_search);
        octomap::OcTreeNode *node = _octree->search(temp);        
        if (node != NULL && _octree->isNodeOccupied(node))
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
      
      if(_occupied_flag && cell_z_max < m_min_z_local){
        m_min_z_local = cell_z_max;
        // cout << "m_min_z_local: " << m_min_z_local << endl;
      }

      // cout << "cell_z_max given: " << cell_z_max << endl;

      if(_occupied_flag){
        Eigen::Vector3d cell_xyz(x_search, y_search, cell_z_max);
        // ROS_WARN_STREAM("cell_xyz: " << cell_xyz);
        terrain_cell_set.push_back(cell_xyz);
      }
      else{
        Eigen::Vector3d cell_xyz(x_search, y_search, _init_z);
        // ROS_WARN_STREAM("cell_xyz: " << cell_xyz);
        terrain_cell_set.push_back(cell_xyz);
      }

      // // 若搜到占据
      // if(abs(cell_z_max - z_min) > 0.05){
      //   Eigen::Vector3d cell_xyz(x_search, y_search, cell_z_max);
      //   // ROS_WARN_STREAM("cell_xyz: " << cell_xyz);
      //   terrain_cell_set.push_back(cell_xyz);
      // }
    }
  }
  auto stop = std::chrono::high_resolution_clock::now();
  auto timespan = std::chrono::duration<double>(stop - start);
  // std::cout << "it took " << timespan.count() << " seconds.";
  ROS_WARN_STREAM("Extract Height takes: " << timespan.count()*1000 << " ms");
}

bool compare_func_max(Eigen::Vector3d a, Eigen::Vector3d b){
  return a.z() < b.z();
}

bool compare_func_min(Eigen::Vector3d a, Eigen::Vector3d b){
  return a.z() > b.z();
}

void terrain_mapping::get_neighbours(int n_array[], int index) {
	n_array[0] = index - Height_map.info.width;
	n_array[1] = index + Height_map.info.width; 
	n_array[2] = index - Height_map.info.width - 1; 
  n_array[3] = index - Height_map.info.width + 1; 
  n_array[4] = index - 1; 
	n_array[5] = index + 1;
	n_array[6] = index + Height_map.info.width + 1;
	n_array[7] = index + Height_map.info.width - 1;
}

void terrain_mapping::Update_Height_Maps(Eigen::Vector3d cur_position_){
  auto start = std::chrono::high_resolution_clock::now();

  // cout << "terrain_cell_set.size: " << terrain_cell_set.size() << endl;
  // cout << "m_min_z_local: " << m_min_z_local << endl;
  // cout << "_init_z: " << _init_z << endl;

  for(auto cell:terrain_cell_set){
    Eigen::Vector2d cell_xy(cell.x(), cell.y());
    Eigen::Vector2i node_index = World2GridIndex(cell_xy);
    // cout << "terrain_cell_set: " << cell.transpose() << endl;

    int Idx = mapIdx(node_index.x() - 1, node_index.y() - 1);

    // cout << "cell_z_norm: " << cell_z_norm << endl;
    // Height_map.data[Idx] = floor((cell_z_norm));    // else is the initual value 0
    // double _init_z = Initual_map_value/Height_map_Scale;
    if(abs(cell.z() - _init_z) < 5e-2){
      Height_map.data[Idx] = floor((m_min_z_local*Height_map_Scale));    // else is the initual value 0
    }
    else{
      Height_map.data[Idx] = floor((cell.z()*Height_map_Scale));    // else is the initual value 0
    }
  }

  for(int i = 0; i < Height_map.info.height; ++i){
    for(int j = 0; j < Height_map.info.width; ++j){
      int Idx = mapIdx(i, j);

      if((abs(Height_map.data[Idx] - floor(_init_z*Height_map_Scale)) < 5e-2 || abs(Height_map.data[Idx] - floor(m_min_z_local*Height_map_Scale)) < 5e-2)){
        // ROS_WARN_STREAM("Void grid process: Before Hight " << int(Height_map.data[Idx]));
        int locations[N_S_H_dev]; 
        get_neighbours(locations, Idx);

        // 均值滤波
        int count = 0;
        double neighbor_sum = 0;
        for(int k = 0; k < N_S_H_dev; ++k){
          if(locations[k] < Height_map.info.width*Height_map.info.height){
            neighbor_sum += Height_map.data[locations[k]];
            count++;
          }
        }
        double Avg_neighbor;
        Avg_neighbor = neighbor_sum/count;
        Height_map.data[Idx] = Avg_neighbor;

        // // 众数
        // int count = 1;
        // vector<int> neighbor_height;
        // for(int k = 0; k < N_S_H_dev; ++k){
        //   if(locations[k] < Height_map.info.width*Height_map.info.height){
        //     neighbor_height.push_back(Height_map.data[locations[k]]);
        //     cout << "Update_Height_Maps - neighbor height: " << int(Height_map.data[locations[k]]) << endl;
        //   }
        // }
        // sort(neighbor_height.begin(), neighbor_height.end());
        // int maj = neighbor_height[0];
        // for(int k = 1; k < neighbor_height.size(); ++k){
        //   if(abs(maj - neighbor_height[k]) < 5e-2)
        //     count++;
        //   else{
        //     count--;
        //     if(count==0){
        //       maj=neighbor_height[k+1];
        //     }
        //   }
        // }
        // Height_map.data[Idx] = maj;
        // cout << "Update_Height_Maps - majority neighbor height: " << maj << endl;
        // ROS_WARN_STREAM("Void grid process: After Hight " << int(Height_map.data[Idx]));
      }
    }
  }
  auto stop = std::chrono::high_resolution_clock::now();
  auto timespan = std::chrono::duration<double>(stop - start);
  // std::cout << "it took " << timespan.count() << " seconds.";
  ROS_WARN_STREAM("Update_Height_Maps takes: " << timespan.count()*1000 << " ms");
}


void terrain_mapping::Update_Height_Maps_track(Eigen::Vector3d cur_position_){
  auto start = std::chrono::high_resolution_clock::now();

  // cout << "terrain_cell_set.size: " << terrain_cell_set.size() << endl;
  // cout << "m_min_z_local: " << m_min_z_local << endl;
  // cout << "_init_z: " << _init_z << endl;

  for(auto cell:terrain_cell_set){
    Eigen::Vector2d cell_xy(cell.x(), cell.y());
    Eigen::Vector2i node_index = World2GridIndex(cell_xy);
    // cout << "terrain_cell_set: " << cell.transpose() << endl;

    int Idx = mapIdx(node_index.x() - 1, node_index.y() - 1);

    // cout << "cell_z_norm: " << cell_z_norm << endl;
    // Height_map.data[Idx] = floor((cell_z_norm));    // else is the initual value 0
    // double _init_z = Initual_map_value/Height_map_Scale;
    if(abs(cell.z() - _init_z) < 5e-2){
      Height_map.data[Idx] = floor((_init_z*Height_map_Scale));    // else is the initual value 0
      // cout << "origin height:" << _init_z << " int: " << floor(Height_map.data[Idx]) << endl;
    }
    else{
      Height_map.data[Idx] = floor((cell.z()*Height_map_Scale));    // else is the initual value 0
      // cout << "origin height:" << cell.z() << " int: " << floor(Height_map.data[Idx]) << endl;
    }
  }

  auto stop = std::chrono::high_resolution_clock::now();
  auto timespan = std::chrono::duration<double>(stop - start);
  // std::cout << "it took " << timespan.count() << " seconds.";
  ROS_WARN_STREAM("Update_Height_Maps_track takes: " << timespan.count()*1000 << " ms");
}

void terrain_mapping::Update_Height_Maps_wNorm(Eigen::Vector3d cur_position_){

  cout << "terrain_cell_set.size: " << terrain_cell_set.size() << endl;
  auto max_vec = *max_element(terrain_cell_set.begin(), terrain_cell_set.end(), compare_func_max);
  // cout << "max_vec: " << max_vec.transpose() << endl;

  auto min_vec = *min_element(terrain_cell_set.begin(), terrain_cell_set.end(), compare_func_max);
  // cout << "min_vec: " << min_vec.transpose() << endl;

  for(auto cell:terrain_cell_set){
    Eigen::Vector2d cell_xy(cell.x(), cell.y());
    Eigen::Vector2i node_index = World2GridIndex(cell_xy);
    // cout << "terrain_cell_set: " << cell.transpose() << endl;

    int Idx = mapIdx(node_index.x() - 1, node_index.y() - 1);

    // cout << "cell_z: " << cell.z() << endl;
    // 归一化
    double cell_z_norm;
    if(abs(max_vec.z() - min_vec.z()) < 0.05){
      cell_z_norm = (2*(cell.z() - min_vec.z()) - 1)*127;
    }
    else{
      cell_z_norm = (2*(cell.z() - min_vec.z())/(max_vec.z() - min_vec.z()) - 1)*127;
    }    

    // cout << "cell_z_norm: " << cell_z_norm << endl;
    // Height_map.data[Idx] = floor((cell_z_norm));    // else is the initual value 0
    // double _init_z = Initual_map_value/Height_map_Scale;
    if(abs(cell.z() - _init_z) < 5e-2){
      Height_map.data[Idx] = floor((m_min_z_local*Height_map_Scale));    // else is the initual value 0
    }
    else{
      Height_map.data[Idx] = floor((cell.z()*Height_map_Scale));    // else is the initual value 0
    }
  }
}

void terrain_mapping::Update_Height_deviation_map(){
  auto start = std::chrono::high_resolution_clock::now();
        
  // cout << "Height_deviation Height_map.info.resolution: " << Height_map.info.resolution << endl;

  for(int i = 0; i < Height_map.info.height; ++i){
    for(int j = 0; j < Height_map.info.width; ++j){
      int Idx = mapIdx(i, j);

      int locations[N_S_H_dev]; 
      get_neighbours(locations, Idx);

      int count = 0;
      double neighbor_devsum = 0;
      for(int k = 0; k < N_S_H_dev; ++k){
        if(locations[k] < Height_map.info.width*Height_map.info.height){
          neighbor_devsum += (Height_map.data[Idx] - Height_map.data[locations[k]]);
          count++;
        }
      }

      double AvgDev_neighbor;
      AvgDev_neighbor = neighbor_devsum/count;

      if(AvgDev_neighbor/Height_map_Scale <= Height_map.info.resolution*2){
        // cout << "Height_deviation: " << AvgDev_neighbor/Height_map_Scale << endl;
        AvgDev_neighbor = 0;
      }
      // else{
      //   cout << "Height_deviation: " << AvgDev_neighbor/Height_map_Scale << endl;
      // }

      Height_deviation_map.data[Idx] = floor((AvgDev_neighbor));
      // cout << "Height_deviation: " << floorAvgDev_neighbor << endl;    
    }
  }

  auto stop = std::chrono::high_resolution_clock::now();
  auto timespan = std::chrono::duration<double>(stop - start);
  // std::cout << "it took " << timespan.count() << " seconds.";
  ROS_WARN_STREAM("Update Height Deviation Map takes: " << timespan.count()*1000 << " ms");
}


// const int N_S = 24;  // 邻域 5^2-1

GridIndex terrain_mapping::World2GridIndex(double pos_x, double pos_y) {
  GridIndex index;

  // ceil返回大于等于该数的整数值，与floor相反
  index.x() = std::ceil((pos_x - Height_map.info.origin.position.x) /
                        Height_map.info.resolution);
  index.y() = std::ceil((pos_y - Height_map.info.origin.position.y) /
                        Height_map.info.resolution);

  return index;
}

void terrain_mapping::get_neighbours(vector<Eigen::Vector3d>& neibor_pos, double pos_x, double pos_y) {  
  Eigen::Vector3d neighbor_position;
  Eigen::Vector2i neighbor_index;

  int N_extend = sqrt(N_S + 1);
  // cout << "N_extend: " << N_extend << endl;

  for(int i = -N_extend/2; i <= N_extend/2; ++i){
    for(int j = -N_extend/2; j <= N_extend/2; ++j){
      // cout << "i " << i << " j " << j << endl;
      if(i == 0 && j == 0){
        continue;
      }
      else{
        neighbor_position.x() = pos_x + Height_map.info.resolution * i;
        neighbor_position.y() = pos_y + Height_map.info.resolution * j;

        neighbor_index = World2GridIndex(neighbor_position.x(), neighbor_position.y());

        neighbor_position.z() = (float)(Height_map.data[neighbor_index.x() + neighbor_index.y() * Height_map.info.width])/Height_map_Scale;
        // cout << "neigh_pos:  " << neighbor_position.transpose() << endl;
        neibor_pos.push_back(neighbor_position);
      }
    }
  }
}


Eigen::Vector3d terrain_mapping::get_normal_vector(double center_x, double center_y) {

  Eigen::Vector2d center_position(center_x, center_y);
  Eigen::Vector2i center_index = World2GridIndex(center_position);
  // cout << "center_index: " << center_index.transpose() << endl;
  //计算中心点坐标
  double center_z = 0;

  // center_x = x;
  // center_y = y;
  center_z = (float)(Height_map.data[center_index.x() + center_index.y() * Height_map.info.width])/Height_map_Scale;
  // cout << "center_position: " << center_x << " " << center_y << " " << center_z << endl;

  // double locations_x[N_S], locations_y[N_S], locations_z[N_S]; 
  // get_neighbours(locations_x, locations_y, locations_z, center_x, center_y);
  vector<Eigen::Vector3d> neighbors;
  get_neighbours(neighbors, center_x, center_y);

  //计算协方差矩阵
  double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
  for (int i = 0; i < neighbors.size(); i++) {
    // cout << "neighbor " << i << " : " << neighbors[i].x() << " " << neighbors[i].y() << " " << neighbors[i].z() << endl;
    xx += (neighbors[i].x() - center_x) * (neighbors[i].x() - center_x);
    xy += (neighbors[i].x() - center_x) * (neighbors[i].y() - center_y);
    xz += (neighbors[i].x() - center_x) * (neighbors[i].z() - center_z);
    yy += (neighbors[i].y() - center_y) * (neighbors[i].y() - center_y);
    yz += (neighbors[i].y() - center_y) * (neighbors[i].z() - center_z);
    zz += (neighbors[i].z() - center_z) * (neighbors[i].z() - center_z);
  }
  //大小为3*3的协方差矩阵
  Eigen::Matrix3f covMat(3, 3);
  covMat(0, 0) = xx / N_S;
  covMat(0, 1) = covMat(1, 0) = xy / N_S;
  covMat(0, 2) = covMat(2, 0) = xz / N_S;
  covMat(1, 1) = yy / N_S;
  covMat(1, 2) = covMat(2, 1) = yz / N_S;
  covMat(2, 2) = zz / N_S;

  //求特征值与特征向量
  Eigen::EigenSolver<Eigen::Matrix3f> es(covMat);
  Eigen::Matrix3f val = es.pseudoEigenvalueMatrix();
  Eigen::Matrix3f vec = es.pseudoEigenvectors();

  //找到最小特征值t1
  double t1 = val(0, 0);
  int ii = 0;
  if (t1 > val(1, 1)) {
    ii = 1;
    t1 = val(1, 1);
  }
  if (t1 > val(2, 2)) {
    ii = 2;
    t1 = val(2, 2);
  }

  //最小特征值对应的特征向量v_n
  Eigen::Vector3d v(vec(0, ii), vec(1, ii), vec(2, ii));
  // cout << "norm vec: " << v.transpose() << endl;
  //特征向量单位化
  v /= v.norm();
  
  if(v.z() < 0){
    v.x() = -v.x();
    v.y() = -v.y();
    v.z() = -v.z();
  }

  return v;
}


Eigen::Vector2d terrain_mapping::get_roll_pitch(double x, double y, double theta_base) {
  // 根据height map计算pca法向量
  Eigen::Vector3d norm_vec_w = get_normal_vector(x, y);
  cout << "norm_vec_w: " << norm_vec_w.transpose() << endl;

  // 3.0 初始化欧拉角(Z-Y-X，即RPY, 先绕x轴roll,再绕y轴pitch,最后绕z轴yaw)
  Eigen::Vector3d ea(0, 0, theta_base);
  cout << "theta_base: " << theta_base << endl;
  // 3.1 欧拉角转换为旋转矩阵
  Eigen::Matrix3d rotation_matrix3;
  rotation_matrix3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitX()) *
                     Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitZ());
  // cout << "rotation matrix3 =\n" << rotation_matrix3 << endl;

  Eigen::Vector3d norm_vec_r = rotation_matrix3 * norm_vec_w;
  cout << "norm_vec_r: " << norm_vec_r.transpose() << endl;

  double roll = atan2(norm_vec_r.y(), norm_vec_r.z());  // 左侧朝上为负值
  cout << "roll: " << roll << endl;

  double pitch = atan2(norm_vec_r.x(), norm_vec_r.z());  // 朝上为负值
  cout << "pitch: " << pitch << endl;

  Eigen::Vector2d roll_pitch_vect(roll, pitch);
  return roll_pitch_vect;
}


void terrain_mapping::Update_Slope_map(){
  auto start = std::chrono::high_resolution_clock::now();

  // double x = 5;
  // double y = -2;
  for(double x = 0; x < 10; x+=Height_map.info.resolution*8)
  {
    for(double y = -5; y < 5; y+=Height_map.info.resolution*8)
    {
      Eigen::Vector3d norm_vec_w = get_normal_vector(x, y);
      // cout << "norm_vec_w: " << norm_vec_w.transpose() << endl;
      // cout << "pos : " << x << " " << y << endl;
      // //3.0 初始化欧拉角(Z-Y-X，即RPY, 先绕x轴roll,再绕y轴pitch,最后绕z轴yaw)
      // Eigen::Vector3d ea(0, 0, 0);
      // //3.1 欧拉角转换为旋转矩阵
      // Eigen::Matrix3d rotation_matrix3;
      // rotation_matrix3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitX()) * 
      //                     Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) * 
      //                     Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitZ());
      // // cout << "rotation matrix3 =\n" << rotation_matrix3 << endl;   

      // Eigen::Vector3d norm_vec_r = rotation_matrix3 * norm_vec_w;
      // cout << "norm_vec_r: " << norm_vec_r.transpose() << endl;

      // double pitch = atan2(norm_vec_r.x(), norm_vec_r.z());
      // cout << "pitch: " << pitch << endl;

      // double roll = atan2(norm_vec_r.y(), norm_vec_r.z());
      // cout << "roll: " << roll << endl;


      Eigen::Vector3d terrain_norm_center(x, y, (float)(Height_map.data[World2GridIndex(x,y).x() + World2GridIndex(x,y).y() * Height_map.info.width])/Height_map_Scale);
      terrain_norm_center_list.push_back(terrain_norm_center);

      terrain_norm_list.push_back(norm_vec_w);
    }
  }


  auto stop = std::chrono::high_resolution_clock::now();
  auto timespan = std::chrono::duration<double>(stop - start);
  // std::cout << "it took " << timespan.count() << " seconds.";
  ROS_WARN_STREAM("Update Height Deviation Map takes: " << timespan.count()*1000 << " ms");
}

