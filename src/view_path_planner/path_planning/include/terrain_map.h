#ifndef TERRAIN_MAP_H 
#define TERRAIN_MAP_H

#include <log4cxx/logger.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <chrono>
#include <fstream>

#include "../include/fsmi_cal.h"


class terrain_mapping {
public:
  terrain_mapping(Eigen::Vector3d cur_position_, bool smooth_flag);

  ~terrain_mapping();

  nav_msgs::OccupancyGrid Get_Height_map(){return Height_map;}

  nav_msgs::OccupancyGrid Get_Height_deviation_map(){return Height_deviation_map;}
  nav_msgs::OccupancyGrid Get_Slope_map(){return Slope_map;}
  nav_msgs::OccupancyGrid Get_Curvature_map(){return Curvature_map;}

  vector<Eigen::Vector3d> terrain_norm_center_list;
  vector<Eigen::Vector3d> terrain_norm_list;
  nav_msgs::OccupancyGrid Height_deviation_map;

protected:
  nav_msgs::OccupancyGrid Height_map;

  nav_msgs::OccupancyGrid Slope_map;
  nav_msgs::OccupancyGrid Curvature_map;
  Eigen::Vector2d get_roll_pitch(double x, double y, double theta_base);

  GridIndex World2GridIndex(Eigen::Vector2d pos);
  GridIndex World2GridIndex(double pos_x, double pos_y);
  int mapIdx(int i, int j) {  return Height_map.info.width * j + i; }


private:
  void Init_Height_map(Eigen::Vector3d cur_position_);

  void Init_Height_deviation_map(Eigen::Vector3d cur_position_);
  void Init_Slope_map(Eigen::Vector3d cur_position_);
  void Init_Curvature_map(Eigen::Vector3d cur_position_);

  void Extract_Height_data(Eigen::Vector3d cur_position_);

  // bool compare_func(Eigen::Vector3d a, Eigen::Vector3d b);

  void Update_Height_Maps(Eigen::Vector3d cur_position_);
  void Update_Height_Maps_track(Eigen::Vector3d cur_position_);
  void Update_Height_Maps_wNorm(Eigen::Vector3d cur_position_);
  void Update_Height_deviation_map();
  void Update_Slope_map();
  void get_neighbours(vector<Eigen::Vector3d>& neibor_pos, double pos_x, double pos_y); 
  Eigen::Vector3d get_normal_vector(double center_x, double center_y);

  void get_neighbours(int n_array[], int index);

  vector<Eigen::Vector3d> terrain_cell_set;
};
#endif