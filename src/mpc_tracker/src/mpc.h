#ifndef MPC_H
#define MPC_H
#include <log4cxx/logger.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <vector>
#include "Eigen/Core"
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include "gaussian_process_regression/gaussian_process_regression.h"

using namespace std;

#define def_Using_Quaternion 0

#define def_pred_terrain 0
#define def_Using_GP 0    // 记录数据


extern double dt;
extern double max_v;
extern double max_w_inw;
extern nav_msgs::OccupancyGrid Height_map;
const int poly_order = 3;
extern float polyfit_result[poly_order];

extern double _ref_base_yaw;
extern double _ref_base_yaw_last;
extern double cur_cam_roll, cur_cam_pitch, cur_cam_yaw; //定义存储r\p\y的容器
extern double cur_base_roll, cur_base_pitch, cur_base_yaw, cur_base_yaw_origin; //定义存储r\p\y的容器
extern Eigen::Vector3d g_position_fb;
extern Eigen::Vector3d g_orientation_fb;

extern vector<Eigen::Vector3d> test_pos_visual;
extern vector<Eigen::Vector3d> test_rot_visual;

extern GaussianProcessRegression<double > gpr;
typedef Eigen::Matrix<double, 4, 1> input_type;
typedef Eigen::Matrix<double, 2, 1> output_type;
// vector<input_type> train_inputs, test_inputs;
// vector<output_type> train_outputs, test_outputs;


class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.

  Eigen::MatrixXd Solve(Eigen::VectorXd state, Eigen::VectorXd ref_wp, size_t Predict_steps);

  std::vector<double> x_pred_vals;
  std::vector<double> y_pred_vals;
  std::vector<double> theta_b_pred_vals;
  std::vector<double> theta_bc_pred_vals;
  std::vector<double> theta_c_pred_vals;
  std::vector<double> pitch_b_pred_vals;
  std::vector<double> pitch_bc_pred_vals;
  std::vector<double> pitch_c_pred_vals;

  std::vector<Eigen::Quaternion<double>> quat_pred;

};

#endif /* MPC_H */