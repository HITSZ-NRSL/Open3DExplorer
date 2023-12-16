#include "mpc.h"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <chrono>

#include "Eigen/Core"
using CppAD::AD;

typedef Eigen::Vector2i GridIndex;
typedef CPPAD_TESTVECTOR(double) Dvector;


using CppAD::NearEqual;
using Eigen::Matrix;
using Eigen::Dynamic;
//
typedef Matrix< double     , Dynamic, Dynamic > ei_matrix;
typedef Matrix< AD<double> , Dynamic, Dynamic > ad_matrix;
//
typedef Matrix< double ,     Dynamic , 1>       ei_vector;
typedef Matrix< AD<double> , Dynamic , 1>       ad_vector;

GaussianProcessRegression<double > gpr(4, 2);

double dt = 0.2;
// double max_v = 0.6;
// double max_gama = 0.6;

// Terrain
double max_v = 0.6;
double max_w_base = 0.4;
double max_gama = 0.4;    // gimbal yaw velocity
double max_w_inw = 0.4;
double max_beta = 0.4;

// const int N_S = 8;  // 邻域 3^2-1
// const int N_S = 24;               // 邻域 5^2-1
// const int N_S = 48;               // 邻域 7^2-1
const int N_S = 80;               // 邻域 9^2-1
const int Height_map_Scale = 25;  // -127~127
float m_min_height = -2;

double _ref_base_yaw;
double _ref_base_yaw_last;

double cur_cam_roll, cur_cam_pitch, cur_cam_yaw; //定义存储r\p\y的容器
double cur_base_roll, cur_base_pitch, cur_base_yaw, cur_base_yaw_origin; //定义存储r\p\y的容器
Eigen::Vector3d g_position_fb(0, 0, 0);
Eigen::Vector3d g_orientation_fb(0, 0, 0);

nav_msgs::OccupancyGrid Height_map;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.

size_t N = 10;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
// state
size_t x_start = 0;
size_t y_start = x_start + N;
size_t theta_b_start = y_start + N;        // base orientation
size_t theta_bc_start = theta_b_start + N;  // camera orientation yaw
size_t pitch_b_start = theta_bc_start + N;  // camera orientation pitch
size_t pitch_bc_start = pitch_b_start + N;  // camera orientation pitch
size_t roll_b_start = pitch_bc_start + N;  // camera orientation pitch

// control input
size_t v_start = roll_b_start + N;
size_t w_start = v_start + N - 1;
size_t gama_start = w_start + N - 1;
size_t beta_start = gama_start + N - 1;

vector<double> ref_base_yaw_vec;

GridIndex World2GridIndex_mpc(Eigen::Vector2d pos) {
  GridIndex index;

  // ceil返回大于等于该数的整数值，与floor相反
  index.x() = std::ceil((pos.x() - Height_map.info.origin.position.x) /
                        Height_map.info.resolution);
  index.y() = std::ceil((pos.y() - Height_map.info.origin.position.y) /
                        Height_map.info.resolution);

  return index;
}

GridIndex World2GridIndex_mpc(double pos_x, double pos_y) {
  GridIndex index;

  // ceil返回大于等于该数的整数值，与floor相反
  index.x() = std::ceil((pos_x - Height_map.info.origin.position.x) /
                        Height_map.info.resolution);
  index.y() = std::ceil((pos_y - Height_map.info.origin.position.y) /
                        Height_map.info.resolution);

  return index;
}


/**
 * Compute the quaternion corresponding to euler angles zyx
 *
 * @param [in] eulerAnglesZyx
 * @return The corresponding quaternion
 */
template <typename SCALAR_T>
Eigen::Quaternion<SCALAR_T> getQuaternionFromEulerAnglesZYX(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesZyx) {
  // clang-format off
  return Eigen::AngleAxis<SCALAR_T>(eulerAnglesZyx(0), Eigen::Matrix<SCALAR_T, 3, 1>::UnitZ()) *
        Eigen::AngleAxis<SCALAR_T>(eulerAnglesZyx(1), Eigen::Matrix<SCALAR_T, 3, 1>::UnitY()) *
        Eigen::AngleAxis<SCALAR_T>(eulerAnglesZyx(2), Eigen::Matrix<SCALAR_T, 3, 1>::UnitX());
  // clang-format on
}

/**
 * Compute the quaternion corresponding to euler angles zyx
 *
 * @param [in] eulerAnglesZyx
 * @return The corresponding quaternion
 */
template <typename SCALAR_T>
Eigen::Quaternion<SCALAR_T> getQuaternionFromEulerAnglesXYZ(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesZyx) {
  // clang-format off
  return Eigen::AngleAxis<SCALAR_T>(eulerAnglesZyx(0), Eigen::Matrix<SCALAR_T, 3, 1>::UnitX()) *
        Eigen::AngleAxis<SCALAR_T>(eulerAnglesZyx(1), Eigen::Matrix<SCALAR_T, 3, 1>::UnitY()) *
        Eigen::AngleAxis<SCALAR_T>(eulerAnglesZyx(2), Eigen::Matrix<SCALAR_T, 3, 1>::UnitZ());
  // clang-format on
}

/**
 * Compute the quaternion corresponding to euler angles zyx
 *
 * @param [in] eulerAnglesZyx
 * @return The corresponding quaternion
 */
template <typename SCALAR_T>
Eigen::Quaternion<SCALAR_T> getQuatFromEuler(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesZyx) {
  SCALAR_T halfYaw = SCALAR_T(eulerAnglesZyx(1)) * SCALAR_T(0.5);  
  SCALAR_T halfPitch = SCALAR_T(eulerAnglesZyx(0)) * SCALAR_T(0.5);  
  SCALAR_T halfRoll = SCALAR_T(eulerAnglesZyx(2)) * SCALAR_T(0.5);  
  SCALAR_T cosYaw = CppAD::cos(halfYaw);
  SCALAR_T sinYaw = CppAD::sin(halfYaw);
  SCALAR_T cosPitch = CppAD::cos(halfPitch);
  SCALAR_T sinPitch = CppAD::sin(halfPitch);
  SCALAR_T cosRoll = CppAD::cos(halfRoll);
  SCALAR_T sinRoll = CppAD::sin(halfRoll);
  Eigen::Quaternion<SCALAR_T> ret_quat;
  ret_quat.x() = -(cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw);
  ret_quat.y() = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
  ret_quat.z() = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
  ret_quat.w() = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;

  return ret_quat;
}

/**
 * Compute a quaternion from a matrix
 *
 * @param [in] R: Rotation Matrix.
 * @return A quaternion representing an equivalent rotation to R.
 */
template <typename SCALAR_T>
Eigen::Quaternion<SCALAR_T> matrixToQuaternion(const Eigen::Matrix<SCALAR_T, 3, 3>& R) {
  return Eigen::Quaternion<SCALAR_T>(R);
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> getRotationMatrixFromZyxEulerAngles(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles){
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);

  const SCALAR_T c1 = CppAD::cos(z);
  const SCALAR_T c2 = CppAD::cos(y);
  const SCALAR_T c3 = CppAD::cos(x);
  const SCALAR_T s1 = CppAD::sin(z);
  const SCALAR_T s2 = CppAD::sin(y);
  const SCALAR_T s3 = CppAD::sin(x);

  // clang-format off
  Eigen::Matrix<SCALAR_T, 3, 3> rotationMatrix;
  rotationMatrix << c1 * c2,      c1 * s2 * s3 - s1 * c3,       c1 * s2 * c3 + s1 * s3,
                    s1 * c2,      s1 * s2 * s3 + c1 * c3,       s1 * s2 * c3 - c1 * s3,
                      -s2,                c2 * s3,                      c2 * c3;
  // clang-format on
  return rotationMatrix;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> getRotationMatrixFromXyzEulerAngles(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles){
  const SCALAR_T x = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T z = eulerAngles(2);

  const SCALAR_T c1 = CppAD::cos(z);
  const SCALAR_T c2 = CppAD::cos(y);
  const SCALAR_T c3 = CppAD::cos(x);
  const SCALAR_T s1 = CppAD::sin(z);
  const SCALAR_T s2 = CppAD::sin(y);
  const SCALAR_T s3 = CppAD::sin(x);

  // clang-format off
  Eigen::Matrix<SCALAR_T, 3, 3> rotationMatrix;
  rotationMatrix << c1 * c2,      c1 * s2 * s3 - s1 * c3,       c1 * s2 * c3 + s1 * s3,
                    s1 * c2,      s1 * s2 * s3 + c1 * c3,       s1 * s2 * c3 - c1 * s3,
                      -s2,                c2 * s3,                      c2 * c3;
  // clang-format on
  return rotationMatrix;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3>  RotateX(SCALAR_T rad)
{
    Eigen::Matrix<SCALAR_T, 3, 3> mat3_rot = Eigen::Matrix<SCALAR_T, 3, 3>::Identity();
    mat3_rot(1,1) = CppAD::cos(rad);
    mat3_rot(1,2) = -CppAD::sin(rad);
    mat3_rot(2,1) = CppAD::sin(rad);
    mat3_rot(2,2) = CppAD::cos(rad);
    return mat3_rot;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3>  RotateY(SCALAR_T rad)
{
    Eigen::Matrix<SCALAR_T, 3, 3> mat3_rot = Eigen::Matrix<SCALAR_T, 3, 3>::Identity();
    mat3_rot(0,0) = CppAD::cos(rad);
    mat3_rot(0,2) = CppAD::sin(rad);
    mat3_rot(2,0) = -CppAD::sin(rad);
    mat3_rot(2,2) = CppAD::cos(rad);
    return mat3_rot;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3>  RotateZ(SCALAR_T rad)
{
    Eigen::Matrix<SCALAR_T, 3, 3> mat3_rot = Eigen::Matrix<SCALAR_T, 3, 3>::Identity();
    mat3_rot(0,0) = CppAD::cos(rad);
    mat3_rot(0,1) = -CppAD::sin(rad);
    mat3_rot(1,0) = CppAD::sin(rad);
    mat3_rot(1,1) = CppAD::cos(rad);
    return mat3_rot;
}


template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> ConvertEulerMobile2RotationMatrix(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles)
{
  return getRotationMatrixFromXyzEulerAngles(eulerAngles);
  const SCALAR_T x = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T z = eulerAngles(2);

  // const int order = 5;  // XYZ  XZY   YXZ   YZX   ZXY  ZYX 

  Eigen::Matrix<SCALAR_T, 3, 3> mat3_rot;
  // switch (order) {
  // case 0:
  //     mat3_rot = RotateX(x) * RotateY(y) * RotateZ(z);
  //     break;
  // case 1:
  //     mat3_rot = RotateX(x) * RotateZ(z) * RotateY(y);
  //     break;
  // case 2:
  //     mat3_rot = RotateY(y) * RotateX(x) * RotateZ(z);
  //     break;
  // case 3:
  //     mat3_rot = RotateY(y) * RotateZ(z) * RotateX(x);
  //     break;
  // case 4:
  //     mat3_rot = RotateZ(z) * RotateX(x) * RotateY(y);
  //     break;
  // case 5:
  //     mat3_rot = RotateZ(z) * RotateY(y) * RotateX(x);
  //     break;
  // }
  mat3_rot = RotateZ(z) * RotateY(y) * RotateX(x);

  return mat3_rot;
}


template <typename SCALAR_T>
Eigen::Quaternion<SCALAR_T>  ConvertRotationMatrix2Quaternion(const Eigen::Matrix<SCALAR_T, 3, 3>& mat3_rot)
{
    return Eigen::Quaternion<SCALAR_T>(mat3_rot);

    /* http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm */
    Eigen::Matrix<SCALAR_T, 4, 1> vec4;

    SCALAR_T tr = mat3_rot(0, 0) + mat3_rot(1, 1) + mat3_rot(2, 2);
    if (CppAD::GreaterThanZero(tr)) {
        SCALAR_T S = CppAD::sqrt(tr + 1.0f) * 2;
        vec4[3] = 0.25f * S;
        vec4[0] = (mat3_rot(2, 1) - mat3_rot(1, 2)) / S;
        vec4[1] = (mat3_rot(0, 2) - mat3_rot(2, 0)) / S;
        vec4[2] = (mat3_rot(1, 0) - mat3_rot(0, 1)) / S;
    } else if ((CppAD::GreaterThanZero(mat3_rot(0, 0) - mat3_rot(1, 1))) & (CppAD::GreaterThanZero(mat3_rot(0, 0) - mat3_rot(2, 2)))) {
        SCALAR_T S = CppAD::sqrt(1.0f + mat3_rot(0, 0) - mat3_rot(1, 1) - mat3_rot(2, 2)) * 2;
        vec4[3] = (mat3_rot(2, 1) - mat3_rot(1, 2)) / S; 
        vec4[0] = 0.25f * S;
        vec4[1] = (mat3_rot(0, 1) + mat3_rot(1, 0)) / S;
        vec4[2] = (mat3_rot(0, 2) + mat3_rot(2, 0)) / S;
    } else if (CppAD::GreaterThanZero(mat3_rot(1, 1) - mat3_rot(2, 2))) {
        SCALAR_T S = CppAD::sqrt(1.0f + mat3_rot(1, 1) - mat3_rot(0, 0) - mat3_rot(2, 2)) * 2;
        vec4[3] = (mat3_rot(0, 2) - mat3_rot(2, 0)) / S;
        vec4[0] = (mat3_rot(0, 1) + mat3_rot(1, 0)) / S;
        vec4[1] = 0.25f * S; 
        vec4[2] = (mat3_rot(1, 2) + mat3_rot(2, 1)) / S;
    } else {
        SCALAR_T S = CppAD::sqrt(1.0f + mat3_rot(2, 2) - mat3_rot(0, 0) - mat3_rot(1, 1)) * 2;
        vec4[3] = (mat3_rot(1, 0) - mat3_rot(0, 1)) / S;
        vec4[0] = (mat3_rot(0, 2) + mat3_rot(2, 0)) / S;
        vec4[1] = (mat3_rot(1, 2) + mat3_rot(2, 1)) / S;
        vec4[2] = 0.25f * S; 
    }

    Eigen::Quaternion<SCALAR_T> quat_vec;
    quat_vec.w() = vec4[3];
    quat_vec.z() = vec4[2];
    quat_vec.y() = vec4[1];
    quat_vec.x() = vec4[0];

    return quat_vec;
}

template <typename SCALAR_T>
Eigen::Quaternion<SCALAR_T> getQuatFromEuler_Nice(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesZyx) {
  // return ConvertRotationMatrix2Quaternion(ConvertEulerMobile2RotationMatrix(eulerAnglesZyx));    // 准确但在cppad优化的时候会报错
  // return matrixToQuaternion(getRotationMatrixFromXyzEulerAngles(eulerAnglesZyx));
  return getQuatFromEuler(eulerAnglesZyx);
}

template <typename SCALAR_T>
Eigen::Quaternion<SCALAR_T> quaternionMultiple(const Eigen::Quaternion<SCALAR_T>& p, const Eigen::Quaternion<SCALAR_T>& q) {
  Eigen::Quaternion<SCALAR_T> q_ret;
  // Eigen::Matrix<SCALAR_T, 3, 1> p_pos, q_pos;
  // p_pos << p.x(), p.y(), p.z();
  // q_pos << q.x(), q.y(), q.z();

  // q_ret.w() = p.w() * q.w() - p.x() * q.x() + p.y() * q.y() + p.z() * q.z();
  q_ret.w() = p.w() * q.w() - p.vec().dot(q.vec());
  
  q_ret.vec() = p.w() * q.vec() + q.w() * p.vec() + p.vec().cross(q.vec());

  return q_ret;
  // return q.w() * qRef.vec() - qRef.w() * q.vec() + q.vec().cross(qRef.vec());
}

template <typename SCALAR_T>
Eigen::Quaternion<SCALAR_T> quaternionInverse(const Eigen::Quaternion<SCALAR_T>& q) {
  Eigen::Quaternion<SCALAR_T> q_ret;
  q_ret.w() = q.w();
  q_ret.vec() = -q.vec();
  return q_ret;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quaternionDistance(const Eigen::Quaternion<SCALAR_T>& q, const Eigen::Quaternion<SCALAR_T>& qRef) {
  return q.w() * qRef.vec() - qRef.w() * q.vec() + q.vec().cross(qRef.vec());
}

template <typename SCALAR_T>
Eigen::Quaternion<SCALAR_T> quaternionDistance_inv(const Eigen::Quaternion<SCALAR_T>& q, const Eigen::Quaternion<SCALAR_T>& qRef) {
  return quaternionMultiple(q, quaternionInverse(qRef));
}

void get_neighbours_height(vector<Eigen::Vector3d> &neibor_pos, double pos_x,
                    double pos_y, double pos_z) {
  Eigen::Vector3d neighbor_position;
  Eigen::Vector2i neighbor_index;

  int N_extend = sqrt(N_S + 1);
  // cout << "N_extend: " << N_extend << endl;

  double _min_z = 1e5;
  double height_array[5];
  for (int i = -N_extend / 2; i <= N_extend / 2; ++i) {
    for (int j = -N_extend / 2; j <= N_extend / 2; ++j) {
      // cout << "i " << i << " j " << j << endl;
      if (i == 0 && j == 0) {
        continue;
      } else {
        neighbor_position.x() = pos_x + Height_map.info.resolution * i;
        neighbor_position.y() = pos_y + Height_map.info.resolution * j;

        neighbor_index =
            World2GridIndex_mpc(neighbor_position.x(), neighbor_position.y());

        neighbor_position.z() =
            (float)(Height_map
                        .data[neighbor_index.x() +
                              neighbor_index.y() * Height_map.info.width]) /
            Height_map_Scale;
        

        if(abs(m_min_height - pos_z) <= Height_map.info.resolution){    // center在未知区域
          if(abs(m_min_height - neighbor_position.z()) > Height_map.info.resolution){ // 邻域点已知，则有效
            // cout << "center unknown neigh_pos:  " << neighbor_position.transpose() << endl;
            neibor_pos.push_back(neighbor_position);
          }
        }
        else{ // center已知
          if(abs(neighbor_position.z() - pos_z) < 3 * Height_map.info.resolution){  // 邻域点与中心点在一定误差内
            neibor_pos.push_back(neighbor_position);
            // cout << "accept neigh_pos:  " << neighbor_position.transpose() << endl;
          }
          else{ // 邻域点与中心点超过一定误差
            if(abs(m_min_height - neighbor_position.z()) <= Height_map.info.resolution){  // 且邻域点未知
              // cout << "defused initvalue neigh_pos:  " << neighbor_position.transpose() << endl;
              neibor_pos.push_back(neighbor_position);
            }
            else{   //且邻域点与未知最低高度大于误差， Wall
              // cout << "defused neigh_pos:  " << neighbor_position.transpose() << endl;
            }          
          }  
        }              
      }
    }
  }
  // cout << "min_z in neighbor: " << _min_z << endl;

  // for(int i = 0; i < neibor_pos.size(); ++i){
  //   if(abs(neibor_pos[i].z() - m_min_height) < 5e-2){
  //     neibor_pos[i].z() = _min_z;
  //     cout << "emit unknown grid: " << neibor_pos[i].z() << " -> " << _min_z << endl;
  //   }
  // }
}

void get_neighbours(vector<Eigen::Vector3d> &neibor_pos, double pos_x,
                    double pos_y, double pos_z) {
  Eigen::Vector3d neighbor_position;
  Eigen::Vector2i neighbor_index;

  int N_extend = sqrt(N_S + 1);
  // cout << "N_extend: " << N_extend << endl;

  double _min_z = 1e5;
  for (int i = -N_extend / 2; i <= N_extend / 2; ++i) {
    for (int j = -N_extend / 2; j <= N_extend / 2; ++j) {
      // cout << "i " << i << " j " << j << endl;
      if (i == 0 && j == 0) {
        continue;
      } else {
        neighbor_position.x() = pos_x + Height_map.info.resolution * i;
        neighbor_position.y() = pos_y + Height_map.info.resolution * j;

        neighbor_index =
            World2GridIndex_mpc(neighbor_position.x(), neighbor_position.y());


        neighbor_position.z() =
            (float)(Height_map
                        .data[neighbor_index.x() +
                              neighbor_index.y() * Height_map.info.width]) /
            Height_map_Scale;

        if(abs(m_min_height - pos_z) <= Height_map.info.resolution){    // center在未知区域
          if(abs(m_min_height - neighbor_position.z()) > Height_map.info.resolution){
            // cout << "center unknown neigh_pos:  " << neighbor_position.transpose() << endl;
            neibor_pos.push_back(neighbor_position);
          }
        }
        else{
          if(abs(neighbor_position.z() - pos_z) < 3 * Height_map.info.resolution){
            neibor_pos.push_back(neighbor_position);
            // cout << "accept neigh_pos:  " << neighbor_position.transpose() << endl;
          }
          else{
            if(abs(m_min_height - neighbor_position.z()) <= Height_map.info.resolution){
              // cout << "defused initvalue neigh_pos:  " << neighbor_position.transpose() << endl;
              neibor_pos.push_back(neighbor_position);
            }
            else{   // Wall
              // cout << "defused neigh_pos:  " << neighbor_position.transpose() << endl;
            }          
          }  
        }              
      }
    }
  }
  // cout << "min_z in neighbor: " << _min_z << endl;

  // for(int i = 0; i < neibor_pos.size(); ++i){
  //   if(abs(neibor_pos[i].z() - m_min_height) < 5e-2){
  //     neibor_pos[i].z() = _min_z;
  //     cout << "emit unknown grid: " << neibor_pos[i].z() << " -> " << _min_z << endl;
  //   }
  // }
}

bool compare_func_max(Eigen::Vector3d a, Eigen::Vector3d b){
  return a.z() < b.z();
}

bool compare_func_min(Eigen::Vector3d a, Eigen::Vector3d b){
  return a.z() > b.z();
}

Eigen::Vector3d get_normal_vector(double center_x, double center_y) {
  Eigen::Vector2d center_position(center_x, center_y);
  Eigen::Vector2i center_index = World2GridIndex_mpc(center_position);
  // cout << "center_index: " << center_index.transpose() << endl;
  //计算中心点坐标
  double center_z = 0;

  center_z =
      (float)(Height_map.data[center_index.x() +
                              center_index.y() * Height_map.info.width]) /
      Height_map_Scale;
  cout << "center_position: " << center_x << " " << center_y << " " << center_z
       << endl;

  Eigen::Vector3d _query_pt_3d(center_x, center_y, center_z + 0.4);
  test_pos_visual.push_back(_query_pt_3d);

  // double locations_x[N_S], locations_y[N_S], locations_z[N_S];
  // get_neighbours(locations_x, locations_y, locations_z, center_x, center_y);
  vector<Eigen::Vector3d> neighbors;
  get_neighbours(neighbors, center_x, center_y, center_z);

  double height_sum = 0;
  double height_avg = 0;
  int count_sum = 0;
  for(int i = 0; i < neighbors.size(); ++i){
    if(abs(neighbors[i].z() - m_min_height) > Height_map.info.resolution){
      height_sum += neighbors[i].z();
      count_sum++;
    }
  }
  height_avg = height_sum/count_sum;
  cout << "height_avg:" << height_avg << endl;

  if(abs(m_min_height - center_z) < Height_map.info.resolution){
    center_z = height_avg;
  }

  for(int i = 0; i < neighbors.size(); ++i){
    if(abs(neighbors[i].z() - m_min_height) < Height_map.info.resolution){
      if(i == 0){
        cout << "emit unknown grid" << i << ": " << neighbors[i].z(); 
        neighbors[i].z() = height_avg;
        cout << " -> " << neighbors[i].z() << endl;
      }
      else if(i == neighbors.size() - 1){
        cout << "emit unknown grid" << i << ": " << neighbors[i].z(); 
        neighbors[i].z() = neighbors[i-1].z();
        cout << " -> " << neighbors[i].z() << endl;
      }
      else{
        cout << "emit unknown grid" << i << ": " << neighbors[i].z(); 
        // neighbors[i].z() = height_avg;
        // neighbors[i].z() = (neighbors[i-1].z() + neighbors[i+1].z())/2;
        neighbors[i].z() = neighbors[i-1].z();
        cout << " -> " << neighbors[i].z() << endl;
      }
    }
  }

  if(neighbors.size() < N_S/2){
    Eigen::Vector3d ret_unit(0,0,1);
    return ret_unit;
  }

  // cout << "max_vec begin" << endl;
  // auto max_vec = *max_element(neighbors.begin(), neighbors.end(), compare_func_max);
  // cout << "max_vec: " << max_vec.transpose() << endl;

  // auto min_vec = *min_element(neighbors.begin(), neighbors.end(), compare_func_max);
  // cout << "min_vec: " << min_vec.transpose() << endl;

  // if(abs(min_vec.z() - max_vec.z()) <= Height_map.info.resolution + 1e-2){
  //   cout << "exterior voxel: " << abs(min_vec.z() - max_vec.z()) << endl;
  //   Eigen::Vector3d ret_unit(0,0,1);
  //   return ret_unit;
  // }

  //计算协方差矩阵
  double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
  for (int i = 0; i < neighbors.size(); i++) {
    // cout << "neighbor " << i << " : " << neighbors[i].x() << " "
    //      << neighbors[i].y() << " " << neighbors[i].z() << endl;
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
  // cout << "covMat: " << endl << covMat << endl;

  //求特征值与特征向量
  Eigen::EigenSolver<Eigen::Matrix3f> es(covMat);
  Eigen::Matrix3f val = es.pseudoEigenvalueMatrix();
  Eigen::Matrix3f vec = es.pseudoEigenvectors();
  // cout << "Eigenval matrix: " << endl << val << endl;
  // cout << "Eigenvec matrix: " << endl << vec << endl;

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

  if (v.z() < 0) {
    v.x() = -v.x();
    v.y() = -v.y();
    v.z() = -v.z();
  }

  return v;
}

Eigen::Vector2d get_roll_pitch_fromline(double x, double y, double theta_base){
  double ret_pitch;
  if(x < 3.5 || (x >= 7 && x < 15) || (x >= 20 && x < 27.5) || (x >= 31 && x < 39) || (x >= 44)){
    ret_pitch = 0.032;    // plt底盘的静态误差
    // ret_pitch = 0.0;    // 底盘的静态误差
  }
  else if((x >= 3.5 && x < 7) || (x >= 27.5 && x < 31)){
    ret_pitch = 0.407;
  }
  else if((x >= 15 && x < 20) || (x >= 39 && x < 44)){
    ret_pitch = -0.257;
  }
  
  Eigen::Vector2d roll_pitch_vect(0, ret_pitch);
  return roll_pitch_vect;
}

Eigen::Vector2d get_roll_pitch_oneramp(double x, double y, double theta_base){
  double ret_pitch;
  if(x < 3.5 || (x >= 7 && x < 15) || (x >= 20)){
    ret_pitch = 0.032;    // plt底盘的静态误差
    // ret_pitch = 0.0;    // 底盘的静态误差
  }
  else if((x >= 3.5 && x < 7)){
    ret_pitch = 0.407;
  }
  else if((x >= 15 && x < 20)){
    ret_pitch = -0.257;
  }
  
  Eigen::Vector2d roll_pitch_vect(0, ret_pitch);
  return roll_pitch_vect;
}

Eigen::Vector3d norm_vec_w;
Eigen::Vector2d get_roll_pitch(double x, double y, double theta_base) {
  // 根据height map计算pca法向量
  Eigen::Vector3d norm_vec_w = get_normal_vector(x, y); // 这里需要减半个车身的长度，不能以相机的位置
  cout << "norm_vec_w: " << norm_vec_w.transpose() << endl;

  if((abs(norm_vec_w(0)) < 1e-2 && abs(norm_vec_w(1)) < 1e-2 && abs(norm_vec_w(2) - 1) < 1e-2)){
    Eigen::Vector2d roll_pitch_vect(0, 0);
    return roll_pitch_vect;
  }

  // 3.0 初始化欧拉角(X-Y-Z，即RPY, 先绕x轴roll,再绕y轴pitch,最后绕z轴yaw)
  Eigen::Vector3d ea(0, 0, theta_base);
  cout << "theta_base: " << theta_base << endl;
  // 3.1 欧拉角转换为旋转矩阵
  Eigen::Matrix3d rotation_matrix3;

  Eigen::Matrix<double, 3, 1> _euler_r(0, 0, theta_base);
  rotation_matrix3 = getRotationMatrixFromXyzEulerAngles(_euler_r);
  // cout << "getRotationMatrixFromZyxEulerAngles rotation matrix3 =\n" << rotation_matrix3 << endl;

  Eigen::Vector3d norm_vec_r = norm_vec_w.transpose() * rotation_matrix3;
  cout << "norm_vec_r: " << norm_vec_r.transpose() << endl;

  double roll = atan2(norm_vec_r.y(), norm_vec_r.z());  // 左侧朝上为负值
  cout << "roll: " << roll << endl;

  double pitch = -atan2(norm_vec_r.x(), norm_vec_r.z());  // 朝上为负值
  cout << "pitch: " << pitch << endl;

  Eigen::Vector3d _query_rot(roll, pitch - M_PI/2, theta_base);
  test_rot_visual.push_back(_query_rot);

  Eigen::Vector2d roll_pitch_vect(roll, pitch);
  return roll_pitch_vect;
}


/**
 * @brief Fit polynomial using Least Square Method.
 *
 * @param X X-axis coordinate vector of sample data.
 * @param Y Y-axis coordinate vector of sample data.
 * @param orders Fitting order which should be larger than zero.
 * @return Eigen::VectorXf Coefficients vector of fitted polynomial.
 */
Eigen::VectorXf FitterLeastSquareMethod(vector<float> &X, vector<float> &Y, uint8_t orders)
{
    // abnormal input verification
    if (X.size() < 2 || Y.size() < 2 || X.size() != Y.size() || orders < 1)
        exit(EXIT_FAILURE);

    // map sample data from STL vector to eigen vector
    Eigen::Map<Eigen::VectorXf> sampleX(X.data(), X.size());
    Eigen::Map<Eigen::VectorXf> sampleY(Y.data(), Y.size());

    Eigen::MatrixXf mtxVandermonde(X.size(), orders + 1);  // Vandermonde matrix of X-axis coordinate vector of sample data
    Eigen::VectorXf colVandermonde = sampleX;              // Vandermonde column

    // construct Vandermonde matrix column by column
    for (size_t i = 0; i < orders + 1; ++i)
    {
        if (0 == i)
        {
            mtxVandermonde.col(0) = Eigen::VectorXf::Constant(X.size(), 1, 1);
            continue;
        }
        if (1 == i)
        {
            mtxVandermonde.col(1) = colVandermonde;
            continue;
        }
        colVandermonde = colVandermonde.array()*sampleX.array();
        mtxVandermonde.col(i) = colVandermonde;
    }

    // calculate coefficients vector of fitted polynomial
    Eigen::VectorXf result = (mtxVandermonde.transpose()*mtxVandermonde).inverse()*(mtxVandermonde.transpose())*sampleY;

    return result;
}

double m_profile_length = 10;
int _extend_step = 10;
// const int poly_order = 3;
float polyfit_result[poly_order];
vector<Eigen::Vector3d> test_pos_visual;
vector<Eigen::Vector3d> test_rot_visual;

void get_pitch_profile(const Eigen::VectorXd & ref_wp){
  double _height_eps = 0.02;

  Eigen::Vector2d _start_pt, _end_pt;
  _start_pt << ref_wp[0], ref_wp[y_start];
  _end_pt << ref_wp[y_start - 1], ref_wp[theta_b_start - 1];
  cout << "start pt: " << _start_pt.transpose() << " end pt: " << _end_pt.transpose() << endl;

  Eigen::Vector2d _direction_vec = (_end_pt - _start_pt).normalized();

  // test_pos_visual.clear();

  Eigen::Vector2d _sample_pt;
  vector<float> _interp_dis_vec;
  vector<float> _interp_height_vec;
  for(int _idx = 0; _idx <= _extend_step; ++_idx){
    _sample_pt = _start_pt + _direction_vec * _idx * Height_map.info.resolution;
    // cout << "idx: " << _idx * Height_map.info.resolution << endl;
    // cout << "_sample_pt: " << _sample_pt.transpose() << endl;

    Eigen::Vector2i _sample_index = World2GridIndex_mpc(_sample_pt);
    // cout << "center_index: " << center_index.transpose() << endl;
    //计算中心点坐标
    float center_z = 0;

    center_z =
        (float)(Height_map.data[_sample_index.x() +
                                _sample_index.y() * Height_map.info.width]) /
        Height_map_Scale;
    // cout << "sample pt center z: " << center_z << endl;
    if(center_z <= m_min_height){
      continue;
    }

    // vector<Eigen::Vector3d> neighbors;
    // get_neighbours(neighbors, _sample_pt.x(), _sample_pt.y());

    // //计算协方差矩阵
    // for (int i = 0; i < neighbors.size(); i++) {
    //   center_z += neighbors[i].z();
    // }
    // float center_z_avg = center_z/(neighbors.size() + 1);
    // cout << "sample pt center neighbor z: " << center_z_avg << endl;
    // cout << _idx * Height_map.info.resolution << ", " << center_z_avg << endl;
    cout << _idx * Height_map.info.resolution << ", " << center_z << endl;
    
    // Eigen::Vector3d _query_pt_3d(_sample_pt.x(), _sample_pt.y(), center_z);

    // test_pos_visual.push_back(_query_pt_3d);

    _interp_dis_vec.push_back(_idx * Height_map.info.resolution);   // -4~4
    _interp_height_vec.push_back(center_z);
  }

  Eigen::VectorXf result_ = FitterLeastSquareMethod(_interp_dis_vec, _interp_height_vec, poly_order);
  for(int i = 0; i < result_.size(); ++i){
    polyfit_result[i] = result_(i);
    cout << "polyfit_result[" << i << "]: " << polyfit_result[i] << endl;
  }
}

void test_heightProfile(const Eigen::VectorXd & ref_wp){
  int _step = ref_wp.size()/4;
  if(_step <= 2){
    return;
  }

  get_pitch_profile(ref_wp);  

  Eigen::Vector2d _start_pt, _end_pt;
  _start_pt << ref_wp[0], ref_wp[y_start];
  _end_pt << ref_wp[y_start - 1], ref_wp[theta_b_start - 1];
  cout << "start pt: " << _start_pt.transpose() << " end pt: " << _end_pt.transpose() << endl;

  Eigen::Vector2d _direction_vec = (_end_pt - _start_pt).normalized();

  test_pos_visual.clear();
  // 若ref_wp过短，则选取当前位置（ref_wp[0]）最大坡面作为pitch角, pitch根据theta确定
  // 否则，选取ref_wp所在直线上的点做曲线拟合
  
  {
    for(int i = 0; i < _step; ++i){
      Eigen::Vector2d _query_pt(ref_wp[i], ref_wp[y_start + i]);
      cout << "_query_pt: " << _query_pt.transpose() << endl;

      float _query_idx = (_query_pt - _start_pt).dot(_direction_vec)/_direction_vec.norm();
      cout << "_query_idx: " << _query_idx << endl;

      float _curvature = 0;
      float _curve_z = 0;

      // cout << "poly_order: " << poly_order << endl;
      // f(x) = result[0] + result[1] * x + result[2] * x^2 + result[3] * x^3
      // f'(x) = result[1] + 2 * result[2] * x + 3 * result[3] * x^2
      
      _curve_z = polyfit_result[0] + polyfit_result[1] * pow(_query_idx, 1) + polyfit_result[2] * pow(_query_idx, 2) + polyfit_result[3] * pow(_query_idx, 3);
      _curvature = polyfit_result[1] * pow(_query_idx, 0) * 1 + 2 * polyfit_result[2] * pow(_query_idx, 1) + 3 * polyfit_result[3] * pow(_query_idx, 2);

      Eigen::Vector3d _query_pt_3d(_query_pt.x(), _query_pt.y(), _curve_z);
      Eigen::Vector3d _query_rot(0, _curvature, 0);
      test_pos_visual.push_back(_query_pt_3d);
      test_rot_visual.push_back(_query_rot);

      // _curvature = polyfit_result[1] * pow(_query_idx, 0) * 1 + 2 * polyfit_result[2] * pow(_query_idx, 1) + 3 * polyfit_result[3] * pow(_query_idx, 2);
      // cout << "term: " << polyfit_result[1] * pow(_query_idx, 0) * 1 << " " << 2 * polyfit_result[2] * pow(_query_idx, 1) << " " << 3 * polyfit_result[3] * pow(_query_idx, 2) << endl;
      cout << "_curve_z: " << _curve_z << endl;
      cout << "_curvature: " << _curvature << endl;
      // pitch_vec.push_back(_curvature);
    }
  }
}

vector<double> pitch_vec;
void get_pitchFromProfile(const Eigen::VectorXd & ref_wp){
  get_pitch_profile(ref_wp);
  int _step = ref_wp.size()/4;

  Eigen::Vector2d _start_pt, _end_pt;
  _start_pt << ref_wp[0], ref_wp[y_start];
  _end_pt << ref_wp[y_start - 1], ref_wp[theta_b_start - 1];
  cout << "start pt: " << _start_pt.transpose() << " end pt: " << _end_pt.transpose() << endl;

  Eigen::Vector2d _direction_vec = (_end_pt - _start_pt).normalized();

  pitch_vec.clear();
  // 若ref_wp过短，则选取当前位置（ref_wp[0]）最大坡面作为pitch角, pitch根据theta确定
  // 否则，选取ref_wp所在直线上的点做曲线拟合
  if(_step <= 2){

  }
  else{
    for(int i = 0; i < _step; ++i){
      Eigen::Vector2d _query_pt(ref_wp[i], ref_wp[y_start + i]);
      cout << "_query_pt: " << _query_pt.transpose() << endl;

      float _query_idx = (_query_pt - _start_pt).dot(_direction_vec)/_direction_vec.norm();
      cout << "_query_idx: " << _query_idx << endl;

      float _curvature = 0;
      // cout << "poly_order: " << poly_order << endl;
      // f(x) = result[0] + result[1] * x + result[2] * x^2 + result[3] * x^3
      // f'(x) = result[1] + 2 * result[2] * x + 3 * result[3] * x^2
      
      _curvature = polyfit_result[1] * pow(_query_idx, 0) * 1 + 2 * polyfit_result[2] * pow(_query_idx, 1) + 3 * polyfit_result[3] * pow(_query_idx, 2);
      cout << "term: " << polyfit_result[1] * pow(_query_idx, 0) * 1 << " " << 2 * polyfit_result[2] * pow(_query_idx, 1) << " " << 3 * polyfit_result[3] * pow(_query_idx, 2) << endl;
      cout << "_curvature: " << _curvature << endl;
      pitch_vec.push_back(_curvature);
    }
  }
}

void get_ref_base_yaw(Eigen::VectorXd & state, Eigen::VectorXd & ref_wp){
    
  ref_base_yaw_vec.clear();

  cout << "ref_base_yaw[0]: " << state[2] << endl;
  ref_base_yaw_vec.push_back(state[2]);

  for(int t = 1; t < N - 2; t++)    
  {
    Eigen::Vector2d prev_pos(ref_wp[t], ref_wp[y_start + t]);
    Eigen::Vector2d last_pos(ref_wp[t + 1], ref_wp[y_start + t + 1]);
    cout << "prev_pos: " << prev_pos.transpose() << endl;
    cout << "last_pos: " << last_pos.transpose() << endl;

    Eigen::Vector2d _dir = last_pos - prev_pos;
    double _ref_base_yaw = atan2(_dir.y(), _dir.x());
    
    _ref_base_yaw = (_ref_base_yaw - ref_base_yaw_vec[t-1]) > M_PI/2 ? (_ref_base_yaw - M_PI) : _ref_base_yaw;    // 机器人可能倒着走

    ref_base_yaw_vec.push_back(_ref_base_yaw);

    cout << "ref_base_yaw: " << _ref_base_yaw << endl;
    cout << "ref_wp_theta: " << ref_wp[theta_b_start + t] << endl;
  }
  ref_base_yaw_vec.push_back(ref_base_yaw_vec.back());
  ref_base_yaw_vec.push_back(ref_base_yaw_vec.back());
}


AD<double> _threshold = 4.8;
AD<double> _pitch_set = 0.49;
AD<double> _zero_set = 0.0;
class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd ref_wp;

  FG_eval(Eigen::VectorXd ref_wp) { this->ref_wp = ref_wp; }

  AD<double> get_pitch(AD<double> pos_x){
    return CondExpGt(pos_x, _threshold, _pitch_set, _zero_set);
    // AD<double> _xd = 4.5 - pos_x;
  }
  
  double get_pitch(double pos_x){
    return pos_x > 4.6 ? -0.49 : 0;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector &fg, const ADvector &vars) {
    // `fg` a vector of the cost constraints, `vars` is a vector of variable
    // values (state & actuators) NOTE: You'll probably go back and forth
    // between this function and the Solver function below.

    // ----------------------- Cost
    fg[0] = 0;

    // minimize the tracking error
    for (int t = 1; t < N; t++) {
      // double _lamda_err = 1.0/float(t);    // 步数t越大，lamda越小，ERROR cost占比越小
      double _lamda_err = 5.0;

      fg[0] +=
          10 * _lamda_err * CppAD::pow(vars[t] - ref_wp[t], 2);  // CppAD::exp(-t*0.1) * 
      fg[0] += 10 * _lamda_err * CppAD::pow(vars[y_start + t] - ref_wp[y_start + t], 2);
      
      // fg[0] +=
      //     10 * CppAD::pow(vars[theta_b_start + t] - _ref_base_yaw,
      //                      2);
      
      cout << "_lamda_err: " << _lamda_err << endl;

      // fg[0] +=
      //     10 * CppAD::pow(vars[theta_c_start + t] - ref_wp[theta_b_start + t],
      //                      2);  // ref_wp[theta_b_start] not real theta base
      //                           // but index, make sure the camera orientation
      //                           // fg[0] += 100 * CppAD::pow(vars[t] - 2, 2);

#if def_Using_Quaternion
      Eigen::Matrix<AD<double>, 3, 1> _euler_base(vars[roll_b_start + t], vars[pitch_b_start + t], vars[theta_b_start + t]);
      // Eigen::Quaternion<AD<double> > _quat_base = getQuaternionFromEulerAnglesXYZ(_euler_base);
      Eigen::Quaternion<AD<double> > _quat_base = getQuatFromEuler_Nice(_euler_base);

      AD<double> _yaw_bc_iter = vars[theta_bc_start + t];
      // _yaw_bc_iter = CondExpGt(vars[theta_bc_start + t], AD<double>(M_PI), vars[theta_bc_start + t] - AD<double>(2*M_PI), (CondExpGt(-vars[theta_bc_start + t], AD<double>(-M_PI), vars[theta_bc_start + t] + AD<double>(2*M_PI), vars[theta_bc_start + t])));

      // if(_yaw_bc_iter > M_PI){
      //   _yaw_bc_iter = vars[theta_bc_start + t] - AD<double>(2*M_PI);
      // }
      // else if(_yaw_bc_iter < -M_PI){
      //   _yaw_bc_iter = vars[theta_bc_start + t] + AD<double>(2*M_PI);
      // }
      Eigen::Matrix<AD<double>, 3, 1> _euler_base2cam(0, vars[pitch_bc_start + t], _yaw_bc_iter);
      // Eigen::Quaternion<AD<double> > _quat_base2cam = getQuaternionFromEulerAnglesXYZ(_euler_base2cam);
      Eigen::Quaternion<AD<double> > _quat_base2cam = getQuatFromEuler_Nice(_euler_base2cam);

      Eigen::Quaternion<AD<double> > _quat_cam = quaternionMultiple(_quat_base, _quat_base2cam);


      Eigen::Matrix<AD<double>, 3, 1> _euler_cam_ref(0, ref_wp[theta_bc_start + t], ref_wp[theta_b_start + t]);
      // Eigen::Quaternion<AD<double> > _quat_cam_ref = getQuaternionFromEulerAnglesXYZ(_euler_cam_ref);
      Eigen::Quaternion<AD<double> > _quat_cam_ref = getQuatFromEuler_Nice(_euler_cam_ref);

      // Eigen::Matrix<AD<double>, 3, 1> _quat_err = quaternionDistance(_quat_cam, _quat_cam_ref);
      // Eigen::Matrix<AD<double>, 3, 1> _euler_cam_mpc = _quat_cam.matrix().eulerAngles(0,1,2);
      Eigen::Matrix<AD<double>, 3, 1> _quat_err =  (_quat_cam.vec() - _quat_cam_ref.vec());


      // fg[0] += 100 * (CppAD::pow((_euler_cam_mpc-_euler_cam_ref)(1), 2) + CppAD::pow((_euler_cam_mpc-_euler_cam_ref)(2), 2));
      fg[0] +=
          100 * (CppAD::pow(_quat_err.x(), 2) + CppAD::pow(_quat_err.y(), 2) + CppAD::pow(_quat_err.z(), 2) + CppAD::pow((_quat_cam.w() - _quat_cam_ref.w()), 2));  // + CppAD::pow((_quat_cam.w() - _quat_cam_ref.w()), 2)
#else
      fg[0] +=
          10 * _lamda_err * CppAD::pow(vars[theta_bc_start + t] + vars[theta_b_start + t] - ref_wp[theta_b_start + t],
                           2);  
      fg[0] +=
                10 * _lamda_err * CppAD::pow(vars[pitch_bc_start + t] + vars[pitch_b_start + t] - ref_wp[theta_bc_start + t],
                                2);
#endif
      
      // fg[0] += 1 * CppAD::pow(vars[theta_bc_start + t], 2);   // ! 实物实验中用到，适用无滑环
      // fg[0] += 1 * CppAD::pow(vars[theta_bc_start + t] + vars[theta_b_start + t], 2);
    }
    
    double _lamda_motion = 1.5 - 1.5*exp(-0.1*N);    // N:0-11->lamda:0-1;步数N越大，lamda越大，motion cost占比越大
    // double _lamda_motion = 0;
    // Minimize the use of actuators
    for (int t = 0; t < N - 1; t++) {
      fg[0] += _lamda_motion * 2 * CppAD::pow(vars[v_start + t], 2);
      fg[0] += _lamda_motion * 1 * CppAD::pow(vars[w_start + t], 2);
      fg[0] += _lamda_motion * 1 * CppAD::pow(vars[gama_start + t], 2);
      // fg[0] += _lamda_motion * 1.5 * CppAD::pow(vars[w_start + t] + vars[gama_start + t], 2);   //! 约束相机相对于世界坐标系的速度,对slam
      fg[0] += _lamda_motion * 2 * CppAD::pow(vars[beta_start + t], 2);
    }

    // -----------------------  model constraints

    // initial constraints
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + theta_b_start] = vars[theta_b_start];
    fg[1 + theta_bc_start] = vars[theta_bc_start];
    fg[1 + pitch_b_start] = vars[pitch_b_start];
    fg[1 + pitch_bc_start] = vars[pitch_bc_start];
    fg[1 + roll_b_start] = vars[roll_b_start];

    test_pos_visual.clear();
    test_rot_visual.clear();
    // get_pitchFromProfile(ref_wp);
    // rest of the constraints
    for (int t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> theta_b1 = vars[theta_b_start + t];
      AD<double> theta_bc1 = vars[theta_bc_start + t];
      AD<double> pitch_b1 = vars[pitch_b_start + t];
      AD<double> pitch_bc1 = vars[pitch_bc_start + t];
      AD<double> roll_b1 = vars[roll_b_start + t];

      // AD<double> pitch_b1 = get_pitch(ref_wp[t]);
      // cout << "pitch_b1: " << pitch_b1 << endl;

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> theta_b0 = vars[theta_b_start + t - 1];
      AD<double> theta_bc0 = vars[theta_bc_start + t - 1];
      AD<double> pitch_b0 = vars[pitch_b_start + t - 1];
      AD<double> pitch_bc0 = vars[pitch_bc_start + t - 1];
      AD<double> roll_b0 = vars[roll_b_start + t - 1];

      AD<double> v0 = vars[v_start + t - 1];
      AD<double> w0 = vars[w_start + t - 1];
      AD<double> gama0 = vars[gama_start + t - 1];
      AD<double> beta0 = vars[beta_start + t - 1];

      AD<double> _pre_pitch = pitch_b0;    // 不考虑地形影响，假设地形都与当前状态一致
      AD<double> _pre_roll = roll_b0;
      // cout << "_pre_pitch: " << _pre_pitch << endl;
      
      // equations for the model:
      // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
      // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
      // v_[t] = v[t-1] + a[t-1] * dt

      fg[1 + x_start + t] = x1 - (x0 + v0 * cos(theta_b0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * sin(theta_b0) * dt);
      fg[1 + theta_b_start + t] = theta_b1 - (theta_b0 + w0 * dt);
      fg[1 + theta_bc_start + t] = theta_bc1 - (theta_bc0 + gama0 * dt);

      fg[1 + pitch_b_start + t] = pitch_b1 - _pre_pitch;    //  pitch_b0
      fg[1 + pitch_bc_start + t] = pitch_bc1 - (pitch_bc0 + beta0 * dt);

      fg[1 + roll_b_start + t] = roll_b1 - _pre_roll;    // roll_b0
    }

  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

Eigen::MatrixXd MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd ref_wp,
                           size_t Predict_steps) {
  cout << "MPC solve of robot with Gimbal..." << endl;
  bool ok = true;
  // size_t i;

  N = Predict_steps;
  // state
  x_start = 0;
  y_start = x_start + N;
  theta_b_start = y_start + N;        // base orientation
  theta_bc_start = theta_b_start + N;  // camera orientation pitch
  pitch_b_start = theta_bc_start + N;  // camera orientation pitch
  pitch_bc_start = pitch_b_start + N;  // camera orientation pitch
  roll_b_start = pitch_bc_start + N;

  // control input
  v_start = roll_b_start + N;
  w_start = v_start + N - 1;
  gama_start = w_start + N - 1;
  beta_start = gama_start + N - 1;
    
  if(Predict_steps < 2){
    cout << "Unexcuted node less than 2........" << endl;
    Eigen::MatrixXd U_result = Eigen::MatrixXd::Zero(4, N);
    Eigen::Vector4d u_k;
    // std::vector<double> w_pred_vals;
    for (int i = 0; i < N; i++) {
      u_k(0) = 0;
      u_k(1) = 0;
      u_k(2) = state[3] - ref_wp[theta_b_start];
      // u_k(3) = state[5] - ref_wp[theta_c_start];
      u_k(3) = 0;
      U_result.col(i) = u_k;
      std::cout << "U " << i << " : " << endl << u_k.transpose() << endl;
    }

    return U_result;
  }
  
  _ref_base_yaw = state[2];    // 以cur_yaw为参考

  // Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // x, y, theta_b, theta_c; v, w, gama, beta
  size_t n_vars = 7 * N + 4 * (N - 1);
  // Set the number of constraints, 运动学模型约束
  size_t n_constraints = 7 * N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) vars[i] = 0;

  // set initial state
  vars[x_start] = state[0];
  vars[y_start] = state[1];
  vars[theta_b_start] = state[2];
  vars[theta_bc_start] = state[3] - state[2];     // ! 这里应该从joint_states读取
  vars[pitch_b_start] = state[4];
  vars[pitch_bc_start] = state[5];    // ! 从joint_states读取的pitch角
  vars[roll_b_start] = state[6];

  // for(int i = 1; i < N; ++i){
  //   vars[pitch_c_start + i] = 0.05 * i;
  // }

  // 添加状态量-相机旋转速度的约束，即底盘不管怎么转，保证相机的转速在一定范围内即可。

  vars[v_start] = state[7];
  vars[w_start] = state[8];
  vars[gama_start] = state[9];
  vars[beta_start] = state[10];

  cout << "initial vars: " << vars << endl;

  // 变量的上下限
  // Set lower and upper limits for variables.
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  // position, 
  for (int i = 0; i < theta_b_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  // theta_base
  for (int i = theta_b_start; i < theta_bc_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  // theta_base
  for (int i = theta_bc_start; i < pitch_b_start; i++) {
    // vars_lowerbound[i] = -M_PI;
    // vars_upperbound[i] = M_PI;
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  // theta_pitch_bc 绝对值
  for (int i = pitch_b_start; i < pitch_bc_start; i++) {
    vars_lowerbound[i] = -1.5;
    vars_upperbound[i] = 1.5;
  }

  // theta_pitch in world 绝对值
  for (int i = pitch_bc_start; i < v_start; i++) {
    vars_lowerbound[i] = -1.5;
    vars_upperbound[i] = 1.5;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // base control velocity - v
  for (int i = v_start; i < w_start; i++) {
    vars_lowerbound[i] = -max_v;
    vars_upperbound[i] = max_v;
  }

  // Acceleration/decceleration upper and lower limits.
  // base control velocity - w
  for (int i = w_start; i < gama_start; i++) {
    vars_lowerbound[i] = -max_w_base;
    vars_upperbound[i] = max_w_base;
  }

  // Acceleration/decceleration upper and lower limits.
  // gimbal control velocity - pitch, yaw
  for (int i = gama_start; i < beta_start; i++) {
    vars_lowerbound[i] = -max_gama;
    vars_upperbound[i] = max_gama;
  }

  for (int i = beta_start; i < n_vars; i++) {
    vars_lowerbound[i] = -max_beta;
    vars_upperbound[i] = max_beta;
  }

  // 等式约束
  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    // 除初始值外其余为等式约束，运动学模型
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // 等式约束
  constraints_lowerbound[x_start] = state[0];
  constraints_lowerbound[y_start] = state[1];
  constraints_lowerbound[theta_b_start] = state[2];
  constraints_lowerbound[theta_bc_start] = state[3] - state[2];     // ! 这里应该从joint_states读取
  constraints_lowerbound[pitch_b_start] = state[4];
  constraints_lowerbound[pitch_bc_start] = state[5];    // ! 从joint_states读取的pitch角
  constraints_lowerbound[roll_b_start] = state[6];

  constraints_upperbound[x_start] = state[0];
  constraints_upperbound[y_start] = state[1];
  constraints_upperbound[theta_b_start] = state[2];
  constraints_upperbound[theta_bc_start] = state[3] - state[2];     // ! 这里应该从joint_states读取
  constraints_upperbound[pitch_b_start] = state[4];
  constraints_upperbound[pitch_bc_start] = state[5];    // ! 从joint_states读取的pitch角
  constraints_upperbound[roll_b_start] = state[6];

  // object that computes objective and constraints
  FG_eval fg_eval(ref_wp);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost: " << cost << std::endl;

  // set x_pred_vals and y_pred_vals for plotting
  x_pred_vals.clear();
  y_pred_vals.clear();
  theta_b_pred_vals.clear();
  theta_bc_pred_vals.clear();
  theta_c_pred_vals.clear();
  pitch_b_pred_vals.clear();
  pitch_bc_pred_vals.clear();
  pitch_c_pred_vals.clear();

  quat_pred.clear();

  for (int i = 0; i < N; ++i) {
    // x_pred_vals.push_back(solution.x[x_start + i] - ref_wp[i]);
    // y_pred_vals.push_back(solution.x[y_start + i] - ref_wp[y_start + i]);
    // theta_b_pred_vals.push_back(solution.x[theta_b_start + i]);
    // theta_c_pred_vals.push_back(solution.x[theta_c_start + i] - ref_wp[theta_b_start + i]);
    // pitch_c_pred_vals.push_back(solution.x[pitch_c_start + i] - ref_wp[theta_c_start + i]);

    x_pred_vals.push_back(solution.x[x_start + i]);
    y_pred_vals.push_back(solution.x[y_start + i]);
    theta_b_pred_vals.push_back(solution.x[theta_b_start + i]);
    theta_bc_pred_vals.push_back(solution.x[theta_bc_start + i]);
    theta_c_pred_vals.push_back(solution.x[theta_b_start + i] + solution.x[theta_bc_start + i]);
    pitch_b_pred_vals.push_back(solution.x[pitch_b_start + i]);
    pitch_bc_pred_vals.push_back(solution.x[pitch_bc_start + i]);
    pitch_c_pred_vals.push_back(solution.x[pitch_b_start + i] + solution.x[pitch_bc_start + i]);
#if def_Using_Quaternion
    Eigen::Matrix<double, 3, 1> _euler_base_visual(solution.x[roll_b_start + i], -solution.x[pitch_b_start + i], solution.x[theta_b_start + i]);
    Eigen::Quaternion<double> _quat_base_visual = getQuatFromEuler_Nice(_euler_base_visual);

    Eigen::Matrix<double, 3, 1> _euler_base2cam_visual(0, -solution.x[pitch_bc_start + i], solution.x[theta_bc_start + i]);
    Eigen::Quaternion<double> _quat_base2cam_visual = getQuatFromEuler_Nice(_euler_base2cam_visual);

    Eigen::Quaternion<double> _quat_cam_visual = quaternionMultiple(_quat_base_visual, _quat_base2cam_visual);
    quat_pred.push_back(_quat_cam_visual);

// !For DEBUG
    Eigen::Matrix<double, 3, 1> _euler_base(solution.x[roll_b_start + i], solution.x[pitch_b_start + i], solution.x[theta_b_start + i]);
    // Eigen::Matrix<double, 3, 1> _euler_base(-0.00753109, 0.264214, -1.00798);
    Eigen::Quaternion<double> _quat_base = getQuatFromEuler_Nice(_euler_base);
    cout << "_euler_base: " << solution.x[roll_b_start + i] << " "  << solution.x[pitch_b_start + i] << " "  << solution.x[theta_b_start + i] << endl;
    cout << "_quat_base: " << _quat_base.x() << " "  << _quat_base.y() << " "  << _quat_base.z() << " "  << _quat_base.w() << " " << endl;
    // _quat_base = matrixToQuaternion(getRotationMatrixFromZyxEulerAngles(_euler_base));
    // cout << "_quat_base getRotationMatrixFromZyxEulerAngles: " << _quat_base.x() << " "  << _quat_base.y() << " "  << _quat_base.z() << " "  << _quat_base.w() << " " << endl;
    // _quat_base = getQuaternionFromEulerAnglesXYZ((_euler_base));
    // cout << "_quat_base getQuaternionFromEulerAnglesXYZ: " << _quat_base.x() << " "  << _quat_base.y() << " "  << _quat_base.z() << " "  << _quat_base.w() << " " << endl;

    // _quat_base = ConvertRotationMatrix2Quaternion(ConvertEulerMobile2RotationMatrix(_euler_base));
    // cout << "_quat_base ConvertEulerMobile2RotationMatrix: " << _quat_base.x() << " "  << _quat_base.y() << " "  << _quat_base.z() << " "  << _quat_base.w() << " " << endl;


    Eigen::Matrix<double, 3, 1> _euler_base2cam(0, solution.x[pitch_bc_start + i], solution.x[theta_bc_start + i]);
    Eigen::Quaternion<double> _quat_base2cam = getQuatFromEuler_Nice(_euler_base2cam);
    cout << "_euler_base2cam: " << 0 << " "  << solution.x[pitch_bc_start + i] << " "  << solution.x[theta_bc_start + i] << endl;
    cout << "_quat_base2cam: " << _quat_base2cam.x() << " "  << _quat_base2cam.y() << " "  << _quat_base2cam.z() << " "  << _quat_base2cam.w() << " " << endl;

    Eigen::Quaternion<double> _quat_cam = quaternionMultiple(_quat_base, _quat_base2cam);    
    cout << "_quat_cam: " << _quat_cam.x() << " "  << _quat_cam.y() << " "  << _quat_cam.z() << " "  << _quat_cam.w() << " " << endl;
    
    // Eigen::Matrix<double, 3, 1> _euler_cam_mpc = _quat_cam.toRotationMatrix().eulerAngles(0,1,2);
    // cout << "_euler_cam: " << _euler_cam_mpc(0) << " "  << _euler_cam_mpc(1) << " "  << _euler_cam_mpc(2) << endl;

    
    Eigen::Matrix<double, 3, 1> _euler_cam_ref(0, ref_wp[theta_bc_start + i], ref_wp[theta_b_start + i]);
    Eigen::Quaternion<double> _quat_cam_ref = getQuatFromEuler_Nice(_euler_cam_ref);
    cout << "_euler_cam_real: " << cur_cam_roll << " "  << cur_cam_pitch << " "  << cur_cam_yaw << endl;
    cout << "_euler_cam_ref: " << 0 << " "  << ref_wp[theta_bc_start + i] << " "  << ref_wp[theta_b_start + i] << endl;
    cout << "_quat_cam_ref: " << _quat_cam_ref.x() << " "  << _quat_cam_ref.y() << " "  << _quat_cam_ref.z() << " "  << _quat_cam_ref.w() << " " << endl;
      
    Eigen::Matrix<double, 3, 1> _quat_err = quaternionDistance(_quat_cam, _quat_cam_ref);
    cout << "_quat_err: " << _quat_err.x() << " " << _quat_err.y() << " " << _quat_err.z() << endl;
    cout << "_quat_vec_err: " << (_quat_cam.vec() - _quat_cam_ref.vec()).transpose() << endl;

#endif
    ROS_WARN_STREAM("x_pred_err_vals: " << x_pred_vals[i] - ref_wp[i] << " y_pred_err_vals : " << y_pred_vals[i] - ref_wp[y_start + i]);

    // ROS_WARN_STREAM("theta_base_err_vals: " << (solution.x[theta_b_start + i]) - ref_base_yaw_vec[i]);
    cout << "mpc_pred_yaw: " << solution.x[theta_b_start + i] + solution.x[theta_bc_start + i] << endl;
    cout << "ref_yaw: " << ref_wp[theta_b_start + i] << endl;
    ROS_WARN_STREAM("yaw_err_vals: " << (solution.x[theta_b_start + i] + solution.x[theta_bc_start + i]) - ref_wp[theta_b_start + i]);

    cout << "mpc_pred_pitch: " << (solution.x[pitch_b_start + i] + solution.x[pitch_bc_start + i]) << endl;
    cout << "ref_pitch: " << ref_wp[theta_bc_start + i] << endl;
    ROS_WARN_STREAM("pitch_err_vals: " << (solution.x[pitch_b_start + i] + solution.x[pitch_bc_start + i]) - ref_wp[theta_bc_start + i]);

    // std::cout << "x_error_vals: " << x_pred_vals[i] << " " << y_pred_vals[i]
    //           << " " << theta_c_pred_vals[i]
    //           << " " << pitch_c_pred_vals[i] << std::endl;

    std::cout << "x_pred_vals: " << x_pred_vals[i] << " " << y_pred_vals[i]
              << " " << theta_b_pred_vals[i] << " " << theta_bc_pred_vals[i] << " " << theta_c_pred_vals[i]
              << " " << pitch_b_pred_vals[i] << " " << pitch_bc_pred_vals[i] << " " << pitch_c_pred_vals[i] << std::endl;
  }

  //  Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  Eigen::MatrixXd U_result = Eigen::MatrixXd::Zero(4, N);
  Eigen::Vector4d u_k;
  // std::vector<double> w_pred_vals;
  for (int i = 0; i < N - 1; i++) {
    u_k(0) = solution.x[v_start + i];
    u_k(1) = solution.x[w_start + i];
    u_k(2) = solution.x[gama_start + i];
    u_k(3) = solution.x[beta_start + i];
    U_result.col(i) = u_k;
    // std::cout << "U " << i << " : " << u_k.transpose() << endl;
  }

  return U_result;
}
