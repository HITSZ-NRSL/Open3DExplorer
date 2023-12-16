
#include <ros/ros.h>
#include <ros/console.h>
#include <log4cxx/logger.h>

#include <vector>
#include <iostream>
#include <octomap/octomap.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
//#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <fstream>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <chrono>
#include <nodehash.h>

using namespace std;

typedef Eigen::Matrix<double, 3, 6> Matrix36;
typedef Eigen::Matrix<double, 3, 3> Matrix33;
typedef Eigen::Matrix<double, 3, 4> Matrix34;
typedef Eigen::Matrix<double, 3, 3> Matrix33;
typedef Eigen::Matrix<double, 6, 6> Matrix66;

// unordered_map<Node_hash, int, NodeHash> density_map;
// unordered_map<Node_hash, int, NodeHash> Color_density_map;


octomap::OcTree* maptree_igo_;
Eigen::Matrix4d g_T_w_c_Fisherinfo = Eigen::Matrix4d::Identity();
double Octomap_Resolution_ = 0.4;   // 分辨率

// 计算雅克比矩阵，用于费雪信息的计算
Matrix36 Cal_Jacobi(const Eigen::Vector3d &p_w, const Eigen::Matrix4d &T_c_w)
{
  Matrix34 T_c_w_34 = T_c_w.block<3, 4>(0, 0);

  Eigen::Vector4d p_w_41;
  p_w_41.block<3, 1>(0, 0) = p_w;
  p_w_41(3, 0) = 1;

  Eigen::Vector3d p_c = T_c_w_34 * p_w_41;
  // ROS_INFO_STREAM("Eigen::Vector3d : " << p_c.transpose());

  const double n = p_c.norm();
  //   Matrix33 jac =
  Eigen::Matrix3d df_dpc = (1 / n) * Eigen::Matrix3d::Identity() -
                           (1 / (n * n * n)) * p_c * p_c.transpose();
  //   return jac;

  Matrix36 jac;
  jac.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

  Eigen::Matrix3d skew;
  // skewMatrix(v, &skew);
  // CHECK_NOTNULL(&skew);

  skew.setZero();
  (skew)(0, 1) = -p_w[2];
  (skew)(1, 0) = p_w[2];
  (skew)(0, 2) = p_w[1];
  (skew)(2, 0) = -p_w[1];
  (skew)(1, 2) = -p_w[0];
  (skew)(2, 1) = p_w[0];

  //   return skew;

  jac.block<3, 3>(0, 3) = (-1.0) * skew;

  Matrix36 dpc_dse3 = T_c_w.block<3, 3>(0, 0) * jac;

  //   Eigen::Matrix3d df_dpc = dBearing_dPointCam(p_c);
  //   Matrix36 dpc_dse3 = -dPoint_dse3(T_c_w, p_w);
  return df_dpc * dpc_dse3;
}


void save_time_consuming(const std::string file_name, float time){
  std::ofstream out_file(file_name, ios::app);
  if (out_file.is_open())
  {
    out_file << time << endl;
  }
  else
  {
    ROS_ERROR("Error open file...");
  }
}

const static Eigen::IOFormat EigenSpaceSeparatedFmt(
    Eigen::FullPrecision, Eigen::DontAlignCols, " ", "\n");

template <typename Type, int Rows, int Cols>
void save(const std::string file, const Eigen::Matrix<Type, Rows, Cols> &mat,
          const Eigen::IOFormat &io_format = EigenSpaceSeparatedFmt)
{
  std::ofstream out(file);
  // CHECK(out.is_open());
  if (out.is_open())
  {
    out << mat.format(io_format);
  }
  else
  {
    ROS_ERROR("Error open file");
  }
}

// 边界条件
Eigen::Matrix4d Get_Twc_Infomation()
{
  Eigen::Matrix4d t_w_c = Eigen::Matrix4d::Identity();

  std::string camera_frame = "d435i_depth_optical_frame";
  std::string world_frame = "virtual_world";

  tf::TransformListener tf_listener_;
  tf::StampedTransform Camera_World_Tf;

  try
  {
    if (tf_listener_.waitForTransform(world_frame, camera_frame, ros::Time(0), ros::Duration(5)))
    {
      // ROS_WARN_STREAM("time now" << ros::Time::now());
      ROS_WARN_STREAM("Enter tf");
      tf_listener_.lookupTransform(world_frame, camera_frame, ros::Time(0), Camera_World_Tf);
    }
    // ROS_WARN_STREAM("Frame_id: 0." << cloud_msg->header.frame_id);

    // kinect_orig_ = point3d(Camera_World_Tf.getOrigin().x(), Camera_World_Tf.getOrigin().y(), Camera_World_Tf.getOrigin().z());
    // ROS_WARN_STREAM("kinect_origin_: " << kinect_orig_);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
    return t_w_c;
  }

  //////////////////////////////////////////////////////////////////////////////////////
  // Convert tf to matrix
  // Eigen::Quaterniond quat;
  // quat.w() = Camera_World_Tf.getRotation().getW();
  // quat.x() = Camera_World_Tf.getRotation().getX();
  // quat.y() = Camera_World_Tf.getRotation().getY();
  // quat.z() = Camera_World_Tf.getRotation().getZ();
  // t_w_c.block<3,3>(0,0) = quat.toRotationMatrix();
  t_w_c.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(3, 3);
  Eigen::Vector3d translate;
  translate[0] = Camera_World_Tf.getOrigin().getX();
  translate[1] = Camera_World_Tf.getOrigin().getY();
  translate[2] = Camera_World_Tf.getOrigin().getZ();
  t_w_c.block<3, 1>(0, 3) = translate;

  t_w_c.block<1, 4>(3, 0) << 0, 0, 0, 1;

  // ROS_INFO_STREAM("t_w_c : " << t_w_c);
  cout << "\n t_w_c =\n" << t_w_c << endl << endl;

  return t_w_c;
}


// Calculate the infomation of One cell
double Cal_Cell_Info(const Eigen::Vector3d &point, const Eigen::Matrix4d &t_w_c)
{
  // Eigen::Vector3d p1(1,0,0);
  // Eigen::Vector3d p2(2,0,0);
  // Eigen::Vector3d t(0,0,0);
  // t_w_c.block<3,1>(0,3) = t;

  // Matrix36 J1 = Cal_Jacobi(p1, t_w_c.inverse());
  // Matrix66 curr_info1;
  // curr_info1 = J1.transpose() * J1;
  // double fisher_info1 = curr_info1.trace();

  // Matrix36 J2 = Cal_Jacobi(p2, t_w_c.inverse());
  // Matrix66 curr_info2;
  // curr_info2 = J2.transpose() * J2;
  // double fisher_info2 = curr_info2.trace();


  Matrix36 J = Cal_Jacobi(point, t_w_c.inverse());

  // ROS_INFO_STREAM("Infomation Matrix : " << J);

  Matrix66 curr_info;
  curr_info = J.transpose() * J;
  double fisher_info = curr_info.trace();

  return fisher_info;
}

Eigen::Vector3i Get_DnesityMap_Index(Eigen::Vector3d p){
  int index_x, index_y, index_z;
  if(p.x() > 0){
    index_x = floor((p.x() + Octomap_Resolution_)/Octomap_Resolution_);
  }
  else if(p.x() < 0){
    index_x = floor(p.x()/Octomap_Resolution_);
  }
  else{
    index_x = -1;
  }

  if(p.y() > 0){
    index_y = floor((p.y() + Octomap_Resolution_)/Octomap_Resolution_);
  }
  else if(p.y() < 0){
    index_y = floor(p.y()/Octomap_Resolution_);
  }
  else{
    index_y = -1;
  }

  if(p.z() > 0){
    index_z = floor((p.z() + Octomap_Resolution_)/Octomap_Resolution_);
  }
  else if(p.z() < 0){
    index_z = floor(p.z()/Octomap_Resolution_);
  }
  else{
    index_z = -1;
  }

  Eigen::Vector3i index(index_x, index_y, index_z);
  return index;
}

int Get_Point_Density(Eigen::Vector3d point){
  int density_val = 0;

  Eigen::Vector3i Index_Density = Get_DnesityMap_Index(point);

  auto res_search = density_map.find(Node_hash(Index_Density.x(), Index_Density.y(), Index_Density.z()));
  if(res_search != density_map.end()){
    // std::cout << "search position: " << point.x() << " " << point.y() << " " << point.z() << " "  << std::endl;
    // std::cout << "search density: " << res_search->second << std::endl;
    
    density_val = res_search->second;
  }
  else{
    // std::cout << "search position: " << point.x() << " " << point.y() << " " << point.z() << " "  << std::endl;
    // std::cout << "search density: " << 0 << std::endl;
  }

  return density_val;
}

double Dens_Thresh_info = 3;    // 1000:only dense; 1:dense+info
// 考虑density_map和Information
double Get_Point_Infogain(Eigen::Vector3d point){
  double gain = 0.0;
  double gain_info = 1.0;
  int gain_dens = 1;

  gain_dens = Get_Point_Density(point);
  // std::cout << "search DensityInfo: " << gain_dens << std::endl;

  // density 大于3 才进行信息计算，滤出杂点
  if(gain_dens >= Dens_Thresh_info){
    octomap::point3d temp(point.x(), point.y(), point.z());

    octomap::OcTreeNode *node = maptree_igo_->search(temp);

    if (node != NULL && maptree_igo_->isNodeOccupied(node))
    {
      Eigen::Vector3d p_w_(temp.x(), temp.y(), temp.z());

      gain_info = Cal_Cell_Info(p_w_, g_T_w_c_Fisherinfo);
      // cout << "g_T_w_c_Fisherinfo: " << g_T_w_c_Fisherinfo << endl;
      // gain_info = gain_info * 10;
      // std::cout << "search FisherInfo: " << gain_info << std::endl;
    }
  }

  return gain_dens * gain_info; 
}
// 考虑Information
double Get_Point_FisherInfogain(Eigen::Vector3d point){
  double gain = 0.0;
  double gain_info = 1.0;

  // density 大于3 才进行信息计算，滤出杂点
  // if(gain_dens >= Dens_Thresh_info)
  {
    octomap::point3d temp(point.x(), point.y(), point.z());

    octomap::OcTreeNode *node = maptree_igo_->search(temp);

    if (node != NULL && maptree_igo_->isNodeOccupied(node))
    {
      Eigen::Vector3d p_w_(temp.x(), temp.y(), temp.z());

      gain_info = Cal_Cell_Info(p_w_, g_T_w_c_Fisherinfo);
      std::cout << "search FisherInfo: " << gain_info << std::endl;
    }
  }

  return gain_info; 
}


double Get_Max_z(Eigen::Vector3d point){
  double max_z = 0;
  int density_val = 0;
  int max_density = -100;

  int Z_depth = 10;
  Eigen::Vector3d point_inZ(point.x(), point.y(), point.z()+ (Z_depth/2) * Octomap_Resolution_);
  
  density_val = Get_Point_Density(point_inZ);

  for(int j = 0; j <= Z_depth; ++j){
    if(density_val > max_density){
      max_z = point_inZ.z();
      max_density = density_val;
      cout << "max z position: " << point_inZ << endl;
      cout << "max z axis: " << max_z << endl;
      cout << "max_density: " << max_density << endl;
    }
    point_inZ.z() -= Octomap_Resolution_;
    density_val = Get_Point_Density(point_inZ);
  }

  return max_z;
}

int Get_Zaxis_Density(Eigen::Vector3d point){
  int density_val = 0;

  int Z_depth = 6;
  Eigen::Vector3d point_inZ(point.x(), point.y(), point.z()+ (Z_depth/2) * Octomap_Resolution_);
  
  density_val += Get_Point_Density(point_inZ);

  for(int j = 0; j <= Z_depth; ++j){
    point_inZ.z() -= Octomap_Resolution_;
    density_val += Get_Point_Density(point_inZ);
  }

  return density_val;
}

int Get_Neighbor_Density(Eigen::Vector3d point){
  int density_val = 0;

  int Z_depth = 20;
  int X_depth = 0;
  int Y_depth = 0;

  for(int i = -X_depth; i <= X_depth; ++i){
    for(int j = -Y_depth; j <= Y_depth; ++j){
      Eigen::Vector3d point_inZ(point.x() + i * Octomap_Resolution_, point.y() + j * Octomap_Resolution_, point.z() + (Z_depth/2) * Octomap_Resolution_);
  
      density_val += Get_Point_Density(point_inZ);

      for(int j = 0; j <= Z_depth; ++j){
        point_inZ.z() -= Octomap_Resolution_;
        density_val += Get_Point_Density(point_inZ);
      }
    }
  }
  return density_val;
}


// 考虑Information
double Get_Point_Infogain_(Eigen::Vector3d point){
  double gain = 0.0;
  double gain_info = 1.0;

  // density 大于3 才进行信息计算，滤出杂点
  {
    octomap::point3d temp(point.x(), point.y(), point.z());

    // octomap::OcTreeNode *node = maptree_igo_->search(temp);

    // if (node != NULL && maptree_igo_->isNodeOccupied(node))
    {
      Eigen::Vector3d p_w_(temp.x(), temp.y(), temp.z());

      gain_info = Cal_Cell_Info(p_w_, g_T_w_c_Fisherinfo);
      std::cout << "search FisherInfo: " << gain_info << std::endl;
    }
  }

  // gain_info = 1;
  return gain_info; 
}

float Get_Neighbor_Info(Eigen::Vector3d point){
  float density_val = 0;

  int Z_depth = 6;
  int X_depth = 0;
  int Y_depth = 0;

  for(int i = -X_depth; i <= X_depth; ++i){
    for(int j = -Y_depth; j <= Y_depth; ++j){
      Eigen::Vector3d point_inZ(point.x() + i * Octomap_Resolution_, point.y() + j * Octomap_Resolution_, point.z() + (Z_depth/2) * Octomap_Resolution_);
  
      density_val += Get_Point_Infogain(point_inZ);

      for(int j = 0; j <= Z_depth; ++j){
        point_inZ.z() -= Octomap_Resolution_;
        density_val += Get_Point_Infogain(point_inZ);
      }
    }
  }
  return density_val;
}


float Get_Neighbor_FisherInfo(Eigen::Vector3d point){
  float density_val = 0;

  int Z_depth = 6;
  int X_depth = 0;
  int Y_depth = 0;

  float l_density_val = 0;
  float l_max_density = -100;
  double z_depth = point.z();
  point.z() = point.z() + (Z_depth/2) * Octomap_Resolution_;
  // cout << "z_depth point: " << point << endl;

  for(int k = 0; k <= Z_depth; ++k){
    l_density_val = Get_Point_Density(point);

    // cout << "z_depth l_density_val: " << l_density_val << endl;
    // cout << "z_depth point: " << point << endl;
    if(l_density_val > l_max_density){
      l_max_density = l_density_val;
      z_depth = point.z();
      // cout << "z_depth l_max_density: " << l_max_density << endl;
      // cout << "z_depth best: " << z_depth << endl;
    
    }

    point.z() -= Octomap_Resolution_;
  }

  if(l_max_density == 0){
    return 0;
  }
  // cout << "z_depth best: " << z_depth << endl;

  for(int i = -X_depth; i <= X_depth; ++i){
    for(int j = -Y_depth; j <= Y_depth; ++j){
      Eigen::Vector3d point_inZ(point.x() + i * Octomap_Resolution_, point.y() + j * Octomap_Resolution_, z_depth);
  
      density_val += Get_Point_FisherInfogain(point_inZ);

      // for(int k = 0; k <= Z_depth; ++k){
      //   point_inZ.z() -= Octomap_Resolution_;
      //   density_val += Get_Point_Infogain(point_inZ);
      // }
    }
  }
  return density_val;
}


void CalAllCell_Infos_Save(octomap::OcTree* map_tree_)
{

  std::vector<Eigen::Vector3d> point_pos_vec;
  std::vector<double> fisherinfo_save;
  std::vector<double> densityinfo_save;
  std::vector<double> totalinfo_save;

  int leafs_count = 0;

  // Eigen::Matrix4d t_w_c = Get_Twc_Infomation();
  Eigen::Matrix4d t_w_c = g_T_w_c_Fisherinfo;

  int Z_depth = 6;

  for (octomap::OcTree::leaf_iterator it = map_tree_->begin_leafs(), end = map_tree_->end_leafs(); it != end; ++it)
  {
    if (map_tree_->isNodeOccupied(*it))
    {
      leafs_count++;

      octomap::point3d p = it.getCoordinate();
      // ROS_INFO_STREAM("Eigen::Vector3d : " << p);

      // if(p.z() < -0.8 || p.z() > -0.4){
      //   return;
      // }

      Eigen::Vector3d p_w_(p.x(), p.y(), p.z());
      point_pos_vec.push_back(p_w_);

      double gain_total = 1.0;
      double gain_info = 1.0;
      int gain_dens = 1;

      gain_dens = Get_Zaxis_Density(p_w_);

      float density_val = 0;
      for(int j = -Z_depth/2; j <= Z_depth/2; ++j){
        p_w_.z() = j * Octomap_Resolution_;
        density_val += Get_Point_FisherInfogain(p_w_);
      }

      gain_info = density_val;

      // density 大于3 才进行信息计算，滤出杂点
      // if(gain_dens >= Dens_Thresh_info)
      // {
      //   octomap::point3d temp(p_w_.x(), p_w_.y(), p_w_.z());

      //   octomap::OcTreeNode *node = maptree_igo_->search(temp);

      //   if (node != NULL && maptree_igo_->isNodeOccupied(node))
      //   {
      //     Eigen::Vector3d p_w_(temp.x(), temp.y(), temp.z());

      //     gain_info = Cal_Cell_Info(p_w_, g_T_w_c_Fisherinfo);
      //     std::cout << "search FisherInfo: " << gain_info << std::endl;
      //   }
      // }

      // ROS_INFO_STREAM("Fisher Infomation : " << fisher_info);

      fisherinfo_save.push_back(gain_info);
      densityinfo_save.push_back(gain_dens);

      gain_total = gain_dens * gain_info;
      totalinfo_save.push_back(gain_total);
    }
  }

  ROS_INFO_STREAM("leafs_count : " << leafs_count);
  // LOG(WARNING) << "Saving density information took " << timer.stop() * 1000 << " ms";
  // LOG(WARNING) << "after feature points size : " << points_world.size() << "......\n";
}


void CalAllCell_Infos_SaveZaxis(octomap::OcTree* map_tree_)
{

  std::vector<Eigen::Vector3d> point_pos_vec;
  std::vector<double> fisherinfo_save;
  std::vector<double> densityinfo_save;
  std::vector<double> totalinfo_save;

  int leafs_count = 0;

  // Eigen::Matrix4d t_w_c = Get_Twc_Infomation();
  Eigen::Matrix4d t_w_c = g_T_w_c_Fisherinfo;

  for (octomap::OcTree::leaf_iterator it = map_tree_->begin_leafs(), end = map_tree_->end_leafs(); it != end; ++it)
  {
    if (map_tree_->isNodeOccupied(*it))
    {
      leafs_count++;

      octomap::point3d p = it.getCoordinate();
      // ROS_INFO_STREAM("Eigen::Vector3d : " << p);

      // if(p.z() < -0.8 || p.z() > -0.4){
      //   return;
      // }

      Eigen::Vector3d p_w_(p.x(), p.y(), p.z());
      point_pos_vec.push_back(p_w_);

      double gain_total = 1.0;
      double gain_info = 1.0;
      int gain_dens = 1;

      gain_dens = Get_Point_Density(p_w_);

      // density 大于3 才进行信息计算，滤出杂点
      // if(gain_dens >= Dens_Thresh_info)
      {
        octomap::point3d temp(p_w_.x(), p_w_.y(), p_w_.z());

        octomap::OcTreeNode *node = maptree_igo_->search(temp);

        if (node != NULL && maptree_igo_->isNodeOccupied(node))
        {
          Eigen::Vector3d p_w_(temp.x(), temp.y(), temp.z());

          gain_info = Cal_Cell_Info(p_w_, g_T_w_c_Fisherinfo);
          std::cout << "search FisherInfo: " << gain_info << std::endl;
        }
      }

      // ROS_INFO_STREAM("Fisher Infomation : " << fisher_info);

      fisherinfo_save.push_back(gain_info);
      densityinfo_save.push_back(gain_dens);

      gain_total = gain_dens * gain_info;
      totalinfo_save.push_back(gain_total);
    }
  }

  ROS_INFO_STREAM("leafs_count : " << leafs_count);

  // LOG(WARNING) << "Saving density information took " << timer.stop() * 1000 << " ms";
  // LOG(WARNING) << "after feature points size : " << points_world.size() << "......\n";
}

void CalAllCell_Info(octomap::OcTree* map_tree_)
{

  std::vector<Eigen::Vector3d> point_pos_vec;
  std::vector<double> fisherinfo_save;

  int leafs_count = 0;

  // Eigen::Matrix4d t_w_c = Get_Twc_Infomation();
  Eigen::Matrix4d t_w_c = g_T_w_c_Fisherinfo;

  for (octomap::OcTree::leaf_iterator it = map_tree_->begin_leafs(), end = map_tree_->end_leafs(); it != end; ++it)
  {
    if (map_tree_->isNodeOccupied(*it))
    {
      leafs_count++;

      octomap::point3d p = it.getCoordinate();
      // ROS_INFO_STREAM("Eigen::Vector3d : " << p);

      Eigen::Vector3d p_w_(p.x(), p.y(), p.z());
      point_pos_vec.push_back(p_w_);

      Matrix36 J = Cal_Jacobi(p_w_, t_w_c.inverse());

      // ROS_INFO_STREAM("Infomation Matrix : " << J);

      Matrix66 curr_info;
      curr_info = J.transpose() * J;
      double fisher_info = curr_info.trace();

      // ROS_INFO_STREAM("Fisher Infomation : " << fisher_info);

      fisherinfo_save.push_back(fisher_info);
    }
  }

  ROS_INFO_STREAM("leafs_count : " << leafs_count);

  // LOG(WARNING) << "Saving density information took " << timer.stop() * 1000 << " ms";
  // LOG(WARNING) << "after feature points size : " << points_world.size() << "......\n";
}
