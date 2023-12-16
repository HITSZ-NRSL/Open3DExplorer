#include <ros/ros.h>

#include <chrono>
#include <numeric>
#include <queue>
#include <nav_msgs/OccupancyGrid.h>
// #include <range_mi/MIGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <vector> 
// #include "range_mi/grid_mi.hpp"
// #include "range_mi/grid_line.hpp"

using namespace std;

#define def_Predefined_traj_TEST 0

#define def_USING_ORBSLAM 1

#define def_w_uncertainty 1   // 1: semantic
#define def_octo_generator 1  // 0:using octoserver for maze; 1:using octomap genrator for plane and terrain

#define m_Frontier3D_depth (int)(15)

extern bool def_maze_mode;
extern bool def_terrain_mode;

extern float explore_range_blx;
extern float explore_range_bly;
extern float explore_range_urx;
extern float explore_range_ury;


extern double g_param_2dfsmi_range;
extern double g_param_3dfsmi_range;
extern bool g_param_buffer_strategy;
extern double g_param_weight_direction;
extern double g_param_weight_distance;
extern double g_param_visualize_fpsize;
extern double g_param_thresh_toexplore;


// Map data

extern ros::Publisher mi_map_pub;
extern ros::Publisher frontier_pub;

// Initialize a place for mutual information

typedef Eigen::Vector2i GridIndex;
//从世界坐标系转换到栅格坐标系
// GridIndex ConvertWorld2GridIndex(Eigen::Vector2d pos);
// Eigen::Vector2d ConvertGridIndex2World(Eigen::Vector2i index);

#if def_octo_generator
extern octomap::ColorOcTree *_octree;
#else
extern octomap::OcTree *_octree;
#endif
// extern octomap::OcTree* frontier3d_octree;
// extern octomap::ColorOcTree *frontier3d_octree;

extern std::vector<Eigen::Vector3d> g_Frontiers3D_pos;
extern std::vector<Eigen::Vector3d> g_Frontiers_Cluster_pos;
extern vector<Eigen::Vector3d> Frontier3D_black_list;

// extern std::vector<Eigen::Vector2i> g_Frontier2D_pos;
extern std::vector<Eigen::Vector2i> g_Frontier2D_Cluster_pos;

extern std::vector<double> fsmi_data;
extern vector<Eigen::Vector2i> frontier2d_vector;
extern Eigen::Vector3d m_Cur_position;

extern float frontier_range;

extern nav_msgs::OccupancyGrid _gridmap;
// typedef Eigen::Vector2i GridIndex;
//从世界坐标系转换到栅格坐标系
GridIndex ConvertWorld2GridIndex(Eigen::Vector2d pos);
GridIndex ConvertWorld2GridIndex(double pos_x, double pos_y);

//从栅格坐标系转换到世界坐标系
Eigen::Vector2d ConvertGridIndex2World(Eigen::Vector2i index);
Eigen::Vector2d ConvertGridIndex2World(int index_x, int index_y);
bool IsInLimitArea(const Eigen::Vector2d& _query_point);
double get_mapz(const Eigen::Vector2d _pos);

extern bool Explore_Done_2d;
extern bool All_FP_inarea;

void draw(
    unsigned int height,
    unsigned int width,
    double x,
    double y,
    double theta,
    unsigned int range,
    unsigned int *const line,
    double *const widths,
    unsigned int &num_cells);

double func(double delta, double vacancy);

void visualize_frontier(const vector<Eigen::Vector2i> frontier2d_vector);

void draw_fsmi_map();


// 最佳观测角度,0代表朝上，逆时针
extern double theta_best;
extern Eigen::Vector2d Body_dir_avg;
extern Eigen::Vector2d Motion_dir;
extern Eigen::Vector2i cur_goal;
// 这里的x，y是index
double compute_fsmi_point(unsigned int x, unsigned int y, double _range, bool _area_limit);

extern nav_msgs::OccupancyGrid g_Height_deviation_map;

void test_mi();

// float find_best_theta(unsigned int x, unsigned int y);
float find_best_theta(unsigned int x, unsigned int y, Eigen::Vector2d _query_pos, float _range, int _sub_beam, bool _area_limit);
float find_best_theta(unsigned int x, unsigned int y, float _range, int _sub_beam, bool _area_limit);

// float find_best_yaw(unsigned int x, unsigned int y, float _range, float & ret_info);

Eigen::Vector2i convert_1d_2d(int one_index);

// 实时计算过程中使用，统计每个点的信息
Eigen::Vector2i compute_mi_map_point(const vector<Eigen::Vector2i> frontier2d_vector, Eigen::Vector2i current_pos_idx);

// 用于整张图的可视化
int compute_fsmi_map();

Eigen::Vector2i mutual_information_compute(const nav_msgs::OccupancyGrid &map_msg, vector<Eigen::Vector2i> frontier2d_vector, Eigen::Vector2i current_pos_idx);
void vacancy_construction(const nav_msgs::OccupancyGrid &map_msg);
