#ifndef GRIDMAP_RRT_3D_H 
#define GRIDMAP_RRT_3D_H

#include <log4cxx/logger.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <chrono>
#include <fstream>

#include "../include/fsmi_cal.h"
#include "../include/terrain_map.h"
    
extern int Fail_grow_count;

extern vector<Eigen::VectorXd> track_path_3d;    // 顺序，[0]对应的是起点
extern bool EnsureTrackingAcc;

void Viewpoint_Check_Frontier(Eigen::Vector3d _cur_position, Eigen::Vector3d _cur_orientation);
extern bool Explore_Done_3d;
class rrt_3d:public terrain_mapping{
 public:
  rrt_3d(Eigen::Vector3d start_position, Eigen::Vector3d start_orientation,
         Eigen::Vector3d end_position, Eigen::Vector3d end_orientation,
         int max_iter, float step_size);

  ~rrt_3d();

  struct Node_3d;
  struct Node_3d {
    vector<Node_3d *> children;
    Node_3d *parent;
    Eigen::Vector3d position;
    Eigen::Vector3d orientation;
    float gain;
    float total_gain;
    float cost_goal;
    float total_cost_goal;
    float cost_motion;
    float total_cost_motion;
    bool is_adjust_construct;
    bool is_adjust_explore;
  };

  Eigen::Vector3d getBBXMax();
  Eigen::Vector3d getBBXMin();

  void draw_line(unsigned int height, unsigned int width, double x, double y,
                 double theta, unsigned int range, unsigned int *const line,
                 double *const widths, unsigned int &num_cells);
  bool isInfeatureless(Eigen::Vector3d point_position, Eigen::Vector3d point_orientation);
  bool isNearWall(Eigen::Vector3d point_position);

  Eigen::Vector2i Convert2pos(int index);

  bool isViewInfeatureless(Eigen::Vector3d point_position, Eigen::Vector3d point_orientation);
  bool isInKnown_Feature_Area(Eigen::Vector3d point_position);

  bool OutofRange(Eigen::Vector3d point_position);

  void deleteNodes(Node_3d *root);

  bool In_Circle_Range(Eigen::Vector3d rand_position);
  
  Node_3d * getRandomNotObstacleNode_3d();
  Node_3d * getRandomNotObstacleNode_3d_inRange();
  Node_3d * getNewNode_3d(Node_3d *q_rand, Node_3d *q_nearest, Eigen::Vector3d direction);
  Node_3d * getNewNode_3d(Node_3d *q_rand, Node_3d *q_nearest, Eigen::Vector3d direction_position, Eigen::Vector3d direction_orientation);
  Eigen::Vector3d getRandom_aroundFrontier(Eigen::Vector3d _Position);

  Node_3d * findNearestNode(Eigen::Vector3d current_position);
  Node_3d * findNearestNode(Node_3d *q_node);

  float gain_exploration_2d(Node_3d *q_new);

  bool fpInFOV(Node_3d *q_node, Eigen::Vector3d frontierpoint);
  bool fpInFOV(Eigen::Vector3d _sample_around_Frontier, Eigen::Vector3d _frontierCluster, Eigen::Vector3d _frontierpoint, int& _unknown_num);

  int fpNumber_InFOV(Eigen::Vector3d _sample_around_Frontier, Eigen::Vector3d _frontierCluster, int& count_unknow);

  // bool get_intersect_point(Eigen::Vector3d node_position, Eigen::Vector3d intersect_vec, octomap::point3d& _Intersected_point);
  void intersection_point_visualization(Node_3d *q_new);
  
  Eigen::VectorXd Extract_BestObserve_Frontier3d();

  float gain_perception(Eigen::Vector3d position, Eigen::Vector3d orinientation);
  float gain_perception(Node_3d *q_new);
  double test_yaw(Eigen::Vector3d node_pos, Eigen::Vector3d frontierpoint);
  int gain_construction(Node_3d *q_new);
  bool Frontier3D_inRange(Node_3d *q_node);


  void addNewNode(Node_3d *q_nearest, Node_3d *q_new);
  bool addNewNode_rrtstar(Node_3d *q_new);

  bool isArrived(Node_3d *node);

  // bool run_w_perception();
  bool run_w_perception_layered();

  Node_3d *chooseParent(Node_3d *&q);
  Node_3d *chooseParent_gain(Node_3d *&q);

  void rewire(Node_3d *&q);
  void rewire_gain(Node_3d *&q);

  void writeMap();

  void writeInfo2File(std::string output_name);

  void deleteAllNodes(Node_3d *root);

  void deleteNode(Node_3d *node);
  void Update_Track_path();
  void Enter_recover_mode();
  bool Line_NoCollision(Eigen::Vector3d _start, Eigen::Vector3d _end);
  bool isInfeatureless(Eigen::Vector3d point_position);

  bool Flag_visualize_intersact_points = false;
  vector<Eigen::Vector3d> rrt3d_intersection_point_visualization;

  vector<Eigen::Vector3d> rrt3d_sample_position;
  vector<Eigen::Vector3d> rrt3d_sample_orientation;

  vector<Node_3d *> path_3d;    // 倒序，[0]对应的是终点
  vector<float> rrt3d_sample_point_gain;
  vector<Eigen::Vector3d> g_BestPositionSet_aroundFrontier;
  vector<Eigen::Vector3d> g_BestOrinientSet_aroundFrontier;
  vector<float> g_sampleSet_fpNum;
  

  // nav_msgs::OccupancyGrid Height_map;

  // nav_msgs::OccupancyGrid Height_deviation_map;
  // nav_msgs::OccupancyGrid Slope_map;
  // nav_msgs::OccupancyGrid Curvature_map;

    Eigen::Vector3d end_position_;
    Eigen::Vector3d end_orientation_;
    vector<Eigen::Vector3d> rrt3d_intersection_point;


  private:
    Eigen::Vector3d get_intersect_point_frontierOctomap(Eigen::Vector3d node_position, Eigen::Vector3d intersect_line_dir);
    Eigen::Vector3d get_OrinientFromVector(Eigen::Vector3d _origin_point, Eigen::Vector3d _target_point);
    void Insert_node_withorinient();

    Node_3d * findNearestNode_Inpath3d(Eigen::Vector3d current_position);
    void Insert_path3d_node();
    void Smooth_node_path();
    void Find_Exploration2d_node();
    void Find_Frontier2d_node();
    void AdjustNode_for_perception();

    void Extract_Subtree_First_maxfp();

    int max_iter_;
    //step_size: 1 unit based
    float step_size_position;
    float step_size_orientation = 0.2;
    float radius;   //range to choose new parent node

    vector<Node_3d *> rrt2d_nodes_;

    Eigen::Vector3d start_position_;
    Eigen::Vector3d start_orientation_;

    // vector<Node_3d *> path_3d;
    Node_3d *root_;
    Node_3d *lastNode_;

    double bestGain_;
    double bestGain_notarrive;
    Node_3d *bestNode_;
    Node_3d *bestNode_notarrive;

    vector<Node_3d*> Near;//没有进行初始化,并且为什么要用*
    vector<double> cost;//起点到近邻点的距离
    vector<double> dist;//近邻点到新节点的距离
    vector<double> m_gain_vec;//近邻点到新节点的距离
    vector<double> m_totalgain_vec;//近邻点到新节点的距离
    double minCost;
    double maxGain;
    int idx_maxCost;


    float lamda = 1;
    float success_distance = 3;
    float m_choose_radius;
    

    // octomap::point3d seed_fp_infov{0,0,0};

};
#endif