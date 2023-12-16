#ifndef GRIDMAP_RRT_
#define GRIDMAP_RRT_

#include <ros/ros.h>
#include <ros/console.h>
#include <log4cxx/logger.h>
#include <nav_msgs/OccupancyGrid.h>

#include "../include/fsmi_cal.h"
#include <chrono>
#include <fstream>

// struct Node;
struct Node {
    vector<Node *> children;
    Node *parent;
    Eigen::Vector3d position;
    // Eigen::Vector3d orientation;
    float gain;
    float total_gain;
};

// double gain_;


/**
 * @brief initialize base information of this planning
 * @param start_position
 * @param end_position
 * @param map
 * @param max_iter
 * @param step_size
 */
void RRT2D_init(Eigen::Vector3d start_position, Eigen::Vector3d end_position, int max_iter, float step_size, short radius);
void RRT3D_init(Eigen::Vector3d start_position, Eigen::Vector3d start_orientation, Eigen::Vector3d end_position, Eigen::Vector3d end_orientation, int max_iter, float step_size, short radius);

bool isInfeatureless(Eigen::Vector3d point_position);


void deleteNodes(Node *root);

Node *getRandomNotObstacleNode();

Node *getRandomNode();

Node *findNearestNode(Eigen::Vector3d current_position);

/**
 * @brief 从树中离采样点最近的节点开始，向采样点延伸stepsize，获得q_new,但是另外两个参数没啥用啊
 * @param q_rand
 * @param q_nearest
 * @param direction
 * @return
 */
Node *getNewNode(Node *q_rand, Node *q_nearest, Eigen::Vector3d direction);

/**
 * @brief 检测q_new是否位于不可检测区域，分多段检测，就像之前写的demo一样，如果只检测q_new位置出有没有障碍
 * 是不合适的，因为从q_nearest到q_new这段距离，如果中间遇到障碍物也是不可以的，这样处理也可以适应不规则障碍物
 * @param q_new
 * @param q_nearest
 * @param direction
 * @return
 */
bool isNewNodeCollision(Eigen::Vector3d q_new, Eigen::Vector3d q_nearest, Eigen::Vector3d direction);

double border_gain(Node *q_new);

int gain_freeVoxels(Node *q_new);

double compute_gain(Node *q_new);

/**
 * @brief 如果q_new合适的话，就把它填入路径
 * @param q_nearest
 * @param q_new
 */
void addNewNode(Node *q_nearest, Node *q_new);

/**
 * @brief 判断是否到达目的地
 * @return
 */
bool isArrived();

extern vector<Eigen::Vector3d> rrt2d_sample_point;
extern vector<Node *> path_;
extern vector<float> rrt2d_sample_point_gain;

// bool run_wo_perception();

bool run_w_perception();

Node *chooseParent(Node *&q);

void rewire(Node *&q);

/**
 * @brief visualize path by writing path into map module
 */
void writeMap();

void writeInfo2File(std::string output_name);

/**
 * @brief Delete all nodes using DFS technique.
 * @param root
 */
void deleteAllNodes(Node *root);

void deleteNode(Node *node);

#endif