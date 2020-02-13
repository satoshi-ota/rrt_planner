#ifndef RRT_PLANNER_RRT_PLANNER_H
#define RRT_PLANNER_RRT_PLANNER_H

#include <math.h>
#include <random>
#include <Eigen/Core>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

namespace rrt_planner
{

const double start_x = -2.0;
const double start_y = 0.0;
const double start_z = 0.0;
const double start_w = 0.0;

const double goal_x = 4.0;
const double goal_y = 2.0;
const double goal_z = 2.0;
const double goal_w = 1.0;

enum Category
{
    NONE  = 0,
    START = 1,
    GOAL  = 2,
};

struct Node
{
    Node()
        :c_pos(Eigen::VectorXd::Zero(4)),
         cat(NONE){}
    Node(Eigen::VectorXd& _pos, Category _cat)
        :c_pos(_pos),
         cat(_cat){}
    ~Node(){}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d pos;

    Eigen::VectorXd c_pos;

    int parent_node;
    Category cat;
};

struct Obstacle
{
    Obstacle(Eigen::Vector3d& _pos, Eigen::Vector3d& _size)
        :pos(_pos),
         size(_size){}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d pos;
    Eigen::Vector3d size;
};

class RRTPlanner
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RRTPlanner();
    ~RRTPlanner();

    void run();

    void setParams(double delta, double goal_tolerance);

    void initTree(Eigen::VectorXd& start);

    void initObstacles();
    void addObstacles(Eigen::Vector3d& pos, Eigen::Vector3d& size);
    void updateObstacles(Eigen::Vector3d& pos, int index);

    void setStart(double s_x, double s_y, double s_z);
    void setGoal(double g_x, double g_y, double g_z);

    bool search();
    void pathMake();
    bool checkGoal(Node& node);
    bool checkConstrain(Node& node);
    bool checkObstacle(Node& node);

    void copyToMsg(nav_msgs::Path& path_msg);
    void showArrow(visualization_msgs::MarkerArray& marker_array);

private:
    Eigen::VectorXd start_;
    Eigen::VectorXd goal_;

    Eigen::Vector3d start_w_{-2.0, 2.0, 4.0};
    Eigen::Vector3d goal_w_{2.0, 2.0, 4.0};

    Eigen::VectorXd start_config_;
    Eigen::VectorXd goal_config_;

    Eigen::Vector3d search_range_min_;
    Eigen::Vector3d search_range_max_;

    std::vector<Node> nodes_;
    std::vector<Node> path_;

    std::vector<Obstacle> obstacles_;

    double delta_{0.05};
    double goal_th_{0.1};
    double select_goal_th_{0.1};
    double obstacle_margin_{1.0};

    unsigned int max_itr{2000};

    bool goal_flag_{false};

    std::random_device seed_gen;
    // std::mt19937 engine(seed_gen());

    // 一様実数分布
    // [-1.0, 1.0)の値の範囲で、等確率に実数を生成する
    // std::uniform_real_distribution<> dist1(-1.0, 1.0);
    // std::uniform_real_distribution<> dist_x(-10.0, 10.0);
    // std::uniform_real_distribution<> dist_y(-10.0, 10.0);
    // std::uniform_real_distribution<> dist_z(-10.0, 10.0);
};

} //namespace rrt_planner

#endif //RRT_PLANNER_RRT_PLANNER_H
