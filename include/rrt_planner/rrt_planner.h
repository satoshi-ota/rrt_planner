#ifndef RRT_PLANNER_RRT_PLANNER_H
#define RRT_PLANNER_RRT_PLANNER_H

#include <math.h>
#include <random>
#include <Eigen/Core>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include "rrt_planner/RRTParametersConfig.h"

namespace rrt_planner
{

enum ConfigDim
{
    DIM1 = 1,
    DIM2 = 2,
    DIM3 = 3,
    DIM4 = 4,
    DIM5 = 5,
    DIM6 = 6,
};

enum Category
{
    NONE  = 0,
    START = 1,
    GOAL  = 2,
};

enum MarkerName
{
    MARKER_OBSTACLE  = 0,
    MARKER_START     = 1,
    MARKER_GOAL      = 2,
};

struct Node
{
    Node(const ConfigDim& _dim)
        :c_state(Eigen::VectorXd::Zero((int)_dim)),
         dim(_dim),
         cat(NONE){}
    Node(Eigen::VectorXd& _pos, Category _cat)
        :c_state(_pos),
         cat(_cat){}
    ~Node(){}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::VectorXd c_state;

    Eigen::Vector3d uav_pos;
    Eigen::Vector3d win_pos;

    int id;
    int parent_id;
    ConfigDim dim;
    Category cat;
};

struct Obstacle
{
    Obstacle(const Eigen::Vector3d& _pos, const Eigen::Vector3d& _size)
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
    RRTPlanner(const ConfigDim& config_dim);
    ~RRTPlanner();

    void run();

    void setParams(double delta, double goal_tolerance, double obstacle_margin, double select_goal_rate_, double max_itr_);

    void reconfig(rrt_planner::RRTParametersConfig &config);

    void initTree(Eigen::VectorXd& start);
    void initPath();

    void initObstacles();
    void addObstacles(const Eigen::Vector3d& pos, const Eigen::Vector3d& size);
    void updateObstacles(Eigen::Vector3d& pos, int index);

    void setStart(const Eigen::VectorXd& c_state_init);
    void setGoal(const Eigen::VectorXd& c_state_goal);

    void setCSpace(const Eigen::VectorXd& c_space_min,
                   const Eigen::VectorXd& c_space_max);

    void setConstrains(const Eigen::VectorXd& c_constrain_min,
                       const Eigen::VectorXd& c_constrain_max);

    bool search();
    void pathMake();
    bool checkGoal(const Node& node);
    bool checkConstrain(const Node& node);
    bool checkObstacle(const Node& node);
    bool checkDim(const Eigen::VectorXd& vec);

    void projection3D(Node& node);

    void copyToMsg(nav_msgs::Path& path_msg, nav_msgs::Path& win_path_msg);
    void showArrow(visualization_msgs::MarkerArray& marker_array);

private:
    Eigen::Vector3d win_init_{-2.0, 2.0, 4.0};
    Eigen::Vector3d win_goal_{2.0, 2.0, 4.0};

    Eigen::VectorXd c_state_init_;
    Eigen::VectorXd c_state_goal_;

    Eigen::VectorXd c_space_min_;
    Eigen::VectorXd c_space_max_;

    Eigen::VectorXd c_constrain_min_;
    Eigen::VectorXd c_constrain_max_;

    std::vector<Node> tree_;
    std::vector<Node> path_;

    std::vector<Obstacle> obstacles_;

    double dt_ref_{0.05};
    double goal_tolerance_{0.1};
    double select_goal_rate_{0.1};
    double obstacle_margin_{1.0};

    unsigned int max_itr_{2000};

    bool goal_flag_{false};
    bool init_flag_{false};

    ConfigDim config_dim_;

    std::random_device seed_gen_;
    std::vector<std::uniform_real_distribution<>> rands_;
};

} //namespace rrt_planner

#endif //RRT_PLANNER_RRT_PLANNER_H
