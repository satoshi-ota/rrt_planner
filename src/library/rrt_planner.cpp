#include "rrt_planner/rrt_planner.h"

namespace rrt_planner
{

RRTPlanner::RRTPlanner()
{
    start_ << start_x, start_y, start_z;
    goal_ << goal_x, goal_y, goal_z;

    nodes_.clear();
    nodes_.push_back(start_);
}

RRTPlanner::~RRTPlanner(){}

void RRTPlanner::setStart(double s_x, double s_y, double s_z)
{
    start_ << s_x, s_y, s_z;
}

void RRTPlanner::setGoal(double g_x, double g_y, double g_z)
{
    goal_ << g_x, g_y, g_z;
}

void RRTPlanner::search()
{
    std::mt19937 engine(seed_gen());
    std::uniform_real_distribution<> dist1(-1.0, 1.0);
    std::uniform_real_distribution<> dist_x(-10.0, 10.0);
    std::uniform_real_distribution<> dist_y(-10.0, 10.0);
    std::uniform_real_distribution<> dist_z(-10.0, 10.0);
    double r1 = dist1(engine);

    Eigen::Vector3d pos;

    if(r1 < select_goal_th_)
    {
        pos.x() = dist_x(engine);
        pos.y() = dist_y(engine);
        pos.z() = dist_z(engine);
    } else {
        pos.x() = goal_x;
        pos.y() = goal_y;
        pos.z() = goal_z;
    }

    double distance = INFINITY;

    Eigen::Vector3d nearest_node;

    for(int i = 0; i < nodes_.size(); i++)
    {
        Eigen::Vector3d node = nodes_[i];
        double d = (pos - node).norm();

        if(d < distance) nearest_node = node;
    }
    Eigen::Vector3d dx = (pos - nearest_node).normalized();
    dx = dx.array() * delta_;
    Eigen::Vector3d new_node = nearest_node + dx;

    nodes_.push_back(new_node);
    checkGoal(new_node);
    // double d = (goal_ - new_node).norm();
    // if(d < goal_th_) goal_flag_ = true;
}

void RRTPlanner::checkGoal(Eigen::Vector3d& node)
{
    double d = (goal_ - node).norm();
    if(d < goal_th_) goal_flag_ = true;
}

void RRTPlanner::run()
{
    int count = 0;
    while(goal_flag_ != true)
    {
        search();
        count++;
    }

    Eigen::Vector3d vec = nodes_.back();

    printf("FIND ITR:%d %f %f %f\n", count, vec.x(), vec.y(), vec.z());
}

} //namespace rrt_planner
