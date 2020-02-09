#include "rrt_planner/rrt_planner.h"

namespace rrt_planner
{

RRTPlanner::RRTPlanner()
{
    nodes_.clear();
    nodes_.push_back(start_);
}

RRTPlanner::~RRTPlanner(){}

void RRTPlanner::search()
{
    double r1 = dist1(engine);

    Eigen::Vector3d pos;

    if(r1 < select_goal_th_)
    {
        pos.x() = dist_x(engine);
        pos.y() = dist_y(engine);
        pos.z() = dist_z(engine);
    } else {
        pos.x() = x_goal;
        pos.y() = y_goal;
        pos.z() = z_goal;
    }

    double distance = INFINITY;

    Eigen::Vector3d nearest_node;

    for(int i = 0; i < nodes_.size(); i++)
    {
        Eigen::Vector3d node = nodes_[i];
        double d = (pos - node).norm();

        if(d < distance) nearest_node = node;
    }

    Eigen::Vector3d new_node = nearest_node + (pos - nearest_node).normalize() * delta_;

    nodes_.push_back(new_node);

    double d = (goal_ - new_node).norm();
    if(d < goal_th_) goal_flag_ = true;
}

void RRTPlanner::run()
{
    while(goal_flag_ != true)
    {
        search();
    }

    Eigen::Vector3d vec = nodes_.back();

    printf("FIND %f %f %f\n", vec.x(), vec.y(), vec.z());
}

} //namespace rrt_planner
