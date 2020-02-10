#ifndef RRT_PLANNER_PATH_PLANNING_NODE_H
#define RRT_PLANNER_PATH_PLANNING_NODE_H

#include <ros/ros.h>

#include "rrt_planner/rrt_planner.h"

namespace rrt_planner
{

class PathPlanningNode
{
public:
    PathPlanningNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~PathPlanningNode();

    void run();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    RRTPlanner rrt_planner_;

};

} //namespace rrt_planner

#endif //RRT_PLANNER_PATH_PLANNING_NODE_H
