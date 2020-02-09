#ifndef RRT_PLANNER_PATH_PLANNING_NODE_H
#define RRT_PLANNER_PATH_PLANNING_NODE_H

#include <ros/ros.h>

namespace rrt_planner
{

class PathPlanningNode
{
public:
PathPlanningNode();
~PathPlanningNode();

private:

RRTPlanner rrt_planner_;

};

} //namespace rrt_planner

#endif //RRT_PLANNER_PATH_PLANNING_NODE_H
