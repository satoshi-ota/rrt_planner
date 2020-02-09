#include "path_planning_node.h"

namespace rrt_planner
{

class PathPlanningNode
{

PathPlanningNode::PathPlanningNode(){}

PathPlanningNode::~PathPlanningNode(){}

};

} //namespace rrt_planner

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrt_path_planning_node");
    ros::NodeHandle nh;


    return 0;
}
