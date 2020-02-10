#include "path_planning_node.h"

namespace rrt_planner
{

PathPlanningNode::PathPlanningNode
    (const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh){}

PathPlanningNode::~PathPlanningNode(){}

void PathPlanningNode::run()
{
    rrt_planner_.run();
}

} //namespace rrt_planner

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrt_path_planning_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    rrt_planner::PathPlanningNode path_planning_node(nh, private_nh);

    path_planning_node.run();

    ros::spin();

    return 0;
}
