#ifndef RRT_PLANNER_PATH_PLANNING_NODE_H
#define RRT_PLANNER_PATH_PLANNING_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <dynamic_reconfigure/server.h>
#include <interactive_markers/interactive_marker_server.h>

#include "rrt_planner/rrt_planner.h"
#include "rrt_planner/RRTParametersConfig.h"

namespace rrt_planner
{

class PathPlanningNode
{
public:
    PathPlanningNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~PathPlanningNode();

    void setInteractiveMarkers();
    void setStart();
    void setGoal();
    void run();

    void createInteractiveMarker(const Eigen::Vector3d& init_pos, const Eigen::Vector3d& size,
                                 unsigned int id, std::string frame);

    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void paramsReconfig(rrt_planner::RRTParametersConfig &config, uint32_t level);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher path_pub_;
    ros::Publisher marker_pub_;

    boost::shared_ptr<dynamic_reconfigure::Server<rrt_planner::RRTParametersConfig>> srv_;

    RRTPlanner rrt_planner_;

    interactive_markers::InteractiveMarkerServer *server_;

};

} //namespace rrt_planner

#endif //RRT_PLANNER_PATH_PLANNING_NODE_H
