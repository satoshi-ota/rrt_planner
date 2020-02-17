#ifndef RRT_PLANNER_PATH_DISPLAY_NODE_H
#define RRT_PLANNER_PATH_DISPLAY_NODE_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

namespace rrt_planner
{

class PathDisplayNode
{
public:
    PathDisplayNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~PathDisplayNode();

    void marker_cb(const visualization_msgs::MarkerArrayConstPtr &feedback);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber array_sub_;
    ros::Publisher marker_pub_;
};

} //namespace rrt_planner

#endif //RRT_PLANNER_PATH_DISPLAY_NODE_H
