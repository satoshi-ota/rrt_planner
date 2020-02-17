#include "path_display_node.h"

namespace rrt_planner
{

PathDisplayNode::PathDisplayNode
    (const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh)
{
    array_sub_ = nh_.subscribe("marker_array", 1,
                    &PathDisplayNode::marker_cb, this);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("marker", 1);
}

PathDisplayNode::~PathDisplayNode(){}

void PathDisplayNode::marker_cb(const visualization_msgs::MarkerArrayConstPtr &feedback)
{
    for(int i = 0; i < feedback->markers.size(); i++)
    {
        visualization_msgs::Marker marker;

        marker = feedback->markers[i];

        marker.scale.x = 0.05;
        marker.scale.y = 0.1;
        marker.scale.z = 0.03;

        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;

        marker.lifetime = ros::Duration(0.05);

        marker_pub_.publish(marker);
        ros::Duration(0.05).sleep();
    }
}

} //namespace rrt_planner

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrt_path_display_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    rrt_planner::PathDisplayNode path_display_node(nh, private_nh);

    ros::spin();

    return 0;
}
