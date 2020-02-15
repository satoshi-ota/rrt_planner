#include "path_planning_node.h"

namespace rrt_planner
{

PathPlanningNode::PathPlanningNode
    (const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh)
{
    initPlanner();
    setStart();
    setGoal();

    path_pub_ = nh_.advertise<nav_msgs::Path>("rrt_path",1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("marker_array", 1);
    server_ = new interactive_markers::InteractiveMarkerServer("simple_marker");

    srv_ = boost::make_shared
            <dynamic_reconfigure::Server<rrt_planner::RRTParametersConfig>>(private_nh);
    dynamic_reconfigure::Server<rrt_planner::RRTParametersConfig>::CallbackType cb
        = boost::bind(&PathPlanningNode::paramsReconfig, this, _1, _2);
    srv_->setCallback(cb);

    setInteractiveMarkers();
}

PathPlanningNode::~PathPlanningNode(){}

void PathPlanningNode::createInteractiveMarker(const Eigen::Vector3d& init_pos,
                                               const Eigen::Vector3d& size,
                                               unsigned int id, std::string frame)
{
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = frame;
    int_marker.header.stamp = ros::Time::now();
    std::ostringstream oss;
    oss << id;
    int_marker.name = oss.str();
    int_marker.description = "Obstacle";
    int_marker.pose.position.x = init_pos.x();
    int_marker.pose.position.y = init_pos.y();
    int_marker.pose.position.z = init_pos.z();
    int_marker.pose.orientation.w = 1.0f;

    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.id = id;
    box_marker.scale.x = size.x();
    box_marker.scale.y = size.y();
    box_marker.scale.z = size.z();
    box_marker.color.r = 0.5;
    box_marker.color.g = 0.5;
    box_marker.color.b = 0.5;
    box_marker.color.a = 1.0;
    box_marker.pose.orientation.w = 1.0f;

    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back(box_marker);

    int_marker.controls.push_back(box_control);

    visualization_msgs::InteractiveMarkerControl move_control;
    move_control.name = "move_xy";
    move_control.orientation.w = 0.707107f;
    move_control.orientation.x = 0;
    move_control.orientation.y = 0.707107f;
    move_control.orientation.z = 0;
    move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(move_control);

    move_control.name = "move_yz";
    move_control.orientation.w = 0;
    move_control.orientation.x = 0;
    move_control.orientation.y = 0;
    move_control.orientation.z = 1.0f;
    move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(move_control);

    auto cb = boost::bind(&PathPlanningNode::processFeedback, this, _1);
    server_->insert(int_marker, cb);

    rrt_planner_->addObstacles(init_pos, size);
}

void PathPlanningNode::setInteractiveMarkers()
{
    Eigen::Vector3d init_pos;
    Eigen::Vector3d size;
    init_pos << 1.0, 1.0, 1.0;
    size << 0.2, 0.2, 0.2;
    createInteractiveMarker(init_pos, size, 0, "/map");
    init_pos << 1.0, -1.0, 1.0;
    size << 0.2, 0.2, 0.2;
    createInteractiveMarker(init_pos, size, 1, "/map");

    server_->applyChanges();
}

void PathPlanningNode::initPlanner()
{
    rrt_planner_ = new RRTPlanner(DIM4);

    Eigen::VectorXd c_space_min;
    c_space_min.resize(4, 1);
    c_space_min << -10.0, -10.0, -10.0, -10.0;
    Eigen::VectorXd c_space_max;
    c_space_max.resize(4, 1);
    c_space_max << 10.0, 10.0, 10.0, 10.0;
    rrt_planner_->setCSpace(c_space_min, c_space_max);

    Eigen::VectorXd c_constrain_min;
    c_constrain_min.resize(4, 1);
    c_constrain_min << -5.0, -3.0, 0.0, 0.0;
    Eigen::VectorXd c_constrain_max;
    c_constrain_max.resize(4, 1);
    c_constrain_max << 5.0, 3.0, 5.0, 1.0;
    rrt_planner_->setConstrains(c_constrain_min, c_constrain_max);
}

void PathPlanningNode::paramsReconfig(rrt_planner::RRTParametersConfig &config, uint32_t level)
{
    rrt_planner_->reconfig(config);
}

void PathPlanningNode::setStart()
{
    Eigen::VectorXd c_state_init;
    c_state_init.resize(4, 1);
    c_state_init << -2.0, 0.0, 0.0, 0.0;
    rrt_planner_->initTree(c_state_init);
}

void PathPlanningNode::setGoal()
{
    Eigen::VectorXd c_state_goal;
    c_state_goal.resize(4, 1);
    c_state_goal << 4.0, 0.0, 2.0, 1.0;
    rrt_planner_->setGoal(c_state_goal);
}

void PathPlanningNode::run()
{
    rrt_planner_->run();

    nav_msgs::Path path_msg;
    path_msg.poses.clear();
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id="/map";
    rrt_planner_->copyToMsg(path_msg);
    path_pub_.publish(path_msg);

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.clear();
    rrt_planner_->showArrow(marker_array);
    marker_pub_.publish(marker_array);
}

void PathPlanningNode::processFeedback
    (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    std::stringstream ss(feedback->marker_name);
    int index;
    ss >> index;

    Eigen::Vector3d pos(feedback->pose.position.x,
                        feedback->pose.position.y,
                        feedback->pose.position.z);

    rrt_planner_->updateObstacles(pos, index);

    setStart();
}

} //namespace rrt_planner

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrt_path_planning_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    rrt_planner::PathPlanningNode path_planning_node(nh, private_nh);

    while(ros::ok())
    {
        path_planning_node.run();
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }

    return 0;
}
