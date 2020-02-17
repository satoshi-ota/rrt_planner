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

    uav_path_pub_ = nh_.advertise<nav_msgs::Path>("rrt_path/uav",1);
    win_path_pub_ = nh_.advertise<nav_msgs::Path>("rrt_path/winch",1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("marker_array", 1);
    bound_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBox>("bound", 1);
    server_ = new interactive_markers::InteractiveMarkerServer("simple_marker");

    srv_ = boost::make_shared
            <dynamic_reconfigure::Server<rrt_planner::RRTParametersConfig>>(private_nh);
    dynamic_reconfigure::Server<rrt_planner::RRTParametersConfig>::CallbackType cb
        = boost::bind(&PathPlanningNode::paramsReconfig, this, _1, _2);
    srv_->setCallback(cb);

    setMarkerPreset();
    setInteractiveMarkers();
}

PathPlanningNode::~PathPlanningNode(){}

void PathPlanningNode::setMarkerPreset()
{
    obstacle_preset_.r = 0.5;
    obstacle_preset_.g = 0.5;
    obstacle_preset_.b = 0.5;
    obstacle_preset_.a = 1.0;

    start_preset_.r = 0.0;
    start_preset_.g = 0.0;
    start_preset_.b = 1.0;
    start_preset_.a = 1.0;

    goal_preset_.r = 1.0;
    goal_preset_.g = 0.0;
    goal_preset_.b = 0.0;
    goal_preset_.a = 1.0;
}

void PathPlanningNode::createInteractiveMarker(const Eigen::Vector3d& init_pos,
                                               const Eigen::Vector3d& size,
                                               const std_msgs::ColorRGBA& color,
                                               unsigned int id, std::string frame, std::string name)
{
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = frame;
    int_marker.header.stamp = ros::Time::now();
    std::ostringstream oss;
    oss << id;
    int_marker.name = oss.str();
    int_marker.description = name;
    int_marker.pose.position.x = init_pos.x();
    int_marker.pose.position.y = init_pos.y();
    int_marker.pose.position.z = init_pos.z();
    int_marker.pose.orientation.w = 1.0f;

    visualization_msgs::Marker sphere_marker;
    sphere_marker.type = visualization_msgs::Marker::SPHERE;
    sphere_marker.id = id;
    sphere_marker.scale.x = size.x();
    sphere_marker.scale.y = size.y();
    sphere_marker.scale.z = size.z();
    sphere_marker.color = color;
    sphere_marker.pose.orientation.w = 1.0f;

    visualization_msgs::InteractiveMarkerControl sphere_control;
    sphere_control.name = name;
    sphere_control.always_visible = true;
    sphere_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
    sphere_control.markers.push_back(sphere_marker);

    int_marker.controls.push_back(sphere_control);

    auto cb = boost::bind(&PathPlanningNode::processFeedback, this, _1);
    server_->insert(int_marker, cb);

    if(name == "Obstacle") rrt_planner_->addObstacles(init_pos, size);
}

void PathPlanningNode::setInteractiveMarkers()
{
    Eigen::Vector3d init_pos;
    Eigen::Vector3d size;
    init_pos << 1.0, 1.0, 1.0;
    size << 1.0, 1.0, 1.0;
    createInteractiveMarker(init_pos, size, obstacle_preset_, 0, "/map", "Obstacle");
    init_pos << 1.0, -1.0, 1.0;
    size << 1.0, 1.0, 1.0;
    createInteractiveMarker(init_pos, size, obstacle_preset_, 1, "/map", "Obstacle");
    init_pos << 1.0, -1.0, 2.0;
    size << 1.0, 1.0, 1.0;
    createInteractiveMarker(init_pos, size, obstacle_preset_, 2, "/map", "Obstacle");
    init_pos << 3.0, -1.0, 2.0;
    size << 1.0, 1.0, 1.0;
    createInteractiveMarker(init_pos, size, obstacle_preset_, 3, "/map", "Obstacle");

    init_pos << -2.0, 0.0, 0.0;
    size << 0.2, 0.2, 0.2;
    createInteractiveMarker(init_pos, size, start_preset_, 4, "/map", "Start");
    init_pos << 4.0, 0.0, 2.0;
    size << 0.2, 0.2, 0.2;
    createInteractiveMarker(init_pos, size, goal_preset_, 5, "/map", "Goal");

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
    c_constrain_min << -5.0, -2.0, 0.0, 0.0;
    Eigen::VectorXd c_constrain_max;
    c_constrain_max.resize(4, 1);
    c_constrain_max << 5.0, 2.0, 5.0, 1.0;
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

    nav_msgs::Path uav_path_msg;
    uav_path_msg.poses.clear();
    uav_path_msg.header.stamp = ros::Time::now();
    uav_path_msg.header.frame_id="/map";
    nav_msgs::Path win_path_msg;
    win_path_msg.poses.clear();
    win_path_msg.header.stamp = ros::Time::now();
    win_path_msg.header.frame_id="/map";
    rrt_planner_->copyToMsg(uav_path_msg, win_path_msg);
    uav_path_pub_.publish(uav_path_msg);
    win_path_pub_.publish(win_path_msg);

    jsk_recognition_msgs::BoundingBox bound_msg;
    bound_msg.header.stamp = ros::Time::now();
    bound_msg.header.frame_id = "/map";
    bound_msg.pose.position.x = 0.0;
    bound_msg.pose.position.y = 0.0;
    bound_msg.pose.position.z = 2.5;
    bound_msg.pose.orientation.w = 1.0;
    bound_msg.dimensions.x = 10.0;
    bound_msg.dimensions.y = 4.0;
    bound_msg.dimensions.z = 5.0;
    bound_pub_.publish(bound_msg);

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.clear();
    rrt_planner_->showArrow(marker_array);
    marker_pub_.publish(marker_array);
}

void PathPlanningNode::processFeedback
    (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    if (feedback->control_name == "Obstacle")
    {
        std::stringstream ss(feedback->marker_name);
        int index;
        ss >> index;

        Eigen::Vector3d pos(feedback->pose.position.x,
                            feedback->pose.position.y,
                            feedback->pose.position.z);

        rrt_planner_->updateObstacles(pos, index);
    }

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
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }

    return 0;
}
