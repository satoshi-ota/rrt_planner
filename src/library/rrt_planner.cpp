#include "rrt_planner/rrt_planner.h"

namespace rrt_planner
{

RRTPlanner::RRTPlanner()
{
    goal_.resize(4, 1);
    goal_ << goal_x, goal_y, goal_z, goal_w;

    initObstacles();

    Eigen::Vector3d pos(1.0, 1.0, 1.0);
    Eigen::Vector3d size(0.2, 0.2, 0.2);
    addObstacles(pos, size);
    pos << 1.0, -1.0, 1.0;
    size <<0.2, 0.2, 0.2;
    addObstacles(pos, size);
}

RRTPlanner::~RRTPlanner(){}

void RRTPlanner::initTree(Eigen::VectorXd& start)
{
    init_flag_ = true;
    goal_flag_ = false;

    Node node(start, START);
    nodes_.clear();
    nodes_.push_back(node);
}

void RRTPlanner::initObstacles()
{
    obstacles_.clear();
}

void RRTPlanner::initPath()
{
    path_.clear();
}

void RRTPlanner::addObstacles(Eigen::Vector3d& pos, Eigen::Vector3d& size)
{
    Obstacle obstacle(pos, size);
    obstacles_.push_back(obstacle);
}

void RRTPlanner::updateObstacles(Eigen::Vector3d& pos, int index)
{
    if(index < obstacles_.size())
    {
        obstacles_[index].pos = pos;
    } else {
        ROS_ERROR("Invalid obstacle index. ");
    }
}

void RRTPlanner::setParams(double delta, double goal_tolerance)
{
    delta_ = delta;
    goal_th_ = goal_tolerance;
}

void RRTPlanner::reconfig(rrt_planner::RRTParametersConfig &config)
{
    delta_ = config.dt_ref;
    goal_th_ = config.goal_tolerance;
    obstacle_margin_ = config.obstacle_margin;
    max_itr = config.max_itr;
}

void RRTPlanner::setStart(double s_x, double s_y, double s_z)
{
    start_ << s_x, s_y, s_z;
}

void RRTPlanner::setGoal(double g_x, double g_y, double g_z)
{
    goal_ << g_x, g_y, g_z;
}

bool RRTPlanner::search()
{
    std::mt19937 engine(seed_gen());
    std::uniform_real_distribution<> dist1(-1.0, 1.0);
    std::uniform_real_distribution<> dist_x(-10.0, 10.0);
    std::uniform_real_distribution<> dist_y(-10.0, 10.0);
    std::uniform_real_distribution<> dist_z(-10.0, 10.0);
    std::uniform_real_distribution<> dist_w(-10.0, 10.0);
    double r1 = dist1(engine);

    Eigen::VectorXd pos;
    pos.resize(4, 1);

    if(r1 < select_goal_th_)
    {
        pos(0) = dist_x(engine);
        pos(1) = dist_y(engine);
        pos(2) = dist_z(engine);
        pos(3) = dist_w(engine);
    } else {
        pos(0) = goal_x;
        pos(1) = goal_y;
        pos(2) = goal_z;
        pos(3) = goal_w;
    }

    double distance = INFINITY;

    Eigen::VectorXd nearest_node;
    Node new_node;

    for(int i = 0; i < nodes_.size(); i++)
    {
        Eigen::VectorXd node = nodes_[i].c_pos;
        double d = (pos - node).norm();

        if(d < distance){
            distance = d;
            new_node.parent_node = i;
            nearest_node = node;
        }
    }

    Eigen::VectorXd dx = (pos - nearest_node).normalized();
    dx = dx.array() * delta_;

    new_node.c_pos = nearest_node + dx;

    if(checkObstacle(new_node) && checkConstrain(new_node))
    {
        nodes_.push_back(new_node);
        if(checkGoal(new_node)) return true;
    }

    return false;
}

bool RRTPlanner::checkGoal(Node& node)
{
    double d = (goal_ - node.c_pos).norm();
    if(d < goal_th_) return true;

    return false;
}

bool RRTPlanner::checkConstrain(Node& node)
{
    if((-5.0 > node.c_pos(0)) || (5.0 < node.c_pos(0))) return false;
    if((-5.0 > node.c_pos(1)) || (5.0 < node.c_pos(1))) return false;
    if((0 > node.c_pos(2)) || (5.0 < node.c_pos(2))) return false;
    if((0 > node.c_pos(3)) || (1.0 < node.c_pos(3))) return false;

    return true;
}

void RRTPlanner::pathMake()
{
    initPath();

    Node end = nodes_[nodes_.size()-1];
    path_.push_back(end);
    int i = end.parent_node;

    while(1)
    {
        path_.push_back(nodes_[i]);
        if(nodes_[i].cat == START) break;
        i = nodes_[i].parent_node;
    }
}

bool RRTPlanner::checkObstacle(Node& node)
{
    Eigen::Vector3d dx = (goal_w_ - start_w_).array() * node.c_pos(3);
    Eigen::Vector3d w = start_w_ + dx;

    Eigen::Vector3d vec = (w - node.c_pos.topLeftCorner(3, 1)).normalized();

    for(int i = 0; i < obstacles_.size(); i++)
    {
        Eigen::Vector3d d = obstacles_[i].pos - node.c_pos.topLeftCorner(3, 1);

        double a = vec.x()*d.x() + vec.y()*d.y() + vec.z()*d.z();
        double b = vec.x()*vec.x() + vec.y()*vec.y() + vec.z()*vec.z();
        double t = a/b;

        Eigen::Vector3d v = vec.array() * t;
        Eigen::Vector3d q = node.c_pos.topLeftCorner(3, 1) + v;

        if((q - obstacles_[i].pos).norm() < obstacle_margin_) return false;
    }

    return true;
}

void RRTPlanner::copyToMsg(nav_msgs::Path& path_msg)
{
    for(int i = 0; i <path_.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = path_[i].c_pos(0);
        pose.pose.position.y = path_[i].c_pos(1);
        pose.pose.position.z = path_[i].c_pos(2);
        path_msg.poses.push_back(pose);
    }
}

void RRTPlanner::showArrow(visualization_msgs::MarkerArray& marker_array)
{
    // marker_array.markers.resize(path_.size());
    for(int i = 0; i <path_.size(); i++)
    {
        geometry_msgs::Point linear_start;
        linear_start.x = start_w_.x() + (goal_w_ - start_w_).x() * path_[i].c_pos(3);
        linear_start.y = start_w_.y() + (goal_w_ - start_w_).y() * path_[i].c_pos(3);
        linear_start.z = start_w_.z() + (goal_w_ - start_w_).z() * path_[i].c_pos(3);
        geometry_msgs::Point linear_end;
        linear_end.x = path_[i].c_pos(0);
        linear_end.y = path_[i].c_pos(1);
        linear_end.z = path_[i].c_pos(2);
        geometry_msgs::Vector3 arrow;
        arrow.x = 0.02;
        arrow.y = 0.04;
        arrow.z = 0.01;

        visualization_msgs::Marker marker;

        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "";
        marker.id = i;
        marker.lifetime = ros::Duration(1.0);

        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale=arrow;

        marker.points.resize(2);
        marker.points[0]=linear_start;
        marker.points[1]=linear_end;

        marker.color.r = 0.7f;
        marker.color.g = 0.5f;
        marker.color.b = 0.7f;
        marker.color.a = 0.5f;

        // marker_array.markers[i].header.frame_id = "/map";
        // marker_array.markers[i].header.stamp = ros::Time::now();
        // marker_array.markers[i].ns = "";
        // marker_array.markers[i].id = i;
        // marker_array.markers[i].lifetime = ros::Duration();
        //
        // marker_array.markers[i].type = visualization_msgs::Marker::ARROW;
        // marker_array.markers[i].action = visualization_msgs::Marker::ADD;
        // marker_array.markers[i].scale=arrow;
        //
        // marker_array.markers[i].points.resize(2);
        // marker_array.markers[i].points[0]=linear_start;
        // marker_array.markers[i].points[1]=linear_end;
        //
        // marker_array.markers[i].color.r = 0.7f;
        // marker_array.markers[i].color.g = 0.5f;
        // marker_array.markers[i].color.b = 0.7f;
        // marker_array.markers[i].color.a = 0.5f;

        marker_array.markers.push_back(marker);
    }
}

void RRTPlanner::run()
{
    if(init_flag_)
    {
        int count = 0;
        while(1)
        {
            if(search()){
                ROS_INFO("Path planner find path. Itr:%d", count);
                pathMake();
                break;
            }

            count++;

            if(max_itr < count)
            {
                ROS_WARN("Reached max iteration. Itr:%d", count);
                initPath();
                break;
            }
        }

        init_flag_ = false;
    }
}

} //namespace rrt_planner
