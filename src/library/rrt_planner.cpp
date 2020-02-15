#include "rrt_planner/rrt_planner.h"

namespace rrt_planner
{

RRTPlanner::RRTPlanner(const ConfigDim& config_dim)
    :config_dim_(config_dim)
{
    initPath();
    initObstacles();
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

void RRTPlanner::addObstacles(const Eigen::Vector3d& pos, const Eigen::Vector3d& size)
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

void RRTPlanner::setParams(double delta, double goal_tolerance, double obstacle_margin, double select_goal_rate, double max_itr)
{
    dt_ref_ = delta;
    goal_tolerance_ = goal_tolerance;
    obstacle_margin_ = obstacle_margin;
    select_goal_rate_ = select_goal_rate;
    max_itr_ = max_itr;
}

void RRTPlanner::reconfig(rrt_planner::RRTParametersConfig &config)
{
    dt_ref_ = config.dt_ref;
    goal_tolerance_ = config.goal_tolerance;
    obstacle_margin_ = config.obstacle_margin;
    select_goal_rate_ = config.select_goal_rate;
    max_itr_ = config.max_itr;
}

void RRTPlanner::setCSpace(const Eigen::VectorXd& c_space_min,
                           const Eigen::VectorXd& c_space_max)
{
    if(checkDim(c_space_min) && checkDim(c_space_max))
    {
        c_space_min_ = c_space_min;
        c_space_max_ = c_space_max;

        rands_.clear();
        for(int i = 0; i < (int)config_dim_; i++)
        {
            std::uniform_real_distribution<> rand(c_space_min_(i),
                                                  c_space_max_(i));
            rands_.push_back(rand);
        }
    }
}

void RRTPlanner::setStart(const Eigen::VectorXd& c_state_init)
{
    if(checkDim(c_state_init)) c_state_init_ = c_state_init;
}

void RRTPlanner::setGoal(const Eigen::VectorXd& c_state_goal)
{
    if(checkDim(c_state_goal)) c_state_goal_ = c_state_goal;
}

void RRTPlanner::setConstrains(const Eigen::VectorXd& c_constrain_min,
                               const Eigen::VectorXd& c_constrain_max)
{
    if(checkDim(c_constrain_min)) c_constrain_min_ = c_constrain_min;
    if(checkDim(c_constrain_max)) c_constrain_max_ = c_constrain_max;
}

bool RRTPlanner::search()
{
    std::mt19937 engine(seed_gen_());
    std::uniform_real_distribution<> goal_or_not(-1.0, 1.0);
    double choice = goal_or_not(engine);

    Eigen::VectorXd c_state;
    c_state.resize((int)config_dim_, 1);

    if(choice < select_goal_rate_)
    {
        for(int i = 0; i < (int)config_dim_; i++) c_state(i) = rands_[i](engine);
    } else {
        for(int i = 0; i < (int)config_dim_; i++) c_state(i) = c_state_goal_(i);
    }

    double distance = INFINITY;

    Eigen::VectorXd nearest_c_state;
    Node new_node(config_dim_);

    for(int i = 0; i < nodes_.size(); i++)
    {
        Eigen::VectorXd ith_c_state = nodes_[i].c_state;
        double d = (c_state - ith_c_state).norm();

        if(d < distance){
            distance = d;
            new_node.parent_node = i;
            nearest_c_state = ith_c_state;
        }
    }

    new_node.c_state
        = nearest_c_state + (c_state - nearest_c_state).normalized() * dt_ref_;

    if(checkObstacle(new_node) && checkConstrain(new_node))
    {
        nodes_.push_back(new_node);
        if(checkGoal(new_node)) return true;
    }

    return false;
}

bool RRTPlanner::checkGoal(const Node& node)
{
    double d = (c_state_goal_ - node.c_state).norm();
    if(d < goal_tolerance_) return true;

    return false;
}

bool RRTPlanner::checkConstrain(const Node& node)
{
    for(int i = 0; i < (int)config_dim_; i++)
        if((node.c_state(i) < c_constrain_min_(i)) ||
           (c_constrain_max_(i) < node.c_state(i))) return false;

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

bool RRTPlanner::checkObstacle(const Node& node)
{
    Eigen::Vector3d w = start_w_ + (goal_w_ - start_w_) * node.c_state(3);

    Eigen::Vector3d vec = (w - node.c_state.topLeftCorner(3, 1)).normalized();

    for(int i = 0; i < obstacles_.size(); i++)
    {
        Eigen::Vector3d d = obstacles_[i].pos - node.c_state.topLeftCorner(3, 1);

        double a = vec.x()*d.x() + vec.y()*d.y() + vec.z()*d.z();
        double b = vec.x()*vec.x() + vec.y()*vec.y() + vec.z()*vec.z();
        double t = a / b;

        Eigen::Vector3d q = node.c_state.topLeftCorner(3, 1) + vec * t;

        if((q - obstacles_[i].pos).norm() < obstacle_margin_) return false;
    }

    return true;
}

bool RRTPlanner::checkDim(const Eigen::VectorXd& vec)
{
    if(vec.size() == (int)config_dim_) return true;

    ROS_WARN("Invalid param. Incoherent vector dimension. ");
    return false;
}

void RRTPlanner::copyToMsg(nav_msgs::Path& path_msg)
{
    for(int i = 0; i <path_.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = path_[i].c_state(0);
        pose.pose.position.y = path_[i].c_state(1);
        pose.pose.position.z = path_[i].c_state(2);
        path_msg.poses.push_back(pose);
    }
}

void RRTPlanner::showArrow(visualization_msgs::MarkerArray& marker_array)
{
    for(int i = 0; i <path_.size(); i++)
    {
        geometry_msgs::Point linear_start;
        linear_start.x = start_w_.x() + (goal_w_ - start_w_).x() * path_[i].c_state(3);
        linear_start.y = start_w_.y() + (goal_w_ - start_w_).y() * path_[i].c_state(3);
        linear_start.z = start_w_.z() + (goal_w_ - start_w_).z() * path_[i].c_state(3);
        geometry_msgs::Point linear_end;
        linear_end.x = path_[i].c_state(0);
        linear_end.y = path_[i].c_state(1);
        linear_end.z = path_[i].c_state(2);
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

        marker_array.markers.push_back(marker);
    }
}

void RRTPlanner::run()
{
    if(init_flag_)
    {
        ROS_INFO("Searching path. ");
        int count = 0;
        while(1)
        {
            if(search()){
                ROS_INFO("Path planner find path. ");
                pathMake();
                break;
            }

            count++;

            if(max_itr_ < count)
            {
                ROS_WARN("Reached max iteration. ");
                initPath();
                break;
            }
        }

        init_flag_ = false;
        ROS_INFO("End RRT Path-Planning. Itr:%d", count);
    }
}

} //namespace rrt_planner
