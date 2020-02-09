#ifndef RRT_PLANNER_RRT_PLANNER_H
#define RRT_PLANNER_RRT_PLANNER_H

#include <math.h>
#include <random>
#include <Eigen/Core>

namespace rrt_planner
{

const double start_x = 0.0;
const double start_y = 0.0;
const double start_z = 0.0;

const double goal_x = 1.0;
const double goal_y = 2.0;
const double goal_z = 3.0;

class RRTPlanner
{
public:
    RRTPlanner();
    ~RRTPlanner();

    void run();

    void search();
    void path_make();
    void check_goal();

private:
    Eigen::Vector3d start_;
    Eigen::Vector3d goal_;

    Eigen::Vector3d search_range_min_;
    Eigen::Vector3d search_range_max_;

    std::vector<Eigen::Vector3d> nodes_;

    double delta_{0.1};
    double goal_th_{0.1};
    double select_goal_th_{0.1};

    bool goal_flag_{false};

    std::random_device seed_gen;
    std::mt19937 engine(seed_gen());

    // 一様実数分布
    // [-1.0, 1.0)の値の範囲で、等確率に実数を生成する
    std::uniform_real_distribution<> dist1(-1.0, 1.0);
    std::uniform_real_distribution<> dist_x(-10.0, 10.0);
    std::uniform_real_distribution<> dist_y(-10.0, 10.0);
    std::uniform_real_distribution<> dist_z(-10.0, 10.0);
};

} //namespace rrt_planner

#endif //RRT_PLANNER_RRT_PLANNER_H
