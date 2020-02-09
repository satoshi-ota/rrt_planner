cmake_minimum_required(VERSION 2.8.3)
project(rrt_planner)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES rrt_planner
    CATKIN_DEPENDS roscpp
    DEPENDS
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(rrt_planner
    src/library/rrt_planner.cpp)

target_link_libraries(rrt_planner ${catkin_LIBRARIES})
add_dependencies(rrt_planner ${catkin_EXPORTED_TARGETS})

add_executable(path_planning_node src/nodes/path_planning_node.cpp)
add_dependencies(path_planning_node rrt_planner ${catkin_LIBRARIES})
target_link_libraries(path_planning_node rrt_planner ${catkin_LIBRARIES})
