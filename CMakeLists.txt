cmake_minimum_required(VERSION 2.8.3)
project(rrt_planner)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  interactive_markers
  dynamic_reconfigure
  jsk_recognition_msgs
  message_generation
  pluginlib
  rviz
)

add_message_files(
  FILES
  MarkerArrayStamped.msg
)

generate_dynamic_reconfigure_options(
  cfg/RRTParameters.cfg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  visualization_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES rrt_planner
    CATKIN_DEPENDS roscpp nav_msgs geometry_msgs visualization_msgs interactive_markers jsk_recognition_msgs
    DEPENDS
)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED
  Core
  Widgets
)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

include_directories("/usr/include/eigen3")

add_library(rrt_planner
    src/library/rrt_planner.cpp
    src/rrt_rviz_plugin/path_visual.cpp
)

target_link_libraries(rrt_planner ${catkin_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(rrt_planner ${catkin_EXPORTED_TARGETS})

add_executable(path_planning_node src/nodes/path_planning_node.cpp)
add_dependencies(path_planning_node rrt_planner rrt_planner_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_planning_node rrt_planner ${catkin_LIBRARIES})

add_executable(path_display_node src/nodes/path_display_node.cpp)
add_dependencies(path_display_node rrt_planner ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_display_node rrt_planner ${catkin_LIBRARIES})
