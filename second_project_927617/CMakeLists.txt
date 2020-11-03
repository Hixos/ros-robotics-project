cmake_minimum_required(VERSION 2.8.3)
project(second_project_927617)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  geodesy
  sensor_msgs
  nav_msgs
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

## Executables
add_executable(lla2enu_node src/lla2enu_node.cpp)
target_link_libraries(lla2enu_node ${catkin_LIBRARIES})
