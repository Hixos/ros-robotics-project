cmake_minimum_required(VERSION 2.8.3)
project(first_project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  geodesy
  sensor_msgs
  nav_msgs
  message_generation
  dynamic_reconfigure
)

add_message_files(
  FILES
  CollisionStatus.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  CalculateDistance.srv
)

generate_dynamic_reconfigure_options(
  cfg/CollisionThresholds.cfg
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

add_executable(distance_service src/distance_service.cpp)
target_link_libraries(distance_service ${catkin_LIBRARIES})
add_dependencies(distance_service ${PROJECT_NAME}_gencpp)

add_executable(collision_status_node src/collision_status_node.cpp)
target_link_libraries(collision_status_node ${catkin_LIBRARIES})
add_dependencies(collision_status_node ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)