cmake_minimum_required(VERSION 2.8.3)
project(tank_sensor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  rostime
  mavros_agrodrone
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS mavros_agrodrone roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(tank_sensor_node src/tank_sensor_node.cpp)
target_link_libraries(tank_sensor_node ${catkin_LIBRARIES})
