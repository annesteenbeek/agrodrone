cmake_minimum_required(VERSION 2.8.3)
project(tank_sensor)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  rostime
  mavros_agrodrone
  std_msgs
)

# Used to export all the build targets so they can be used, this should be done before defining any targets with add_library() or add_executable()
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tank_sensor
  CATKIN_DEPENDS mavros_agrodrone roscpp std_msgs
)

# to define which directories contain header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# include a library 
add_library(tank_sensor
    src/lib/mcp3008Spi.cpp 
)

# define the actual executable
add_executable(
    tank_sensor_node 
    src/tank_sensor_node.cpp
)

# link the executable, the library and additional libraries 
target_link_libraries(
    tank_sensor_node
    tank_sensor
    ${catkin_LIBRARIES}
)
