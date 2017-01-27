cmake_minimum_required(VERSION 2.8.3)
project(sprayer_controller)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  rostime
  std_msgs
)

# Used to export all the build targets so they can be used, this should be done before defining any targets with add_library() or add_executable()
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sprayer_controller
  CATKIN_DEPENDS roscpp std_msgs
)

# to define which directories contain header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# include a library 
add_library(sprayer_controller
    src/lib/pwmBit.cpp
    src/lib/ShiftReg.cpp
)

# define the actual executable
add_executable(
    sprayer_controller_node
    src/sprayer_controller_node.cpp
)

FIND_LIBRARY(WIRINGPI_LIBRARY
    wiringPi /root/wiringPi
)


# link the executable, the library and additional libraries 
target_link_libraries(
    sprayer_controller_node
    sprayer_controller
    ${catkin_LIBRARIES}
    ${WIRINGPI_LIBRARY}
)

add_dependencies(
    sprayer_controller
    wiringPi
    )
