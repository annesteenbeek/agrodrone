cmake_minimum_required(VERSION 2.8.3)
project(mavros_agrodrone)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  mavros
  mavros_msgs
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
   DIRECTORY msg
   FILES
   TankLevel.msg
 )

include(EnableCXX11)
include(MavrosMavlink)

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  LIBRARIES mavros_agrodrone
  CATKIN_DEPENDS mavros std_msgs message_runtime
  DEPENDS Boost
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${mavlink_INCLUDE_DIRS}
)

add_library(mavros_agrodrone
    src/plugins/tank_level.cpp
    src/plugins/agro_mode.cpp
)

target_link_libraries(mavros_agrodrone
  ${mavros_LIBRARIES}
  ${catkin_LIBRARIES}
)

# make sure messages are generated first (causes problems on lower cored machines)
add_dependencies(mavros_agrodrone mavros_agrodrone_generate_messages_cpp)

install(
  TARGETS
  mavros_agrodrone
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  FILES
  mavros_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# vim: ts=2 sw=2 et:
