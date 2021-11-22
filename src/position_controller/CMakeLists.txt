cmake_minimum_required(VERSION 2.8.3)
project(position_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  nav_msgs
  geometry_msgs
  message_generation
  dynamic_reconfigure
)

find_package(gazebo REQUIRED)


## Generate messages in the 'msg' folder
add_message_files(
   FILES
   msgCoordinates.msg
   msgData.msg
   msgTrajectory.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
 #)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   nav_msgs
   std_msgs
 )


## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/PID.cfg
#   cfg/DynReconf2.cfg
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES position_controller
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


## Mark cpp header files for installation
 install(DIRECTORY include/position_controller/msgTrajectory.h
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)



add_executable(position_controller src/position_controller.cpp)
target_link_libraries(position_controller ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} pthread)
add_dependencies(position_controller position_controller_generate_messages_cpp position_controller_gencfg)

