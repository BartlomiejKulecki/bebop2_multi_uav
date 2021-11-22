cmake_minimum_required(VERSION 2.8.3)
project(formation_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  position_controller
  message_generation
)

find_package(Curses REQUIRED)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
 #  FILES
#   StartTrajectory.srv
   
 #)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   nav_msgs#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES formation_controller
#  CATKIN_DEPENDS nav_msgs roscpp std_msgs
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
  ${position_controller_INCLUDE_DIRS}
  ${CURSES_INCLUDE_DIR}
)


add_executable(formation_controller src/formation_controller.cpp)
target_link_libraries(formation_controller ${catkin_LIBRARIES} ${CURSES_LIBRARIES})

add_executable(collision_avoidance src/collision_avoidance.cpp)
target_link_libraries(collision_avoidance ${catkin_LIBRARIES})
#add_dependencies(formation_controller  )
