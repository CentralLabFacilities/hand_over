cmake_minimum_required(VERSION 2.8.3)
project(hand_over_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  std_msgs
  moveit_msgs
  message_generation
)


add_action_files(FILES 
  HandOver.action 
  TakeOver.action
  MeasureForce.action
)

generate_messages(   
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  moveit_msgs
) 


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hand_over
  CATKIN_DEPENDS actionlib_msgs std_msgs moveit_msgs
#  DEPENDS system_lib
)

