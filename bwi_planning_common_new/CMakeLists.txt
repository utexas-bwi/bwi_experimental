cmake_minimum_required(VERSION 2.8.3)
project(bwi_planning_common_new)

find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  roscpp
  std_msgs 
  std_srvs
  actionlib_msgs
  bwi_planning_common
)

find_package(PkgConfig REQUIRED)

add_action_files(
  DIRECTORY 
    action
  FILES
    ActionInterface.action
)

generate_messages(
  DEPENDENCIES
    std_msgs
    actionlib_msgs
    bwi_planning_common
)

catkin_package(
  INCLUDE_DIRS 

  CATKIN_DEPENDS 
    message_runtime 
    std_msgs 
    actionlib_msgs 
    bwi_planning_common
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

