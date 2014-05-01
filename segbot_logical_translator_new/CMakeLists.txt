cmake_minimum_required(VERSION 2.8.3)
project(segbot_logical_translator_new)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  bwi_mapper
  bwi_planning_common
  bwi_planning_common_new
  message_filters
  move_base_msgs
  nav_msgs
  roscpp
  tf
  segbot_logical_translator
)

catkin_package(
  LIBRARIES 
    segbot_logical_translator_new
  CATKIN_DEPENDS
    actionlib
    bwi_mapper
    bwi_planning_common
    bwi_planning_common_new
    message_filters
    move_base_msgs
    nav_msgs
    roscpp
    tf
    segbot_logical_translator
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(segbot_logical_navigator2 src/segbot_logical_navigator2.cpp)
target_link_libraries(segbot_logical_navigator2 
  ${catkin_LIBRARIES}
)

install(TARGETS 
  segbot_logical_navigator2 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  include/segbot_logical_translator_new/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

