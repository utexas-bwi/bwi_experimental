cmake_minimum_required(VERSION 2.8.3)
project(bwi_action_executor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  segbot_gui
  roslib
  bwi_kr
)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package()

###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp executable
set(ACTEXEC_SRC)
add_subdirectory(src)
add_executable(bwi_action_executor_node ${ACTEXEC_SRC} )
add_dependencies(bwi_action_executor_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(bwi_action_executor_node ${catkin_LIBRARIES})

#############
## Install ##
#############


#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
