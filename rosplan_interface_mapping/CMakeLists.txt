cmake_minimum_required(VERSION 2.8.3)
project(rosplan_interface_mapping)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosplan_knowledge_msgs
  nav_msgs
  mongodb_store
  geometry_msgs
  diagnostic_msgs
  visualization_msgs
  tf
  occupancy_grid_utils
)

find_package(Boost REQUIRED COMPONENTS
  filesystem
)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs rosplan_knowledge_msgs nav_msgs mongodb_store geometry_msgs diagnostic_msgs visualization_msgs tf
  DEPENDS
)

###########
## Build ##
###########

## include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## roadmap sources
set(RPROADMAP_SOURCES
	src/RPRoadmapServer.cpp
	src/RPRoadmapVisualization.cpp)

## simple map sources
set(RPSIMPLEMAP_SOURCES
  src/RPSimpleMapServer.cpp
  src/RPSimpleMapVisualization.cpp)

## Declare cpp executables
add_executable(rproadmapServer ${RPROADMAP_SOURCES})
add_dependencies(rproadmapServer ${catkin_EXPORTED_TARGETS})

add_executable(rpsimplemapServer ${RPSIMPLEMAP_SOURCES})
add_dependencies(rpsimplemapServer ${catkin_EXPORTED_TARGETS})

## Specify libraries against which to link a library or executable target
target_link_libraries(rproadmapServer ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(rpsimplemapServer ${catkin_LIBRARIES} ${Boost_LIBRARIES})

##########
## Test ##
##########

# please do not use add_rosttest_gtest (seems to be interfering with qtcreator and cmake)

#############
## Install ##
#############

install(TARGETS rproadmapServer rpsimplemapServer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
