cmake_minimum_required(VERSION 2.8.3)
project(rosplan_knowledge_base)

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

## knowledge sources
set(KNOWLEDGE_SOURCES
	src/KnowledgeBase.cpp
	src/PlanningFilter.cpp)

## Declare cpp executables
add_executable(knowledgeBase ${KNOWLEDGE_SOURCES})

## Specify libraries against which to link a library or executable target
target_link_libraries(knowledgeBase ${catkin_LIBRARIES})

##########
## Test ##
##########

# please do not use add_rosttest_gtest (seems to be interfering with qtcreator and cmake)
