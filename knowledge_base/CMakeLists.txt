cmake_minimum_required(VERSION 2.8.3)
project(knowledge_base)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  planning_knowledge_msgs
)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  LIBRARIES knowledge_base
  CATKIN_DEPENDS roscpp rospy std_msgs planning_knowledge_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## val sources
set(KNOWLEDGE_SOURCES
	src/KnowledgeBase.h
	src/KnowledgeBase.cpp
	src/PlanningFilter.cpp)

## Declare cpp executables
add_executable(knowledgeBase
	${KNOWLEDGE_SOURCES})

add_executable(testHarness src/TestHarness.cpp)

## Specify libraries against which to link a library or executable target
target_link_libraries(knowledgeBase
  ${catkin_LIBRARIES}
)

target_link_libraries(testHarness
  ${catkin_LIBRARIES}
)
