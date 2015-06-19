cmake_minimum_required(VERSION 2.8.3)
project(rosplan_planning_system)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  diagnostic_msgs
  rosplan_dispatch_msgs
  rosplan_knowledge_msgs
  rosplan_knowledge_base
)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs diagnostic_msgs rosplan_dispatch_msgs rosplan_knowledge_msgs rosplan_knowledge_base
)

###########
## Build ##
###########

## include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${rosplan_knowledge_base_INCLUDE_DIRS}
  include
)

## planning system sources
set(PLANNING_SOURCES
	src/PlanningSystem.cpp
	src/PlanDispatcher.cpp
	src/PlanParser.cpp
	src/PDDLProblemGenerator.cpp
	src/PlanningEnvironment.cpp)

## Declare cpp executables
add_executable(planner ${PLANNING_SOURCES} ${VAL_SOURCES})
add_dependencies(planner ${catkin_EXPORTED_TARGETS} ${rosplan_knowledge_base_LIBRARIES})

## Specify libraries against which to link a library or executable target
target_link_libraries(planner ${catkin_LIBRARIES} ${rosplan_knowledge_base_LIBRARIES})
