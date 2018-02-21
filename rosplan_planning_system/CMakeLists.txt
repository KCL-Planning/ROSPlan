cmake_minimum_required(VERSION 2.8.3)
project(rosplan_planning_system)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib
  std_msgs
  std_srvs
  diagnostic_msgs
  rosplan_dispatch_msgs
  rosplan_knowledge_msgs
)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS} src/VALfiles
  LIBRARIES ${PROJECT_NAME} rosplan_action_interface
  CATKIN_DEPENDS roscpp rospy actionlib std_msgs std_srvs diagnostic_msgs rosplan_dispatch_msgs rosplan_knowledge_msgs
  DEPENDS
)

###########
## Build ##
###########

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(src/VALfiles)

find_package(FLEX REQUIRED)

# Disable deprecated declarations warning (about std::auto_ptr)
add_definitions(-Wno-deprecated-declarations)

## visualisation
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})

## val sources
set(VAL_SOURCES
	src/VALfiles/DebugWriteController.cpp
	src/VALfiles/FastEnvironment.cpp
	src/VALfiles/FuncAnalysis.cpp
	src/VALfiles/SASActions.cpp
	src/VALfiles/SimpleEval.cpp
	src/VALfiles/TIM.cpp
	src/VALfiles/TimSupport.cpp
	src/VALfiles/ToFunction.cpp
	src/VALfiles/TypedAnalyser.cpp
	src/VALfiles/ValidatorAPI.cpp
	src/VALfiles/instantiation.cpp
	src/VALfiles/pddl+.cpp
	src/VALfiles/ptree.cpp
	src/VALfiles/typecheck.cpp
	src/VALfiles/Validator.cpp
	src/VALfiles/Action.cpp
	src/VALfiles/Proposition.cpp
	src/VALfiles/FuncExp.cpp
	src/VALfiles/RepairAdvice.cpp
	src/VALfiles/State.cpp
	src/VALfiles/Plan.cpp
	src/VALfiles/Ownership.cpp
	src/VALfiles/Environment.cpp
	src/VALfiles/Events.cpp
	src/VALfiles/Polynomial.cpp
	src/VALfiles/TrajectoryConstraints.cpp
	src/VALfiles/RobustAnalyse.cpp
	src/VALfiles/PrettyPrinter.cpp
	src/VALfiles/random.cpp
	src/VALfiles/LaTeXSupport.cpp
	src/VALfiles/Utils.cpp)

## Declare cpp executables
add_executable(problemInterface src/ProblemGeneration/ProblemInterface.cpp src/ProblemGeneration/PDDLProblemGenerator.cpp)
add_executable(popf_planner_interface src/PlannerInterface/POPFPlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(ff_planner_interface src/PlannerInterface/FFPlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(pddl_simple_plan_parser src/PlanParsing/PDDLSimplePlanParser.cpp src/PlanParsing/PlanParser.cpp)
add_executable(pddl_esterel_plan_parser src/PlanParsing/PDDLEsterelPlanParser.cpp src/PlanParsing/PlanParser.cpp)
add_executable(pddl_simple_plan_dispatcher src/PlanDispatch/SimplePlanDispatcher.cpp)
add_executable(pddl_esterel_plan_dispatcher src/PlanDispatch/EsterelPlanDispatcher.cpp)
add_executable(simulatedAction src/ActionInterface/RPSimulatedActionInterface.cpp src/ActionInterface/RPActionInterface.cpp)

## Add dependencies
add_dependencies(problemInterface ${catkin_EXPORTED_TARGETS})
add_dependencies(popf_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(ff_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(pddl_simple_plan_parser ${catkin_EXPORTED_TARGETS})
add_dependencies(pddl_esterel_plan_parser ${catkin_EXPORTED_TARGETS})
add_dependencies(pddl_simple_plan_dispatcher ${catkin_EXPORTED_TARGETS})
add_dependencies(pddl_esterel_plan_dispatcher ${catkin_EXPORTED_TARGETS})
add_dependencies(simulatedAction ${catkin_EXPORTED_TARGETS})

## Specify libraries against which to link a library or executable target
target_link_libraries(problemInterface ${catkin_LIBRARIES})
target_link_libraries(popf_planner_interface ${catkin_LIBRARIES})
target_link_libraries(ff_planner_interface ${catkin_LIBRARIES})
target_link_libraries(pddl_simple_plan_parser ${catkin_LIBRARIES})
target_link_libraries(pddl_esterel_plan_parser ${catkin_LIBRARIES})
target_link_libraries(pddl_simple_plan_dispatcher ${catkin_LIBRARIES})
target_link_libraries(pddl_esterel_plan_dispatcher ${catkin_LIBRARIES})
target_link_libraries(simulatedAction ${catkin_LIBRARIES})

## Declare libraries
add_library(${PROJECT_NAME} ${PLANNING_SOURCES} ${VAL_SOURCES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_library(rosplan_action_interface src/ActionInterface/RPActionInterface.cpp)
add_dependencies(rosplan_action_interface ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} rosplan_action_interface simulatedAction
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY include/rosplan_action_interface
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY src/VALfiles/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# This should probably go to the package bin directory,
# but there is no simple way to determine this path from
# within a launch file similar to "$(find ...)".
# The chosen path allows to use the same roslaunch parameter,
# when run from the workspace or when installed.
file(GLOB COMMON_BINS common/bin/*)
install(PROGRAMS ${COMMON_BINS}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common/bin
)

