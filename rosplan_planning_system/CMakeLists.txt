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

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES rosplan_action_interface
  CATKIN_DEPENDS roscpp rospy actionlib std_msgs std_srvs diagnostic_msgs rosplan_dispatch_msgs rosplan_knowledge_msgs
  DEPENDS
)

###########
## Build ##
###########

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

find_package(FLEX REQUIRED)

# Disable deprecated declarations warning (about std::auto_ptr)
add_definitions(-Wno-deprecated-declarations)

## visualisation
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})

## Declare cpp executables
add_executable(problemInterface src/ProblemGeneration/ProblemInterface.cpp src/ProblemGeneration/PDDLProblemGenerator.cpp
                                                                           src/ProblemGeneration/ProblemGenerator.cpp
                                                                           src/ProblemGeneration/RDDLProblemGenerator.cpp
                                                                           src/ProblemGeneration/ProblemGeneratorFactory.cpp)
add_executable(popf_planner_interface src/PlannerInterface/POPFPlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(ff_planner_interface src/PlannerInterface/FFPlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(metricff_planner_interface src/PlannerInterface/FFPlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(smt_planner_interface src/PlannerInterface/SMTPlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(fd_planner_interface src/PlannerInterface/FDPlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(tfd_planner_interface src/PlannerInterface/TFDPlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(lpg_planner_interface src/PlannerInterface/LPGPlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(upm_planner_interface src/PlannerInterface/UPMPlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(pprp_planner_interface src/PlannerInterface/PPRPPlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(pddl_simple_plan_parser src/PlanParsing/PDDLSimplePlanParser.cpp src/PlanParsing/PlanParser.cpp)
add_executable(pddl_esterel_plan_parser src/PlanParsing/PDDLEsterelPlanParser.cpp src/PlanParsing/PlanParser.cpp)
add_executable(pddl_simple_plan_dispatcher src/PlanDispatch/SimplePlanDispatcher.cpp)
add_executable(pddl_esterel_plan_dispatcher src/PlanDispatch/EsterelPlanDispatcher.cpp)
add_executable(simulatedAction src/ActionInterface/RPSimulatedActionInterface.cpp src/ActionInterface/RPActionInterface.cpp)

## Add dependencies
add_dependencies(problemInterface ${catkin_EXPORTED_TARGETS})
add_dependencies(popf_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(ff_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(metricff_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(smt_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(fd_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(tfd_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(lpg_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(upm_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(pprp_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(pddl_simple_plan_parser ${catkin_EXPORTED_TARGETS})
add_dependencies(pddl_esterel_plan_parser ${catkin_EXPORTED_TARGETS})
add_dependencies(pddl_simple_plan_dispatcher ${catkin_EXPORTED_TARGETS})
add_dependencies(pddl_esterel_plan_dispatcher ${catkin_EXPORTED_TARGETS})
add_dependencies(simulatedAction ${catkin_EXPORTED_TARGETS})

## Specify libraries against which to link a library or executable target
target_link_libraries(problemInterface ${catkin_LIBRARIES})
target_link_libraries(popf_planner_interface ${catkin_LIBRARIES})
target_link_libraries(ff_planner_interface ${catkin_LIBRARIES})
target_link_libraries(metricff_planner_interface ${catkin_LIBRARIES})
target_link_libraries(smt_planner_interface ${catkin_LIBRARIES})
target_link_libraries(fd_planner_interface ${catkin_LIBRARIES})
target_link_libraries(tfd_planner_interface ${catkin_LIBRARIES})
target_link_libraries(lpg_planner_interface ${catkin_LIBRARIES})
target_link_libraries(upm_planner_interface ${catkin_LIBRARIES})
target_link_libraries(pprp_planner_interface ${catkin_LIBRARIES})
target_link_libraries(pddl_simple_plan_parser ${catkin_LIBRARIES})
target_link_libraries(pddl_esterel_plan_parser ${catkin_LIBRARIES})
target_link_libraries(pddl_simple_plan_dispatcher ${catkin_LIBRARIES})
target_link_libraries(pddl_esterel_plan_dispatcher ${catkin_LIBRARIES})
target_link_libraries(simulatedAction ${catkin_LIBRARIES})

## Declare libraries
# rosplan_planning_system does not dispatch VALfiles as a library anymore.
#add_library(${PROJECT_NAME} ${PLANNING_SOURCES} )
#add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_library(rosplan_action_interface src/ActionInterface/RPActionInterface.cpp)
add_dependencies(rosplan_action_interface ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS rosplan_action_interface simulatedAction
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

# This should probably go to the package bin directory,
# but there is no simple way to determine this path from
# within a launch file similar to "$(find ...)".
# The chosen path allows to use the same roslaunch parameter,
# when run from the workspace or when installed.
file(GLOB COMMON_BINS common/bin/*)
install(PROGRAMS ${COMMON_BINS}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common/bin
)

