cmake_minimum_required(VERSION 2.8.3)
project(rosplan_planning_system)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

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
  rosplan_dependencies
  roslib
  rostest
)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES rosplan_action_interface
  CATKIN_DEPENDS roscpp rospy actionlib std_msgs std_srvs diagnostic_msgs
                 rosplan_dispatch_msgs rosplan_knowledge_msgs roslib rostest
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

## Declare cpp executables
add_executable(problemInterface src/ProblemGeneration/ProblemInterface.cpp src/ProblemGeneration/PDDLProblemGenerator.cpp
                                                                           src/ProblemGeneration/ProblemGenerator.cpp
                                                                           src/ProblemGeneration/RDDLProblemGenerator.cpp
                                                                           src/ProblemGeneration/CHIMPFluent.cpp
                                                                           src/ProblemGeneration/CHIMPProblem.cpp
                                                                           src/ProblemGeneration/CHIMPProblemGenerator.cpp
                                                                           src/ProblemGeneration/ProblemGeneratorFactory.cpp)
add_executable(popf_planner_interface src/PlannerInterface/POPFPlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(panda_planner_interface src/PlannerInterface/PANDAPlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(ff_planner_interface src/PlannerInterface/FFPlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(metricff_planner_interface src/PlannerInterface/FFPlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(cff_planner_interface src/PlannerInterface/CFFPlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(rddlsim_planner_interface src/PlannerInterface/RDDLSIMPlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(smt_planner_interface src/PlannerInterface/SMTPlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(fd_planner_interface src/PlannerInterface/FDPlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(tfd_planner_interface src/PlannerInterface/TFDPlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(lpg_planner_interface src/PlannerInterface/LPGPlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(upm_planner_interface src/PlannerInterface/UPMPlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(pprp_planner_interface src/PlannerInterface/PPRPPlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(chimp_planner_interface src/PlannerInterface/CHIMPPlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(online_planner_interface src/PlannerInterface/OnlinePlannerInterface.cpp src/PlannerInterface/PlannerInterface.cpp)
add_executable(pddl_simple_plan_parser src/PlanParsing/PDDLSimplePlanParser.cpp src/PlanParsing/PlanParser.cpp)
add_executable(pddl_esterel_plan_parser src/PlanParsing/PDDLEsterelPlanParser.cpp src/PlanParsing/PlanParser.cpp)
add_executable(pddl_simple_plan_dispatcher src/PlanDispatch/SimplePlanDispatcher.cpp src/PlanDispatch/PlanDispatcher.cpp)
add_executable(pddl_esterel_plan_dispatcher src/PlanDispatch/EsterelPlanDispatcher.cpp src/PlanDispatch/PlanDispatcher.cpp)
add_executable(online_plan_dispatcher src/PlanDispatch/OnlinePlanDispatcher.cpp src/PlanDispatch/PlanDispatcher.cpp)
add_executable(simulatedAction src/ActionInterface/RPSimulatedActionInterface.cpp src/ActionInterface/RPActionInterface.cpp)

## Add dependencies
add_dependencies(problemInterface ${catkin_EXPORTED_TARGETS})
add_dependencies(popf_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(panda_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(ff_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(cff_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(rddlsim_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(metricff_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(smt_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(fd_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(tfd_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(lpg_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(upm_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(pprp_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(online_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(chimp_planner_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(pddl_simple_plan_parser ${catkin_EXPORTED_TARGETS})
add_dependencies(pddl_esterel_plan_parser ${catkin_EXPORTED_TARGETS})
add_dependencies(pddl_simple_plan_dispatcher ${catkin_EXPORTED_TARGETS})
add_dependencies(pddl_esterel_plan_dispatcher ${catkin_EXPORTED_TARGETS})
add_dependencies(online_plan_dispatcher ${catkin_EXPORTED_TARGETS})
add_dependencies(simulatedAction ${catkin_EXPORTED_TARGETS})

## Specify libraries against which to link a library or executable target
target_link_libraries(problemInterface ${catkin_LIBRARIES})
target_link_libraries(popf_planner_interface ${catkin_LIBRARIES})
target_link_libraries(panda_planner_interface ${catkin_LIBRARIES})
target_link_libraries(ff_planner_interface ${catkin_LIBRARIES})
target_link_libraries(metricff_planner_interface ${catkin_LIBRARIES})
target_link_libraries(cff_planner_interface ${catkin_LIBRARIES})
target_link_libraries(rddlsim_planner_interface ${catkin_LIBRARIES})
target_link_libraries(smt_planner_interface ${catkin_LIBRARIES})
target_link_libraries(fd_planner_interface ${catkin_LIBRARIES})
target_link_libraries(tfd_planner_interface ${catkin_LIBRARIES})
target_link_libraries(lpg_planner_interface ${catkin_LIBRARIES})
target_link_libraries(upm_planner_interface ${catkin_LIBRARIES})
target_link_libraries(pprp_planner_interface ${catkin_LIBRARIES})
target_link_libraries(online_planner_interface ${catkin_LIBRARIES})
target_link_libraries(chimp_planner_interface ${catkin_LIBRARIES})
target_link_libraries(pddl_simple_plan_parser ${catkin_LIBRARIES})
target_link_libraries(pddl_esterel_plan_parser ${catkin_LIBRARIES})
target_link_libraries(pddl_simple_plan_dispatcher ${catkin_LIBRARIES})
target_link_libraries(pddl_esterel_plan_dispatcher ${catkin_LIBRARIES})
target_link_libraries(online_plan_dispatcher ${catkin_LIBRARIES})
target_link_libraries(simulatedAction ${catkin_LIBRARIES})

## Declare libraries
# rosplan_planning_system does not dispatch VALfiles as a library anymore.
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
# file(GLOB COMMON_BINS common/bin/*)
# install(PROGRAMS ${COMMON_BINS}
# DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common/bin
#)
install(DIRECTORY common/bin
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common/bin/
  FILE_PERMISSIONS OWNER_EXECUTE
)

# install pddl files in install space
install(DIRECTORY test/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)

# install launch files in install space
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
##  Tests  ##
#############

# to build the test do: "catkin build --no-deps rosplan_planning_system --make-args run_tests"
# to run the test do e.g.: "rostest rosplan_planning_system foo.test --text"
# where "--text" is optional, if you want to see the console output of your test
if (CATKIN_ENABLE_TESTING)
    # POPF Planner Interface
    add_rostest_gtest(PopfPlannerInterfaceTests test/launch/popf_planner_interface.test test/src/PopfPlannerInterfaceTests.cpp)
    target_link_libraries(PopfPlannerInterfaceTests ${catkin_LIBRARIES})
    # Contingent-FF Planner Interface
    add_rostest_gtest(CffPlannerInterfaceTests test/launch/cff_planner_interface.test test/src/CffPlannerInterfaceTests.cpp)
    target_link_libraries(CffPlannerInterfaceTests ${catkin_LIBRARIES})    
    # Problem Interface
    add_rostest_gtest(ProblemInterfaceTests test/launch/problem_interface.test test/src/ProblemInterfaceTests.cpp)
    target_link_libraries(ProblemInterfaceTests ${catkin_LIBRARIES})
    # Parsing Interface
    add_rostest_gtest(ParsingInterfaceTests test/launch/parsing_interface.test test/src/ParsingInterfaceTests.cpp)
    target_link_libraries(ParsingInterfaceTests ${catkin_LIBRARIES})
    # Plan Dispatch Interface
    add_rostest_gtest(PlanDispatchTests test/launch/plan_dispatch.test test/src/PlanDispatchTests.cpp)
    target_link_libraries(PlanDispatchTests ${catkin_LIBRARIES})
    # Planner Interface
    add_rostest_gtest(ChimpPlannerInterfaceTests test/launch/chimp_planner_interface.test test/src/ChimpPlannerInterfaceTests.cpp)
    target_link_libraries(ChimpPlannerInterfaceTests ${catkin_LIBRARIES})
    # CHIMP Problem
    catkin_add_gtest(CHIMPProblemUTest test/src/CHIMPProblemTests.cpp src/ProblemGeneration/CHIMPFluent.cpp
                                                                      src/ProblemGeneration/CHIMPProblem.cpp)
    target_link_libraries(CHIMPProblemUTest ${catkin_LIBRARIES})
endif()
