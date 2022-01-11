cmake_minimum_required(VERSION 2.8.3)
project(rosplan_knowledge_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  rosplan_knowledge_msgs
  nav_msgs
  geometry_msgs
  diagnostic_msgs
  visualization_msgs
  tf
  rosplan_dependencies
  rostest
)

find_package(FLEX REQUIRED)

find_package(Boost REQUIRED COMPONENTS
  filesystem
)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs rosplan_knowledge_msgs nav_msgs geometry_msgs diagnostic_msgs visualization_msgs tf rosplan_dependencies
  DEPENDS
)

###########
## Build ##
###########

## include_directories(include)
include_directories(src)
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${rosplan_dependencies_DEVEL_PREFIX}/include/hddl_parser)

## val sources
add_definitions(-Wno-deprecated-declarations) # Disable deprecated declarations warning (about std::auto_ptr)
set(VAL_SOURCES
        src/VALfiles/DebugWriteController.cpp
        src/VALfiles/pddl+.cpp
        src/VALfiles/ptree.cpp
)

## knowledge sources
set(KNOWLEDGE_SOURCES
    src/KnowledgeBaseFactory.cpp
    src/KnowledgeBase.cpp
    src/KnowledgeComparitor.cpp
    src/PDDLDomainParser.cpp
    src/PDDLProblemParser.cpp
    src/PDDLKnowledgeBase.cpp
    src/HDDLKnowledgeBase.cpp
    src/RDDLTaskParser.cpp
    src/RDDLKnowledgeBase.cpp
    src/RDDLExprUtils.cpp
    src/RDDLUtils.cpp
    src/VALVisitorOperator.cpp
    src/VALVisitorPredicate.cpp
    src/VALVisitorProblem.cpp
    src/PPDDLKnowledgeBase.cpp
    src/PPDDLParser.cpp
    src/PPDDLUtils.cpp
)

## Declare cpp executables
add_executable(knowledgeBase ${KNOWLEDGE_SOURCES} ${VAL_SOURCES})
add_dependencies(knowledgeBase ${catkin_EXPORTED_TARGETS})

## Specify libraries against which to link a library or executable target
target_link_libraries(knowledgeBase ${catkin_LIBRARIES})

##########
## Test ##
##########

# please do not use add_rosttest_gtest (seems to be interfering with qtcreator and cmake)

#############
## Install ##
#############

install(TARGETS knowledgeBase
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
##  Tests  ##
#############

# to build the test do: "catkin build --no-deps rosplan_planning_system --make-args run_tests"
# to run the test do e.g.: "rostest rosplan_planning_system foo.test --text"
# where "--text" is optional, if you want to see the console output of your test
if (CATKIN_ENABLE_TESTING)
    # Knowledge Base
    add_rostest_gtest(KnowledgeBaseTests test/launch/knowledge_base.test test/src/KnowledgeBaseTests.cpp)
    target_link_libraries(KnowledgeBaseTests ${catkin_LIBRARIES})
endif()
