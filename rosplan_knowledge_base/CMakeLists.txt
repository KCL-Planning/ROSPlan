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
  mongodb_store
  geometry_msgs
  diagnostic_msgs
  visualization_msgs
  tf
  rosplan_dependencies
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
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs rosplan_knowledge_msgs nav_msgs mongodb_store geometry_msgs diagnostic_msgs visualization_msgs tf
  DEPENDS
)

###########
## Build ##
###########

## include_directories(include)
include_directories(src)
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

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
    src/RDDLTaskParser.cpp
    src/RDDLKnowledgeBase.cpp
    src/RDDLExprUtils.cpp
    src/RDDLUtils.cpp
    src/VALVisitorOperator.cpp
	src/VALVisitorPredicate.cpp
	src/VALVisitorProblem.cpp
	src/PPDDLKnowledgeBase.cpp
	src/PPDDLParser.cpp
	src/PPDDLUtils.cpp)


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
