cmake_minimum_required(VERSION 2.8.3)
project(rosplan_knowledge_base)

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
)

find_package(FLEX REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(MONGO mongo-cxx-driver)
if(${MONGO_FOUND})
	message(STATUS "Mongo CXX drivers have been found. Compiling with persistent KB.")
	add_definitions(-DUSE_MONGO_CXX)
else(${MONGO_FOUND})
	message(WARNING "Mongo CXX drivers could not be found. Compiling without persistent KB.")
endif(${MONGO_FOUND})

find_package(Boost REQUIRED COMPONENTS
  filesystem
)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs rosplan_knowledge_msgs nav_msgs geometry_msgs diagnostic_msgs visualization_msgs tf
  DEPENDS
)

###########
## Build ##
###########

## include_directories(include)
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(src/VALfiles)
if(${MONGO_FOUND})
	include_directories(${MONGO_INCLUDE_DIRS})
endif(${MONGO_FOUND})

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

## knowledge sources
set(KNOWLEDGE_SOURCES
	src/main.cpp
	src/KnowledgeBase.cpp
	src/PlanFilter.cpp
	src/KnowledgeComparitor.cpp
	src/DomainParser.cpp
	src/VALVisitorOperator.cpp
	src/VALVisitorPredicate.cpp)

if(${MONGO_FOUND})
	set(KNOWLEDGE_SOURCES
		${KNOWLEDGE_SOURCES}
		src/KnowledgeBasePersistent.cpp
		src/MongoInterface.cpp
	)
endif(${MONGO_FOUND})

## Declare cpp executables
add_executable(knowledgeBase ${KNOWLEDGE_SOURCES} ${VAL_SOURCES})
add_dependencies(knowledgeBase ${catkin_EXPORTED_TARGETS})

## Specify libraries against which to link a library or executable target
if(${MONGO_FOUND})
	target_link_libraries(knowledgeBase ${MONGO_LIBRARIES})
endif(${MONGO_FOUND})
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

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
