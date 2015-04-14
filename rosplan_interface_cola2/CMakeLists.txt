cmake_minimum_required(VERSION 2.8.3)
project(rosplan_interface_cola2)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  diagnostic_msgs
  rosplan_dispatch_msgs
  rosplan_knowledge_msgs
  auv_msgs
  cola2_control
  actionlib
  mongodb_store
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
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs actionlib rosplan_dispatch_msgs rosplan_knowledge_msgs auv_msgs cola2_control mongodb_store diagnostic_msgs
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

## Declare cpp executables
add_executable(rpcola2 src/RPCola2.cpp)

## Specify libraries against which to link a library or executable target
target_link_libraries(rpcola2 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

##########
## Test ##
##########

# please do not use add_rosttest_gtest (seems to be interfering with qtcreator and cmake)
# see test documentation: http://wiki.ros.org/gtest
#add_rostest_gtest(${PROJECT_NAME}_utest test/utest.cpp)
#target_link_libraries(testPlanningSystem ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
