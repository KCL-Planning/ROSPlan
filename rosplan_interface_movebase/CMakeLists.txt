cmake_minimum_required(VERSION 2.8.3)
project(rosplan_interface_movebase)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  diagnostic_msgs
  geometry_msgs
  move_base_msgs
  rosplan_dispatch_msgs
  rosplan_knowledge_msgs
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
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs actionlib rosplan_dispatch_msgs rosplan_knowledge_msgs mongodb_store diagnostic_msgs
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
add_executable(rpmovebase src/RPMoveBase.cpp)
add_dependencies(rpmovebase ${catkin_EXPORTED_TARGETS})

## Specify libraries against which to link a library or executable target
target_link_libraries(rpmovebase ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# add_dependencies(rpmovebase ${learning_actionlib_EXPORTED_TARGETS})

##########
## Test ##
##########
# please do not use add_rosttest_gtest (seems to be interfering with qtcreator and cmake)
# see test documentation: http://wiki.ros.org/gtest
#add_rostest_gtest(${PROJECT_NAME}_utest test/utest.cpp)
#target_link_libraries(testPlanningSystem ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

#############
## Install ##
#############

install(TARGETS rpmovebase 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
