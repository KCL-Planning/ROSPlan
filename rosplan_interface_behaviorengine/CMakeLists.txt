cmake_minimum_required(VERSION 2.8.3)
project(rosplan_interface_behaviorengine)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  diagnostic_msgs
  fawkes_msgs
  mongodb_store
  roscpp
  rosplan_dispatch_msgs
  rosplan_knowledge_msgs
  rosplan_planning_system
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
#find_package(Boost REQUIRED COMPONENTS format)

include(cxx_atleast)
cxx_atleast(14)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rosplan_interface_behaviorengine
  CATKIN_DEPENDS actionlib diagnostic_msgs fawkes_msgs mongodb_store roscpp rosplan_dispatch_msgs rosplan_knowledge_msgs rosplan_planning_system
#  DEPENDS boost
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${boost_INCLUDE_DIRS}
)

add_executable(rosplan_interface_behaviorengine src/rosplan_interface_behaviorengine.cpp)
add_executable(rosplan_kb_updater_navgraph src/kb_updater_navgraph.cpp)

add_dependencies(rosplan_interface_behaviorengine ${catkin_EXPORTED_TARGETS})
add_dependencies(rosplan_kb_updater_navgraph ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(rosplan_interface_behaviorengine
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${boost_LIBRARIES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(rosplan_kb_updater_navgraph
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${boost_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS rosplan_interface_behaviorengine rosplan_interface_behaviorengine_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

