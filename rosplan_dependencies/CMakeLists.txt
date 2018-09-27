cmake_minimum_required(VERSION 2.8.3)
project(rosplan_dependencies)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${catkin_INCLUDE_DIRS} VAL/include
  LIBRARIES VAL
  CATKIN_DEPENDS roscpp
  DEPENDS
)

########################################################################################################################
## VAL
########################################################################################################################
message(STATUS "Preparing VAL dependency...")

###########
## Build ##
###########

find_package(FLEX REQUIRED)

# Disable deprecated declarations warning (about std::auto_ptr)
add_definitions(-Wno-deprecated-declarations)

## val sources
set(VAL_SOURCES
        VAL/src/DebugWriteController.cpp
        VAL/src/FuncAnalysis.cpp
        VAL/src/SASActions.cpp
        VAL/src/SimpleEval.cpp
        VAL/src/TIM.cpp
        VAL/src/TimSupport.cpp
        VAL/src/ToFunction.cpp
        VAL/src/TypedAnalyser.cpp
        VAL/src/instantiation.cpp
        VAL/src/pddl+.cpp
        VAL/src/ptree.cpp
        VAL/src/typecheck.cpp
        VAL/src/Validator.cpp
        VAL/src/Action.cpp
        VAL/src/Proposition.cpp
        VAL/src/FuncExp.cpp
        VAL/src/RepairAdvice.cpp
        VAL/src/State.cpp
        VAL/src/Plan.cpp
        VAL/src/Ownership.cpp
        VAL/src/Environment.cpp
        VAL/src/Events.cpp
        VAL/src/Polynomial.cpp
        VAL/src/TrajectoryConstraints.cpp
        VAL/src/RobustAnalyse.cpp
        VAL/src/PrettyPrinter.cpp
        VAL/src/random.cpp
        VAL/src/LaTeXSupport.cpp
        VAL/src/Utils.cpp)
include_directories("VAL/include")

add_library(VAL ${VAL_SOURCES})
add_dependencies(VAL ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############
 install(TARGETS VAL
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY VAL/include
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

########################################################################################################################
########################################################################################################################

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/rosplan_dependencies.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/rosplan_dependencies_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
