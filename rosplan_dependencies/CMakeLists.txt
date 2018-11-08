cmake_minimum_required(VERSION 2.8.3)
project(rosplan_dependencies)

cmake_policy(SET CMP0054 NEW)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

# Get the git submodules...
message(STATUS "[${PROJECT_NAME}] Checking out the submodules...")
execute_process(COMMAND git -C ${CMAKE_CURRENT_SOURCE_DIR} submodule init OUTPUT_QUIET WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
execute_process(COMMAND git -C ${CMAKE_CURRENT_SOURCE_DIR} submodule update --force OUTPUT_QUIET WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

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
        INCLUDE_DIRS ${catkin_INCLUDE_DIRS} VAL/include rddl_parser
        LIBRARIES val rddl_parser
        CATKIN_DEPENDS roscpp
        DEPENDS
)

# COMMON FLAGS
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fprofile-arcs -ftest-coverage")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -fomit-frame-pointer -DNDEBUG")


########################################################################################################################
########################################################################################################################
## RDDL PARSER
########################################################################################################################
message(STATUS "[${PROJECT_NAME}] Preparing RDDL parser dependency...")

###########
## Build ##
###########
execute_process(COMMAND getconf LONG_BIT OUTPUT_VARIABLE LBITS)
if (LBITS EQUAL 64)
    SET (RDDL_CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64")
else()
    SET (RDDL_CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32")
endif(LBITS EQUAL 64)
SET (RDDL_CMAKE_CXX_FLAGS "${RDDL_CMAKE_CXX_FLAGS} -g -Wall -W -Wno-sign-compare -Wno-deprecated -ansi -pedantic -Werror -std=c++11")

# Lexer and parser
find_package(BISON REQUIRED)
find_package(FLEX REQUIRED)

BISON_TARGET(RDDL_PARSER rddl_parser/parser.ypp ${CMAKE_CURRENT_BINARY_DIR}/parser.tab.cc COMPILE_FLAGS -d)
FLEX_TARGET(RDDL_SCANNER rddl_parser/lexer.l ${CMAKE_CURRENT_BINARY_DIR}/lex.yy.cc)
ADD_FLEX_BISON_DEPENDENCY(RDDL_SCANNER RDDL_PARSER)

set(RDDL_PARSER_SOURCES
        rddl_parser/rddl.cc
        rddl_parser/instantiator.cc
        rddl_parser/preprocessor.cc
        rddl_parser/task_analyzer.cc
        rddl_parser/evaluatables.cc
        rddl_parser/logical_expressions.cc
        rddl_parser/states.cc
        rddl_parser/probability_distribution.cc
        rddl_parser/utils/timer.cc
        rddl_parser/utils/string_utils.cc
        rddl_parser/utils/system_utils.cc
        rddl_parser/utils/math_utils.cc
        rddl_parser/RDDLParser.cpp)
include_directories("rddl_parser")

add_library(rddl_parser SHARED  ${BISON_RDDL_PARSER_OUTPUTS} ${FLEX_RDDL_SCANNER_OUTPUTS} ${RDDL_PARSER_SOURCES})
add_dependencies(rddl_parser ${catkin_EXPORTED_TARGETS})
SET_TARGET_PROPERTIES(rddl_parser PROPERTIES COMPILE_FLAGS "${RDDL_CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

#############
## Install ##
#############

install(TARGETS rddl_parser
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## Mark cpp header files for installation
install(DIRECTORY rddl_parser/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )

########################################################################################################################
########################################################################################################################
## VAL
########################################################################################################################
message(STATUS "[${PROJECT_NAME}] Preparing VAL dependency...")

###########
## Build ##
###########

find_package(FLEX REQUIRED)

# Disable deprecated declarations warning (about std::auto_ptr)
add_definitions(-Wno-deprecated-declarations)

## val sources
set(VAL_SOURCES
        VAL/src/DebugWriteController.cpp
        VAL/src/pddl+.cpp
        VAL/src/ptree.cpp
        )

include_directories("VAL/include")

add_library(val SHARED ${VAL_SOURCES})
add_dependencies(val ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############
install(TARGETS val
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## Mark cpp header files for installation
install(DIRECTORY VAL/include/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION} # To have the same layout as the include/
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )

########################################################################################################################
########################################################################################################################
## rddlsim
########################################################################################################################
message(STATUS "[${PROJECT_NAME}] Preparing rddlsim dependency...")

###########
## Build ##
###########

if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/rddlsim/bin)
    message(STATUS "[${PROJECT_NAME}] Compiling rddlsim...")
    execute_process(COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/rddlsim/compile WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rddlsim/ OUTPUT_QUIET ERROR_QUIET)
    message(STATUS "[${PROJECT_NAME}] Done!")
else(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/rddlsim/bin)
    message(STATUS "[${PROJECT_NAME}] rddlsim is already compiled!")
endif(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/rddlsim/bin)

#############
## Install ##
#############
