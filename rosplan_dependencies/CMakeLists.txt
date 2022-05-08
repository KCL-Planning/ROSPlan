cmake_minimum_required(VERSION 2.8.3)
project(rosplan_dependencies)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS rosplan_knowledge_msgs)

# Get the git submodules...
message(STATUS "[${PROJECT_NAME}] Checking out the submodules...")
execute_process(COMMAND git -C ${CMAKE_CURRENT_SOURCE_DIR} submodule init OUTPUT_QUIET ERROR_QUIET WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
execute_process(COMMAND git -C ${CMAKE_CURRENT_SOURCE_DIR} submodule update --force OUTPUT_QUIET ERROR_QUIET WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

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
        INCLUDE_DIRS ${catkin_INCLUDE_DIRS} VAL/include rddl_parser ippc_server/include ippc_server/cpp-base64 ppddl_parser/include ppddl_parser/src hddl_parser/include
        LIBRARIES val_kcl rddl_parser ippc_server ppddl_parser hddl_parser
        CATKIN_DEPENDS roscpp rosplan_knowledge_msgs
        DEPENDS
)

# COMMON FLAGS
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fprofile-arcs -ftest-coverage")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -fomit-frame-pointer -DNDEBUG")

########################################################################################################################
########################################################################################################################
########################################################################################################################
## HDDL PARSER
########################################################################################################################
message(STATUS "[${PROJECT_NAME}] Preparing HDDL parser dependency...")

###########
## Build ##
###########

# Lexer and parser
find_package(BISON 3.0.4 REQUIRED)
find_package(FLEX REQUIRED)

# make directory ${CATKIN_DEVEL_PREFIX}/include, CATKIN_DEVEL_PREFIX=$HOME/ros_ws/.../devel/.private/rosplan_dependencies
add_custom_target(foo ALL COMMAND cmake -E make_directory ${CATKIN_DEVEL_PREFIX}/include/hddl_parser)

BISON_TARGET(HDDL_PARSER hddl_parser/src/hddl_parser.yacc
             ${CATKIN_DEVEL_PREFIX}/include/hddl_parser/hddl_parser.tab.cpp)
FLEX_TARGET(HDDL_SCANNER hddl_parser/src/hddl_scanner.lex
             ${CATKIN_DEVEL_PREFIX}/include/hddl_parser/hddl_scanner.yacc.cpp)
ADD_FLEX_BISON_DEPENDENCY(HDDL_SCANNER HDDL_PARSER)

set(HDDL_PARSER_SOURCES hddl_parser/src/hddl_parser.cpp)
include_directories(hddl_parser/include)
include_directories(${CATKIN_DEVEL_PREFIX}/include/hddl_parser)

add_library(hddl_parser ${BISON_HDDL_PARSER_OUTPUTS} ${FLEX_HDDL_SCANNER_OUTPUTS} ${HDDL_PARSER_SOURCES})
add_dependencies(hddl_parser ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# hddl parser is already installed under devel using the lines above
# BISON_TARGET & FLEX_TARGET
# see : $HOME/ros_ws/.../devel/.private/rosplan_dependencies/include/hddl_parser

########################################################################################################################
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
SET (RDDL_CMAKE_CXX_FLAGS "${RDDL_CMAKE_CXX_FLAGS} -g -Wall -W -Wno-sign-compare -Wno-deprecated -ansi -pedantic -Werror -std=c++11 -O3 -DNDEBUG -fomit-frame-pointer -Wno-error=deprecated-copy")

# Lexer and parser
find_package(BISON 3.0.4 REQUIRED)
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
SET_TARGET_PROPERTIES(rddl_parser PROPERTIES COMPILE_FLAGS "${RDDL_CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS}" LINK_FLAGS "-g -O3")

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

add_library(val_kcl SHARED ${VAL_SOURCES})
add_dependencies(val_kcl ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############
install(TARGETS val_kcl
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

########################################################################################################################
########################################################################################################################
########################################################################################################################
## PPDDL PARSER
########################################################################################################################
message(STATUS "[${PROJECT_NAME}] Preparing PPDDL parser dependency...")

set(PPDDL_DIR ${CMAKE_CURRENT_LIST_DIR}/ppddl_parser/src/PPDDLParser)
# Lexer and parser
# Compile FLEX and BISON
find_package(BISON)
find_package(FLEX)


BISON_TARGET(PPDDL_PARSER ${PPDDL_DIR}/parser.yy ${CMAKE_CURRENT_BINARY_DIR}/ppddl_parser.tab.cc COMPILE_FLAGS -d)
FLEX_TARGET(PPDDL_SCANNER ${PPDDL_DIR}/tokenizer.ll ${CMAKE_CURRENT_BINARY_DIR}/ppddl_lex.yy.cc)
ADD_FLEX_BISON_DEPENDENCY(PPDDL_SCANNER PPDDL_PARSER)

# Define variables
set(FLEX_BISON_OUTPUTS ${BISON_PPDDL_PARSER_OUTPUTS}  ${FLEX_PPDDL_SCANNER_OUTPUTS} CACHE INTERNAL "" FORCE)

set(PPDDL_PARSER_SOURCES
        ${PPDDL_DIR}/actions.cc
        ${PPDDL_DIR}/domains.cc
        ${PPDDL_DIR}/effects.cc
        ${PPDDL_DIR}/expressions.cc
        ${PPDDL_DIR}/formulas.cc
        ${PPDDL_DIR}/functions.cc
        ${PPDDL_DIR}/predicates.cc
        ${PPDDL_DIR}/problems.cc
        ${PPDDL_DIR}/rational.cc
        ${PPDDL_DIR}/requirements.cc
        ${PPDDL_DIR}/terms.cc
        ${PPDDL_DIR}/types.cc)

set(PPDDL_PARSER_HEADERS
        ${PPDDL_DIR}/actions.h
        ${PPDDL_DIR}/domains.h
        ${PPDDL_DIR}/effects.h
        ${PPDDL_DIR}/expressions.h
        ${PPDDL_DIR}/formulas.h
        ${PPDDL_DIR}/functions.h
        ${PPDDL_DIR}/predicates.h
        ${PPDDL_DIR}/problems.h
        ${PPDDL_DIR}/rational.h
        ${PPDDL_DIR}/refcount.h
        ${PPDDL_DIR}/requirements.h
        ${PPDDL_DIR}/terms.h
        ${PPDDL_DIR}/types.h)

set(PPDDL_PARSER_ALL ${PPDDL_PARSER_SOURCES}
        ${PPDDL_PARSER_HEADERS}
        ${FLEX_BISON_OUTPUTS})

include_directories(ppddl_parser/include ppddl_parser/src ${PPDDL_DIR} VAL/include) # We add the src as include because the PPDDLParser headers are there

add_library(ppddl_parser SHARED
        ppddl_parser/include/PPDDLDeterminizator.h ppddl_parser/src/PPDDLDeterminizator.cpp
        ppddl_parser/src/PPDDLParserInterface.cpp ppddl_parser/include/PPDDLParserInterface.h
        ppddl_parser/src/PDDLPrinter.cpp ppddl_parser/include/PDDLPrinter.h
        ppddl_parser/src/VALConversion.cpp ppddl_parser/include/VALConversion.h
        ${PPDDL_PARSER_ALL}
        ppddl_parser/src/Strategies/MLODeterminizator.cpp ppddl_parser/include/Strategies/MLODeterminizator.h
        ppddl_parser/src/Strategies/AODeterminizator.cpp ppddl_parser/include/Strategies/AODeterminizator.h
        ppddl_parser/src/Strategies/TLDeterminizator.cpp ppddl_parser/include/Strategies/TLDeterminizator.h
        ppddl_parser/src/PPDDLDeterminizatorFactory.cpp ppddl_parser/include/PPDDLDeterminizatorFactory.h)
target_link_libraries(ppddl_parser val_kcl)

#############
## Install ##
#############

install(TARGETS ppddl_parser
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY ppddl_parser/include/ ppddl_parser/src/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

########################################################################################################################
########################################################################################################################
########################################################################################################################
## ippc_server
########################################################################################################################
message(STATUS "[${PROJECT_NAME}] Adding ippc_server dependency...")

###########
## Build ##
###########

include_directories(ippc_server/include ippc_server/cpp-base64 ${catkin_INCLUDE_DIRS})

add_library(ippc_server SHARED ippc_server/src/ippc_server.cpp ippc_server/src/strxml.cpp ippc_server/cpp-base64/base64.cpp)
add_dependencies(ippc_server ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS ippc_server
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## Mark cpp header files for installation
install(DIRECTORY ippc_server/include/ ippc_server/cpp-base64/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)




########################################################################################################################
########################################################################################################################
########################################################################################################################
## CHIMP
########################################################################################################################

###########
## Build ##
###########

if (ROSPLAN_COMPILE_CHIMP)
    message(STATUS "[${PROJECT_NAME}] Preparing chimp dependency...")
    message(STATUS "[${PROJECT_NAME}] Compiling CHIMP...")
    execute_process(COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/chimp/gradlew build WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/chimp/)
    message(STATUS "[${PROJECT_NAME}] Done!")
else (ROSPLAN_COMPILE_CHIMP)
    message(STATUS "[${PROJECT_NAME}] ROSPLAN_COMPILE_CHIMP is not set: CHIMP will not be compiled")
endif(ROSPLAN_COMPILE_CHIMP)

#############
## Install ##
#############

########################################################################################################################
########################################################################################################################
