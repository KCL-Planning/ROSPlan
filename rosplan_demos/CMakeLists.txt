cmake_minimum_required(VERSION 2.8.3)
project(rosplan_demos)

find_package(catkin REQUIRED)


###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
)


install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
