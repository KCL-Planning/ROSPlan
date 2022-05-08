cmake_minimum_required(VERSION 3.0.2)
project(rosplan_action_interface)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib_msgs
  diagnostic_msgs
  roscpp
  rospy
  std_msgs
  genmsg
  actionlib_msgs
)

catkin_python_setup()

catkin_package(
#  CATKIN_DEPENDS
)

###########
## Build ##
###########

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

