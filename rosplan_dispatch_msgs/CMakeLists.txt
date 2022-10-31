cmake_minimum_required(VERSION 2.8.3)
project(rosplan_dispatch_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib_msgs
  diagnostic_msgs
  roscpp
  rospy
  std_msgs
  genmsg
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   EsterelPlan.msg
   EsterelPlanEdge.msg
   EsterelPlanNode.msg
   CompletePlan.msg
   ActionDispatch.msg
   ActionFeedback.msg
)

add_service_files(
  FILES
  PlanningService.srv
  ProblemService.srv
  ParsingService.srv
  GetPlanningParams.srv
  DispatchService.srv
)

add_action_files(
  FILES
  Plan.action
  NonBlockingDispatch.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs
   diagnostic_msgs
   std_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_generation message_runtime diagnostic_msgs roscpp rospy std_msgs genmsg
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
