#!/bin/bash
echo "Generating waypoints.";
rosservice call /kcl_rosplan/roadmap_server/create_prm "{nr_waypoints: 10, min_distance: 0.3, casting_distance: 2.0, connecting_distance: 8.0, occupancy_threshold: 10, total_attempts: 10}";
echo "Adding initial state and goals to knowledge base.";
param_goal="knowledge:";
param="knowledge:
- knowledge_type: 0
  instance_type: 'robot'
  instance_name: 'kenny'
  attribute_name: ''
  function_value: 0.0";
for i in $(seq 0 $(( $(rosservice call /kcl_rosplan/get_current_instances "type_name: 'waypoint'" | sed 's/wp/\n/g' | wc -l) - 2)) )
do
param_goal="$param_goal
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'visited'
  values:
  - {key: 'wp', value: 'wp$i'}
  function_value: 0.0"
done;
param="$param
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'robot_at'
  values:
  - {key: 'v', value: 'kenny'}
  - {key: 'wp', value: 'wp0'}
  function_value: 0.0";
rosservice call /kcl_rosplan/update_knowledge_base_array "update_type: 0
$param"
rosservice call /kcl_rosplan/update_knowledge_base_array "update_type: 1
$param_goal"
echo "Calling problem generator.";
rosservice call /rosplan_problem_interface/problem_generation_server;
echo "Calling planner interface.";
rosservice call /rosplan_planner_interface/planning_server;
echo "Calling plan parser.";
rosservice call /rosplan_parsing_interface/parse_plan;
echo "Calling plan dispatcher.";
rosservice call /rosplan_plan_dispatcher/dispatch_plan;
echo "Finished!";
