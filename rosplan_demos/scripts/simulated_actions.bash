#!/bin/bash
param_goals="knowledge:";
param="knowledge:
- knowledge_type: 0
  instance_type: 'robot'
  instance_name: 'kenny'
  attribute_name: ''
  function_value: 0.0
- knowledge_type: 0
  instance_type: 'robot'
  instance_name: 'beth'
  attribute_name: ''
  function_value: 0.0";
for i in $(seq 0 4 )
do
param="$param
- knowledge_type: 0
  instance_type: 'waypoint'
  instance_name: 'wp$i'
  attribute_name: ''
  function_value: 0.0";
param_goals="$param_goals
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'visited'
  values:
  - {key: 'wp', value: 'wp$i'}
  function_value: 0.0"
done;
param_goals="$param_goals
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'docked'
  values:
  - {key: 'v', value: 'kenny'}
  function_value: 0.0
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'docked'
  values:
  - {key: 'v', value: 'beth'}
  function_value: 0.0"
param="$param
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'robot_at'
  values:
  - {key: 'v', value: 'kenny'}
  - {key: 'wp', value: 'wp0'}
  function_value: 0.0
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'robot_at'
  values:
  - {key: 'v', value: 'beth'}
  - {key: 'wp', value: 'wp0'}
  function_value: 0.0
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'docked'
  values:
  - {key: 'v', value: 'kenny'}
  function_value: 0.0
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'docked'
  values:
  - {key: 'v', value: 'beth'}
  function_value: 0.0
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'dock_at'
  values:
  - {key: 'wp', value: 'wp0'}
  function_value: 0.0";

rosservice call /kcl_rosplan/update_knowledge_base_array "update_type: 0
$param"
rosservice call /kcl_rosplan/update_knowledge_base_array "update_type: 1
$param_goals"
rosservice call /rosplan_problem_interface/problem_generation_server;
rosservice call /rosplan_planner_interface/planning_server;
rosservice call /rosplan_parsing_interface/parse_plan;
rosservice call /rosplan_plan_dispatcher/dispatch_plan;
