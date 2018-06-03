#!/bin/bash

#objects

rosservice call /kcl_rosplan/update_knowledge_base_array "update_type: 0
knowledge:
- knowledge_type: 0
  instance_type: 'robot'
  instance_name: 'kinba'
  attribute_name: ''
  function_value: 0.0
- knowledge:
  knowledge_type: 0
  instance_type: 'floor'
  instance_name: 'fl0'
  attribute_name: ''
  function_value: 0.0
- knowledge:
  knowledge_type: 0
  instance_type: 'floor'
  instance_name: 'fl1'
  attribute_name: ''
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'robot_at'
  values:
  - {key: 'v', value: 'kinba'}
  - {key: 'wp', value: 'wp0-0'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'docked'
  values:
  - {key: 'v', value: 'kinba'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'outside_elevator'
  values:
  - {key: 'v', value: 'kinba'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'allowed_goto_waypoint'
  values:
  - {key: 'v', value: 'kinba'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'dock_at'
  values:
  - {key: 'wp', value: 'wp0-0'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'elevator_waypoint'
  values:
  - {key: 'wp', value: 'wp0-6'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'elevator_waypoint'
  values:
  - {key: 'wp', value: 'wp1-6'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'current_elevator_waypoint'
  values:
  - {key: 'wp', value: 'wp0-6'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'elevator_access_waypoint'
  values:
  - {key: 'wp', value: 'wp0-5'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'elevator_access_waypoint'
  values:
  - {key: 'wp', value: 'wp1-7'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'greeting_waypoint'
  values:
  - {key: 'wp', value: 'wp0-1'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'destination_waypoint'
  values:
  - {key: 'wp', value: 'wp1-8'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'hallway_waypoint'
  values:
  - {key: 'wp', value: 'wp0-0'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'hallway_waypoint'
  values:
  - {key: 'wp', value: 'wp0-1'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'hallway_waypoint'
  values:
  - {key: 'wp', value: 'wp0-5'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'hallway_waypoint'
  values:
  - {key: 'wp', value: 'wp1-7'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'hallway_waypoint'
  values:
  - {key: 'wp', value: 'wp1-8'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'waypoint_at_floor'
  values:
  - {key: 'wp', value: 'wp0-0'}
  - {key: 'fl', value: 'fl0'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'waypoint_at_floor'
  values:
  - {key: 'wp', value: 'wp0-1'}
  - {key: 'fl', value: 'fl0'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'waypoint_at_floor'
  values:
  - {key: 'wp', value: 'wp0-5'}
  - {key: 'fl', value: 'fl0'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'waypoint_at_floor'
  values:
  - {key: 'wp', value: 'wp0-6'}
  - {key: 'fl', value: 'fl0'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'waypoint_at_floor'
  values:
  - {key: 'wp', value: 'wp1-6'}
  - {key: 'fl', value: 'fl1'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'waypoint_at_floor'
  values:
  - {key: 'wp', value: 'wp1-7'}
  - {key: 'fl', value: 'fl1'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'waypoint_at_floor'
  values:
  - {key: 'wp', value: 'wp1-8'}
  - {key: 'fl', value: 'fl1'}
  function_value: 0.0";

rosservice call /kcl_rosplan/update_knowledge_base_array "update_type: 1
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'person_greeted'
  values:
  - {key: 'v', value: 'kinba'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'person_guided'
  values:
  - {key: 'v', value: 'kinba'}
  function_value: 0.0
- knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'docked'
  values:
  - {key: 'v', value: 'kinba'}
  function_value: 0.0";

echo "Complete";

#call planner to create and dispatch plan
rosservice call /kcl_rosplan/planning_server;
