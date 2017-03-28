#!/bin/bash
rosservice call /kcl_rosplan/roadmap_server/create_prm "{nr_waypoints: 10, min_distance: 0.3, casting_distance: 2.0, connecting_distance: 8.0, occupancy_threshold: 10, total_attempts: 10}";
rosservice call /kcl_rosplan/update_knowledge_base "update_type: 0
knowledge:
  knowledge_type: 0
  instance_type: 'robot'
  instance_name: 'kenny'
  attribute_name: ''
  function_value: 0.0";
rosservice call /kcl_rosplan/update_knowledge_base "update_type: 0
knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'robot_at'
  values:
  - {key: 'v', value: 'kenny'}
  - {key: 'wp', value: 'wp0'}
  function_value: 0.0";
for i in $(seq 0 $(( $(rosservice call /kcl_rosplan/get_current_instances "type_name: 'waypoint'" | sed 's/wp/\n/g' | wc -l) - 2)) )
do
rosservice call /kcl_rosplan/update_knowledge_base "update_type: 1
knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'visited'
  values:
  - {key: 'wp', value: 'wp$i'}
  function_value: 0.0"
done;
rosservice call /kcl_rosplan/update_knowledge_base "update_type: 0
knowledge:
  knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'robot_at'
  values:
  - {key: 'v', value: 'kenny'}
  - {key: 'wp', value: 'wp0'}
  function_value: 0.0";
rosservice call /kcl_rosplan/planning_server;
