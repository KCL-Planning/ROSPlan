#!/bin/bash
rosservice call /rosplan_problem_interface/problem_generation_server;
rosservice call /rosplan_planner_interface/planning_server;
rosservice call /rosplan_parsing_interface/parse_plan;
rosservice call /rosplan_plan_dispatcher/dispatch_plan;
