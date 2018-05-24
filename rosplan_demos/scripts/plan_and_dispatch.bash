echo "Calling problem generator.";
rosservice call /rosplan_problem_interface/problem_generation_server;

echo "Calling planner interface.";
rosservice call /rosplan_planner_interface/planning_server;

echo "Parsing Plan.";
rosservice call /rosplan_parsing_interface/parse_plan;

echo "Dispatching Plan.";
rosservice call /rosplan_plan_dispatcher/dispatch_plan;
