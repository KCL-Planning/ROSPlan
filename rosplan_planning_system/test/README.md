# rosplan_planning_system - unit tests

## build tests manually

Build rosplan_planning_system tests:

        catkin build --no-deps rosplan_planning_system --make-args run_tests

Build all catkin ws tests:

        catkin build --make-args run_tests

## run tests manually

Planner Interface tests:

        rostest rosplan_planning_system planner_interface.test --text

CHIMP Planner Interface tests:

        rostest rosplan_planning_system chimp_planner_interface.test --text

Problem Interface tests:

        rostest rosplan_planning_system problem_interface.test --text

Parsing Interface tests:

        rostest rosplan_planning_system parsing_interface.test --text

Plan Dispatch Interface tests:

        rostest rosplan_planning_system plan_dispatch.test --text

Run all catkin ws tests:

        catkin run_tests

