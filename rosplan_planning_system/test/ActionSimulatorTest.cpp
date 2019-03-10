/*
 * Copyright [2019] <KCL - ISR - DFKI collaboration>
 *
 * KCL: King's College London
 * ISR: Institue for Systems and Robotics
 * DFKI: German Research Institute for Artificial Inteligence
 *
 * Author: Michael Cashmore (michael.cashmore@kcl.ac.uk), Oscar Lima (olima@isr.tecnico.ulisboa.pt, oscar.lima@dfki.de)
 *
 * Unit tests for the Action Simulator (ActionSimulator.h)
 *
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <rosplan_planning_system/PlanDispatch/CSPExecGenerator.h>
#include <string>

// declare object of the class ActionSimulator
ActionSimulator *action_simulator_tester;

bool compareVectors(std::vector<std::string> &vector1, std::vector<std::string> &vector2)
{
    // iterate over vector 1, see if all of its elements can be found in vector 2

    // init response to true, might get modified below
    bool vectors_are_equivalent = true;

    // iterate over vector1 and find its elements in vector2
    for(auto vit=vector1.begin(); vit!=vector1.end(); vit++) {
        if(!(std::find(vector2.begin(), vector2.end(), *vit) != vector2.end())) {
            // element not found
            vectors_are_equivalent = false;
            ROS_ERROR("element (%s) missing", vit->c_str());
        }
    }

    return vectors_are_equivalent;
}

TEST(action_simulator_test, get_operator_names)
{
    std::vector<std::string> operator_names;
    action_simulator_tester->getOperatorNames(operator_names);
    ROS_INFO("get operator names test");

    std::vector<std::string> expected_operator_names = {"goto_waypoint", "localise", "dock", "undock", \
        "wait_load_at_machine", "switch_machine_on", "wait_unload", "ditch"};

    ASSERT_EQ(compareVectors(operator_names, expected_operator_names), true);
}

TEST(action_simulator_test, get_domain_predicates)
{
    std::vector<std::string> domain_predicates;
    action_simulator_tester->getAllPredicateNames(domain_predicates);
    ROS_INFO("get all predicate names test");

    std::vector<std::string> expected_domain_predicates = {"robot_at", "undocked", "docked", "localised", \
        "dock_at", "carrying_order", "nocarrying_order", "asked_unload", "order_delivered", \
        "delivery_destination", "machine_on", "machine_off"};

    ASSERT_EQ(compareVectors(domain_predicates, expected_domain_predicates), true);
}

TEST(action_simulator_test, save_kb_snapshot_and_find_fact_and_find_goal)
{
    ROS_INFO("print KB test");

    action_simulator_tester->saveKBSnapshot();
    action_simulator_tester->printInternalKBFacts();
    action_simulator_tester->printInternalKBGoals();

    std::string action;
    std::vector<std::string> params;

    action = "robot_at";
    params = {"robot0", "wp0"};
    ASSERT_EQ(action_simulator_tester->findFactInternal(action, params), true);

    action = "nocarrying_order";
    params = {"robot0"};
    ASSERT_EQ(action_simulator_tester->findFactInternal(action, params), true);

    action = "undocked";
    params = {"robot0"};
    ASSERT_EQ(action_simulator_tester->findFactInternal(action, params), true);

    action = "localised";
    ASSERT_EQ(action_simulator_tester->findFactInternal(action, params), true);

    action = "robot_at";
    params = {"robot1", "wp0"};
    ASSERT_EQ(action_simulator_tester->findFactInternal(action, params), true);

    action = "nocarrying_order";
    params = {"robot1"};
    ASSERT_EQ(action_simulator_tester->findFactInternal(action, params), true);

    action = "undocked";
    params = {"robot1"};
    ASSERT_EQ(action_simulator_tester->findFactInternal(action, params), true);

    action = "localised";
    params = {"robot1"};
    ASSERT_EQ(action_simulator_tester->findFactInternal(action, params), true);

    /* we do not check for this facts:
    (robot_at robot2 wp0)
    (nocarrying_order robot2)
    (undocked robot2)
    (localised robot2)
    (machine_off machine0)
    (machine_off machine1)
    (machine_off machine2)
    (machine_off machine3)
    (machine_off machine4)
    (delivery_destination wp4)
    (delivery_destination wp2)
    (delivery_destination wp3)*/

    // find goals
    std::string goal= "order_delivered";

    params = {"wp4"};
    ASSERT_EQ(action_simulator_tester->findGoalInternal(goal, params), true);

    params = {"wp2"};
    ASSERT_EQ(action_simulator_tester->findGoalInternal(goal, params), true);

    params = {"wp3"};
    ASSERT_EQ(action_simulator_tester->findGoalInternal(goal, params), true);
}

TEST(action_simulator_test, remove_predicate_from_kb)
{
    std::string action = "localised";
    std::vector<std::string> params = {"robot1"};
    action_simulator_tester->removeFactInternal(action, params);
    ASSERT_EQ(action_simulator_tester->findFactInternal(action, params), false);
}

TEST(action_simulator_test, action_applicability)
{
    // action is applicable
    std::string action = "goto_waypoint";
    std::vector<std::string> params = {"robot0", "wp0", "wp1"};
    ASSERT_EQ(action_simulator_tester->isActionApplicable(action, params), true);

    // action not applicable
    action = "wait_load_at_machine";
    params = {"robot0", "robot1", "machine2"};
    ASSERT_EQ(action_simulator_tester->isActionApplicable(action, params), false);
}

TEST(action_simulator_test, simulate_action)
{
    // action is applicable
    std::string action = "goto_waypoint";
    std::vector<std::string> params = {"robot0", "wp0", "wp1"};

    // simulate action start
    action_simulator_tester->simulateActionStart(action, params);

    // check effects
    // robot not at wp0 anymore
    std::string predicate_name = "robot_at";
    params = {"robot0", "wp0"};
    ASSERT_EQ(action_simulator_tester->findFactInternal(predicate_name, params), false);

    predicate_name = "asked_unload";
    params = {"robot0"};
    ASSERT_EQ(action_simulator_tester->findFactInternal(predicate_name, params), false);

    // robot at destination (mm not yet, this is at end effect)
    predicate_name = "robot_at";
    params = {"robot0", "wp1"};
    ASSERT_EQ(action_simulator_tester->findFactInternal(predicate_name, params), false);

    // simulate action end
    action = "goto_waypoint";
    params = {"robot0", "wp0", "wp1"};
    action_simulator_tester->simulateActionEnd(action, params);

    // robot at destination
    predicate_name = "robot_at";
    params = {"robot0", "wp1"};
    ASSERT_EQ(action_simulator_tester->findFactInternal(predicate_name, params), true);
}

TEST(action_simulator_test, goals_achieved)
{
    // reset internal kb
    action_simulator_tester->saveKBSnapshot();

    // goals should be false at this point
    ASSERT_EQ(action_simulator_tester->areGoalsAchieved(), false);

    std::string action;
    std::vector<std::string> params;

    // simulate entire plan
    action = "goto_waypoint";
    params = {"robot0", "wp0", "machine4"};
    action_simulator_tester->simulateActionStart(action, params);
    action_simulator_tester->simulateActionEnd(action, params);

    action = "switch_machine_on";
    params = {"robot0", "machine4"};
    action_simulator_tester->simulateActionStart(action, params);
    action_simulator_tester->simulateActionEnd(action, params);

    action = "goto_waypoint";
    params = {"robot2", "wp0", "machine4"};
    action_simulator_tester->simulateActionStart(action, params);
    action_simulator_tester->simulateActionEnd(action, params);

    action = "wait_load_at_machine";
    params = {"robot0", "robot2", "machine4"};
    action_simulator_tester->simulateActionStart(action, params);
    action_simulator_tester->simulateActionEnd(action, params);

    action = "goto_waypoint";
    params = {"robot0", "machine4", "wp3"};
    action_simulator_tester->simulateActionStart(action, params);
    action_simulator_tester->simulateActionEnd(action, params);

    action = "wait_unload";
    params = {"robot0", "wp3"};
    action_simulator_tester->simulateActionStart(action, params);
    action_simulator_tester->simulateActionEnd(action, params);

    action = "goto_waypoint";
    params = {"robot0", "wp3", "machine4"};
    action_simulator_tester->simulateActionStart(action, params);
    action_simulator_tester->simulateActionEnd(action, params);

    action = "wait_load_at_machine";
    params = {"robot0", "robot2", "machine4"};
    action_simulator_tester->simulateActionStart(action, params);
    action_simulator_tester->simulateActionEnd(action, params);

    action = "goto_waypoint";
    params = {"robot0", "machine4", "wp2"};
    action_simulator_tester->simulateActionStart(action, params);
    action_simulator_tester->simulateActionEnd(action, params);

    action = "wait_unload";
    params = {"robot0", "wp2"};
    action_simulator_tester->simulateActionStart(action, params);
    action_simulator_tester->simulateActionEnd(action, params);

    action = "goto_waypoint";
    params = {"robot1", "wp0", "machine4"};
    action_simulator_tester->simulateActionStart(action, params);
    action_simulator_tester->simulateActionEnd(action, params);

    action = "wait_load_at_machine";
    params = {"robot1", "robot2", "machine4"};
    action_simulator_tester->simulateActionStart(action, params);
    action_simulator_tester->simulateActionEnd(action, params);

    action = "goto_waypoint";
    params = {"robot1", "machine4", "wp4"};
    action_simulator_tester->simulateActionStart(action, params);
    action_simulator_tester->simulateActionEnd(action, params);

    action = "wait_unload";
    params = {"robot1", "wp4"};
    action_simulator_tester->simulateActionStart(action, params);
    action_simulator_tester->simulateActionEnd(action, params);

    // goal should be achieved now! after plan execution
    ASSERT_EQ(action_simulator_tester->areGoalsAchieved(), true);
}

TEST(action_simulator_test, revert_action)
{
    // reset internal kb
    action_simulator_tester->saveKBSnapshot();

    // simulate action
    std::string action = "goto_waypoint";
    std::vector<std::string> params = {"robot0", "wp0", "wp1"};
    action_simulator_tester->simulateActionStart(action, params);
    action_simulator_tester->simulateActionEnd(action, params);

    // revert action
    action_simulator_tester->revertActionStart(action, params);
    action_simulator_tester->revertActionEnd(action, params);

    std::string predicate_name;

    // check robot should be at same location as it was
    predicate_name = "robot_at";
    params = {"robot0", "wp0"};
    // should be there
    ASSERT_EQ(action_simulator_tester->findFactInternal(predicate_name, params), true);

    predicate_name = "robot_at";
    params = {"robot0", "wp1"};
    // should not be there
    ASSERT_EQ(action_simulator_tester->findFactInternal(predicate_name, params), false);

    predicate_name = "asked_unload";
    params = {"robot0"};
    // should not be there
    ASSERT_EQ(action_simulator_tester->findFactInternal(predicate_name, params), false);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_simulator_test_node");
    testing::InitGoogleTest(&argc, argv);
    action_simulator_tester = new ActionSimulator(true, true);
    return RUN_ALL_TESTS();
}
