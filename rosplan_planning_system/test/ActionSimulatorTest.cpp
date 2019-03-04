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

std::string event_out_response;
bool callback_received = false;

// get operator names test
TEST(action_simulator_test, success_test)
{
    // create object of the node class (ActionSimulator)
    ActionSimulator action_simulator_tester(true, true);

    std::vector<std::string> operator_names;
    action_simulator_tester.getOperatorNames(operator_names);
    ROS_INFO("get operator names test");

    std::vector<std::string> expected_operator_names = {"goto_waypoint", "localise", "dock", "undock", \
        "wait_load_at_machine", "switch_machine_on", "wait_unload", "ditch"};

    bool vectors_are_equivalent = true;

    // iterate over operator_names vector and find its elements in operator_names
    for(auto onit=expected_operator_names.begin(); onit!=expected_operator_names.end(); onit++) {
        // find operator names in the fetched ones
        if(!(std::find(operator_names.begin(), operator_names.end(), *onit) != operator_names.end())) {
            // element not found
            vectors_are_equivalent = false;
            ROS_ERROR("operator name (%s) missing in domain", onit->c_str());
        }
    }

    ASSERT_EQ(vectors_are_equivalent, true);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    // init node
    ros::init(argc, argv, "action_simulator_test");
    ROS_INFO("Test initialized");

    return RUN_ALL_TESTS();
}


//
//     // snippet: get all domain predicate names
//     std::vector<std::string> domain_predicates;
//     action_simulator_tester.getAllPredicateNames(domain_predicates);
//     ROS_DEBUG("get all predicate names:");
//     ROS_DEBUG("================");
//     for(auto it=domain_predicates.begin(); it != domain_predicates.end(); it++)
//     {
//         ROS_DEBUG("%s", it->c_str());
//     }
//
//     // save all predicates to internal KB
//     action_simulator_tester.saveKBSnapshot();
//
//     // snippet: print internal KB facts
//     ROS_DEBUG("print internal KB facts:");
//     ROS_DEBUG("================");
//     action_simulator_tester.printInternalKBFacts();
//
//     // snippet: print internal KB goals
//     ROS_DEBUG("print internal KB goals:");
//     ROS_DEBUG("================");
//     action_simulator_tester.printInternalKBGoals();
//
//     // snippet: find fact in KB (with no args)
//     ROS_DEBUG("find fact in internal KB no args:");
//     ROS_DEBUG("================");
//     std::string predicate_name2 = "person_descending";
//     if(action_simulator_tester.findFactInternal(predicate_name2))
//         ROS_DEBUG("predicate : (person_descending) found in internal KB");
//     else
//         ROS_DEBUG("predicate : (person_descending) not found in internal KB");
//     // alternative option
//     ROS_DEBUG("find fact in internal KB no args (alternative option):");
//     ROS_DEBUG("================");
//     std::string predicate_name3 = "person_descending";
//     std::vector<std::string> args1 = {""};
//     if(action_simulator_tester.findFactInternal(predicate_name3, args1))
//         ROS_DEBUG("predicate : (person_descending) found in internal KB");
//     else
//         ROS_DEBUG("predicate : (person_descending) not found in internal KB");
//
//     // snippet: find fact in KB (with args)
//     ROS_DEBUG("find fact in internal KB with args:");
//     ROS_DEBUG("================");
//     std::string predicate_name4 = "has_driver_license";
//     std::vector<std::string> args2 = {"batdad"};
//     if(action_simulator_tester.findFactInternal(predicate_name4, args2))
//         ROS_DEBUG("predicate : (has_driver_license batdad) found in internal KB");
//     else
//         ROS_DEBUG("predicate : (has_driver_license batdad) not found in internal KB");
//
//     // snippet: remove predicate from KB (with args)
//     std::string predicate_name = "has_driver_license";
//     std::vector<std::string> args3 = {"batdad"};
//     action_simulator_tester.removeFactInternal(predicate_name, args3);
//     ROS_DEBUG("test: remove predicate with args (has_driver_license batdad):");
//     ROS_DEBUG("================");
//     action_simulator_tester.printInternalKBFacts();
//
//     // snippet: remove predicate from KB (no args)
//     std::string predicate_name5 = "person_descending";
//     std::vector<std::string> args4 = {""};
//     ROS_DEBUG("test: remove predicate with no args (person_descending):");
//     ROS_DEBUG("================");
//     action_simulator_tester.removeFactInternal(predicate_name5, args4);
//     action_simulator_tester.printInternalKBFacts();
//
//     // snippet: see if action is applicable
//     std::string action_name = "get_down_from_car";
//     std::vector<std::string> params = {"batdad","car","ben_school"}; // person, car, location
//     ROS_DEBUG("check if action is applicable: (get_down_from_car batdad car ben_school), expected outcome is true");
//     ROS_DEBUG("================");
//     if(action_simulator_tester.isActionApplicable(action_name, params))
//         ROS_DEBUG("action is applicable!");
//     else
//         ROS_DEBUG("action is not applicable");
//
//     // snippet: simulate an action start
//     ROS_DEBUG("simulate action start: (get_down_from_car batdad car ben_school)");
//     ROS_DEBUG("================");
//     ROS_DEBUG("KB before simulation");
//     action_simulator_tester.printInternalKBFacts();
//     std::string action_name2 = "get_down_from_car";
//     std::vector<std::string> params2 = {"batdad","car","ben_school"};
//     action_simulator_tester.simulateActionStart(action_name2, params2);
//     ROS_DEBUG("KB after simulation");
//     action_simulator_tester.printInternalKBFacts();
//
//     // snippet: test areGoalsAchieved()
//     ROS_DEBUG("Check if goals are achieved:");
//     ROS_DEBUG("================");
//     ROS_DEBUG("KB goals:");
//     action_simulator_tester.printInternalKBGoals();
//     ROS_DEBUG("KB facts:");
//     action_simulator_tester.printInternalKBFacts();
//     if(action_simulator_tester.areGoalsAchieved())
//         ROS_DEBUG("goals achieved");
//     else
//         ROS_DEBUG("goals not achieved");
//
//     // snippet: test revert actions
//     ROS_DEBUG("Check reverting action: (get_down_from_car batdad car ben_school)");
//     ROS_DEBUG("================");
//     ROS_DEBUG("KB facts:");
//     action_simulator_tester.printInternalKBFacts();
//     std::string action_name3 = "get_down_from_car";
//     std::vector<std::string> params3 = {"batdad","car","ben_school"}; // person, car, location
//     if(action_simulator_tester.revertActionStart(action_name3, params3))
//         ROS_DEBUG("action reverted succesfully");
//     else
//         ROS_ERROR("failed to revert action");
//     ROS_DEBUG("KB facts after reverting action:");
//     action_simulator_tester.printInternalKBFacts();
