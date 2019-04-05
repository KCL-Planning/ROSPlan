//
// Created by martin on 04/04/19.
//
#include <iostream>
#include <gtest/gtest.h>
#include <ros/package.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "rosplan_dispatch_msgs/ParsingService.h"
#include "rosplan_dispatch_msgs/CompletePlan.h"
#include "rosplan_dispatch_msgs/EsterelPlan.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"

bool plan_messages_received;
std::vector<rosplan_dispatch_msgs::ActionDispatch> simple_plan_messages;

void testCallback(const rosplan_dispatch_msgs::CompletePlan completePlan) {

    for (auto rit=completePlan.plan.begin(); rit!=completePlan.plan.end(); rit++) {
        simple_plan_messages.push_back(*rit);
    }

    plan_messages_received = true;
}


bool esterel_plan_messages_received;
rosplan_dispatch_msgs::EsterelPlan retrieved_esterelPlan;

void esterelTestCallback(const rosplan_dispatch_msgs::EsterelPlan esterelPlan) {

    retrieved_esterelPlan = esterelPlan;

    esterel_plan_messages_received = true;
}

GTEST_TEST(ParsingInterfaceTests, Test1_parse_plan) {

    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<std_srvs::Empty>("/rosplan_parsing_interface/parse_plan");
    std_srvs::Empty srv;

    ASSERT_EQ(1, client1.call(srv));
}

GTEST_TEST(ParsingInterfaceTests, Test2_parse_plan_from_file) {

    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<rosplan_dispatch_msgs::ParsingService>("/rosplan_parsing_interface/parse_plan_from_file");
    rosplan_dispatch_msgs::ParsingService srv;

    std::string rosplan_planning_system_path = ros::package::getPath("rosplan_planning_system");

    srv.request.plan_path = rosplan_planning_system_path + "/test/pddl/test_plan.pddl";

    client1.call(srv);

    ASSERT_EQ(1, srv.response.plan_parsed);

}


GTEST_TEST(ParsingInterfaceTests, Test3_simple_plan_messages_published_on_plan_topic) {

    ros::NodeHandle n("~");

    ros::Subscriber sub = n.subscribe<rosplan_dispatch_msgs::CompletePlan>("/rosplan_parsing_interface/complete_plan", 1000, &testCallback);


    ros::ServiceClient client1 = n.serviceClient<rosplan_dispatch_msgs::ParsingService>("/rosplan_parsing_interface/parse_plan_from_file");
    rosplan_dispatch_msgs::ParsingService srv;

    std::string rosplan_planning_system_path = ros::package::getPath("rosplan_planning_system");

    srv.request.plan_path = rosplan_planning_system_path + "/test/pddl/test_plan.pddl";

    client1.call(srv);

    ros::Rate loop_rate = 10;
    plan_messages_received = false;
    while (!plan_messages_received && ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    std::vector<std::string> actual_action_names;
    std::vector<std::string> expected_action_names = {"load-truck", "walk", "board-truck", "drive-truck", "drive-truck", "unload-truck"};


    for (auto rit=simple_plan_messages.begin(); rit!=simple_plan_messages.end(); rit++) {
        actual_action_names.push_back(rit->name);
    }

    ASSERT_EQ(expected_action_names, actual_action_names);

}


GTEST_TEST(ParsingInterfaceTests, Test4_action_id_unique) {

    bool unique = true;
    int prevId = 1000;
    int currId;
    for (auto rit=simple_plan_messages.begin(); rit!=simple_plan_messages.end(); rit++) {
        currId = rit->action_id;
        if(currId == prevId){
            unique = false;
            break;
         }
        else{
            prevId = rit->action_id;
        }
    }
    ASSERT_EQ(true, unique);
}

GTEST_TEST(ParsingInterfaceTests, Test5_full_complete_plan) {

    std::vector<int> actual_parameter_size;
    std::vector<float> actual_durations;
    std::vector<float> actual_dispatch_time;

    std::vector<int> expected_parameter_size = {3, 3, 3, 4, 4, 3};
    std::vector<float> expected_durations = {0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
    std::vector<float> expected_dispatch_time = {0, 0, 0.001, 0.002, 0.003, 0.004};


    for (auto rit=simple_plan_messages.begin(); rit!=simple_plan_messages.end(); rit++) {
        actual_parameter_size.push_back(rit->parameters.size());
        actual_durations.push_back(rit->duration);
        actual_dispatch_time.push_back(rit->dispatch_time);
    }


    ASSERT_EQ(expected_parameter_size, actual_parameter_size);
    ASSERT_EQ(expected_durations, actual_durations);
    ASSERT_EQ(expected_dispatch_time, actual_dispatch_time);
}


GTEST_TEST(ParsingInterfaceTests, Test6_esterel_plan_messages_published_on_plan_topic) {


    ros::NodeHandle n("~");

    ros::Subscriber sub = n.subscribe<rosplan_dispatch_msgs::EsterelPlan>("/rosplan_parsing_interface_esterel/complete_plan", 1000, &esterelTestCallback);

    ros::ServiceClient client1 = n.serviceClient<rosplan_dispatch_msgs::ParsingService>("/rosplan_parsing_interface_esterel/parse_plan_from_file");
    rosplan_dispatch_msgs::ParsingService srv;

    std::string rosplan_planning_system_path = ros::package::getPath("rosplan_planning_system");

    srv.request.plan_path = rosplan_planning_system_path + "/test/pddl/test_plan.pddl";

    client1.call(srv);

    ros::Rate loop_rate = 10;
    esterel_plan_messages_received = false;
    while (!esterel_plan_messages_received && ros::ok()) {
    loop_rate.sleep();
    ros::spinOnce();
    }

    int actual_number_of_nodes = retrieved_esterelPlan.nodes.size();
    int actual_number_of_edges = retrieved_esterelPlan.edges.size();

    int expected_number_of_nodes = 13;
    int expected_number_of_edges = 21;

    ASSERT_EQ(expected_number_of_nodes, actual_number_of_nodes);
    ASSERT_EQ(expected_number_of_edges, actual_number_of_edges);

}

GTEST_TEST(ParsingInterfaceTests, Test7_esterel_plan_node_id_unique) {

    bool unique = true;
    int prevId = 1000;
    int currId;

    for (auto rit=retrieved_esterelPlan.nodes.begin(); rit!=retrieved_esterelPlan.nodes.end(); rit++) {

        currId = rit->node_id;
        if(currId == prevId){
            unique = false;
            break;
        }
        else{
            prevId = rit->node_id;
        }
    }
    ASSERT_EQ(true, unique);
}


GTEST_TEST(ParsingInterfaceTests, Test8_esterel_plan_edge_id_unique) {

    bool unique = true;
    int prevId = 1000;
    int currId;

    for (auto rit=retrieved_esterelPlan.edges.begin(); rit!=retrieved_esterelPlan.edges.end(); rit++) {

        std::cout << rit->edge_id << std::endl;
        currId = rit->edge_id;
        if(currId == prevId){
            unique = false;
            break;
        }
        else{
            prevId = rit->edge_id;
        }
    }
    ASSERT_EQ(true, unique);
}




int main(int argc, char **argv) {

    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "ParsingInterfaceTests");

    ros::NodeHandle n("~");

    return RUN_ALL_TESTS();
}
