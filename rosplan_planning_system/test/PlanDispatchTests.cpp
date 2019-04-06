//
// Created by martin on 05/04/19.
//

#include <iostream>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <thread>
#include <chrono>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "rosplan_dispatch_msgs/ParsingService.h"
#include "rosplan_dispatch_msgs/CompletePlan.h"
#include "rosplan_dispatch_msgs/EsterelPlan.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "rosplan_dispatch_msgs/DispatchService.h"


bool actions_received;
rosplan_dispatch_msgs::ActionDispatch retrieved_action;

void testCallback(const rosplan_dispatch_msgs::ActionDispatch action) {
    std::cout << "In callbaack" << std::endl;


    retrieved_action = action;

    actions_received = true;
}


bool graph_received;
std::string retrieved_graph;

void graphTestCallback(const std_msgs::String::ConstPtr &graph) {
    std::cout << "In callbaack" << std::endl;


    retrieved_graph = graph->data;

    graph_received = true;
}

bool feedback_received;
rosplan_dispatch_msgs::ActionFeedback retrieved_feedback;

void feedbackTestCallback(const rosplan_dispatch_msgs::ActionFeedback feedback) {

    std::cout << "aight" << std::endl;

    retrieved_feedback = feedback;

    feedback_received = true;
}


// Documentation incorrectly states that dispatch_plan service is of type std_srvs/Empty

GTEST_TEST(PlanDispatchTests, Test1_plan_dispatch_simple) {

    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<std_srvs::Empty>("/rosplan_problem_interface/problem_generation_server");
    std_srvs::Empty srv1;

    ros::ServiceClient client2 = n.serviceClient<std_srvs::Empty>("/rosplan_planner_interface/planning_server");
    std_srvs::Empty srv2;

    ros::ServiceClient client3 = n.serviceClient<std_srvs::Empty>("/rosplan_parsing_interface/parse_plan");
    std_srvs::Empty srv3;

    ros::ServiceClient client4 = n.serviceClient<rosplan_dispatch_msgs::DispatchService>("/rosplan_plan_dispatcher/dispatch_plan");
    rosplan_dispatch_msgs::DispatchService srv4;

    client1.call(srv1);
    client2.call(srv2);
    client3.call(srv3);
    client4.call(srv4);

    ASSERT_EQ(true, srv4.response.success);

    std::this_thread::sleep_for (std::chrono::seconds(1));

    ASSERT_EQ(true, srv4.response.goal_achieved);


}

GTEST_TEST(PlanDispatchTests, Test2_published_action_dispatch) {

    ros::NodeHandle n("~");
    ros::Subscriber sub = n.subscribe<rosplan_dispatch_msgs::ActionDispatch>("/rosplan_plan_dispatcher/action_dispatch", 1000, &testCallback);


    ros::ServiceClient client1 = n.serviceClient<std_srvs::Empty>("/rosplan_problem_interface/problem_generation_server");
    std_srvs::Empty srv1;

    ros::ServiceClient client2 = n.serviceClient<std_srvs::Empty>("/rosplan_planner_interface/planning_server");
    std_srvs::Empty srv2;

    ros::ServiceClient client3 = n.serviceClient<std_srvs::Empty>("/rosplan_parsing_interface/parse_plan");
    std_srvs::Empty srv3;

    ros::ServiceClient client4 = n.serviceClient<rosplan_dispatch_msgs::DispatchService>("/rosplan_plan_dispatcher/dispatch_plan");
    rosplan_dispatch_msgs::DispatchService srv4;

    client1.call(srv1);
    client2.call(srv2);
    client3.call(srv3);
    client4.call(srv4);

    ASSERT_EQ(true, srv4.response.success);

    std::this_thread::sleep_for (std::chrono::seconds(1));

    ASSERT_EQ(true, srv4.response.goal_achieved);

    ros::Rate loop_rate = 10;
    actions_received = false;
    while (!actions_received && ros::ok()) {
        std::cout << "In while" << std::endl;

        loop_rate.sleep();
        ros::spinOnce();
    }

    int actual_action_id = retrieved_action.action_id;
    std::string actual_name = retrieved_action.name;
    int actual_parameter_size = retrieved_action.parameters.size();
    float actual_duration = retrieved_action.duration;
    float actual_dispatch_time = retrieved_action.dispatch_time;

    int expected_action_id = 11;
    std::string expected_name = "dock";
    int expected_parameter_size = 2;
    float expected_duration = 30;
    float expected_dispatch_time = 610.011;

    ASSERT_EQ(expected_action_id, actual_action_id);
    ASSERT_EQ(expected_name, actual_name);
    ASSERT_EQ(expected_parameter_size, actual_parameter_size);
    ASSERT_EQ(expected_duration, actual_duration);
    ASSERT_EQ(expected_dispatch_time, actual_dispatch_time);

}

GTEST_TEST(PlanDispatchTests, Test3_published_action_feedback) {

    ros::NodeHandle n("~");
    ros::Subscriber sub = n.subscribe<rosplan_dispatch_msgs::ActionFeedback>("/rosplan_plan_dispatcher/action_feedback", 1000, &feedbackTestCallback);


    ros::ServiceClient client1 = n.serviceClient<std_srvs::Empty>("/rosplan_problem_interface/problem_generation_server");
    std_srvs::Empty srv1;

    ros::ServiceClient client2 = n.serviceClient<std_srvs::Empty>("/rosplan_planner_interface/planning_server");
    std_srvs::Empty srv2;

    ros::ServiceClient client3 = n.serviceClient<std_srvs::Empty>("/rosplan_parsing_interface/parse_plan");
    std_srvs::Empty srv3;

    ros::ServiceClient client4 = n.serviceClient<rosplan_dispatch_msgs::DispatchService>("/rosplan_plan_dispatcher/dispatch_plan");
    rosplan_dispatch_msgs::DispatchService srv4;

    client1.call(srv1);
    client2.call(srv2);
    client3.call(srv3);
    client4.call(srv4);

    ASSERT_EQ(true, srv4.response.success);

    std::this_thread::sleep_for (std::chrono::seconds(1));

    ASSERT_EQ(true, srv4.response.goal_achieved);

    ros::Rate loop_rate = 10;
    feedback_received = false;
    while (!feedback_received && ros::ok()) {
        std::cout << "In while" << std::endl;

        loop_rate.sleep();
        ros::spinOnce();
    }
    std::cout << retrieved_feedback.action_id << std::endl;

    int actual_action_id = retrieved_feedback.action_id;
    std::string actual_status = retrieved_feedback.status;
    int actual_information_size = retrieved_feedback.information.size();


    int expected_action_id = 10;
    std::string expected_status = "action achieved";
    int expected_information_size = 0;



    ASSERT_EQ(expected_action_id, actual_action_id);
    ASSERT_EQ(expected_status, actual_status);
    ASSERT_EQ(expected_information_size, actual_information_size);

}

GTEST_TEST(PlanDispatchTests, Test4_published_plan_graph) {

    ros::NodeHandle n("~");
    ros::Subscriber sub = n.subscribe<std_msgs::String>("/rosplan_plan_dispatcher/plan_graph", 1000, &graphTestCallback);


    ros::ServiceClient client1 = n.serviceClient<std_srvs::Empty>("/rosplan_problem_interface/problem_generation_server");
    std_srvs::Empty srv1;

    ros::ServiceClient client2 = n.serviceClient<std_srvs::Empty>("/rosplan_planner_interface/planning_server");
    std_srvs::Empty srv2;

    ros::ServiceClient client3 = n.serviceClient<std_srvs::Empty>("/rosplan_parsing_interface/parse_plan");
    std_srvs::Empty srv3;

    ros::ServiceClient client4 = n.serviceClient<rosplan_dispatch_msgs::DispatchService>("/rosplan_plan_dispatcher/dispatch_plan");
    rosplan_dispatch_msgs::DispatchService srv4;

    client1.call(srv1);
    client2.call(srv2);
    client3.call(srv3);
    client4.call(srv4);

    ASSERT_EQ(true, srv4.response.success);

    std::this_thread::sleep_for (std::chrono::seconds(1));

    ASSERT_EQ(true, srv4.response.goal_achieved);

    ros::Rate loop_rate = 10;
    graph_received = false;
    while (!graph_received && ros::ok()) {
        std::cout << "In while" << std::endl;

        loop_rate.sleep();
        ros::spinOnce();
    }

std::string expected_graph = "digraph plan {\n0[ label=\"plan_start\",style=filled,fillcolor=black,fontcolor=white];\n}\n";

ASSERT_EQ(expected_graph, retrieved_graph);

}







int main(int argc, char **argv) {

    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "PlanDispatchTests");

    ros::NodeHandle n("~");

    return RUN_ALL_TESTS();
}
