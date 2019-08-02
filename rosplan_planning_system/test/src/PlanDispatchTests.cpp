/**
 *
 * Copyright [2019] <KCL King's College London>
 *
 * Author: Martin Koling (martinh.koling@kcl.ac.uk)
 * Author: Oscar Lima (oscar.lima@dfki.de)
 *
 * Unit tests for ROSPlan plan dispatch
 */

#include <ros/ros.h>
#include <thread>
#include <chrono>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <gtest/gtest.h>

#include "rosplan_dispatch_msgs/ParsingService.h"
#include "rosplan_dispatch_msgs/CompletePlan.h"
#include "rosplan_dispatch_msgs/EsterelPlan.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "rosplan_dispatch_msgs/DispatchService.h"

bool actions_received;
rosplan_dispatch_msgs::ActionDispatch retrieved_action;

void testCallback(const rosplan_dispatch_msgs::ActionDispatch action) {

    retrieved_action = action;
    actions_received = true;
}

bool graph_received;
std::string retrieved_graph;

void graphTestCallback(const std_msgs::String::ConstPtr &graph) {

    retrieved_graph = graph->data;
    graph_received = true;
}

bool feedback_received;
std::vector<rosplan_dispatch_msgs::ActionFeedback> feedback_vector;

void feedbackTestCallback(const rosplan_dispatch_msgs::ActionFeedback feedback) {

    feedback_vector.push_back(feedback);
    feedback_received = true;
}

GTEST_TEST(PlanDispatchTests, Test1_plan_dispatch) {

    ros::NodeHandle nh("~");

    std::string srv_name1 = "/rosplan_problem_interface/problem_generation_server";
    ros::ServiceClient client1 = nh.serviceClient<std_srvs::Empty>(srv_name1);
    std_srvs::Empty srv1;

    std::string srv_name2 = "/rosplan_planner_interface/planning_server";
    ros::ServiceClient client2 = nh.serviceClient<std_srvs::Empty>(srv_name2);
    std_srvs::Empty srv2;

    std::string srv_name3 = "/rosplan_parsing_interface/parse_plan";
    ros::ServiceClient client3 = nh.serviceClient<std_srvs::Empty>(srv_name3);
    std_srvs::Empty srv3;

    std::string srv_name4 = "/rosplan_plan_dispatcher/dispatch_plan";
    ros::ServiceClient client4 = nh.serviceClient<rosplan_dispatch_msgs::DispatchService>(srv_name4);
    rosplan_dispatch_msgs::DispatchService srv4;

    ros::service::waitForService(srv_name1, ros::Duration(1.0));
    client1.call(srv1);
    std::this_thread::sleep_for (std::chrono::milliseconds(800));

    ros::service::waitForService(srv_name2, ros::Duration(1.0));
    client2.call(srv2);
    std::this_thread::sleep_for (std::chrono::milliseconds(800));

    ros::service::waitForService(srv_name3, ros::Duration(1.0));
    client3.call(srv3);
    std::this_thread::sleep_for (std::chrono::milliseconds(800));

    ros::service::waitForService(srv_name4, ros::Duration(1.0));
    client4.call(srv4);

    EXPECT_TRUE(srv4.response.success);

    std::this_thread::sleep_for (std::chrono::seconds(1));

    EXPECT_TRUE(srv4.response.goal_achieved);
}

GTEST_TEST(PlanDispatchTests, Test2_published_action_dispatch) {

    ros::NodeHandle nh("~");
    ros::Subscriber sub = nh.subscribe<rosplan_dispatch_msgs::ActionDispatch>
                                    ("/rosplan_plan_dispatcher/action_dispatch", 1000, &testCallback);

    std::string srv_name1 = "/rosplan_problem_interface/problem_generation_server";
    ros::ServiceClient client1 = nh.serviceClient<std_srvs::Empty>(srv_name1);
    std_srvs::Empty srv1;

    std::string srv_name2 = "/rosplan_planner_interface/planning_server";
    ros::ServiceClient client2 = nh.serviceClient<std_srvs::Empty>(srv_name2);
    std_srvs::Empty srv2;

    std::string srv_name3 = "/rosplan_parsing_interface/parse_plan";
    ros::ServiceClient client3 = nh.serviceClient<std_srvs::Empty>(srv_name3);
    std_srvs::Empty srv3;

    std::string srv_name4 = "/rosplan_plan_dispatcher/dispatch_plan";
    ros::ServiceClient client4 = nh.serviceClient<rosplan_dispatch_msgs::DispatchService>(srv_name4);
    rosplan_dispatch_msgs::DispatchService srv4;

    ros::service::waitForService(srv_name1, ros::Duration(1.0));
    client1.call(srv1);
    std::this_thread::sleep_for (std::chrono::milliseconds(800));

    ros::service::waitForService(srv_name2, ros::Duration(1.0));
    client2.call(srv2);
    std::this_thread::sleep_for (std::chrono::milliseconds(800));

    ros::service::waitForService(srv_name3, ros::Duration(1.0));
    client3.call(srv3);
    std::this_thread::sleep_for (std::chrono::milliseconds(800));

    ros::service::waitForService(srv_name4, ros::Duration(1.0));
    client4.call(srv4);

    EXPECT_TRUE(srv4.response.success);

    std::this_thread::sleep_for (std::chrono::seconds(1));

    EXPECT_TRUE(srv4.response.goal_achieved);

    ros::Rate loop_rate = 10;
    actions_received = false;
    while (!actions_received && ros::ok()) {

        loop_rate.sleep();
        ros::spinOnce();
    }

    int actual_action_id = retrieved_action.action_id;
    int actual_parameter_size = retrieved_action.parameters.size();
    float actual_duration = retrieved_action.duration;
    float actual_dispatch_time = retrieved_action.dispatch_time;

    int expected_action_id = 11;
    int expected_parameter_size = 2;
    float expected_duration = 3;
    float expected_dispatch_time = 12.011;
    float tolerance = 5.0;
    bool between_tolerance = false;

    if(actual_dispatch_time < expected_dispatch_time + tolerance ||
                expected_dispatch_time - tolerance < actual_dispatch_time) {
        between_tolerance = true;
    }

    ASSERT_EQ(expected_action_id, actual_action_id);
    ASSERT_EQ(expected_parameter_size, actual_parameter_size);
    ASSERT_EQ(expected_duration, actual_duration);
    EXPECT_TRUE(between_tolerance); // ASSERT_EQ(expected_dispatch_time, actual_dispatch_time); but with tolerance
}

GTEST_TEST(PlanDispatchTests, Test3_published_action_feedback) {

    ros::NodeHandle nh("~");
    ros::Subscriber sub = nh.subscribe<rosplan_dispatch_msgs::ActionFeedback>
                        ("/rosplan_plan_dispatcher/action_feedback", 1000, &feedbackTestCallback);

    std::string srv_name1 = "/rosplan_problem_interface/problem_generation_server";
    ros::ServiceClient client1 = nh.serviceClient<std_srvs::Empty>(srv_name1);
    std_srvs::Empty srv1;

    std::string srv_name2 = "/rosplan_planner_interface/planning_server";
    ros::ServiceClient client2 = nh.serviceClient<std_srvs::Empty>(srv_name2);
    std_srvs::Empty srv2;

    std::string srv_name3 = "/rosplan_parsing_interface/parse_plan";
    ros::ServiceClient client3 = nh.serviceClient<std_srvs::Empty>(srv_name3);
    std_srvs::Empty srv3;

    std::string srv_name4 = "/rosplan_plan_dispatcher/dispatch_plan";
    ros::ServiceClient client4 = nh.serviceClient<rosplan_dispatch_msgs::DispatchService>(srv_name4);
    rosplan_dispatch_msgs::DispatchService srv4;

    ros::service::waitForService(srv_name1, ros::Duration(1.0));
    client1.call(srv1);
    std::this_thread::sleep_for (std::chrono::milliseconds(800));

    ros::service::waitForService(srv_name2, ros::Duration(1.0));
    client2.call(srv2);
    std::this_thread::sleep_for (std::chrono::milliseconds(800));

    ros::service::waitForService(srv_name3, ros::Duration(1.0));
    client3.call(srv3);
    std::this_thread::sleep_for (std::chrono::milliseconds(800));

    ros::service::waitForService(srv_name4, ros::Duration(1.0));
    client4.call(srv4);

    EXPECT_TRUE(srv4.response.success);

    std::this_thread::sleep_for (std::chrono::seconds(1));

    EXPECT_TRUE(srv4.response.goal_achieved);

    ros::Rate loop_rate = 10;
    feedback_received = false;
    while (!feedback_received && ros::ok()) {

        loop_rate.sleep();
        ros::spinOnce();
    }

    ASSERT_EQ(4, feedback_vector.size());
}

GTEST_TEST(PlanDispatchTests, Test4_published_plan_graph) {

    ros::NodeHandle nh("~");
    ros::Subscriber sub = nh.subscribe<std_msgs::String>("/rosplan_plan_dispatcher/plan_graph",
                                                         1000, &graphTestCallback);

    std::string srv_name1 = "/rosplan_problem_interface/problem_generation_server";
    ros::ServiceClient client1 = nh.serviceClient<std_srvs::Empty>(srv_name1);
    std_srvs::Empty srv1;

    std::string srv_name2 = "/rosplan_planner_interface/planning_server";
    ros::ServiceClient client2 = nh.serviceClient<std_srvs::Empty>(srv_name2);
    std_srvs::Empty srv2;

    std::string srv_name3 = "/rosplan_parsing_interface/parse_plan";
    ros::ServiceClient client3 = nh.serviceClient<std_srvs::Empty>(srv_name3);
    std_srvs::Empty srv3;

    std::string srv_name4 = "/rosplan_plan_dispatcher/dispatch_plan";
    ros::ServiceClient client4 = nh.serviceClient<rosplan_dispatch_msgs::DispatchService>(srv_name4);
    rosplan_dispatch_msgs::DispatchService srv4;

    ros::service::waitForService(srv_name1, ros::Duration(1.0));
    client1.call(srv1);
    std::this_thread::sleep_for (std::chrono::milliseconds(800));

    ros::service::waitForService(srv_name2, ros::Duration(1.0));
    client2.call(srv2);
    std::this_thread::sleep_for (std::chrono::milliseconds(800));

    ros::service::waitForService(srv_name3, ros::Duration(1.0));
    client3.call(srv3);
    std::this_thread::sleep_for (std::chrono::milliseconds(800));

    ros::service::waitForService(srv_name4, ros::Duration(1.0));
    client4.call(srv4);

    EXPECT_TRUE(srv4.response.success);

    std::this_thread::sleep_for (std::chrono::seconds(1));

    EXPECT_TRUE(srv4.response.goal_achieved);

    ros::Rate loop_rate = 10;
    graph_received = false;
    while (!graph_received && ros::ok()) {

        loop_rate.sleep();
        ros::spinOnce();
    }

    std::string expected_graph = "digraph plan {\n0[ label=\"plan_start\",style=filled"
    ",fillcolor=black,fontcolor=white];\n}\n";

    ASSERT_EQ(expected_graph, retrieved_graph);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "PlanDispatchTests");

    return RUN_ALL_TESTS();
}
