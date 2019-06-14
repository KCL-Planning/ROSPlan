/**
 *
 * Copyright [2019] <KCL King's College London>
 *
 * Author: Martin Koling (martinh.koling@kcl.ac.uk)
 * Author: Oscar Lima (oscar.lima@dfki.de)
 *
 * Unit tests for ROSPlan problem interface
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <gtest/gtest.h>

#include "rosplan_planning_system/ProblemGeneration/PDDLProblemGenerator.h"
#include "rosplan_planning_system/ProblemGeneration/ProblemGenerator.h"
#include "rosplan_dispatch_msgs/ProblemService.h"

bool problem_received;
std::string last_problem;

void testCallback(const std_msgs::String::ConstPtr &problem) {
    std::cout << "here-problem" << std::endl;
    ROS_INFO("I heard: [%s]", problem->data.c_str());
    last_problem = problem->data;
    problem_received = true;
}

GTEST_TEST(ProblemInterfaceTests, Test1_problem_generated) {

    ros::NodeHandle nh("~");

    std::string srv_name = "/rosplan_problem_interface/problem_generation_server_params";
    ros::ServiceClient client1 = nh.serviceClient<rosplan_dispatch_msgs::ProblemService>(srv_name);
    rosplan_dispatch_msgs::ProblemService srv;

    std::string rosplan_planning_system_path = ros::package::getPath("rosplan_planning_system");

    srv.request.problem_path = rosplan_planning_system_path + "/test/pddl/amazon/generated_test_problem.pddl";
    srv.request.problem_string_response = true;

    ros::service::waitForService(srv_name, ros::Duration(1));
    EXPECT_TRUE(client1.call(srv));
    EXPECT_TRUE(srv.response.problem_generated);
}

GTEST_TEST(ProblemInterfaceTests, Test2_problem_string_against_known_problem) {

    ros::NodeHandle nh("~");

    std::string srv_name = "/rosplan_problem_interface/problem_generation_server_params";
    ros::ServiceClient client = nh.serviceClient<rosplan_dispatch_msgs::ProblemService>(srv_name);
    rosplan_dispatch_msgs::ProblemService srv;

    std::string rosplan_planning_system_path = ros::package::getPath("rosplan_planning_system");

    srv.request.problem_path = rosplan_planning_system_path + "/test/pddl/amazon/generated_test_problem.pddl";
    srv.request.problem_string_response = true;

    ros::service::waitForService(srv_name, ros::Duration(1));
    client.call(srv);

    std::string known_problem = "(define (problem task)\n(:domain driverlog-simple)\n"
    "(:objects\n    home amazon london myhouse - place\n    driver - driver\n    truck - truck"
    "\n    mydvd - item\n)\n(:init\n    (at driver home)\n    (at truck amazon)\n    (at mydvd amazon)"
    "\n\n\n    (link amazon london)\n    (link london amazon)\n    (link london myhouse)\n    (link myhouse london)\n"
    "\n    (path home amazon)\n    (path amazon home)\n\n)\n(:goal (and\n    (at mydvd myhouse)\n))\n)\n";

    EXPECT_TRUE(srv.response.problem_generated);
    ASSERT_EQ(known_problem, srv.response.problem_string);
}

GTEST_TEST(ProblemInterfaceTests, Test3_problem_instance_against_known_problem) {

    ros::NodeHandle nh("~");

    ros::Subscriber sub = nh.subscribe("/rosplan_problem_interface/problem_instance", 1000, &testCallback);

    std::string srv_name = "/rosplan_problem_interface/problem_generation_server_params";
    ros::ServiceClient client1 = nh.serviceClient<rosplan_dispatch_msgs::ProblemService>(srv_name);
    rosplan_dispatch_msgs::ProblemService srv;

    std::string rosplan_planning_system_path = ros::package::getPath("rosplan_planning_system");

    srv.request.problem_path = rosplan_planning_system_path + "/test/pddl/amazon/generated_test_problem.pddl";
    srv.request.problem_string_response = true;

    ros::service::waitForService(srv_name, ros::Duration(1));
    client1.call(srv);

    // Listen to callback at 10 hz
    ros::Rate loop_rate = 10;

    // flag to wait until callback is received
    problem_received = false;
    while (!problem_received && ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    // reset flag
    problem_received = false;

    std::string known_problem = "(define (problem task)\n(:domain driverlog-simple)\n(:objects\n    "
    "home amazon london myhouse - place\n    driver - driver\n    truck - truck\n    mydvd - item\n)\n"
    "(:init\n    (at driver home)\n    (at truck amazon)\n    (at mydvd amazon)\n\n\n    (link amazon "
    "london)\n    (link london amazon)\n    (link london myhouse)\n    (link myhouse london)\n\n    "
    "(path home amazon)\n    (path amazon home)\n\n)\n(:goal (and\n    (at mydvd myhouse)\n))\n)\n";

    EXPECT_TRUE(srv.response.problem_generated);
    ASSERT_EQ(last_problem, known_problem);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "ProblemInterfaceTests");

    return RUN_ALL_TESTS();
}
