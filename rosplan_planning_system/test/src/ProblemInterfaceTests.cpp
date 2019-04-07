//
// Created by martin on 14/03/19.
//

#include <iostream>
#include <gtest/gtest.h>
#include <ros/package.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "rosplan_planning_system/ProblemGeneration/PDDLProblemGenerator.h"
#include "rosplan_planning_system/ProblemGeneration/ProblemGenerator.h"
#include "rosplan_dispatch_msgs/ProblemService.h"
#include <cstdlib>

bool problem_received;
std::string last_problem;

void testCallback(const std_msgs::String::ConstPtr &problem) {
    std::cout << "here-problem" << std::endl;
    ROS_INFO("I heard: [%s]", problem->data.c_str());
    last_problem = problem->data;
    problem_received = true;
}

GTEST_TEST(ProblemInterfaceTests, Test1_problem_generated) {

        ros::NodeHandle n("~");

        ros::ServiceClient client1 = n.serviceClient<rosplan_dispatch_msgs::ProblemService>("/rosplan_problem_interface/problem_generation_server_params");
        rosplan_dispatch_msgs::ProblemService srv;

        std::string rosplan_planning_system_path = ros::package::getPath("rosplan_planning_system");

        srv.request.problem_path = rosplan_planning_system_path + "/test/pddl/generated_test_problem.pddl";
        srv.request.problem_string_response = true;

        ASSERT_EQ(1, client1.call(srv));
        ASSERT_EQ(1, srv.response.problem_generated);

}

GTEST_TEST(ProblemInterfaceTests, Test2_problem_string_against_known_problem) {

        ros::NodeHandle n("~");

        ros::ServiceClient client1 = n.serviceClient<rosplan_dispatch_msgs::ProblemService>("/rosplan_problem_interface/problem_generation_server_params");
        rosplan_dispatch_msgs::ProblemService srv;

        std::string rosplan_planning_system_path = ros::package::getPath("rosplan_planning_system");

        srv.request.problem_path = rosplan_planning_system_path + "/test/pddl/generated_test_problem.pddl";
        srv.request.problem_string_response = true;

        client1.call(srv);

        std::string known_problem = "(define (problem task)\n(:domain driverlog-simple)\n(:objects\n    home amazon london myhouse - place\n    driver - driver\n    truck - truck\n    mydvd - item\n)\n(:init\n    (at driver home)\n    (at truck amazon)\n    (at mydvd amazon)\n\n\n    (link amazon london)\n    (link london amazon)\n    (link london myhouse)\n    (link myhouse london)\n\n    (path home amazon)\n    (path amazon home)\n\n)\n(:goal (and\n    (at mydvd myhouse)\n))\n)\n";

        ASSERT_EQ(1, srv.response.problem_generated);

        ASSERT_EQ(known_problem, srv.response.problem_string);

}


GTEST_TEST(ProblemInterfaceTests, Test3_problem_instance_against_known_problem) {

        ros::NodeHandle n("~");

        ros::Subscriber sub = n.subscribe("/rosplan_problem_interface/problem_instance", 1000, &testCallback);

        ros::ServiceClient client1 = n.serviceClient<rosplan_dispatch_msgs::ProblemService>("/rosplan_problem_interface/problem_generation_server_params");
        rosplan_dispatch_msgs::ProblemService srv;

        std::string rosplan_planning_system_path = ros::package::getPath("rosplan_planning_system");

        srv.request.problem_path = rosplan_planning_system_path + "/test/pddl/generated_test_problem.pddl";
        srv.request.problem_string_response = true;

        client1.call(srv);

        ros::Rate loop_rate = 10;
        problem_received = false;
        while (!problem_received && ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();
        }

        std::string known_problem = "(define (problem task)\n(:domain driverlog-simple)\n(:objects\n    home amazon london myhouse - place\n    driver - driver\n    truck - truck\n    mydvd - item\n)\n(:init\n    (at driver home)\n    (at truck amazon)\n    (at mydvd amazon)\n\n\n    (link amazon london)\n    (link london amazon)\n    (link london myhouse)\n    (link myhouse london)\n\n    (path home amazon)\n    (path amazon home)\n\n)\n(:goal (and\n    (at mydvd myhouse)\n))\n)\n";

        ASSERT_EQ(1, srv.response.problem_generated);
        ASSERT_EQ(last_problem, known_problem);

}


int main(int argc, char **argv) {

    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "ProblemInterfaceTests");


    return RUN_ALL_TESTS();
}
