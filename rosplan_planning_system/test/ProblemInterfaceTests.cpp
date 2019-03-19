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

        std::string rosplan_demos_path = ros::package::getPath("rosplan_demos");

        srv.request.problem_path = rosplan_demos_path + "/common/generated_test_problem.pddl";
        srv.request.problem_string_response = true;

        bool callResponse = client1.call(srv);

        ASSERT_EQ(1, callResponse);
        //std::cout << srv.response.problem_string << std::endl;
        ASSERT_EQ(1, srv.response.problem_generated);
        //false, check generated problem file
}

GTEST_TEST(ProblemInterfaceTests, Test2_problem_string_against_known_problem) {

        ros::NodeHandle n("~");

        ros::Subscriber sub = n.subscribe("/rosplan_planner_interface/problem_instance", 1, testCallback);

        ros::ServiceClient client1 = n.serviceClient<rosplan_dispatch_msgs::ProblemService>("/rosplan_problem_interface/problem_generation_server_params");
        rosplan_dispatch_msgs::ProblemService srv;

        std::string rosplan_demos_path = ros::package::getPath("rosplan_demos");

        srv.request.problem_path = rosplan_demos_path + "/common/generated_test_problem.pddl";
        srv.request.problem_string_response = true;

        client1.call(srv);

        std::string known_problem = "todo: KNOWN PROBLEM";

        ASSERT_EQ(1, srv.response.problem_generated);
        ASSERT_EQ(known_problem, srv.response.problem_string);

}
/*
GTEST_TEST(ProblemInterfaceTests, Test3_problem_instance_against_known_problem) {

        ros::NodeHandle n("~");

        ros::Subscriber sub = n.subscribe("/rosplan_planner_interface/problem_instance", 1, testCallback);

        ros::ServiceClient client1 = n.serviceClient<rosplan_dispatch_msgs::ProblemService>("/rosplan_problem_interface/problem_generation_server_params");
        rosplan_dispatch_msgs::ProblemService srv;

        std::string rosplan_demos_path = ros::package::getPath("rosplan_demos");

        srv.request.problem_path = rosplan_demos_path + "/common/generated_test_problem.pddl";
        srv.request.problem_string_response = true;

        client1.call(srv);

        ros::Rate loop_rate = 10;
        problem_received = false;
        while (!problem_received && ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();
        }

        std::string known_problem = "todo: KNOWN PROBLEM";

        ASSERT_EQ(1, srv.response.problem_generated);
        ASSERT_EQ(last_problem, known_problem);

}
*/


// TIL


int main(int argc, char **argv) {

    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "ProblemInterfaceTests");


    return RUN_ALL_TESTS();
}