//
// Created by martin on 11/02/19.
//
#include <iostream>
#include <gtest/gtest.h>
#include <ros/package.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "rosplan_planning_system/PlannerInterface/POPFPlannerInterface.h"
#include "rosplan_planning_system/PlannerInterface/PlannerInterface.h"
#include "rosplan_dispatch_msgs/PlanningService.h"
#include <cstdlib>


    bool plan_received;
    std::string last_plan;

    void utestCallback(const std_msgs::String::ConstPtr& plan) {
        std::cout << "here" << std::endl;
        ROS_INFO("I heard: [%s]", plan->data.c_str());
        last_plan = plan->data;
        plan_received = true;
    }


    GTEST_TEST(PlannerInterfaceTests, Test1_plan_found) {

        ros::NodeHandle n("~");

        ros::ServiceClient client1 = n.serviceClient<rosplan_dispatch_msgs::PlanningService>("/rosplan_planner_interface/planning_server_params");
        rosplan_dispatch_msgs::PlanningService srv;

        std::string rosplan_demos_path = ros::package::getPath("rosplan_demos");
        std::string rosplan_planning_system_path = ros::package::getPath("rosplan_planning_system");

        srv.request.use_problem_topic = false;
        srv.request.data_path = rosplan_demos_path + "/common/";
        srv.request.domain_path = rosplan_demos_path + "/common/utest_domain.pddl";
        srv.request.problem_path = rosplan_demos_path + "/common/utest_problem.pddl";
        srv.request.planner_command = "timeout 10 " + rosplan_planning_system_path + "/common/bin/popf -n DOMAIN PROBLEM";

        ASSERT_EQ(1, client1.call(srv));
        ASSERT_EQ(1, srv.response.plan_found);

    }


    GTEST_TEST(PlannerInterfaceTests, Test2_planner_output_against_known_plan) {
        ros::NodeHandle n("~");

        ros::Subscriber sub = n.subscribe("/rosplan_planner_interface/planner_output", 1, utestCallback);
        ros::Publisher pub = n.advertise<std_msgs::String>("/rosplan_planner_interface/problem_instance", 1000);

        ros::ServiceClient client1 = n.serviceClient<rosplan_dispatch_msgs::PlanningService>("/rosplan_planner_interface/planning_server_params");
        rosplan_dispatch_msgs::PlanningService srv;

        std::string rosplan_demos_path = ros::package::getPath("rosplan_demos");
        std::string rosplan_planning_system_path = ros::package::getPath("rosplan_planning_system");

        srv.request.use_problem_topic = false;
        srv.request.data_path = rosplan_demos_path + "/common/";
        srv.request.domain_path = rosplan_demos_path + "/common/utest_domain.pddl";
        srv.request.problem_path = rosplan_demos_path + "/common/utest_problem.pddl";
        srv.request.planner_command = "timeout 10 " + rosplan_planning_system_path + "/common/bin/popf -n DOMAIN PROBLEM";

        std::ifstream t(rosplan_demos_path + "/common/utest_problem.pddl");
        std::stringstream ss;
        ss << t.rdbuf();

        std_msgs::String msg;
        msg.data = ss.str();
        pub.publish(msg);

        ros::spinOnce();

        // call service to generate plan
        client1.call(srv);

        // wait to receive the planner output
        ros::Rate loop_rate = 10;
        plan_received = false;
        while (!plan_received && ros::ok()) {
            loop_rate.sleep();
            ros::spinOnce();
        }

        std::string known_plan = "0.000: (movetob ball)  [0.001]\n";

        // checking the plan
        ASSERT_EQ(1, srv.response.plan_found);
        ASSERT_EQ(last_plan, known_plan);

    }


    GTEST_TEST(PlannerInterfaceTests, Test3_problem_without_solution) {
        ros::NodeHandle n("~");

        ros::Subscriber sub = n.subscribe("/rosplan_planner_interface/planner_output", 10, utestCallback);
        ros::Publisher pub = n.advertise<std_msgs::String>("/rosplan_planner_interface/problem_instance", 1000);

        ros::ServiceClient client1 = n.serviceClient<rosplan_dispatch_msgs::PlanningService>("/rosplan_planner_interface/planning_server_params");
        rosplan_dispatch_msgs::PlanningService srv;

        std::string rosplan_demos_path = ros::package::getPath("rosplan_demos");
        std::string rosplan_planning_system_path = ros::package::getPath("rosplan_planning_system");

        srv.request.use_problem_topic = false;
        srv.request.data_path = rosplan_demos_path + "/common/";
        srv.request.domain_path = rosplan_demos_path + "/common/utest_domain.pddl";
        srv.request.problem_path = rosplan_demos_path + "/common/utest_problem_no_solution.pddl";
        srv.request.planner_command = "timeout 10 " + rosplan_planning_system_path + "/common/bin/popf -n DOMAIN PROBLEM";

        std::ifstream t(rosplan_demos_path + "/common/utest_problem_no_solution.pddl");
        std::stringstream ss;
        ss << t.rdbuf();

        std_msgs::String msg;
        msg.data = ss.str();
        pub.publish(msg);

        ros::spinOnce();

        // call service to generate plan
        bool call_answer = client1.call(srv);

        ASSERT_EQ(1, call_answer);


        // checking the plan
        ASSERT_EQ(0, srv.response.plan_found);

    }


    GTEST_TEST(PlannerInterfaceTests, Test4_invalid_pddl_syntax) {
        ros::NodeHandle n("~");

        ros::Subscriber sub = n.subscribe("/rosplan_planner_interface/planner_output", 10, utestCallback);
        ros::Publisher pub = n.advertise<std_msgs::String>("/rosplan_planner_interface/problem_instance", 1000);

        ros::ServiceClient client1 = n.serviceClient<rosplan_dispatch_msgs::PlanningService>("/rosplan_planner_interface/planning_server_params");
        rosplan_dispatch_msgs::PlanningService srv;

        std::string rosplan_demos_path = ros::package::getPath("rosplan_demos");
        std::string rosplan_planning_system_path = ros::package::getPath("rosplan_planning_system");

        srv.request.use_problem_topic = false;
        srv.request.data_path = rosplan_demos_path + "/common/";
        srv.request.domain_path = rosplan_demos_path + "/common/utest_domain.pddl";
        srv.request.problem_path = rosplan_demos_path + "/common/utest_problem_invalid_syntax.pddl";
        srv.request.planner_command = "timeout 10 " + rosplan_planning_system_path + "/common/bin/popf -n DOMAIN PROBLEM";

        std::ifstream t(rosplan_demos_path + "/common/utest_problem_invalid_syntax.pddl");
        std::stringstream ss;
        ss << t.rdbuf();

        std_msgs::String msg;
        msg.data = ss.str();
        pub.publish(msg);

        ros::spinOnce();

        // call service to generate plan
        bool call_answer = client1.call(srv);

        ASSERT_EQ(1, call_answer);


        // checking the plan
        ASSERT_EQ(0, srv.response.plan_found);

    }



    int main(int argc, char **argv) {

            ::testing::InitGoogleTest(&argc, argv);
            ros::init(argc, argv, "utest");

            ros::NodeHandle n("~");

            return RUN_ALL_TESTS();
    }