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
        srv.request.planner_command = "timeout 60 " + rosplan_planning_system_path + "/common/bin/popf -n DOMAIN PROBLEM";

        ASSERT_EQ(1, srv.response.plan_found);
    }

    GTEST_TEST(PlannerInterfaceTests, Test2_planner_output_against_known_plan) {
        ros::NodeHandle n("~");

        ros::ServiceClient client1 = n.serviceClient<rosplan_dispatch_msgs::PlanningService>("/rosplan_planner_interface/planning_server_params");
        rosplan_dispatch_msgs::PlanningService srv;

        std::string rosplan_demos_path = ros::package::getPath("rosplan_demos");
        std::string rosplan_planning_system_path = ros::package::getPath("rosplan_planning_system");


        srv.request.use_problem_topic = false;
        srv.request.data_path = rosplan_demos_path + "/common/";
        srv.request.domain_path = rosplan_demos_path + "/common/utest_domain.pddl";
        srv.request.problem_path = rosplan_demos_path + "/common/utest_problem.pddl";
        srv.request.planner_command = "timeout 60 " + rosplan_planning_system_path + "/common/bin/popf -n DOMAIN PROBLEM";

        std::string known_plan = "";

        ASSERT_EQ(known_plan, "s"/* string published on planner output */);
    }

    GTEST_TEST(PlannerInterfaceTests, Test3_problem_topic) {
        ros::NodeHandle n("~");
        std::cout << n.getNamespace() << std::endl;

        ros::ServiceClient client1 = n.serviceClient<rosplan_dispatch_msgs::PlanningService>("/rosplan_planner_interface/planning_server_params");
        rosplan_dispatch_msgs::PlanningService srv;

        std::string rosplan_demos_path = ros::package::getPath("rosplan_demos");
        std::string rosplan_planning_system_path = ros::package::getPath("rosplan_planning_system");

        srv.request.use_problem_topic = false;
        srv.request.data_path = rosplan_demos_path + "/common/";
        srv.request.domain_path = rosplan_demos_path + "/common/utest_domain.pddl";
        srv.request.problem_path = rosplan_demos_path + "/common/utest_problem.pddl";
        srv.request.planner_command = "timeout 60 " + rosplan_planning_system_path + "/common/bin/popf -n DOMAIN PROBLEM";

        ASSERT_EQ(1, srv.response.plan_found);
    }




    bool plan_received = false;



    void utestCallback(const std_msgs::String::ConstPtr& plan){
        std::cout << "here" << std::endl;
        ROS_INFO("I heard: [%s]", plan->data.c_str());
        plan_received = true;
    }


    int main(int argc, char **argv) {

            ::testing::InitGoogleTest(&argc, argv);
            ros::init(argc, argv, "utest");

            ros::NodeHandle n("~");

            ros::Subscriber sub = n.subscribe("/rosplan_planner_interface/planner_output", 1, utestCallback);

            ros::Publisher pub = n.advertise<std_msgs::String>("/rosplan_planner_interface/problem_instance", 1000);

            std_msgs::String msg;

            std::string rosplan_demos_path = ros::package::getPath("rosplan_demos");
            std::string rosplan_planning_system_path = ros::package::getPath("rosplan_planning_system");

            std::ifstream t(rosplan_demos_path + "/common/utest_problem.pddl");
            std::stringstream ss;
            ss << t.rdbuf();

            msg.data = ss.str();
            std::cout << ss.str() << std::endl;
            pub.publish(msg);

            /*
            ros::Rate loop_rate = 10;
            while (!plan_received && ros::ok()) {
                loop_rate.sleep();
                std::cout << "here!" << std::endl;
                ros::spinOnce();
            }
*/

            ros::spinOnce();


            return RUN_ALL_TESTS();

    }