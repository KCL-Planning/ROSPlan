//
// Created by martin on 11/02/19.
//
#include <iostream>
#include <gtest/gtest.h>

#include "rosplan_planning_system/PlannerInterface/POPFPlannerInterface.h"
#include "rosplan_planning_system/PlannerInterface/PlannerInterface.h"
#include "rosplan_dispatch_msgs/PlanningService.h"
#include <cstdlib>




    GTEST_TEST(ROSTest, TestROSNode_1) {
        // Verifies that ros core is running.
        bool ros_master_exists = ros::master::check();
        EXPECT_TRUE(ros_master_exists);

        if (ros_master_exists) {
            // Tests the ability to instantiate a ROS node.
            ros::NodeHandle nh;
            EXPECT_TRUE(nh.ok());
        }
    }

    GTEST_TEST(ROSTest, TestROSNode_2) {
        ros::NodeHandle n("~");

        ros::ServiceClient client1 = n.serviceClient<rosplan_dispatch_msgs::PlanningService>("/rosplan_planner_interface/planning_server_params");
        rosplan_dispatch_msgs::PlanningService srv;

        srv.request.use_problem_topic = false;
        srv.request.data_path = "rosplan_demos/common/";
        srv.request.domain_path = "rosplan_demos/common/utest_domain.pddl";
        srv.request.problem_path = "rosplan_demos/common/utest_problem.pddl";
        srv.request.planner_command = "timeout 60 rosplan_planning_system/common/bin/popf -n DOMAIN PROBLEM";

        EXPECT_TRUE(client1.call(srv));
        ASSERT_EQ(1, srv.response.plan_found);
    }

    GTEST_TEST(ROSTest, TestROSNode_3) {
    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<rosplan_dispatch_msgs::PlanningService>("/rosplan_planner_interface/planning_server_params");
    rosplan_dispatch_msgs::PlanningService srv;

    srv.request.use_problem_topic = false;
    srv.request.data_path = "rosplan_demos/common/";
    srv.request.domain_path = "rosplan_demos/common/d.pddl";
    srv.request.problem_path = "rosplan_demos/common/p.pddl";
    srv.request.planner_command = "";

    client1.call(srv);
    ASSERT_EQ(0, srv.response.plan_found);
    }

    GTEST_TEST(ROSTest, TestROSNode_4) {
    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<rosplan_dispatch_msgs::PlanningService>("/rosplan_planner_interface/planning_server_params");
    rosplan_dispatch_msgs::PlanningService srv;

    srv.request.use_problem_topic = false;
    srv.request.data_path = "rosplan_demos/common/";
    srv.request.domain_path = "rosplan_demos/common/elevators-domain.pddl";
    srv.request.problem_path = "rosplan_demos/common/elevators-problem.pddl";
    srv.request.planner_command = "timeout 60 rosplan_planning_system/common/bin/popf -n DOMAIN PROBLEM";

    client1.call(srv);
    ASSERT_EQ(0, srv.response.plan_found);
    }


int main(int argc, char **argv) {

    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "utest");

    ros::NodeHandle n("~");
    // ros::Subscriber sub = n.subscribe("/rosplan_planner_interface/planner_output", 1, &KCL_rosplan::PlannerInterface::problemCallback, <std_msgs::String*>);
    ros::Publisher pub = n.advertise<std_msgs::String>("/rosplan_planner_interface/problem_instance", 1, true);

    return RUN_ALL_TESTS();

}
