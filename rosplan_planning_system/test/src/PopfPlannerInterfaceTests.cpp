/**
 *
 * Copyright [2019] <KCL King's College London>
 *
 * Author: Martin Koling (martinh.koling@kcl.ac.uk)
 * Author: Oscar Lima (oscar.lima@dfki.de)
 *
 * Unit tests for ROSPlan planner interface
 */

#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <gtest/gtest.h>

#include "rosplan_planning_system/PlannerInterface/PlannerInterface.h"
#include "rosplan_dispatch_msgs/PlanningService.h"

bool plan_received;
std::string last_plan;

void testCallback(const std_msgs::String::ConstPtr &plan) {
    ROS_INFO("I heard: [%s]", plan->data.c_str());
    last_plan = plan->data;
    plan_received = true;
}

GTEST_TEST(PlannerInterfaceTests, Test1_plan_found) {

    ros::NodeHandle nh("~");

    std::string srv_name = "/rosplan_planner_interface/planning_server_params";
    ros::ServiceClient client = nh.serviceClient<rosplan_dispatch_msgs::PlanningService>(srv_name);
    rosplan_dispatch_msgs::PlanningService srv;

    std::string rosplan_planning_system_path = ros::package::getPath("rosplan_planning_system");

    srv.request.use_problem_topic = false;
    srv.request.data_path = rosplan_planning_system_path + "/test/pddl/test_domain";
    srv.request.domain_path = rosplan_planning_system_path + "/test/pddl/test_domain/domain.pddl";
    srv.request.problem_path = rosplan_planning_system_path + "/test/pddl/test_domain/test_problem.pddl";
    srv.request.planner_command = "timeout 10 " + rosplan_planning_system_path + "/common/bin/popf -n DOMAIN PROBLEM";

    ros::service::waitForService(srv_name, ros::Duration(3));
    bool is_srv_call_successful = false;
    if(client.call(srv))
        is_srv_call_successful = true;
    
    EXPECT_TRUE(is_srv_call_successful);
    EXPECT_TRUE(srv.response.plan_found);
}

GTEST_TEST(PlannerInterfaceTests, Test2_format_published_on_planner_output) {

    ros::NodeHandle nh("~");

    ros::Subscriber sub = nh.subscribe<std_msgs::String>("/rosplan_planner_interface/planner_output", 1000, &testCallback);
    ros::Publisher pub = nh.advertise<std_msgs::String>("/rosplan_problem_interface/problem_instance", 1000);

    std::string srv_name = "/rosplan_planner_interface/planning_server_params";
    ros::ServiceClient client1 = nh.serviceClient<rosplan_dispatch_msgs::PlanningService>(srv_name);
    rosplan_dispatch_msgs::PlanningService srv;

    std::string rosplan_planning_system_path = ros::package::getPath("rosplan_planning_system");

    srv.request.use_problem_topic = false;
    srv.request.data_path = rosplan_planning_system_path + "/test/pddl/test_domain/";
    srv.request.domain_path = rosplan_planning_system_path + "/test/pddl/test_domain/domain.pddl";
    srv.request.problem_path = rosplan_planning_system_path + "/test/pddl/test_domain/test_problem.pddl";
    srv.request.planner_command = "timeout 10 " + rosplan_planning_system_path + "/common/bin/popf -n DOMAIN PROBLEM";

    std::ifstream t(rosplan_planning_system_path + "/test/pddl/test_domain/test_problem.pddl");
    std::stringstream ss;
    ss << t.rdbuf();

    std_msgs::String msg;
    msg.data = ss.str();
    pub.publish(msg);

    ros::spinOnce();

    ros::Rate loop_rate = 10;
    loop_rate.sleep();

    ros::service::waitForService(srv_name, ros::Duration(3));
    client1.call(srv);

    plan_received = false;
    while (!plan_received && ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    std::string known_plan = "0.000: (movetob ball)  [0.001]\n";

    EXPECT_TRUE(srv.response.plan_found);
    ASSERT_EQ(last_plan, known_plan);
}

GTEST_TEST(PlannerInterfaceTests, Test3_problem_without_solution) {

    ros::NodeHandle nh("~");

    std::string srv_name = "/rosplan_planner_interface/planning_server_params";
    ros::ServiceClient client = nh.serviceClient<rosplan_dispatch_msgs::PlanningService>(srv_name);
    rosplan_dispatch_msgs::PlanningService srv;

    std::string rosplan_planning_system_path = ros::package::getPath("rosplan_planning_system");

    srv.request.use_problem_topic = false;
    srv.request.data_path = rosplan_planning_system_path + "/test/pddl/test_domain/";
    srv.request.domain_path = rosplan_planning_system_path + "/test/pddl/test_domain/domain.pddl";
    srv.request.problem_path = rosplan_planning_system_path + "/test/pddl/test_domain/test_problem_no_solution.pddl";
    srv.request.planner_command = "timeout 10 " + rosplan_planning_system_path + "/common/bin/popf -n DOMAIN PROBLEM";

    ros::service::waitForService(srv_name, ros::Duration(3));
    bool is_srv_call_successful = false;
    if(client.call(srv))
        is_srv_call_successful = true;

    EXPECT_TRUE(is_srv_call_successful);
    EXPECT_FALSE(srv.response.plan_found);
}

GTEST_TEST(PlannerInterfaceTests, Test4_invalid_pddl_syntax) {

    ros::NodeHandle nh("~");

    std::string srv_name = "/rosplan_planner_interface/planning_server_params";
    ros::ServiceClient client1 = nh.serviceClient<rosplan_dispatch_msgs::PlanningService>(srv_name);
    rosplan_dispatch_msgs::PlanningService srv;

    std::string rosplan_planning_system_path = ros::package::getPath("rosplan_planning_system");

    srv.request.use_problem_topic = false;
    srv.request.data_path = rosplan_planning_system_path + "/test/pddl/test_domain/";
    srv.request.domain_path = rosplan_planning_system_path + "/test/pddl/test_domain/domain.pddl";
    srv.request.problem_path = rosplan_planning_system_path + "/test/pddl/test_domain/test_problem_invalid_syntax.pddl";
    srv.request.planner_command = "timeout 10 " + rosplan_planning_system_path + "/common/bin/popf -n DOMAIN PROBLEM";

    ros::service::waitForService(srv_name, ros::Duration(3));
    EXPECT_TRUE(client1.call(srv));
    EXPECT_FALSE(srv.response.plan_found);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "PopfPlannerInterfaceTests");

    return RUN_ALL_TESTS();
}
