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




int main(int argc, char **argv) {

    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "ParsingInterfaceTests");

    ros::NodeHandle n("~");

    return RUN_ALL_TESTS();
}
