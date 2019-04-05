//
// Created by martin on 05/04/19.
//

#include <iostream>
#include <gtest/gtest.h>
#include <ros/package.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "rosplan_dispatch_msgs/ParsingService.h"
#include "rosplan_dispatch_msgs/CompletePlan.h"
#include "rosplan_dispatch_msgs/EsterelPlan.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/DispatchService.h"


// Documentation incorrectly states tgat dispatch_plan service is of type std_srvs/Empty

GTEST_TEST(PlanDispatchTests, Test1_plan_dispatch) {

    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<rosplan_dispatch_msgs::ParsingService>("/rosplan_parsing_interface_esterel/parse_plan_from_file");
    rosplan_dispatch_msgs::ParsingService srv;

    std::string rosplan_planning_system_path = ros::package::getPath("rosplan_planning_system");

    srv.request.plan_path = rosplan_planning_system_path + "/test/pddl/test_plan.pddl";

    client1.call(srv);

    ASSERT_EQ(1, srv.response.plan_parsed);

    /*

    ros::ServiceClient client2 = n.serviceClient<rosplan_dispatch_msgs::DispatchService>("/rosplan_plan_dispatcher_esterel/dispatch_plan");
    rosplan_dispatch_msgs::DispatchService dispatchSrv;

    client2.call(dispatchSrv);

    ASSERT_EQ(true, dispatchSrv.response.success);

    ASSERT_EQ(true, dispatchSrv.response.goal_achieved);

     */
}


int main(int argc, char **argv) {

    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "PlanDispatchTests");

    ros::NodeHandle n("~");

    return RUN_ALL_TESTS();
}
