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

        ros::ServiceClient client1 = n.serviceClient<rosplan_dispatch_msgs::PlanningService>("planning_server");
        rosplan_dispatch_msgs::PlanningService srv;

        EXPECT_TRUE(client1.call(srv));
    }


int main(int argc, char **argv) {

    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "utest");

    return RUN_ALL_TESTS();


   // ros::ServiceClient client2 = n.serviceClient<rosplan_dispatch_msgs::PlanningService>("planning_server_params");

   // rosplan_dispatch_msgs::PlanningService srv;

   // client1.call(srv);
   // client2.call(srv);


   // client.call( &KCL_rosplan::PlannerInterface::runPlanningServerDefault);

    // srv.runPlanningServerDefault(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);


   // client.call()
    //bool a = KCL_rosplan::PlannerInterface::testM();

    //ros::Publisher problem_instance = n.advertise<std_msgs::String>("problem_instance", 1000);

    //ros::Subscriber planner_output = n.subscribe("planner_output", 1000, &KCL_rosplan::PlannerInterface::problemCallback,
                                              //   dynamic_cast<KCL_rosplan::PlannerInterface*>(&hej));

    /*

    // call the two services of the planner interface node
    rosservice call /rosplan_planner_interface/planning_server
    rosservice call /rosplan_planner_interface/planning_server_params

    // subscribe to published topic
    subscribe to /rosplan_planner_interface/planner_output

    //publish on the problem topic
    rostopic pub /rosplan_planner_interface/problem_instance std_msgs/String "data: ''"

     */


}
