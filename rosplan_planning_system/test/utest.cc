//
// Created by martin on 11/02/19.
//

#include <gtest/gtest.h>

#include "rosplan_planning_system/ProblemGeneration/ProblemInterface.h"



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
    // This test will fail
    bool false_value = false;
    EXPECT_TRUE(false_value);
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "utest");
    return RUN_ALL_TESTS();
}