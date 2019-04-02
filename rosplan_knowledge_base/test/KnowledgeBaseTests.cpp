//
// Created by martin on 02/04/19.
//

#include <iostream>
#include <gtest/gtest.h>
#include <ros/package.h>
#include "ros/ros.h"
#include "std_msgs/String.h"


GTEST_TEST(KnowledgeBaseTests, Test1_) {

ros::NodeHandle n("~");


ASSERT_EQ(1, 1);


}


int main(int argc, char **argv) {

    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "KnowledgeBaseTests");

    ros::NodeHandle n("~");

    return RUN_ALL_TESTS();
}
