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
#include <cstdlib>


GTEST_TEST(ProblemGenerationTests, Test1_) {

    bool sant = true;
    ASSERT_EQ(1, sant);

}

int main(int argc, char **argv) {

    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "ProblemGeneratorTests");


    return RUN_ALL_TESTS();
}