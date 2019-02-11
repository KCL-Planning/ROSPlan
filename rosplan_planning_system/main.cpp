//
// Created by martin on 11/02/19.
//
#include <gtest/gtest.h>
#include <gmock/gmock.h>

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    RUN_ALL_TESTS();
    return 0;
}
