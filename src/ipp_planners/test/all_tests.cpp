#include <gtest/gtest.h>
#include <ros/ros.h>
#include <iostream>
#include "ipp_planners/gtests/test_coverage.h"
#include "ipp_planners/gtests/test_tigris.h"


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}