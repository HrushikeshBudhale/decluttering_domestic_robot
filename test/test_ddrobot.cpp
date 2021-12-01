/**
 * @file test_ddrobot.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <gtest/gtest.h>
#include <ros/ros.h>

std::shared_ptr<ros::NodeHandle> nh;

TEST(test_navigation_class, test_one) {
    EXPECT_EQ(1, 1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "TestingNode");
    nh.reset(new ros::NodeHandle);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
