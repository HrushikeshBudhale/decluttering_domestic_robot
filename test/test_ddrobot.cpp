
#include <gtest/gtest.h>
#include <ros/ros.h>


TEST(test_navigation_class, test_one) {
    EXPECT_EQ(1,1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "TestingNode");
    nh.reset(new ros::NodeHandle);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
