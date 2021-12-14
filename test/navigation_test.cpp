/**
* MIT License
*
* Copyright(c) 2021
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files(the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* @file navigation_test.cpp
* @author Driver: Hrushikesh B Navigator: Ameya Konkar
* @brief Test file for navigation class
* @version 0.1
* @date 2021-12-09
* 
* @copyright Copyright (c) 2021
* 
*/


#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

#include "navigation/navigation.hpp"

// Global variables
bool message_received;
geometry_msgs::PoseStamped pose_message;
geometry_msgs::Twist velocity_message;

bool wait_for_message(const bool& message_received, double timeout = 5) {
    ros::Time start = ros::Time::now();
    ros::Time now;
    while (!message_received) {
        ros::spinOnce();
        now = ros::Time::now();
        if ((now - start).toSec() > timeout) {
            return false;
        }
    }
    return true;
}

void keep_publishing(const bool& message_received, ros::Publisher* pose_pub,
            geometry_msgs::PoseWithCovarianceStamped msg, double timeout = 5) {
    ros::Time start = ros::Time::now();
    ros::Time now;
    while (!message_received) {
        ros::spinOnce();
        pose_pub->publish(msg);
        now = ros::Time::now();
        if ((now - start).toSec() > timeout) {
            return;
        }
    }
}

void pose_callback(const geometry_msgs::PoseStampedConstPtr& msg) {
    message_received = true;
    pose_message.header = msg->header;
    pose_message.pose = msg->pose;
}

void velocity_callback(const geometry_msgs::TwistConstPtr& msg) {
    message_received = true;
    velocity_message.angular = msg->angular;
}


TEST(test_navigation_class, test_bin_goal_setting) {
    // Arrange
    message_received = false;
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/move_base_simple/goal", 10,
                                                            pose_callback);
    geometry_msgs::PoseStamped expected_msg;
    expected_msg.pose.orientation.w = 1;
    expected_msg.header.frame_id = "map";

    // Act
    Navigation navigator(&nh);
    navigator.set_bin_location_as_goal();

    // Assert
    ASSERT_TRUE(wait_for_message(message_received, 3));
    EXPECT_EQ(pose_message.pose.orientation, expected_msg.pose.orientation);
    EXPECT_EQ(pose_message.pose.position, expected_msg.pose.position);
    EXPECT_EQ(pose_message.header.frame_id, expected_msg.header.frame_id);
}

TEST(test_navigation_class, test_checkpoint_goal_setting) {
    // Arrange
    int number_of_rooms = 5;
    message_received = false;
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/move_base_simple/goal", 10,
                                                            pose_callback);
    geometry_msgs::PoseStamped expected_msg;
    expected_msg.pose.position.z = 0;
    expected_msg.pose.orientation.w = 1;
    expected_msg.header.frame_id = "map";

    // Act
    Navigation navigator(&nh);
    for (int room=0; room < number_of_rooms; room++) {
        navigator.set_next_checkpoint_as_goal();

        // Assert
        ASSERT_TRUE(wait_for_message(message_received, 3));
        EXPECT_EQ(pose_message.pose.orientation, expected_msg.pose.orientation);
        EXPECT_NE(pose_message.pose.position.x, expected_msg.pose.position.x);
        EXPECT_NE(pose_message.pose.position.y, expected_msg.pose.position.y);
        EXPECT_EQ(pose_message.pose.position.z, expected_msg.pose.position.z);
        EXPECT_EQ(pose_message.header.frame_id, expected_msg.header.frame_id);
    }
}


TEST(test_navigation_class, test_object_pose_goal_setting) {
    // Arrange
    message_received = false;
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/move_base_simple/goal", 10,
                                                            pose_callback);
    geometry_msgs::Pose object_pose;
    object_pose.position.x = 5.4;
    object_pose.position.y = 1.2;
    object_pose.position.z = 0.025;
    object_pose.orientation.w = 1;

    geometry_msgs::PoseStamped expected_msg;
    expected_msg.header.frame_id = "map";
    expected_msg.pose = object_pose;

    // Act
    Navigation navigator(&nh);
    navigator.set_object_pose_as_goal(object_pose);

    // Assert
    ASSERT_TRUE(wait_for_message(message_received, 3));
    EXPECT_EQ(pose_message.pose.orientation, expected_msg.pose.orientation);
    EXPECT_EQ(pose_message.pose.position, expected_msg.pose.position);
    EXPECT_EQ(pose_message.header.frame_id, expected_msg.header.frame_id);
}

TEST(test_navigation_class, test_turn_around) {
    // Arrange
    message_received = false;
    ros::NodeHandle nh;
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::
                                                    PoseWithCovarianceStamped>(
                                                            "/robot_pose", 10);
    ros::Subscriber sub = nh.subscribe("/mobile_base_controller/cmd_vel", 10,
                                                            velocity_callback);
    double expected_velocity = 0.6;
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.pose.pose.orientation.w = 1;

    // Act
    Navigation navigator(&nh);
    navigator.turn_state = Navigation::turning::TURNING;
    keep_publishing(message_received, &pose_pub, msg, 3);
    navigator.turn_around();

    // Assert
    ASSERT_TRUE(wait_for_message(message_received, 3));
    EXPECT_EQ(velocity_message.angular.z, 0.6);
}
