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
* @file object_spawner_test.cpp
* @author Driver: Hrushikesh B Navigator: Ameya Konkar
* @brief Test file for ObjectSpawner class
* @version 0.1
* @date 2021-12-09
* 
* @copyright Copyright (c) 2021
* 
*/


#include <gtest/gtest.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/ModelState.h>

#include "object_spawner/object_spawner.hpp"

// Global variables
bool msg_received = false;
gazebo_msgs::ModelState pose_msg;

bool waitForMessage(const bool& msg_received, double timeout = 5) {
    ros::Time start = ros::Time::now();
    ros::Time now;
    while (!msg_received) {
        ros::spinOnce();
        now = ros::Time::now();
        if ((now - start).toSec() > timeout) {
            return false;
        }
    }
    return true;
}

void pose_cb(const gazebo_msgs::ModelStateConstPtr& msg) {
    msg_received = true;
    pose_msg.reference_frame = msg->reference_frame;
    pose_msg.pose = msg->pose;
}


bool spawn_cb(gazebo_msgs::SpawnModel::Request
                                    &req,  // NOLINT(runtime/references)
            gazebo_msgs::SpawnModel::Response
                                    &res ) {  // NOLINT(runtime/references)
    res.success = true;
    return true;
}


TEST(test_object_spawner_class, test_constructor) {
    // Arrange
    msg_received = false;
    bool expected_value = false;
    ros::NodeHandle nh;

    // Act
    ObjectSpawner spawner(&nh);

    // Assert
    ASSERT_EQ(spawner.is_object_in_hand, expected_value);
}

TEST(test_object_spawner_class, test_set_object_pose) {
    // Arrange
    msg_received = false;
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/gazebo/set_model_state", 10,
                                                            pose_cb);
    geometry_msgs::Pose object_pose;
    object_pose.position.x = 0;
    object_pose.position.y = -5;
    object_pose.position.z = 0.025;
    object_pose.orientation.w = 1;

    gazebo_msgs::ModelState expected_msg;
    expected_msg.pose = object_pose;
    expected_msg.reference_frame = "world";

    // Act
    ros::NodeHandle n_h;
    ObjectSpawner spawner(&n_h);
    spawner.set_object_pose(object_pose);

    // Assert
    ASSERT_TRUE(waitForMessage(msg_received, 3));
    EXPECT_EQ(pose_msg.reference_frame, expected_msg.reference_frame);
    EXPECT_EQ(pose_msg.pose, expected_msg.pose);
}

TEST(test_object_spawner_class, set_object_pose_test) {
    // Arrange
    geometry_msgs::Pose object_pose;
    object_pose.position.x = -5.4;
    object_pose.position.y = -1.2;
    object_pose.position.z = 0.025;
    object_pose.orientation.w = 1;

    gazebo_msgs::ModelState expected_msg;
    expected_msg.pose = object_pose;
    expected_msg.reference_frame = "world";

    // Act
    msg_received = false;
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/gazebo/set_model_state", 10,
                                                            pose_cb);
    // Assert
    ASSERT_TRUE(waitForMessage(msg_received, 3));
    // EXPECT_EQ(pose_msg.reference_frame, expected_msg.reference_frame);
    // EXPECT_EQ(pose_msg.pose, expected_msg.pose);
}
