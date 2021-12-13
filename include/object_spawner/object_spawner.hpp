/**
* MIT License
*
* Copyright(c) 2021 Abhijit Mahalle
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
* @file object_spawner.hpp
* @author Abhijit Mahalle
* @brief Header file for the ObjectSpawner class
* @version 0.1
* @date 2021-11-27
* @copyright Copyright (c) 2021
* 
*/

#pragma once

// ROS headers
#include <std_srvs/SetBool.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

// Std C++ headers
#include <stdlib.h>
#include <string>


class ObjectSpawner {
 public:
    explicit ObjectSpawner(ros::NodeHandle*);
    bool spawn_object();
    void set_object_pose(geometry_msgs::Pose);
    bool is_object_in_hand;

 private:
    bool set_object_state_cb(std_srvs::SetBool::Request&,
                            std_srvs::SetBool::Response&);
    void publish_pose(const ros::TimerEvent&);

    unsigned int seed;
    bool is_spawned;
    int map_range[4];

    ros::NodeHandle* nh_;
    ros::ServiceServer update_state_service_;
    ros::ServiceClient spawn_object_client_;
    ros::Timer object_pose_tf_timer_;
    ros::Publisher pose_pub_;

    std::string urdf_string_;
    geometry_msgs::Pose object_pose_;
    std::string object_name;

    tf2_ros::TransformBroadcaster br_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tf_listener_;
};
