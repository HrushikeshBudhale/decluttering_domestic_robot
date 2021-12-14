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
* @file grasp_object.hpp
* @author Driver: Ameya Konkar, Navigator: Hrushikesh Budhale
* @brief Header file for GraspObject class 
* @version 0.1
* @date 2021-12-08
* 
* @copyright Copyright (c) 2021
* 
*/

#pragma once

// ROS headers
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <std_srvs/SetBool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>

class GraspObject {
 public:
    /**
     * @brief Construct a new Grasp Object object
     * 
     */
    explicit GraspObject(ros::NodeHandle*);

     /**
     * @brief move robot to object position 
     * 
     */
    void move_to_object_pose(geometry_msgs::Pose);

    /**
     * @brief Pick the object from the object position
     * 
     */
    void move_to_pick_pose();

    /**
     *  @brief Place the object to the bin position
     * 
     */
    void move_to_place_pose();

 public:
    /**  Node handle  */
    ros::NodeHandle* nh_;
    /** Service client to set object state  */
    ros::ServiceClient set_object_state_client_;

    /**
     *  @brief moves the robot arm to the object location.
     * 
     */
    void move_arm_to_pose(geometry_msgs::Pose);
};
