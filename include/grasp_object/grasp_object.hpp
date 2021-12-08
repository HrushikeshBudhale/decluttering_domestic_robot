/**
 * @file grasp_object.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-29
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
    explicit GraspObject(ros::NodeHandle*);
    void move_to_object_pose(geometry_msgs::Pose);
    void move_to_pick_pose();
    void move_to_place_pose();
    
 public:
    ros::NodeHandle* nh_;
    ros::ServiceClient set_object_state_client_;
    void move_arm_to_pose(geometry_msgs::Pose);
};
