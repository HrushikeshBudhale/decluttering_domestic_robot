/**
 * @file grasp_object.hpp
 * @author Ameya konkar (ameyakonk)
 * @brief 
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
