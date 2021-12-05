/**
 * @file object_spawner.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-28
 * 
 * @copyright Copyright (c) 2021
 * 
 */

##pragma once

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
    bool set_object_state_cb(std_srvs::SetBool::Request&,
                            std_srvs::SetBool::Response&);
    void getRange(std::string);
    void set_object_pose(geometry_msgs::Pose);

    std::pair<float, float> getRandomLocation();

 private:
    std::vector<std::pair<float, float>> mapRange;
};

