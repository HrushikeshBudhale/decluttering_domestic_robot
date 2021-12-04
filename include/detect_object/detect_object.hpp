/**
 * @file detect_object.hpp
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
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>



class DetectObject {
 public:
    DetectObject();
    bool detectColor();
    geometry_msgs::Pose getPosition();

 private:
    int objectColor;
    geometry_msgs::Pose objectPosition;
};
