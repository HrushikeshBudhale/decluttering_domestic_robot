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

#include <geometry_msgs/Pose.h>

class GraspObject {
 public:
    GraspObject();
    void openGrip();
    void closeGrip();
    void pickObject(geometry_msgs::Pose);
    void placeObject(geometry_msgs::Pose);
 public:
    geometry_msgs::Pose pickupLocation;
    geometry_msgs::Pose placeLocation;
};
