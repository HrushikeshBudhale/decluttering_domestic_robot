/**
 * @file navigation.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include "ros/ros.h"


class Navigation {
 public:
    Navigation();
    geometry_msgs::Point getNextCheckpoint();
    geometry_msgs::Point getBinLocation();
    void moveToNextCheckpoint();
    void moveNearObject(geometry_msgs::Pose);
    void turnAround();
};
