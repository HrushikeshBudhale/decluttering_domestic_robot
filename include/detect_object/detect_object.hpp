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

#include <geometry_msgs/Pose.h>


class DetectObject {
 public:
    DetectObject();
    bool detectColor();
    geometry_msgs::Pose getPosition();

 private:
    int objectColor;
    geometry_msgs::Pose objectPosition;
};
