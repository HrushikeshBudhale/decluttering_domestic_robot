/**
 * @file detect_object.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <detect_object/detect_object.hpp>

DetectObject::DetectObject() {
    // set predefined object color
}


bool DetectObject::detectColor() {
    // opencv function to detect color of object
    return true;
}


geometry_msgs::Pose DetectObject::getPosition() {
    // return the position of object in camera frame
    return geometry_msgs::Pose();
}

