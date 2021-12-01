/**
 * @file navigation.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */


#include <navigation/navigation.hpp>

Navigation::Navigation() {
    ROS_INFO_STREAM("Navigation object create");
    // set subscriber, pulisher and services
}

geometry_msgs::Point Navigation::getNextCheckpoint() {
    // functions responsible for getting position of next checkpoint
    return geometry_msgs::Point();
}

geometry_msgs::Point Navigation::getBinLocation() {
    // function responsible for getting bin location after picking up the object
    return geometry_msgs::Point();
}

void Navigation::moveToNextCheckpoint() {
    // function responsible for making robot move to next unattended checkpoint
}

void Navigation::moveNearObject(geometry_msgs::Pose) {
    // function responsible for moving the robot near object after detecting it
}

void Navigation::turnAround() {
    // function for making robot turn around to detect the object
}
