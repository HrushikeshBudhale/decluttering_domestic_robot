/**
 * @file dcrobot.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <dcrobot/dcrobot.hpp>

DCRobot::DCRobot(ros::NodeHandle* node_handle):
                    detectObj(node_handle),
                    graspObj(node_handle),
                    navigator(node_handle),
                    tf_listener_(this->tfBuffer_) {
    nh_ = node_handle;
    state_ = robotState::STARTING;
    ROS_INFO_STREAM("[DCRobot] DCRobot object initialized");
}
