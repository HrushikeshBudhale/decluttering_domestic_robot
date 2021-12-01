/**
 * @file dcrobot_node.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <dcrobot/dcrobot.hpp>

int main(int argc, char *argv[]) {
    // Initialize the node
    ros::init(argc, argv, "dcrobot_node");
    ROS_INFO_STREAM("[dcrobot_node] Started dcrobot_node");
    ros::NodeHandle nh;
    DCRobot dcr();  // Create DCRobot object
    ros::spin();
    return 0;
}