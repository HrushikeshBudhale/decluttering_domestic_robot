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

DetectObject::DetectObject(ros::NodeHandle* node_handle):
                                        image_transport_(*node_handle) {
    // set predefined object color
    image_sub_ = image_transport_.subscribe("xtion/rgb/image_raw", 1,
                                &DetectObject::image_cb, this,
                                image_transport::TransportHints("compressed"));
    is_object_detected = false;
    cv::namedWindow("1", 0);
    cv::namedWindow("2", 0);
    ROS_INFO_STREAM("[DetectObject] DetectObject object initialized");
}


bool DetectObject::detectColor() {
    // opencv function to detect color of object
    return true;
}


geometry_msgs::Pose DetectObject::getPosition() {
    // return the position of object in camera frame
    return geometry_msgs::Pose();
}

