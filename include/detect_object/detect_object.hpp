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

// Std C++ headers
#include <string.h>
#include <vector>

// OpenCV headers
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Std C++ headers
#include <string.h>
#include <vector>

// OpenCV headers
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class DetectObject {
 public:
    explicit DetectObject(ros::NodeHandle*);
    bool detect_object();
    bool is_object_detected;

 private:
    int i;
    ros::NodeHandle* nh_;
    cv::Mat img_bgr_, frame_hsv_, frame_thresh_;
    image_transport::ImageTransport image_transport_;
    image_transport::Subscriber image_sub_;
    std::vector<std::vector<cv::Point>> contours_;
};