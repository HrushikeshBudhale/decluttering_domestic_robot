/**
* MIT License
*
* Copyright(c) 2021 Abhijit Mahalle
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files(the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
* @file detect_object.hpp
* @author Abhijit Mahalle
* @brief Library file for the DetectObject class
* @version 0.1
* @date 2021-12-04
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


class DetectObject {
 public:
   /**
    * @brief Constructor for the DetectObject class
    * 
    * @param node_handle 
    */
    explicit DetectObject(ros::NodeHandle*);

    /**
     * @brief Method to detect pre-defined object
     * 
     * @return true if the object is detected
     * @return false  if the object is not detected 
     */
    bool detect_object();

    /**
     * @brief Flag to check if the object is detected
     */
    bool is_object_detected;

    /**
   * @brief callback function for receiving image published by Tiago
   * 
   * @param msg 
   */
    void image_cb(const sensor_msgs::ImageConstPtr&);

 private:
    ros::NodeHandle* nh_;
    cv::Mat img_bgr_, frame_hsv_, frame_thresh_;
    image_transport::ImageTransport image_transport_;
    image_transport::Subscriber image_sub_;
    std::vector<std::vector<cv::Point>> contours_;
};
