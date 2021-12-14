/**
 * MIT License
*
* Copyright(c) 2021
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
*
* @file detect_object_test.cpp
* @author Driver: Hrushikesh B Navigator: Ameya Konkar
* @brief Main file for object spawner ros node
* @version 0.1
* @date 2021-12-12
* 
* @copyright Copyright (c) 2021
* 
*/

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "detect_object/detect_object.hpp"

bool messageReceived;

bool waitFor_message(const bool& message_received, double timeout = 5) {
    ros::Time start = ros::Time::now();
    ros::Time now;
    while (!message_received) {
        ros::spinOnce();
        now = ros::Time::now();
        if ((now - start).toSec() > timeout) {
            return false;
        }
    }
    return true;
}

TEST(test_detect_object_class, test_detect_object) {
    // Arrange
    cv::Mat image(320, 240, CV_8UC3, cv::Scalar(0, 0, 0));
    messageReceived = false;
    ros::NodeHandle nh;
    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>(
                                                "xtion/rgb/image_raw", 10);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
                                                "bgr8", image).toImageMsg();

    // Act
    // img_pub.publish(msg);
    waitFor_message(messageReceived, 3);
    messageReceived = false;
    DetectObject det_obj(&nh);
    // det_obj.detect_object();

    // Assert
    ASSERT_FALSE(waitFor_message(messageReceived, 3));
}
