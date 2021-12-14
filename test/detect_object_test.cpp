// /**
//  * @file detect_object_test.cpp
//  * @author your name (you@domain.com)
//  * @brief 
//  * @version 0.1
//  * @date 2021-12-12
//  * 
//  * @copyright Copyright (c) 2021
//  * 
//  */


// #include <gtest/gtest.h>
// #include <ros/ros.h>

// #include <sensor_msgs/Image.h>
// #include <std_msgs/Header.h>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/objdetect/objdetect.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>

// // #include "detect_object/detect_object.hpp"
// // #include "grasp_object/grasp_object.hpp"

// bool messageReceived;

// bool waitFor_message(const bool& message_received, double timeout = 5) {
//     ros::Time start = ros::Time::now();
//     ros::Time now;
//     while (!message_received) {
//         ros::spinOnce();
//         now = ros::Time::now();
//         if ((now - start).toSec() > timeout) {
//             return false;
//         }
//     }
//     return true;
// }

// TEST(test_detect_object_class, test_detect_object) {
//     cv::Mat image(320, 240, CV_8UC3, cv::Scalar(0, 0, 0));
//     messageReceived = false;
//     ros::NodeHandle nh;
//     // ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>(
//     //                                                 "xtion/rgb/image_raw", 10);
//     // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
//     //                                                 "bgr8", image).toImageMsg();
//     // img_pub.publish(msg);
//     // waitFor_message(messageReceived, 3);
//     // messageReceived = false;
//     // DetectObject det_obj(&nh);
//     // det_obj.detect_object();
//     GraspObject go(&nh);
//     go.move_to_pick_pose();

//     // Assert
//     ASSERT_TRUE(waitFor_message(messageReceived, 3));

// }