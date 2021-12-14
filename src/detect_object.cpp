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
*
* @file detect_object.cpp
* @author Driver: Abhijit Mahalle (abhimah@umd.edu), Navigator: Hrushikesh B
* @brief Library file for the DetectObject class
* @version 0.1
* @date 2021-12-04
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

bool DetectObject::detect_object() {
    // opencv function to detect color of object
    // Convert from BGR to HSV colorspace
    cv::cvtColor(img_bgr_, frame_hsv_, cv::COLOR_BGR2HSV);

    // Detect the object based on HSV Range Values
    cv::inRange(frame_hsv_, cv::Scalar(69, 50, 0),
                            cv::Scalar(120, 255, 255), frame_thresh_);
    // Show the frames
    cv::findContours(frame_thresh_, contours_, CV_RETR_EXTERNAL,
                                        CV_CHAIN_APPROX_SIMPLE);
    if (contours_.size() > 0) {
        cv::Rect objectBoundingRectangle = cv::Rect(0, 0, 0, 0);
        std::vector<std::vector<cv::Point> > largest_contour;
        largest_contour.push_back(contours_.at(contours_.size()-1));
        objectBoundingRectangle = cv::boundingRect(largest_contour.at(0));
        int x = objectBoundingRectangle.x+objectBoundingRectangle.width/2;
        int y = objectBoundingRectangle.y+objectBoundingRectangle.height/2;
        cv::circle(img_bgr_, cv::Point(x, y), 2, cv::Scalar(0, 255, 0), 2);
        if (is_object_detected == false) {
            is_object_detected = true;
            ROS_INFO_STREAM("[ObjectDetector] Object found in image at u: "
                                                    << x << ",\tv: " << y);
        }
    }
    cv::imshow("1", img_bgr_);
    cv::imshow("2", frame_thresh_);
    cv::waitKey(1);
    return true;
}

void DetectObject::image_cb(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cvPtr;
    try {
        cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cvPtr->image.copyTo(img_bgr_);
    this->detect_object();
}

