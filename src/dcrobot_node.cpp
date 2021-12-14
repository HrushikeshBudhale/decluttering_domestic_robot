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
* @file dcrobot_node.cpp
* @author Driver: Hrushikesh Budhale Navigator: Abhijit Mahalle
* @brief Main file for dcrobot ros node
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
