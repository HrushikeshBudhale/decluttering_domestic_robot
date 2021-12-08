/**
 * @file dcrobot.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <control_msgs/PointHeadAction.h>
#include <tf2_ros/transform_listener.h>
// #include <geometry_msgs/Pose.h>

// C++ standard headers
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

#include "navigation/navigation.hpp"
#include "grasp_object/grasp_object.hpp"
#include "detect_object/detect_object.hpp"
#include "object_spawner/object_spawner.hpp"


// Action interface type for moving TIAGo's head, provided as a typedef
// for convenience
typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction>
                                                            PointHeadClient;
typedef boost::shared_ptr<PointHeadClient> PointHeadClientPtr;

class DCRobot {
 public:
    DCRobot();
    void synchronizeTask();

 private:
    Navigation navigator();
    GraspObject graspObj();
    DetectObject detecteObj();
};
