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
    explicit DCRobot(ros::NodeHandle*);
    void handle_states();
    bool is_obj_within_reach();
    geometry_msgs::Pose get_object_pose(std::string wrt = "map");
    void pick_up_object();
    void place_object();
    enum robotState {
        STARTING,
        IDLE,
        MOVING_TO_CHECKPOINT,
        TURNING_AROUND,
        OBJECT_FOUND,
        MOVING_TOWARDS_OBJECT,
        PICKING_OBJECT,
        MOVING_TO_BIN_LOCATION,
        PLACING_OBJECT,
        STOP
    };
    Navigation navigator;
    GraspObject graspObj;
    DetectObject detectObj;
 private:
    void set_head_down();
    void create_point_head_client(PointHeadClientPtr&);
    ros::NodeHandle* nh_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tf_listener_;
    PointHeadClientPtr point_head_client_;
    robotState state_;
    robotState pre_state_;
};
