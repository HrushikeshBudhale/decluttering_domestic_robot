/**
 * @file dcrobot.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <dcrobot/dcrobot.hpp>

DCRobot::DCRobot(ros::NodeHandle* node_handle):
                    detectObj(node_handle),
                    graspObj(node_handle),
                    navigator(node_handle),
                    tf_listener_(this->tfBuffer_) {
    nh_ = node_handle;
    state_ = robotState::STARTING;
    ROS_INFO_STREAM("[DCRobot] DCRobot object initialized");
}

void DCRobot::handle_states() {
    //  Checks if the state of the robot has been changed
    if (pre_state_ != state_) {
        ROS_INFO_STREAM("[DCRobot] in state: " << state_ << "________________");
        pre_state_ = state_;
    }
    
    //  State Switches to the respective state
    switch (state_) {
        case STARTING:
            // Create a point head action client to move the TIAGo's head
            create_point_head_client(point_head_client_);
            set_head_down();
            ROS_INFO_STREAM("[DCRobot] Setting head position");
            state_ = IDLE;
            break;

        case IDLE:
            // Sets checkpoints for the robot
            navigator.set_next_checkpoint_as_goal();
            state_ = MOVING_TO_CHECKPOINT;
            break;

        case MOVING_TO_CHECKPOINT:
            // Moves the robot to the checkpoint
            if (detectObj.is_object_detected) {
                navigator.stop_moving();
                navigator.set_object_pose_as_goal(get_object_pose());
                state_ = MOVING_TOWARDS_OBJECT;
            } else if (navigator.is_goal_reached()) {
                navigator.turn_state = Navigation::turning::TURN_START;
                state_ = TURNING_AROUND;
            }
            break;

        case TURNING_AROUND:
            // Turns the robot at the checkpoint
            if (detectObj.is_object_detected) {
                navigator.set_object_pose_as_goal(get_object_pose());
                state_ = MOVING_TOWARDS_OBJECT;
            } else if (navigator.turn_state ==
                                        Navigation::turning::TURN_COMPLETE) {
                navigator.set_next_checkpoint_as_goal();
                state_ = MOVING_TO_CHECKPOINT;
            } else {
                navigator.turn_around();
            }
            break;

        case MOVING_TOWARDS_OBJECT:
            //  Moves the robot towards the object
            if (is_obj_within_reach()) {
               navigator.stop_moving();
               state_ = PICKING_OBJECT;
            }
            break;

        case PICKING_OBJECT:
            // Acion of object picking
            pick_up_object();
            navigator.set_bin_location_as_goal();
            state_ = MOVING_TO_BIN_LOCATION;
            break;

        case MOVING_TO_BIN_LOCATION:
            // Moving the object to the bin location
            if (navigator.is_goal_reached())
                state_ = PLACING_OBJECT;
            break;

        case PLACING_OBJECT:
            // Acion of object placing at the bin location
            place_object();
            ROS_INFO_STREAM("[DCRobot] Task accomplished!");
            state_ = STOP;
            break;
    }
}

bool DCRobot::is_obj_within_reach() {
    //  Checks if object is within reach of the robot arm.
    auto objectPose = get_object_pose("base_link");
    double dx = objectPose.position.x;
    double dy = objectPose.position.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    ROS_INFO_STREAM("[DCRobot] object is at distance: " << distance);
    if (distance < 0.7) return true;
    return false;
}

geometry_msgs::Pose DCRobot::get_object_pose(std::string wrt) {
    //  Checks if object is within reach of the robot arm.
    geometry_msgs::Pose objectPose;
    if (detectObj.is_object_detected == false) {
        ROS_INFO_STREAM("[DCRobot] object has not been found yet.");
    } else {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer_.lookupTransform(wrt, "blue_box",
                                                            ros::Time(0));
        objectPose.position.x = transformStamped.transform.translation.x;
        objectPose.position.y = transformStamped.transform.translation.y;
        objectPose.position.z = transformStamped.transform.translation.z;
        // ROS_INFO_STREAM("[DCRobot] Got pose X: " << objectPose.position.x
        //                                 << ", Y: " << objectPose.position.y);
        objectPose.orientation = transformStamped.transform.rotation;
    }
    return objectPose;
}