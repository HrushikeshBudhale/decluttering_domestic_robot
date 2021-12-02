/**
 * @file navigation.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */


#include <navigation/navigation.hpp>

Navigation::Navigation(ros::NodeHandle* node_handle) {
    nh_ = node_handle;
    turn_state = TURN_COMPLETE;
    is_pose_initialized_ = false;
    checkpoint_counter_ = -1;
    bin_location_.position.x = 0;
    bin_location_.position.y = 0;

    goal_pub_ = nh_->advertise<geometry_msgs::PoseStamped>(
                                            "/move_base_simple/goal", 10);
    vel_pub_ = nh_->advertise<geometry_msgs::Twist>(
                                "/mobile_base_controller/cmd_vel", 10);
    cancel_goal_pub_ = nh_->advertise<actionlib_msgs::GoalID>(
                                            "/move_base/cancel", 5);

    cur_pose_sub_ = nh_->subscribe(
        "/robot_pose", 10, &Navigation::robot_pose_cb, this);

    clear_cost_map_client_ = nh_->serviceClient<std_srvs::Empty>(
                                                "/move_base/clear_costmaps");

    initialize_checkpoint_list();
}

void Navigation::set_next_checkpoint_as_goal() {
    checkpoint_counter_++;
    if (checkpoint_counter_ < checkpoints_.size()) {
        geometry_msgs::PoseStamped goalPose;
        goalPose.pose.position = checkpoints_[checkpoint_counter_].position;
        goalPose.pose.orientation.w = 1.0;
        goalPose.header.frame_id = "map";
        goal_pub_.publish(goalPose);
        goal_pub_.publish(goalPose);
        goal_pose_ = goalPose.pose;
        ROS_INFO_STREAM("[Navigation] Publishied next checkpoint pose as goal");
    } else {
        ROS_INFO_STREAM("[Navigation] No more checkpoints available");
    }
    return;
}

geometry_msgs::Point Navigation::getNextCheckpoint() {
    // functions responsible for getting position of next checkpoint
    return geometry_msgs::Point();
}

geometry_msgs::Point Navigation::getBinLocation() {
    // function responsible for getting bin location after picking up the object
    return geometry_msgs::Point();
}

void Navigation::moveToNextCheckpoint() {
    // function responsible for making robot move to next unattended checkpoint
}

void Navigation::moveNearObject(geometry_msgs::Pose) {
    // function responsible for moving the robot near object after detecting it
}

void Navigation::turnAround() {
    // function for making robot turn around to detect the object
}
