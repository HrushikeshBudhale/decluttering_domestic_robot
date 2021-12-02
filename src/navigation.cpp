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

void Navigation::set_bin_location_as_goal() {
    // clear cost map in the existing map
    std_srvs::Empty srv;
    clear_cost_map_client_.call(srv);

    geometry_msgs::PoseStamped goalPose;
    goalPose.pose.position = bin_location_.position;
    goalPose.pose.orientation.w = 1.0;
    goalPose.header.frame_id = "map";
    goal_pub_.publish(goalPose);
    goal_pub_.publish(goalPose);
    goal_pose_ = goalPose.pose;
    ROS_INFO_STREAM("[Navigation] bin location x: " << goalPose.pose.position.x
                                        << ", y: " << goalPose.pose.position.y);
    ROS_INFO_STREAM("[Navigation] Publishied bin pose as goal");
    return;
}

void Navigation::robot_pose_cb(
    const geometry_msgs::PoseWithCovarianceStamped &robot_pose) {
    current_pose_ = robot_pose.pose.pose;
    is_pose_initialized_ = true;
}