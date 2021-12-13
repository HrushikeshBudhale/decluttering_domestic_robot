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

bool Navigation::is_goal_reached() {
    double x_sq = std::pow(
        current_pose_.position.x- goal_pose_.position.x, 2);
    double y_sq = std::pow(
        current_pose_.position.y- goal_pose_.position.y, 2);
    double distance = std::sqrt(x_sq + y_sq);
    if (distance <= 0.1) return true;
    return false;
}

void Navigation::turn_around() {
    if (is_pose_initialized_) {
        if (turn_state == TURN_START) {
            tf2::fromMsg(current_pose_.orientation, initial_tf_quat_);
            initial_tf_quat_ = initial_tf_quat_.inverse();
            turn_state = TURNING;
        } else {
            tf2::Quaternion tf2_quat;
            tf2::fromMsg(current_pose_.orientation, tf2_quat);
            tf2_quat *= initial_tf_quat_;
            auto angle = tf2::getYaw(tf2_quat);
            set_turning_velocity();
            // ROS_INFO_STREAM("[Navigation] Turning around: " << angle);
            if (angle > -0.1 && angle < 0) {
                turn_state = TURN_COMPLETE;
                ROS_INFO_STREAM("[Navigation] Turning complete");
            }
        }
    } else {
        ROS_INFO_STREAM("[Navigation] waiting for initial pose");
    }
}

void Navigation::stop_moving() {
    actionlib_msgs::GoalID new_msg;
    cancel_goal_pub_.publish(new_msg);
    actionlib_msgs::GoalStatusArrayConstPtr reply;
    ROS_INFO_STREAM("[Navigation] Waiting for goal cancellation response.");
    reply = ros::topic::waitForMessage<actionlib_msgs::GoalStatusArray>(
                                                            "/move_base/status",
                                                            ros::Duration(2));
    if (reply == NULL)
        ROS_ERROR_STREAM("[Navigation] Cancellation request not received");
    else
        ROS_INFO_STREAM("[Navigation] Robot stopped moving");
}

void Navigation::set_object_pose_as_goal(geometry_msgs::Pose objectPose) {
    geometry_msgs::PoseStamped goalPose;
    // ROS_INFO_STREAM("[Navigation] Setting X: " << objectPose.position.x
    //                                 << ", Y: " << objectPose.position.y);
    goalPose.pose.position = objectPose.position;
    goalPose.pose.orientation.w = 1.0;
    goalPose.header.frame_id = "map";
    goal_pub_.publish(goalPose);
    goal_pub_.publish(goalPose);
    goal_pose_ = goalPose.pose;
    ROS_INFO_STREAM("[Navigation] Publishied object pose as goal");
    return;
}

void Navigation::initialize_checkpoint_list() {
    int cp_arr_x[5] = {0, 1, 4, -1, -4};
    int cp_arr_y[5] = {0, -1, -6, -6, -3};
    for (int i = 0; i < 5; i++) {
        geometry_msgs::Pose checkpoint_pose;
        checkpoint_pose.position.x = cp_arr_x[i];
        checkpoint_pose.position.y = cp_arr_y[i];
        checkpoints_.push_back(checkpoint_pose);
    }
    ROS_INFO_STREAM("[Navigation] Navigation object initialized");
}

void Navigation::set_turning_velocity() {
    // function for making robot turn around to detect the object
    geometry_msgs::Twist orientation;
    orientation.angular.z = 0.6;
    vel_pub_.publish(orientation);
}
