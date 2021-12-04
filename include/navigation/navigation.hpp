/**
 * @file navigation.hpp
 * @author Abhijit Mahalle (abhimah@umd.edu)
 * @brief 
 * @version 0.1s
 * @date 2021-01-12
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

// ROS headers
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include <ros/ros.h>

// Std C++ headers
#include <vector>

class Navigation {
 public:
    explicit Navigation(ros::NodeHandle*);
    void set_next_checkpoint_as_goal();
    void set_bin_location_as_goal();
    void robot_pose_cb(const geometry_msgs::PoseWithCovarianceStamped&);
    bool is_goal_reached();
    void stop_moving();
    void set_object_pose_as_goal(geometry_msgs::Pose);
    enum turning {
        TURN_START,
        TURNING,
        TURN_COMPLETE
    };
    turning turn_state;

 private:
    void initialize_checkpoint_list();
    void set_turning_velocity();

    ros::NodeHandle* nh_;
    ros::Subscriber cur_pose_sub_;
    ros::Publisher goal_pub_;
    ros::Publisher vel_pub_;
    ros::Publisher cancel_goal_pub_;
    ros::ServiceClient clear_cost_map_client_;

    bool is_pose_initialized_;

    geometry_msgs::Pose current_pose_;
    geometry_msgs::Pose bin_location_;
    geometry_msgs::Pose goal_pose_;
    tf2::Quaternion initial_tf_quat_;
    std::vector <geometry_msgs::Pose> checkpoints_;
    std::vector<geometry_msgs::Pose>::size_type checkpoint_counter_;
};
