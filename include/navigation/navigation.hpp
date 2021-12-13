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
    /**
     * @brief Construct a new Navigation:: Navigation object
     *        Creates publisher and subscriber objects     
     * 
     * @param node_handle
     * @return void 
     */
    explicit Navigation(ros::NodeHandle*);

    /**
     * @brief Updates new checkpoints as goal for the robot.
     * @return void
     */
    void set_next_checkpoint_as_goal();
    
    /**
     * @brief sets bin location as goal for the robot 
     * @return void
     */
    void set_bin_location_as_goal();

    /**
     * @brief Callback function to obtain current robot position and orientation
     * 
     * @param robot_pose
     * @return void 
     */
    void robot_pose_cb(const geometry_msgs::PoseWithCovarianceStamped&);
    
    /**
     * @brief Checks if a given is reached.
     * 
     * @return true if goal reached.
     * @return false if goal is not reached.
     */
    bool is_goal_reached();

    /**
     * @brief turns the robot by 180 degree 
     *        or till the object is detected. 
     * @return void
     */
    void turn_around();

    /**
     * @brief Stops moving the robot once the robot has reached within a 
     *        certain distance from the object.
     * @return void
     */
    void stop_moving();
    
    /**
     * @brief Sets object coordinates as a goal to the robot
     * @return void
     * @param objectPose 
     */
    void set_object_pose_as_goal(geometry_msgs::Pose);
    
    enum turning {
        TURN_START,
        TURNING,
        TURN_COMPLETE
    };
    turning turn_state;

 private:
   
    /**
     * @brief Published turning velocity to the robot.
     * @return void 
     */
    void set_turning_velocity();

    /**
     * @brief initialize predefined checkpoints
     * @return void
     */
    void initialize_checkpoint_list();

    /** Node handle  */
    ros::NodeHandle* nh_;
    /** subscriber to get current robot pose  */
    ros::Subscriber cur_pose_sub_;
    /** Publisher to set checkpoint goals  */
    ros::Publisher goal_pub_;
    /** Publisher to set turning velocity  of the robot */
    ros::Publisher vel_pub_;
    /** Publisher to cancel a given goal  */
    ros::Publisher cancel_goal_pub_;
    /** ServiceClient to clear cost map  */
    ros::ServiceClient clear_cost_map_client_;

    /** checks if robot pose is received  */
    bool is_pose_initialized_;

    /** stores current robot pose  */
    geometry_msgs::Pose current_pose_;
    /** stores the target bin location  */
    geometry_msgs::Pose bin_location_;
    /** stores the goal pose for the robot  */
    geometry_msgs::Pose goal_pose_;
    /** stores initial robot orientation for the robot to rotate  */
    tf2::Quaternion initial_tf_quat_;
    /** stores fixed checkpoints for the robot  */
    std::vector <geometry_msgs::Pose> checkpoints_;
    /** iterator to change checkpoints  */
    std::vector<geometry_msgs::Pose>::size_type checkpoint_counter_;
};
