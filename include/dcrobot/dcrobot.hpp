/**
 * @file dcrobot.hpp
 * @author Ameya Konkar (ameyakonk)
 * @brief 
 * @version 0.1
 * @date 2021-12-09
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
   /**
     * @brief Construct a new DCRobot object
     * 
     */
    explicit DCRobot(ros::NodeHandle*);
    /**
     * @brief Synchronizes various tasks of the robot
     * 
     */
    void handle_states();
    /**
     * @brief Checks whether the object is within reach of the robot
     * 
     */
    bool is_obj_within_reach();
    /**
     * @brief Gets object pose from the world. 
     * 
     */
    geometry_msgs::Pose get_object_pose(std::string wrt = "map");
    /**
     * @brief Handles object picking by the robot.
     * 
     */
    void pick_up_object();
    /**
     * @brief Handles object placement by the robot.
     * 
     */
    void place_object();
    /**
     * @brief Enumerates various robot functioning states.
     * 
     */
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
    /** Instance for Navigation class created*/
    Navigation navigator;
    /** Instance for graspObj class created*/
    GraspObject graspObj;
    /** Instance for DetectObject class created*/
    DetectObject detectObj;
 private:
   /**
     * @brief Enumerates various object states.
     * 
     */
    void set_head_down();
    /**
     * @brief Enumerates various object states.
     * 
     */
    void create_point_head_client(PointHeadClientPtr&);
    /** Node Handle created*/
    ros::NodeHandle* nh_;
    /** TF buffer created*/
    tf2_ros::Buffer tfBuffer_;
    /** TF listener created*/
    tf2_ros::TransformListener tf_listener_;
    /** PointHeadClientPtr for head movement actions*/
    PointHeadClientPtr point_head_client_;
    /** robotState for checking current state of the robot */
    robotState state_;
    /** robotState for storing previous states of the robot*/
    robotState pre_state_;
};
