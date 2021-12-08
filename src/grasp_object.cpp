/**
 * @file grasp_object.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */


#include <grasp_object/grasp_object.hpp>

GraspObject::GraspObject(ros::NodeHandle* nodeHandle) {
    //  Initializing parameter values
    nh_ = nodeHandle;
    //  Initializing Service client
    set_object_state_client_ = nh_->serviceClient<std_srvs::SetBool>(
                                                            "/setObjectState");
    ROS_INFO_STREAM("[GraspObject] GraspObject object initialized");
}

void GraspObject::move_arm_to_pose(geometry_msgs::Pose inPose) {
    //  create am asynchronous spinner
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //  publishing robot arm goal pose
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "base_footprint";
    goal_pose.pose = inPose;
    moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
    group_arm_torso.setPlannerId("SBLkConfigDefault");
    group_arm_torso.setPoseReferenceFrame("base_footprint");
    group_arm_torso.setPoseTarget(goal_pose);

    ROS_INFO_STREAM("[GraspObject] Planning to move " <<
                    group_arm_torso.getEndEffectorLink()
                    << " to a target pose expressed in " <<
                    group_arm_torso.getPlanningFrame());

    group_arm_torso.setStartStateToCurrentState();
    group_arm_torso.setMaxVelocityScalingFactor(1.0);

    //  Create moveit planner interface.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    group_arm_torso.setPlanningTime(5.0);
    bool success = static_cast<bool>(group_arm_torso.plan(my_plan));

    if (!success)
        throw std::runtime_error("No plan found");

    ROS_INFO_STREAM("[GraspObject] Plan found in " << my_plan.planning_time_
                                                                << " seconds");

    // Execute the plan
    ros::Time start = ros::Time::now();

    moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();
    if (!static_cast<bool>(e))
        throw std::runtime_error("Error executing plan");

    ROS_INFO_STREAM("[GraspObject] Motion duration: "
                                        << (ros::Time::now() - start).toSec());
    spinner.stop();
}



