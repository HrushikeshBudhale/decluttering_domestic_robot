/**
* MIT License
*
* Copyright(c) 2021
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files(the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* @file grasp_object.cpp
* @author Driver: Ameya Konkar (ameyakonk), Navigator: Hrushikesh Budhale
* @brief LIbrary file for GraspObject class
* @version 0.1
* @date 2021-12-08
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

void GraspObject::move_to_object_pose(geometry_msgs::Pose objectPose) {
    tf2::Quaternion quat_tf;
    // orientation while picking up
    quat_tf.setRPY(-0.011, 1.57, 1.57);
    objectPose.position.z = 0.30;
    objectPose.orientation = tf2::toMsg(quat_tf);
    move_arm_to_pose(objectPose);

    std_srvs::SetBool srv;
    srv.request.data = true;
    set_object_state_client_.call(srv);
}

void GraspObject::move_to_pick_pose() {
    geometry_msgs::Pose goalPose;
    tf2::Quaternion quat_tf;
    // orientation after picking up
    quat_tf.setRPY(0.011, 0.011, 1.57);
    goalPose.position.x = 0.2;
    goalPose.position.y = 0.0;
    goalPose.position.z = 0.45;
    goalPose.orientation = tf2::toMsg(quat_tf);
    move_arm_to_pose(goalPose);
}

void GraspObject::move_to_place_pose() {
    geometry_msgs::Pose goalPose;
    tf2::Quaternion quat_tf;
    // orientation while placing the object
    quat_tf.setRPY(-0.011, 1.57, 1.57);
    goalPose.position.x = 0.65;
    goalPose.position.y = 0.0;
    goalPose.position.z = 0.26;
    goalPose.orientation = tf2::toMsg(quat_tf);
    move_arm_to_pose(goalPose);

    std_srvs::SetBool srv;
    srv.request.data = false;
    set_object_state_client_.call(srv);
}
