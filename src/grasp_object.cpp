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



