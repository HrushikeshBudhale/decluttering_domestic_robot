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

GraspObject::GraspObject() {
    // setup moveit params
    // setup publisher, subscriber and services
}


void GraspObject::openGrip() {
    // function responsible for openig the grip
}


void GraspObject::closeGrip() {
    // function responsible for closing the grip
}


void GraspObject::pickObject(geometry_msgs::Pose) {
    // function responsible for picking up the object given its pose
}


void GraspObject::placeObject(geometry_msgs::Pose) {
    // function responsible for placing the object given the pose of place
}

