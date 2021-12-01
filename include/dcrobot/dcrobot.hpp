/**
 * @file dcrobot.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <navigation/navigation.hpp>
#include <grasp_object/grasp_object.hpp>
#include <detect_object/detect_object.hpp>

class DCRobot {
 public:
    DCRobot();
    void synchronizeTask();

 private:
    Navigation navigator();
    GraspObject graspObj();
    DetectObject detecteObj();
};
