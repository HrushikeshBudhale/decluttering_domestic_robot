#pragma once

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <sstream>
#include "ros/ros.h"


class Navigation {
 public:
    Navigation();
    void move_forward(double speed);
    void turn(double speed);
    void stop(void);
    void move_around(void);


 private:
    
};
