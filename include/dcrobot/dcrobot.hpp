#pragma once

#include <navigation/navigation.hpp>


class DCRobot {
 public:
    DCRobot();
    void synchronizeTask();

 private:
    Navigation navigator();
};
