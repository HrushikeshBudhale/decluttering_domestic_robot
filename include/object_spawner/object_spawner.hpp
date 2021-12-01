/**
 * @file object_spawner.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-28
 * 
 * @copyright Copyright (c) 2021
 * 
 */



#pragma once

#include <vector>
#include <utility>
#include <string>

class ObjectSpawner {
 public:
    ObjectSpawner();
    void getRange(std::string);
    std::pair<float, float> getRandomLocation();

 private:
    std::vector<std::pair<float, float>> mapRange;
};
