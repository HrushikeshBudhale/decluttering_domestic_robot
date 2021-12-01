/**
 * @file object_spawner.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-28
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <object_spawner/object_spawner.hpp>

ObjectSpawner::ObjectSpawner() {
    // Spawn object at 0,0 location in map
}

void ObjectSpawner::getRange(std::string file_location) {
    // read map file and set min and max range
}

std::pair<float, float> ObjectSpawner::getRandomLocation() {
    // Return 2d point on map
    return std::pair<float, float>();
}