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

ObjectSpawner::ObjectSpawner(ros::NodeHandle* node_handle):
                                    tf_listener_(this->tfBuffer_) {
    object_name = "blue_box";
    is_spawned = false;
    is_object_in_hand = false;
    seed = 5;
    map_range[0] = -6;   // x min
    map_range[1] = -7;   // y min
    map_range[2] = 5;    // x max
    map_range[3] = 7;    // y max

    nh_ = node_handle;
    pose_pub_ = nh_->advertise<gazebo_msgs::ModelState>(
                                                "/gazebo/set_model_state", 10);

    urdf_string_ = R"(<robot name="simple_box"><link name="object_base_link">
    </link><joint name="object_base_joint" type="fixed">
    <parent link="object_base_link"/><child link="my_box"/>
    <axis xyz="0 0 1" /><origin xyz="0 0 0" rpy="0 0 0"/></joint>
    <link name="my_box"><inertial><origin xyz="0 0 0" />
    <mass value="0.1" /><inertia  ixx="0.0001" ixy="0.0"  
    ixz="0.0"  iyy="0.0001"  iyz="0.0"  izz="0.0001" /></inertial>
    <visual><origin xyz="0 0 0"/><geometry><box size="0.05 0.05 0.05" />
    </geometry></visual><collision><origin xyz="0 0 0"/><geometry>
    <box size="0.05 0.05 0.05" /></geometry></collision></link>
    <gazebo reference="my_box"><material>Gazebo/Blue</material>
    </gazebo><gazebo reference="object_base_link"><gravity>0</gravity>
    </gazebo></robot>)";

    ROS_INFO_STREAM("[ObjectSpawner] ObjectSpawner object initialized");
}

