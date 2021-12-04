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

bool ObjectSpawner::spawn_object() {
    if (is_spawned) {
        ROS_INFO_STREAM("[object_spawner] Object has already been spawned.");
        return 0;
    }
    spawn_object_client_ =
        nh_->serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    gazebo_msgs::SpawnModel srv;
    srv.request.model_name = object_name;
    srv.request.model_xml = urdf_string_;
    srv.request.initial_pose.position.x = (rand_r(&seed)%
                                        (std::abs(map_range[2] - map_range[0]))
                                                            + map_range[0]);
    srv.request.initial_pose.position.y = (rand_r(&seed)%
                                        (std::abs(map_range[3] - map_range[1]))
                                                            + map_range[1]);
    // srv.request.initial_pose.position.x = 4;
    // srv.request.initial_pose.position.y = 2;
    srv.request.initial_pose.position.z = 0.025;
    srv.request.initial_pose.orientation.w = 1;
    srv.request.reference_frame = "world";


    if (spawn_object_client_.call(srv)) {
        ROS_INFO_STREAM("[object_spawner] Object spawned successfully");
        is_spawned = true;
        // Update tf frame
        object_pose_ = srv.request.initial_pose;
        // Create timer to keep updating the frame
        object_pose_tf_timer_ = nh_->createTimer(ros::Duration(0.1),
                                            &ObjectSpawner::publish_pose, this);
        // Start service server to listen if object is in hand
        update_state_service_ = nh_->advertiseService("/setObjectState",
                                    &ObjectSpawner::set_object_state_cb, this);
    } else {
        ROS_ERROR_STREAM("[object_spawner] Failed to spawn object");
        return 1;
    }
    return 0;
}

bool ObjectSpawner::set_object_state_cb(std_srvs::SetBool::Request &req,
                                            std_srvs::SetBool::Response &res) {
    is_object_in_hand = req.data;
    res.message = "ObjectStateUpdated";
    res.success = true;
    return true;
}