
#include <dcrobot/dcrobot.hpp>

int main(int argc, char *argv[]) {
    // Initialize the node
    ros::init(argc, argv, "dcrobot_node");
    ROS_INFO_STREAM("[dcrobot_node] Started dcrobot_node");
    ros::NodeHandle nh;
    DCRobot dcr();  // Create DCRobot object
    ros::spin();
    return 0;
}