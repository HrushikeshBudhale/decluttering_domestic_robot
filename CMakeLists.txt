cmake_minimum_required(VERSION 3.0.2)
project(decluttering_domestic_robot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  image_transport
  cv_bridge
  control_msgs
  actionlib
)

# find_package(OpenCV REQUIRED)
find_package(OpenCV COMPONENTS opencv_highgui opencv_features2d opencv_xfeatures2d opencv_calib3d)
if(NOT OPENCV_FOUND)
	find_package(OpenCV REQUIRED COMPONENTS opencv_highgui opencv_features2d opencv_calib3d)
	add_definitions(-DNO_CV_XFEATURES2D)
endif()

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen3 REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   sensor_msgs#   std_msgs
# )


add_library(dcrobot_library src/dcrobot.cpp)
add_library(navigation_library src/navigation.cpp)
add_library(detect_object_library src/detect_object.cpp)
add_library(grasp_object_library src/grasp_object.cpp)


###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dcrobot_node src/dcrobot_node.cpp)


target_link_libraries(dcrobot_node
  dcrobot_library
  navigation_library
  detect_object_library
  grasp_object_library
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_library(object_spawner_library src/object_spawner.cpp)

add_executable(object_spawner_node src/object_spawner_node.cpp)

target_link_libraries(object_spawner_node
  object_spawner_library
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	add_rostest_gtest(main_rostest 
	  test/main_test.test 
	  test/navigation_test.cpp
	)
	target_link_libraries(
	  main_rostest ${catkin_LIBRARIES}
	)
endif()
