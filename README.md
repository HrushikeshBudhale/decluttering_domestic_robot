[![Build & Test](https://github.com/HrushikeshBudhale/decluttering_domestic_robot/actions/workflows/industrial_ci_action.yml/badge.svg)](https://github.com/HrushikeshBudhale/decluttering_domestic_robot/actions/workflows/industrial_ci_action.yml)
[![Coverage Status](https://coveralls.io/repos/github/HrushikeshBudhale/decluttering_domestic_robot/badge.svg?branch=main)](https://coveralls.io/github/HrushikeshBudhale/decluttering_domestic_robot?branch=main)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
# decluttering_domestic_robot


## Project Overview and Description
This project was created as a demonstration showcasing understanding of core ros concepts and best software development practices while working in robotics software teams.

In this project robot performs a small search and manipulation operation. 
- First, the robot moves through a house in search of a blue colored cube.
- It utilizes ros movebase package for autonomously navigating through the mapped environment.
- While moving, Opencv based filtering is used for object detection purpose. Once the object is detected, robot moves to that object location.
- Moveit package is then used to plan manipulator trajectory towards the object to grasp it.
- In last phase of the state machine, robot brings the object to desired location in the environment. 

<p align="center">
  <img src="https://github.com/HrushikeshBudhale/decluttering_domestic_robot/blob/main/docs/home_organizing_robot.gif?raw=0" alt="ACTIVITY DIAGRAM" width="600"/>
</p>

- complete demo [video](https://youtu.be/3PMW3bMZKI4)

## Development Team
Sprint | #1 | #2 | #3 |
--- | --- | --- | ---
Hrushikesh Budhale | Driver | Design Keeper | Navigator
Abhijit Mahalle | Navigator | Driver | Design Keeper
Ameya Konkar | Design Keeper | Navigator | Driver 

## Table of Contents
   * [Documentation](#documentation)
   * [External Dependencies](#external-dependencies)
   * [Installation instructions](#installation-instructions)
   * [Tests](#tests)
   * [Generating documentation](#generating-documentation)
   * [Installation instructions](#installation-instructions)
   * [Activity Diagram](#activity-diagram)
   * [UML Class Diagram](#uml-class-diagram)
   * [Known Issues](#known-issues)

## Documentation
Doxygen generated documentation for developers can be found [here](https://hrushikeshbudhale.github.io/decluttering_domestic_robot/docs/html/index.html)

- Product backlog sheet [google sheet](https://docs.google.com/spreadsheets/d/1uLx1TDejwb_q-EkkCh65zcsgOdo6YiGDN0ZlcO-tUYo/edit?usp=sharing)
- Sprint planning notes [google docs](https://docs.google.com/document/d/1j18MeeHkREd-rwEOQwoSgQbipBhqeb5pdRHkQmvJkCU/edit?usp=sharing)

## External Dependencies
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [Gazebo](http://gazebosim.org/)
- [Tiago Robot](http://wiki.ros.org/Robots/TIAGo/Tutorials)
- [Moveit](https://moveit.ros.org/)
- [Opencv](https://github.com/opencv/opencv)

## Installation instructions
Install Tiago packages from [wiki.ros.org](http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS)

```
cd <catkin workspace>/src
sudo apt-get install git
git clone --recursive https://github.com/HrushikeshBudhale/decluttering_domestic_robot.git
cd ..
catkin build
source ./devel/setup.bash
```
Copy the existing SLAM configuration files and world file from cloned repo to the tiago directory using following command.
```
sudo cp -r src/decluttering_domestic_robot/configuration/smallHouse/ ~/.pal/tiago_maps/configurations/
sudo cp src/decluttering_domestic_robot/world/smallHouse.world /tiago_public_ws/src/tiago_simulation/tiago_gazebo/worlds/
```

For launching the simulation demo run following command.
This will start a gazebo environment of a house with 5 rooms. Tiago robot will be spawned. Also a small blue color box will be spawned in a random location.
```
roslaunch decluttering_domestic_robot simulation.launch
```

Launch file can be used to record a bag file by passing ```rocord_bag``` argument as follows
```
roslaunch decluttering_domestic_robot simulation.launch record_bag:=true
```

## Tests
To Build the Tests using catkin_make run following command in your catkin workspace.
```
catkin_make tests
```

To run the test execute following command.
```
rostest decluttering_domestic_robot main_test.test
```

## Generating documentation
Run the following command in folder's root directory to generate new documentation
```
doxygen docs/doxygen_config.conf
```
---
## Activity Diagram
<p align="center">
  <img src="https://github.com/HrushikeshBudhale/decluttering_domestic_robot/blob/main/uml/initial/activity_diagram.png?raw=0" alt="ACTIVITY DIAGRAM" width="300"/>
</p>


## UML Class Diagram
<p align="center">
  <img src="https://github.com/HrushikeshBudhale/decluttering_domestic_robot/blob/main/uml/revised/class_diagram.png?raw=0" alt="UML Diagram" width="600"/>
</p>


## Known Issues
Moveit trajectory generation fails while picking objects in places very near to the walls.


