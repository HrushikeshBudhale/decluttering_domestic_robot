[![Build & Test](https://github.com/HrushikeshBudhale/decluttering_domestic_robot/actions/workflows/industrial_ci_action.yml/badge.svg)](https://github.com/HrushikeshBudhale/decluttering_domestic_robot/actions/workflows/industrial_ci_action.yml)
[![Coverage Status](https://coveralls.io/repos/github/HrushikeshBudhale/decluttering_domestic_robot/badge.svg?branch=main)](https://coveralls.io/github/HrushikeshBudhale/decluttering_domestic_robot?branch=main)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
# decluttering_domestic_robot

---

### Project Overview and Description
Since robots are being rapidly deployed for various domestic applications, Acme Robotics is looking to develop a domestic robot as a part of its 5-year product road-map. The robot called the "Domestic Decluttering Robot" will declutter the house floor by identifying different items, collecting them, and placing them at their desired location in the house.  The robot will scout the house floor to detect and grasp the desired object. Once grasped, it will place the object at the predefined location. The robot will have a camera to identify the object, Lidar sensor to make a 3D map of the environment around it, and a 8 DOF robotic manipulator mounted on it.

### Documentation
Doxygen generated documentation for developers can be found [here](https://hrushikeshbudhale.github.io/decluttering_domestic_robot/docs/html/index.html)

- Robot demo [video](https://youtu.be/NrRCCqRFqw4)
- Presentation [video](https://youtu.be/-lPjSiizmCI)
- Presentation [slides](https://docs.google.com/presentation/d/1osl5vrxGRXqx1QkJhoWhC6rh83X2Kr2OMlprzH5XEH0/edit?userstoinvite=abhimah@umd.edu#slide=id.g107542cf1d8_6_10)
- Product backlog sheet [google sheet](https://docs.google.com/spreadsheets/d/1uLx1TDejwb_q-EkkCh65zcsgOdo6YiGDN0ZlcO-tUYo/edit?usp=sharing)
- Sprint planning notes [google docs](https://docs.google.com/document/d/1j18MeeHkREd-rwEOQwoSgQbipBhqeb5pdRHkQmvJkCU/edit?usp=sharing)

### Development Team
Sprint | #1 | #2 | #3 |
--- | --- | --- | ---
Hrushikesh Budhale | Driver | Design Keeper | Navigator
Abhijit Mahalle | Navigator | Driver | Design Keeper
Ameya Konkar | Design Keeper | Navigator | Driver 

### External Dependencies
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [Gazebo](http://gazebosim.org/)
- [Tiago Robot](http://wiki.ros.org/Robots/TIAGo/Tutorials)
- [Moveit](https://moveit.ros.org/)
- [Opencv](https://github.com/opencv/opencv)

### Installation instructions
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

### Tests
To Build the Tests using catkin_make run following command in your catkin workspace.
```
catkin_make tests
```

To run the test execute following command.
```
rostest decluttering_domestic_robot main_test.test
```

### Generating documentation
Run the following command in folder's root directory to generate new documentation
```
doxygen docs/doxygen_config.conf
```
---

### Activity Diagram
<p align="center">
  <img src="https://github.com/HrushikeshBudhale/decluttering_domestic_robot/blob/main/uml/initial/activity_diagram.png" alt="ACTIVITY DIAGRAM" width="300"/>
</p>

---

### UML Class Diagram
<p align="center">
  <img src="https://github.com/HrushikeshBudhale/decluttering_domestic_robot/blob/main/uml/revised/class_diagram.png" alt="UML Diagram" width="600"/>
</p>

---

### Known Issues/Bugs
Picking the object at corner positions is not possible for the robot.



