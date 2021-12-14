![Build & Test](https://github.com/HrushikeshBudhale/decluttering_domestic_robot/workflows/Build%20&%20Test/badge.svg)
[![Coverage Status](https://coveralls.io/repos/github/HrushikeshBudhale/decluttering_domestic_robot/badge.svg?branch=main)](https://coveralls.io/github/HrushikeshBudhale/decluttering_domestic_robot?branch=main)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
# decluttering_domestic_robot

---

### Project Overview and Description
Since robots are being rapidly deployed for various domestic applications, Acme Robotics is looking to develop a domestic robot as a part of its 5-year product road-map. The robot called the "Domestic Decluttering Robot" will declutter the house floor by identifying different items, collecting them, and placing them at their desired location in the house.  The robot will scout the house floor to detect and grasp the desired object. Once grasped, it will place the object at the predefined location. The robot will have a camera to identify the object, Lidar sensor to make a 3D map of the environment around it, and a 8 DOF robotic manipulator mounted on it.

### Presentation Video link

Video link: https://youtu.be/-lPjSiizmCI

### Presentation link

https://docs.google.com/presentation/d/1osl5vrxGRXqx1QkJhoWhC6rh83X2Kr2OMlprzH5XEH0/edit?userstoinvite=abhimah@umd.edu#slide=id.g107542cf1d8_6_10

### Robot Simulation link

Video link: https://youtu.be/NrRCCqRFqw4

### Development Team
#### Sprint 1
* Driver        - Hrushikesh Budhale
* Navigator     - Abhijit Mahalle
* Design Keeper - Ameya Konkar

#### Sprint 1
* Driver        - Abhijit Mahalle
* Navigator     - Ameya Konkar
* Design Keeper - Hrushikesh Budhale

#### Sprint 1
* Driver        - Ameya Konkar
* Navigator     - Hrushikesh Budhale
* Design Keeper - Abhijit Mahalle


### Product Backlog
Product backlog can be found in [google sheet](https://docs.google.com/spreadsheets/d/1uLx1TDejwb_q-EkkCh65zcsgOdo6YiGDN0ZlcO-tUYo/edit?usp=sharing)

### Sprint planning notes
Sprint planning notes can found in [google docs](https://docs.google.com/document/d/1j18MeeHkREd-rwEOQwoSgQbipBhqeb5pdRHkQmvJkCU/edit?usp=sharing)

### Deliverables
* Developer level documentation using doxygen
* Build status using travis CI
* Code coverage using coveralls
* Accurate implementation using generated test cases in gtest
* Tests for ros nodes in rostest
* Steps showing how to build the repository
* Steps showing how to run test and demo applications
* Steps showing how to generate Doxygen documentation
* Valgrind output for memory leak check

### Activity Diagram
<p align="center">
  <img src="https://github.com/HrushikeshBudhale/decluttering_domestic_robot/blob/sprint1/uml/initial/activity_diagram.png" alt="ACTIVITY DIAGRAM" width="400"/>
</p>

---

### UML Class Diagram
<p align="center">
  <img src="https://github.com/HrushikeshBudhale/decluttering_domestic_robot/blob/sprint1/uml/initial/class_diagram.png" alt="UML Diagram" width="600"/>
</p>

---

### External Dependencies
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [Gazebo](http://gazebosim.org/)
- [Tiago Robot](http://wiki.ros.org/Robots/TIAGo/Tutorials)
- [Moveit](https://moveit.ros.org/)
- [Opencv](https://github.com/opencv/opencv)

## Known Issues/Bugs

---
Picking the object at certain positions and orientation is not possible for the robot.

---

### Installing Tiago package

```
Install Tiago packages using the link: http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS

```

### Building and running the Program

```
cd <catkin workspace>/src
sudo apt-get install git
git clone --recursive https://github.com/HrushikeshBudhale/decluttering_domestic_robot.git
cd ..
catkin build
source ./devel/setup.bash
roslaunch decluttering_domestic_robot simulation.launch

```
### To Build the Tests

```
Using Catkin Make: catkin_make tests

```
### To Run the Tests

```
rostest decluttering_domestic_robot main_test.test

```
