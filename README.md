[![Build Status](https://app.travis-ci.com/HrushikeshBudhale/Decluttering-Domestic-Robot.svg?branch=main)](https://app.travis-ci.com/HrushikeshBudhale/Decluttering-Domestic-Robot)
[![Coverage Status](https://coveralls.io/repos/github/HrushikeshBudhale/Decluttering-Domestic-Robot/badge.svg?branch=main)](https://coveralls.io/github/HrushikeshBudhale/Decluttering-Domestic-Robot?branch=main)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
# Decluttering-Domestic-Robot

---

### Project Overview and Description
Since robots are being rapidly deployed for various domestic applications, Acme Robotics is looking to develop a domestic robot as a part of its 5-year product road-map. The robot called the "Domestic Decluttering Robot" will declutter the house floor by identifying different items, collecting them, and placing them at their desired location in the house.  The robot will scout the house floor to detect and grasp the desired object. Once grasped, it will place the object at the predefined location. The robot will have a camera to identify the object, Lidar sensor to make a 3D map of the environment around it, and a 8 DOF robotic manipulator mounted on it.
 
### Assumptions

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
  <img src="https://github.com/HrushikeshBudhale/Decluttering-Domestic-Robot/blob/sprint1/uml/initial/activity_diagram.png" alt="ACTIVITY DIAGRAM" width="400"/>
</p>

---

### UML Class Diagram
<p align="center">
  <img src="https://github.com/HrushikeshBudhale/Decluttering-Domestic-Robot/blob/sprint1/uml/initial/class_diagram.png" alt="UML Diagram" width="600"/>
</p>

---

### External Dependencies
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [Gazebo](http://gazebosim.org/)
- [Fetch Robot](http://docs.fetchrobotics.com/)
- [Moveit](https://moveit.ros.org/)
- [Opencv](https://github.com/opencv/opencv)
