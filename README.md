![Build & Test](https://github.com/HrushikeshBudhale/decluttering_domestic_robot/workflows/Build%20&%20Test/badge.svg)
[![Coverage Status](https://coveralls.io/repos/github/HrushikeshBudhale/decluttering_domestic_robot/badge.svg?branch=sprint1)](https://coveralls.io/github/HrushikeshBudhale/decluttering_domestic_robot?branch=sprint1)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
# decluttering_domestic_robot

---

### Project Overview and Description
Since robots are being rapidly deployed for various domestic applications, Acme Robotics is looking to develop a domestic robot as a part of its 5-year product road-map. The robot called the "Domestic Decluttering Robot" will declutter the house floor by identifying different items, collecting them, and placing them at their desired location in the house.  The robot will scout the house floor to detect and grasp the desired object. Once grasped, it will place the object at the predefined location. The robot will have a camera to identify the object, Lidar sensor to make a 3D map of the environment around it, and a 8 DOF robotic manipulator mounted on it.
 
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
While running the main source code , we are getting segementation core dumped due to issues with the eigen package.
Need to resolve in the next iteration.

Having issue while installing matplotlib.

Added to the backlog for the next iteration.

The issue where the source file might have issue is when the dependancies are not installed properly.

Make sure you install the correct packages.

Check out this link for installing Eigen:
https://eigen.tuxfamily.org/dox/GettingStarted.html

Check out this link for installing matplotlib:
https://matplotlib-cpp.readthedocs.io/en/latest/

Check out this link for visual kinematics - python 
https://github.com/dbddqy/visual_kinematics

---

### Building the Program

```
sudo apt-get install git
cd <catkin workspace>/src
git clone --recursive https://github.com/ameyakonk/ENPM808X_Midterm_Manipulator_IKSolver.git
cd ..
catkin build
source ./devel/setup.bash
roslaunch decluttering_domestic_robot simulation.launch

```
### To Run the Test

```
Run tests: ./test/cpp-test

```