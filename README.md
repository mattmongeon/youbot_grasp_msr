youbot_grasp_msr
================

Package for controlling grasping with the [KUKA youBot](http://www.kuka-labs.com/en/service_robotics/research_education/youbot/) arm.

Introduction
============

This is a working demo for the forward and inverse kinematics of the 5 degree of freedom KUKA youBot arm.  It makes use of a technique that adds "virtual joints" to the robot to make its inverse kinematics easier to solve by transforming its arm into a redundant manipulator. This package includes a modified urdf called "youbot_virtual.urdf", which contains two virtual prismatic joints added to the base of the youBot URDF. The prismatic joints lie on the youBot's driving plane. These two virtual joints are used to define how much the base of the youBot should move in addition to how the arm should move, thus allowing the inverse kinematics to be solved with 7 degrees of freedom instead of just 5 in the arm.

The package dependancies include: 
---------------------------------

The following dependancies are clonable via github:

1) [hrl-kdl](https://github.com/gt-ros-pkg/hrl-kdl)

2) [urdfdom](https://github.com/ros/urdfdom)

3) [brics_actuator](http://wiki.ros.org/brics_actuator) ( The brics messages are required by the hrl-kdl) 

The package also relies on the following packages, installable from apt-get in Hydro. 

4) [youbot_driver](https://github.com/youbot/youbot_driver) 

5) [youbot_driver_ros_interface](https://github.com/youbot/youbot_driver_ros_interface)


Running the Package
================

The youBot arm must be powered on, and the arm must be initialized. Then we simply need to launch the main launch file using

```bash
roslaunch youbot_grasp_msr grasp.launch
```
The inverse and forward kinematics of the arm are calculated in order to drive the arm to a predefined position and grasp a square block. The arm then returns to a predefined pose and then drops the block back where it picked it up.

Additional Information
----------------------

This package is part of a larger project developed by a group of students in the [Northwestern MSR](http://robotics.northwestern.edu/) program. The completed project will be able to command the youBot to grasp an object that is identified by a laser pointer. It will navigate to the object using the [youbot_nav_msr](https://github.com/jihoonkimMSR/youbot_nav_msr) package and determine a precise grasp point using an [ASUS Xtion PRO LIVE](http:/www.asus.com/us/Multimedia/Xtion_PRO_LIVE) attached at the arm to provide visual feedback for grasping.  The object will then be carried by the youBot to a drop-off location.

Note of caution
---------------

The youBot arm is powerful, fast, and does not have any built-in collision detection. Be cautious in choosing desired gripper positions to avoid any damage to the youBot and/or users. 

Resources
=========

Here are a few links that may be useful in working with the youBot and its inverse kinematics: 

The [youbot_arm_test](https://github.com/youbot/youbot_driver_ros_interface) is a good place to get started with controlling the youBot arm.  In particular [this file](https://github.com/youbot/youbot_driver_ros_interface/blob/hydro-devel/src/examples/youbot_arm_test.cpp) provides good information for publishing joint positions comands in C++.

The authors of this package also found [youbot_teleop](https://github.com/adamjardim/youbot_teleop) a helpful tool during the development and testing of the youbot_grasp_msr package. 

[Unified Closed Form Inverse Kinematics for the KUKA youBot](http://ieeexplore.ieee.org/xpl/articleDetails.jsp?reload=true&arnumber=6309496) outlines a nice technique for solving the IK problem for the youBot with full 8-DOF youBot (3 from the base and 5 from the arm). The developers of youbot_grasp_msr wish to implement the method proposed in this paper in the near future. 

[youbot Manipulation](https://github.com/svenschneider/youbot-manipulation): Sven Schneider has some really good examples of solving the IK problems for the youBot. These examples have been largely deprecated by the release of newer versions of ROS, and the transition from the [ROS arm-navigation](http://wiki.ros.org/arm_navigation) stack to [MoveIt!](http://moveit.ros.org/). 

Acknowledgements
================

Thank you to [Jarvis Schultz](https://github.com/jarvisschultz) for contributing the ```dls_ik```, ```dls_ik_position_only```, and ```inverse_biased``` functions and their required utility functions in youbot_grasping_kdl.py.

