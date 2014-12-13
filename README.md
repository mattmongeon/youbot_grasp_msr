youbot_grasp_msr
================

Package for controlling grasping with the KUKA youBot arm.



Introduction
============

This is a working demo for the forward and inverse kinematics of the 5 degree of freedom [KUKA youBot](http://www.kuka-labs.com/en/service_robotics/research_education/youbot/) arm. 

The package makes use of a technique common to manipulators with less than 6 degrees of freedom. The idea is simple, "virtual joints" are added to the youBot's urdf to give a more convenient number of DOFs. The IK problem is then solved with a defined heuristic to deal with the extra degrees of freedom. This package includes a modified urdf called "youbot_virtual.urdf", which contains two virtual prismatic joints added to the top of the youBot urdf. The prismatic joints lie on the Youbots driving plane. These two virtual joints control the linear velocity of the base. With this modification, the youBot manipulator can be modelled as a redundant 7 degree of freedom manipulator and the IK can be solved with the hrl-kdl stack. The values that the IK problem finds for these two virtual joints signifies a small displacement that the base should drive, in order to get the target object into a pose that falls within the workspace of the youBot arm. This ensures that an IK solution will be found.

The package dependancies include: 
---------------------------------

The following dependancies are clonable via github:

1) [ROS hrl-kdl](https://github.com/gt-ros-pkg/hrl-kdl)

2) [ROS urdfdom](https://github.com/ros/urdfdom)

3) [brics_actuator](http://wiki.ros.org/brics_actuator) ( The brics messages are required by the hrl-kdl) 

The package also relies on the following packages, installable from apt-get in Hydro. 
4) [youbot_driver](https://github.com/youbot/youbot_driver) 

5) [youbot_driver_ros_interface](https://github.com/youbot/youbot_driver_ros_interface)


Running the Package
================

The youBot arm must be powered on, and the arm must be initialized. Then we simply
need to launch the main launch file using

```bash
roslaunch youbot_grasp_msr grasp.launch
```
The inverse and forward kinematics of the arm are calculated in order to drive the arm to a predefined position and grasp a square block. The arm then returns to a predefined pose, with the block in its grasp, ready for transportation.

This package is part of a larger project developed by a group of students in the [Northwestern MSR](http://robotics.northwestern.edu/) program. As part of this project, the youBot is commanded which object to grasp using a laser pointer. It navigates to the object using the [youbot_nav_msr
](https://github.com/jihoonkimMSR/youbot_nav_msr) package, fine tunes its position with the inbuilt RGB camera of the youBot. At this point, the action server calls the youbot_grasp_msr, which has the task of grasping the designated object. 

Note of caution
---------------
The youBot arm is powerful, fast, and does not have internal speed limiters. Be cautious in choosing desired gripper positions to avoid any damage to the youBot and/or users. 

Resources
=========

Here are a few links that may be useful in figuring out what is going on with the youBot and its inverse kinematics: 

The [youBot_arm_test](https://github.com/youbot/youbot_driver_ros_interface/blob/hydro-devel/src/examples/youbot_arm_test.cpp) package is a good place to get started with controlling the youBot arm. 

The authors of this package also found [youbot_teleop](https://github.com/adamjardim/youbot_teleop) a helpful tool
during the development and testing of the youbot_grasp_msr package. 

[Unified Closed Form Inverse Kinematics for the KUKA youBot](http://ieeexplore.ieee.org/xpl/articleDetails.jsp?reload=true&arnumber=6309496) outlines a nice technique for solving the IK problem for the youBot with full 8-DOF youBot (3 from the base and 5 from the arm). The developers of youbot_grasp_msr wish to implement the method proposed in this paper in the near future. 

[Youbot Manipulation](https://github.com/svenschneider/youbot-manipulation): Sven Schneider has some really good examples of solving the IK problems for the youBot. These examples have been largely deprecated by the release of newer versions of ROS, and the transition from the [ROS arm-navigation](http://wiki.ros.org/arm_navigation) stack to [MoveIt!](http://moveit.ros.org/). 


