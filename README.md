# ROSPlan Framework

[![Build Status](https://travis-ci.com/KCL-Planning/ROSPlan.svg?branch=master)](https://travis-ci.com/KCL-Planning/ROSPlan)

The main ROSPlan website and documentation is available [here](http://kcl-planning.github.io/ROSPlan).

The ROSPlan framework provides a generic method for task planning in a ROS system. ROSPlan encapsulates both planning and dispatch. It provides with a simple interface, and already includes interfaces to common ROS libraries.

## Important note:

rosplan_demos are moved now to a separate [repository](https://github.com/KCL-Planning/rosplan_demos), check the following [pinned issue](https://github.com/KCL-Planning/ROSPlan/issues/189) for details.

## Installation

ROSPlan supports kinetic and melodic ROS distributions and is developed under Ubuntu 16.04 / 18.04, being linux Ubuntu 18.04 + melodic the recommended setup.

Install dependencies:
```sh
sudo apt install flex bison freeglut3-dev libbdd-dev python-catkin-tools ros-$ROS_DISTRO-tf2-bullet
```

Select or create a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace):
```sh
mkdir -p ROSPlan/src
cd ROSPlan/
```

Get the code:
```sh
cd src/
git clone https://github.com/KCL-Planning/rosplan
```

Compile everything:
```sh
catkin build
```

## Running a demo with the turtlebot 2, using Gazebo simulator (kinetic only)

This demo has been migrated to a separate repo, please follow up this link: [rosplan_demos](https://github.com/KCL-Planning/rosplan_demos)

## Related repositories:

- Automatic localisation and docking action interfaces with the Turtlebot 2 (Kobuki base) 
https://github.com/KCL-Planning/ROSPlan_interface_Turtlebot2

- Integration with the Component Oriented Layered-base Architecture for Autonomy (COLA2). Developed in the Research Center of Underwater Robotics (CIRS) in the University of Girona (UdG). This architecture is used to control the Autonomous Underwater Vehicles (AUVs) developed in this center. (https://bitbucket.org/udg_cirs/cola2)
https://github.com/KCL-Planning/ROSPlan_interface_COLA2

- Action interfaces for piloting a quadrotor from Jindrich Vodrazka, (takeoff, land, fly_square, and fly_waypoint).
https://github.com/fairf4x/ROSPlan_interface_quadrotor
