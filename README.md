# ROSPlan Framework

![Build status](https://github.com/KCL-Planning/ROSPlan/workflows/build/badge.svg)
![Test status](https://github.com/KCL-Planning/ROSPlan/workflows/test/badge.svg)

The main ROSPlan website and documentation is available [here](http://kcl-planning.github.io/ROSPlan).

The ROSPlan framework provides a generic method for task planning in a ROS system. ROSPlan encapsulates both planning and dispatch. It provides with a simple interface, and already includes interfaces to common ROS libraries.

## ROSPlan Demos:

Several demos are available in the [rosplan_demos repository](https://github.com/KCL-Planning/rosplan_demos). More will be added over time.

<p align="center"><img src="https://github.com/KCL-Planning/rosplan_demos/blob/master/rosplan_stage_demo/stage_demo.png" width="50%"><img src="https://github.com/KCL-Planning/rosplan_demos/blob/master/rosplan_stage_exploration_demo/rosplan_exploration_demo.png" width="50%"></p>

## Installation

ROSPlan supports by default noetic ROS1 distribution and is developed under Ubuntu 20.04.

Install dependencies:
```sh
sudo apt install flex bison freeglut3-dev libbdd-dev python3-osrf-pycommon python3-catkin-tools
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

## Using ROSPlan's docker image
Get your docker image from the [Docker Hub](https://hub.docker.com/r/kclplanning/rosplan):

```
docker pull kclplanning/rosplan
```

Run an interactive bash shell inside the docker, ready to run ROSPlan:
```
docker run -it --rm kclplanning/rosplan bash
```

## ROSPlan KB GUI
There is a newly developed interface to interact with ROSPlan's Knowledge Base. It was built by Eden Jia, using `tkinter` in Python. You can find it and use it [here](https://github.com/H0PP3R/rosplan_gui).

## Related repositories:

- Automatic localisation and docking action interfaces with the Turtlebot 2 (Kobuki base) 
https://github.com/KCL-Planning/ROSPlan_interface_Turtlebot2

- Integration with the Component Oriented Layered-base Architecture for Autonomy (COLA2). Developed in the Research Center of Underwater Robotics (CIRS) in the University of Girona (UdG). This architecture is used to control the Autonomous Underwater Vehicles (AUVs) developed in this center. (https://bitbucket.org/udg_cirs/cola2)
https://github.com/KCL-Planning/ROSPlan_interface_COLA2

- Action interfaces for piloting a quadrotor from Jindrich Vodrazka, (takeoff, land, fly_square, and fly_waypoint).
https://github.com/fairf4x/ROSPlan_interface_quadrotor
