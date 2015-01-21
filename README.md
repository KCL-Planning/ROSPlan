ROSPlan Framework
=================

The ROSPlan framework provides a generic method for task planning in a ROS system. ROSPlan encapsulates both planning and dispatch. It possesses as a simple interface, and already includes interfaces to common ROS libraries.

### Installation

Get the prerequisites:

(for Indigo)
```sh
sudo apt-get install ros-indigo-mongodb-store
```
(for Hydro)
```sh
sudo apt-get install ros-hydro-mongodb-store
```
Select a catkin workspace or create a new one:
```sh
mkdir -p ROSPlan/src
cd ROSPlan/
```
Get the code:
```sh
git clone https://github.com/KCL-Planning/ROSPlan.git src/
```
Compile everything:
```sh
source /opt/ros/hydro/setup.bash
catkin_make -j1
```

### Running a demo with the turtlebot

The turtlebot demo is now a simple exploration mission. The turtlebot will visit randomly generated waypoints around a map.

The domain for this demo is in the `rosplan_planning_system` package, as `common/domain.pddl`.

To run the demo first follow the installation instructions and quick-start guide for the turtlebot simulation:

http://wiki.ros.org/turtlebot_gazebo (indigo)

http://wiki.ros.org/turtlebot_simulator (hydro)

Then source the ROSPlan workspace and run:
```sh
roslaunch rosplan_demos turtlebot.launch
sh src/rosplan_demos/scripts/turtlebot_explore.bash
```

`turtlebot.launch` will start the turtlebot simulation, rviz, and ROSPlan.

`turtlebot_explore.bash` calls a ROSPlan component service to generate a roadmap (you can see in rviz); adds exploration goals; and then calls the planning system service.

The turtlebot will move around the waypoints, exploring the environment. You should output from the planning system, something like:
```
...
KCL: (PS) Dispatching plan
KCL: (PS) Dispatching action [0, goto_waypoint, 10.024417, 10.000000]
KCL: (MoveBase) action recieved
KCL: (PS) Feedback received [0,action enabled]
KCL: (MoveBase) action finished: SUCCEEDED
KCL: (PS) Feedback received [0,action achieved]
...
```
