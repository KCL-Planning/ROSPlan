ROSPlan Framework
=================

The main ROSPlan website and documentation is available here:
http://kcl-planning.github.io/ROSPlan/

The ROSPlan framework provides a generic method for task planning in a ROS system. ROSPlan encapsulates both planning and dispatch. It possesses a simple interface, and already includes interfaces to common ROS libraries.

### Installation

Get the prerequisites:

(for Indigo)
```sh
sudo apt-get install flex ros-indigo-mongodb-store ros-indigo-tf2-bullet freeglut3-dev
```
(for Hydro)
```sh
sudo apt-get install flex ros-hydro-mongodb-store ros-hydro-tf2-bullet freeglut3-dev
```
Select a catkin workspace or create a new one:
```sh
mkdir -p ROSPlan/src
cd ROSPlan/
```
Get the code:
```sh
cd src/
git clone https://github.com/clearpathrobotics/occupancy_grid_utils
git clone https://github.com/KCL-Planning/rosplan
# optionally get the turtlebot interface
git clone https://github.com/KCL-Planning/rosplan_interface_turtlebot2
```
Compile everything:
```sh
source /opt/ros/hydro/setup.bash
catkin_make
```

### Running a demo with the turtlebot

The turtlebot demo is now a simple exploration mission. The turtlebot will visit randomly generated waypoints around a map.

The domain for this demo is in the `rosplan_demos` package, as `common/domain_turtlebot_demo.pddl`.

To run the demo first follow the installation instructions and quick-start guide for the Turtlebot Simulator and Gazebo:

[Turtlebot Gazebo](http://wiki.ros.org/turtlebot_gazebo) 

[Turtlebot Simulator](http://wiki.ros.org/turtlebot_simulator) 

Then source the ROSPlan workspace in two terminals.

*1.* In the first terminal, begin the simulation, rviz visualisation, and ROSPlan nodes using the `turtlebot.launch` from the rosplan_demos package:
```
roslaunch rosplan_demos turtlebot.launch
```

*2.* In the second terminal run `turtlebot_explore.bash`, a script which
- adds to the knowledge base the PDDL objects and facts which comprise the initial state;
- adds the goals to the knowledge base; and
- calls the ROSPlan services which generate a plan and dispatch it.
```
rosrun rosplan_demos turtlebot_explore.bash
```

You should see the following output from the script:
```
waypoints: ['wp0', 'wp1', 'wp2', 'wp3', 'wp4', 'wp5']
Adding initial state and goals to knowledge base.
success: True
success: True
Calling problem generator.
Calling planner interface.
Calling plan parser.
Calling plan dispatcher.
Finished!
```

The turtlebot will move around the waypoints, exploring the environment. You should see output in the first terminal, something like:
```
...
KCL: (/rosplan_problem_interface) (problem.pddl) Generating problem file.
KCL: (/rosplan_problem_interface) (problem.pddl) The problem was generated.
KCL: (/rosplan_planner_interface) Problem recieved.
KCL: (/rosplan_planner_interface) (problem.pddl) Writing problem to file.
KCL: (/rosplan_planner_interface) (problem.pddl) Running: timeout 10 /home/michael/ros_indigo/turtlebot/src/rosplan/rosplan_planning_system/common/bin/popf /home/michael/ros_indigo/turtlebot/src/rosplan/rosplan_demos/common/domain_turtlebot_demo.pddl /home/michael/ros_indigo/turtlebot/src/rosplan/rosplan_demos/common/problem.pddl > /home/michael/ros_indigo/turtlebot/src/rosplan/rosplan_demos/common/plan.pddl
KCL: (/rosplan_planner_interface) (problem.pddl) Planning complete
KCL: (/rosplan_planner_interface) (problem.pddl) Plan was solved.
KCL: (/rosplan_parsing_interface) Planner output recieved.
KCL: (/rosplan_parsing_interface) Parsing planner output.
KCL: (/rosplan_plan_dispatcher) Plan recieved.
KCL: (/rosplan_plan_dispatcher) Dispatching plan.
KCL: (/rosplan_plan_dispatcher) Dispatching action [0, goto_waypoint, 0.804106, 10.000000]
KCL: (/rosplan_plan_dispatcher) Feedback received [0, action enabled]
...
```

### Related repositories:

Automatic localisation and docking action interfaces with the Turtlebot 2 (Kobuki base) 
https://github.com/KCL-Planning/ROSPlan_interface_Turtlebot2

Integration with the Component Oriented Layered-base Architecture for Autonomy (COLA2). Developed in the Research Center of Underwater Robotics (CIRS) in the University of Girona (UdG). This architecture is used to control the Autonomous Underwater Vehicles (AUVs) developed in this center. (https://bitbucket.org/udg_cirs/cola2)
https://github.com/KCL-Planning/ROSPlan_interface_COLA2

Action interfaces for piloting a quadrotor from Jindrich Vodrazka, (takeoff, land, fly_square, and fly_waypoint).
https://github.com/fairf4x/ROSPlan_interface_quadrotor
