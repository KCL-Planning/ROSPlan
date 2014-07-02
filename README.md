### About

The ROSPlan framework provides a generic method for task planning in a ROS system.
![Overview of the ROSPlan framework in a system](/overview.png)
The ROSPlan framework performs three major tasks:

#### representing the environment

ROSPlan requires a symbolic representation of the environment. For this reason, sensor data must be parsed and converted into a symbolic format. This part of ROSPlan is known as the Knowledge-base.

There are already existing tools for this task: RoboEarth DB contains a number of object models; OMPL can be used to generate waypoints and trajectories. At its simplest, the Knowledge-base simply acts as an interface for these tools.

However, it is intended to be extensible, and will feature modules to perform translations into symoblic models for some frequently encountered tasks. For example: 2D and 3D path-finding in real environments, searching for objects, and manipulating objects.

#### generating a plan

A plan is a sequence of actions that is estimated to achieve a goal. The planner may optionally optimise a provided metric.

Currently ROSPlan uses the temporal planning language PDDL, version 2.1.

#### dispatching the plan

ROSPlan dispatches actions to possibly many controllers, using the ROS standard actionlib. These controllers form the executor of the plan. The dispatch process is intended to be both flexible and robust.

When the current plan is no longer valid, due to a change in the environment, failed action, or discovery of new information, ROSPlan will reconstruct its model and replan.

Immediate feedback from the executor about action faliure, success, and additional information is taken into account when making this decision. This is the referred to as the short feedback loop.

Sensor data can be continually parsed and passed to ROSPlan. This is used to determine whether the plan will fail sometime in the future, whether further optimisations can be made, or if new goals can be achieved and new actions inserted into the plan.

### Usage

![Diagram of the ROSPlan framework](/framework.png)

#### (A) Planning domain

In the launch file: specify the path to a PDDL2.1 domain.
The following is an example of how to start the ROSPlan framework from a launch file.

```xml
<!-- planning system -->
<node name="planning_ros_system" pkg="planning_ros_system" type="planner" respawn="false" output="screen">

	<!-- path to domain file -->
	<param name="domain_path" value="$(find planning_ros_system)/data/domain.pddl" />
	
	<!-- location to generate problem file -->
	<param name="problem_path" value="$(find planning_ros_system)/data/problem.pddl" />
	
	<!-- directory for other generated files -->
	<param name="data_path" value="$(find planning_ros_system)/data/" />

</node>
```

#### (B) Building the initial environment

**2 ROS services:**

- planning_system/get_type_instances
- planning_system/get_instance_attributes

#### (C) Updating the environment

**2 ROS topics:**

- planning_system/filter
- planning_system/notification

#### (D) Action dispatch and feedback

**2 ROS topics:**

- planning_system/actionDispatch
- planning_system/actionFeedback

#### (E) [optional] Actionlib interface

**2 ROS topics:**

- planning_system/actionDispatch
- planning_system/actionFeedback
