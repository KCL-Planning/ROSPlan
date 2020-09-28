---
title: Tutorial 01 Problem Generation
layout: documentation
permalink: /tutorials/tutorial_01
output: true
---

## 1. Description

This tutorial will teach how to use ROSPlan to generate PDDL problems.

![Problem Generation node diagram](./tutorial_01.png)

## 2. Prior Setup

This tutorial assumes that ROSPlan is already installed, following the instructions on the gihtub README:
[https://github.com/KCL-Planning/ROSPlan](https://github.com/KCL-Planning/ROSPlan).

## 3.1 Launch File

Change directory to the  ROSPlan workspace.

Create a new launch file, *tutorial_01.launch*, in the current directory and paste the following inside it:

```xml
<?xml version="1.0"?>
<launch>

	<!-- arguments -->
	<arg name="domain_path"	default="$(find rosplan_demos)/common/domain_turtlebot.pddl" />
	<arg name="problem_path"	default="$(find rosplan_demos)/common/problem_turtlebot.pddl" />

	<!-- knowledge base -->
	<node name="rosplan_knowledge_base" pkg="rosplan_knowledge_base" type="knowledgeBase" respawn="false" output="screen">
		<param name="domain_path" value="$(arg domain_path)" />
		<param name="problem_path" value="$(arg problem_path)" />
		<!-- conditional planning flags -->
		<param name="use_unknowns" value="false" />
	</node>

	<!-- problem generation -->
	<include file="$(find rosplan_planning_system)/launch/includes/problem_interface.launch">
		<arg name="knowledge_base"   value="rosplan_knowledge_base" />
		<arg name="domain_path"      value="$(arg domain_path)" />
		<arg name="problem_path"     value="$(find rosplan_demos)/common/problem.pddl" />
		<arg name="problem_topic"    value="problem_instance" />
	</include>
</launch>
```

## 3.2 The Launch File Explained

The launch file will start 2 ROSPlan nodes.

```xml
	<!-- arguments -->
	<arg name="domain_path"	default="$(find rosplan_demos)/common/domain_turtlebot.pddl" />
	<arg name="problem_path"	default="$(find rosplan_demos)/common/problem_turtlebot.pddl" />

	<!-- knowledge base -->
	<node name="rosplan_knowledge_base" pkg="rosplan_knowledge_base" type="knowledgeBase" respawn="false" output="screen">
		<param name="domain_path" value="$(arg domain_path)" />
		<param name="problem_path" value="$(arg problem_path)" />
		<!-- conditional planning flags -->
		<param name="use_unknowns" value="false" />
	</node>
```

The first ROSPlan node started is the **Knowledge Base**. This node stores the PDDL model: both the domain and the current state.

The Knowledge Base node takes 4 parameters:
1. `domain_path` is required and specifies a PDDL domain file. You can follow the path to view the domain file.
2. `problem_path` is an optional parameter to load an initial state. If this parameter is not set, then the state will contain no objects, any propositions will be false, and all functions will be initialised to zero.
3. `use_unknowns` is used for conditional planning. If false, then a proposition not added to the Knowledge Base is assumed to be false in the initial state. If true, then it is assumed to be unknown; false propositions have to be explicitly stated.


```xml
	<!-- problem generation -->
	<include file="$(find rosplan_planning_system)/launch/includes/problem_interface.launch">
		<arg name="knowledge_base"   value="rosplan_knowledge_base" />
		<arg name="domain_path"      value="$(arg domain_path)" />
		<arg name="problem_path"     value="$(find rosplan_demos)/common/problem.pddl" />
		<arg name="problem_topic"    value="problem_instance" />
	</include>
```

The second ROSPlan node started is the **Problem Interface**, which is launched from the included launch file.

The Problem Interface also has 4 parameters:
1. `knowledge_base` specifies the node name of the Knowledge Base node that stores the current state.
2. `domain_path` specifies the path to the domain file used by that Knowledge Base. In our launch file, we've declared the domain path as an argument at the top.
3. `problem_path` specifies the path into which the new problem file will be written.
4. `problem_topic` specifies the topic name on which the problem will be published. In our launch file we've made it relative to the node name: *rosplan_problem_interface/problem_instance*.


## 3.3 Launching

From the terminal, launch the file:

```
roslaunch tutorial_01.launch
```

You should see the output from ROSPlan, identified by the prefix "KCL":

```
KCL: (KB) Parsing domain
KCL: (KB) Parsing domain: /home/michael/ros_indigo/turtlebot/src/rosplan/rosplan_demos/common/domain_turtlebot.pddl.
KCL: (KB) Parsing initial state
KCL: (KB) Parsing Problem File: /home/michael/ros_indigo/turtlebot/src/rosplan/rosplan_demos/common/problem_turtlebot.pddl.
KCL: (/rosplan_problem_interface) Ready to receive
KCL: (KB) Ready to receive
```

## 3.4 Generating a Problem

Open another terminal and source the workspace.

```
source devel/setup.bash
```

In the second terminal, take a look at the node list and service list to see what is running:

```
rosnode list
rosservice list
```

In particular you should see the nodes: */rosplan_knowledge_base* and */rosplan_problem_interface*.

Then call the problem generation service (using tab complete is helpful)

```
rosservice call /rosplan_problem_interface/problem_generation_server
```

There will be no output in the second terminal, but in the first terminal you should see the following lines:

```
KCL: (/rosplan_problem_interface) (problem.pddl) Generating problem file.
KCL: (/rosplan_problem_interface) (problem.pddl) The problem was generated.
```

## 3.5 Viewing at the Problem

What does the problem look like?

It should look exectly like the problem that was loaded into the Knolwedge Base with the `problem_path` parameter. Nothing has changed, after all!

You can find the newly generated problem file in two places:

**A.** The file saved in the `problem_path` parameter of the problem_interface node. If you are looking for a file, this is where it is saved.

```
cat src/rosplan_demos/rosplan_demos/common/problem.pddl
```

**B.** Published on the topic specified by the `problem_topic` parameter. Use this command to *echo* the contents of that topic. The flag *-n 1* means that only one message will be printed.

```
rostopic echo /rosplan_problem_interface/problem_instance -n 1 -p
```

## 4. What's Next?

Passing the problem to a planner to produce a plan. The next tutorial: [Tutorial 02: Planning](tutorial_02) 
