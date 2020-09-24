---
title: Tutorial 08 Knowledge Base III
layout: documentation
permalink: /tutorials/tutorial_08
---

	
## 1. Description

This tutorial will cover the ROSPlan **Knowledge Base** node services for updating the current state.

## 2. Prior Setup

This tutorial assumes that you have have already followed [Tutorial 01: Problem Generation](tutorial_01), and will use the same launch files and scripts.

## 3. Launching

Change directory to the  ROSPlan workspace.

Launch the **Knowledge Base** node:

```
roslaunch tutorial_01.launch
```

You know that the node is ready when you see the output:

```
KCL: (KB) Ready to receive
```

## 4.1 Updating the State Information

Open a second terminal and source the workspace.

There are three services we can call.

```
/rosplan_knowledge_base/clear
/rosplan_knowledge_base/update
/rosplan_knowledge_base/update_array
```

## 4.2 Clearing the Knowledge Base

Start by calling the clear service:

```
rosservice call /rosplan_knowledge_base/clear
```

In the first terminal you will see this output:

```
KCL: (KB) Removing whole model
```

Try generating a new problem file, and then view it:

```
rosservice call /rosplan_problem_interface/problem_generation_server
rostopic echo /rosplan_problem_interface/problem_instance -n 1 -p
```

It should be completely empty now.

## 4.3 Adding One Item to the Knowledge Base

Use the following command to look at the update service information:

```
rosservice info /rosplan_knowledge_base/update
```

You should see that the serice uses the type *rosplan_knowledge_msgs/KnowledgeUpdate*. View this now:

```
rossrv show rosplan_knowledge_msgs/KnowledgeUpdateService
```

Not including the body of KnowledgeItem, it looks like this:

```
uint8 ADD_KNOWLEDGE=0
uint8 ADD_GOAL=1
uint8 REMOVE_KNOWLEDGE=2
uint8 REMOVE_GOAL=3
uint8 ADD_METRIC=4
uint8 REMOVE_METRIC=5
uint8 update_type
rosplan_knowledge_msgs/KnowledgeItem knowledge
---
bool success
```

The `update_type` request field specifies the kind of update that's being performed, while the `knowledge` request field holds the actual state information. The KnowledgeItem message type is described fully in the documentation.

Let's add a new object instance to the Knowledge Base.

Create a new file in the workspace, called *update.bash*, and copy in the following code:

```
#!bin/bash
rosservice call /rosplan_knowledge_base/update "update_type: 0
knowledge:
  knowledge_type: 0
  instance_type: 'robot'
  instance_name: 'kenny'" 
```

This will perform a service call with update type `ADD_KNOWLEDGE`, and knowledge type `INSTANCE`. The instance type is *robot* and the instance name is *kenny*.

Give the script permission to be executed, and execute it to call the service.

```
chmod 755 update.bash
./update.bash
```

In the first terminal, you should see output like this:

```
KCL: (KB) Adding instance (robot, kenny)
```

Try generating a new problem file, and then view it:

```
rosservice call /rosplan_problem_interface/problem_generation_server
rostopic echo /rosplan_problem_interface/problem_instance -n 1 -p
```

There is now one robot.

## 4.4 Adding Many Items to the Knowledge Base

Use the following command to look at the **array** update service type:

```
rossrv show rosplan_knowledge_msgs/KnowledgeUpdateServiceArray
```

Not including the body of *KnowledgeItem*, it looks like this:

```
uint8 ADD_KNOWLEDGE=0
uint8 ADD_GOAL=1
uint8 REMOVE_KNOWLEDGE=2
uint8 REMOVE_GOAL=3
uint8 ADD_METRIC=4
uint8 REMOVE_METRIC=5
uint8[] update_type
rosplan_knowledge_msgs/KnowledgeItem[] knowledge
---
bool success
```

It is almost the same as the regular update service, but the update_type and corresponding knowledge items are stored in an array. This allows you to make many edits to the knowledge base with a single service call. The items will be processed in the order they appear in the array.

Open *update.bash*, delete everything, and paste the following code:

```
#!bin/bash
update_type="update_type:"
knowledge="knowledge:"

# ADD WAYPOINT INSTANCES
for i in $(seq 0 9); do
update_type="$update_type
- 0";
knowledge="$knowledge
- knowledge_type: 0
  instance_type: 'waypoint'
  instance_name: 'wp$i'";
done

# ADD DOCK_AT
update_type="$update_type
- 0";
knowledge="$knowledge
- knowledge_type: 1
  attribute_name: 'dock_at'
  values:
  - {key: 'wp', value: 'wp0'}"

# ADD ROBOT_AT
update_type="$update_type
- 0";
knowledge="$knowledge
- knowledge_type: 1
  attribute_name: 'robot_at'
  values:
  - {key: 'wp', value: 'wp0'}"

rosservice call /rosplan_knowledge_base/update_array "
$update_type
$knowledge";
```
This code creates an array for the `update_type`, which are all 0 for `ADD_KNOWLEGE`. It also creates an array of knowledge items, beginning with 10 waypoint instances, then two propositions: *(dock_at wp0)* and *(robot_at wp0)*.

Try generating a new problem file, and then view it:

```
rosservice call /rosplan_problem_interface/problem_generation_server
rostopic echo /rosplan_problem_interface/problem_instance -n 1 -p
```

You should see the new information in the initial state of the problem file.

It is not necessary to use bash scripts to update a Knowledge Base. For more information on how to call services from code, take a look at the ROS tutorials for [c++](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient) and [python](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29).

## 5. What's Next?

[Tutorial 09: Knowledge Base IV](tutorial_09) describes how to perform queries on the current state in the knowledge base.
