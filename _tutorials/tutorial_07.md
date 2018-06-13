---
title: Tutorial 07 Knowledge Base II
layout: documentation
permalink: /tutorials/tutorial_07
---
	
## 1. Description

This tutorial will cover the ROSPlan **Knowledge Base** node services for fetching current state.

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

## 4.1 Fetching the State Information

Open a second terminal and source the workspace.

Use the following command to see a list of the current state services:

```
rosservice list | grep state
```

## 4.2 Object Instances

The state instance service looks like this:

```
string type_name
---
string[] instances
```

Which is straightforward -- calling the service returns a list of all instances which match the PDDL type. Try calling the service:

```
rosservice call /rosplan_knowledge_base/state/instances "type_name: 'robot'" 
rosservice call /rosplan_knowledge_base/state/instances "type_name: 'waypoint'"
```

## 4.3 Propositions and Functions

The proposition (and function) service looks like this:

```
string predicate_name
---
rosplan_knowledge_msgs/KnowledgeItem[] attributes
```

The predicate (or function) name specified in the request data, and the list of propositions which are true (or functions and their values) which match are returned from the Knowledge Base.

Each proposition and function is represented by a *KnowledgeItem* message. this message type is described fully in the [documentation](../documentation/). Let's see an example of it now by calling the proposition service:

```
rosservice call /rosplan_knowledge_base/state/propositions "predicate_name: 'robot_at'" 
```

This will return all propositions with the matching name in the Knowledge Base:

```
attributes: 
  - 
    knowledge_type: 1
    initial_time: 
      secs: 1528284422
      nsecs: 168676390
    is_negative: False
    instance_type: ''
    instance_name: ''
    attribute_name: robot_at
    values: 
      - 
        key: v
        value: kenny
      - 
        key: wp
        value: wp0
    function_value: 0.0
    optimization: ''
    expr: 
      tokens: []
    ineq: 
      comparison_type: 0
      LHS: 
        tokens: []
      RHS: 
        tokens: []
      grounded: False
```

Let's break this down line by line:

```
    knowledge_type: 1
```
The *knowledge_type* specifies what the *KnowledgeItem* message contains.

```
  uint8 INSTANCE=0
  uint8 FACT=1
  uint8 FUNCTION=2
  uint8 EXPRESSION=3
  uint8 INEQUALITY=4
```

These are the possible options, you can see by looking at the *KnowledgeItem* message type with `rosmsg show`.

```
    initial_time: 
      secs: 1528284422
      nsecs: 168676390
```
The initial time records the time at which this fact became true.

```
    is_negative: False
```

If true, this field specifies that the item explicitly represents a negative fact. In PDDL a proposition not included in the state is already considered false; this field is used when using the knowledge base for contingent and conformant planning.

The instance name and type fields are not used, since we are representing a proposition. Similarly, the function_value, optimisation type, and expression are not needed here.

The attribute name represents the proposition:

```
    attribute_name: robot_at
    values: 
      - 
        key: v
        value: kenny
      - 
        key: wp
        value: wp0
```

In this case, the proposition *(robot_at kenny wp0)*.

## 4.4 Goals

Call the goal service with the following command:

```
rosservice call /rosplan_knowledge_base/state/goals "predicate_name: ''"
```

This time the request data does not matter. The response will be a conjunctive goal, represented by a list of *KnowledgeItem* messages:

```
attributes: 
  - 
    knowledge_type: 1
    initial_time: 
      secs: 0
      nsecs:         0
    is_negative: False
    instance_type: ''
    instance_name: ''
    attribute_name: visited
    values: 
      - 
        key: wp
        value: wp0
    function_value: 0.0
```

## 4.5 Metrics

Call the metric service with the following command:

```
rosservice call /rosplan_knowledge_base/state/metric
```

This time there is no request data. The response is a single knowledge item containing an optimisation metric and an expression. The problem instance we loaded into the Knowledge Base did not have any metric, so this will be empty:

```
metric: 
  knowledge_type: 0
  initial_time: 
    secs: 0
    nsecs:         0
  is_negative: False
  instance_type: ''
  instance_name: ''
  attribute_name: ''
  values: []
  function_value: 0.0
  optimization: ''
  expr: 
    tokens: []
  ineq: 
    comparison_type: 0
    LHS: 
      tokens: []
    RHS: 
      tokens: []
    grounded: False
```

## 4.6 Timed-Initial-Literals

The timed-initial-literals (TILs) and numeric fluents are fetched in the same way as the propositions and functions.

```
rosservice call /rosplan_knowledge_base/state/timed_knowledge "predicate_name: 'robot_at'" 
```

The *initial_time* field of the propositions and functions in the response will be some time in the future.

Open the problem file, *rosplan_demos/common/problem_turtlebot.pddl* and in the initial state paste the following lines:

```
	(at 10 (dock_at wp1))
	(at 20 (not (dock_at wp1)))
	(at 30 (dock_at wp1))
	(at 40 (not (dock_at wp1)))
```

This will create a new dock at waypoint *wp1*, which appears for 10 seconds then disappears, twice.

Restart the launch in the first terminal with `ctrl+c` and:

```
roslaunch tutorial_01.launch
```

Wait for some time. After 10 seconds you will see the first TIL trigger. After 40 seconds the output should look like this:

```
[ INFO] [...32.990845370]: KCL: (KB) Adding fact (dock_at wp1)
[ INFO] [...42.990838447]: KCL: (KB) Removing domain attribute (dock_at)
[ INFO] [...52.990941832]: KCL: (KB) Adding fact (dock_at wp1)
[ INFO] [...62.990843924]: KCL: (KB) Removing domain attribute (dock_at)
```

Try to restart the launch file and call the `state/timed_knowledge` service before all of the TILs trigger.

```
attributes: 
  - 
    knowledge_type: 1
    initial_time: 
      secs: 1528294177
      nsecs: 965704122
    is_negative: True
    instance_type: ''
    instance_name: ''
    attribute_name: dock_at
    values: 
      - 
        key: wp
        value: wp1
    function_value: 0.0
    optimization: ''
    expr: 
      tokens: []
    ineq: 
      comparison_type: 0
      LHS: 
        tokens: []
      RHS: 
        tokens: []
      grounded: False

```
This output shows the final TIL.

## 5. What's Next?

[Tutorial 08: Knowledge Base III](tutorial_08) describes how to use the Knowledge Base services to update the current state.

[Tutorial 09: Knowledge Base IV](tutorial_09) describes how to perform queries on the current state in the knowledge base.
