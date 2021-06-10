---
layout: documentation
title: Plan Dispatch
---

![PI](../images/rosplan_dispatch_interface.png){: .big_chart }

Plan Dispatch includes plan execution, and the process of connecting single actions to the processes which are responsible for their execution. An implementation of the Plan Dispatch node subscribes to a plan topic, and is closely tied to the plan representation of plans published on that topic. The plan is executed as a service, which returns true if the plan was executed without errors, and can be preempted.

## Contents:

- Launch
- Services
- Publishers

## Launching Plan Dispatch

There are two implementations of the Parsing Interface provided: *pddl_simple_plan_dispatcher*, and *pddl_esterel_plan_dispatcher*. See the [plan_parsing](03_parsing_interface) node for more information on these representations, and the corresponding message type of the *plan_topic*.

An example launch file showing all the parameters is shown below:

```xml
<launch>

	<!-- arguments -->
	<arg name="node_name"                default="rosplan_plan_dispatcher" />
	<arg name="knowledge_base"           default="rosplan_knowledge_base" />
	<arg name="plan_topic"               default="/rosplan_parsing_interface/complete_plan" />
	<arg name="action_dispatch_topic"    default="action_dispatch" />
	<arg name="action_feedback_topic"    default="action_feedback" />


	<!-- plan dispatching -->
	<node name="$(arg node_name)" pkg="rosplan_planning_system" type="pddl_esterel_plan_dispatcher" respawn="false" output="screen">
		<param name="knowledge_base"        value="$(arg knowledge_base)" />
		<param name="plan_topic"            value="$(arg plan_topic)" />
		<param name="action_dispatch_topic" value="$(arg action_dispatch_topic)" />
		<param name="action_feedback_topic" value="$(arg action_feedback_topic)" />
	</node>

</launch>
```

## Services

Topic: **dispatch_plan**  
Type: *rosplan_dispatch_msgs/DispatchService*  
Begins the execution of the last plan message that was recieved on the *plan_topic*. The service returns when the plan has completed successfuly, or has failed.

Topic: **cancel_plan**  
Type: *std_srvs/Empty*  
Attempt to cancel the plan execution currently underway.

## Publishers

Topic: **action_dispatch**  
Type: *rosplan_dispatch_msgs/ActionDispatch*  
Each action to be executed will be published on this topic.

Topic: **action_feedback**  
Type: *rosplan_dispatch_msgs/ActionFeedback*  
Action feedback is published on this topic. The *plan_dispatch* node will also publish to this topic to provide feedback for actions which were not dispatched due to their preconditions not being achieved.
