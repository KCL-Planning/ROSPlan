---
layout: documentation
title: Problem Interface
---

![PI](../images/rosplan_problem_interface.png){: .big_chart }

The Problem Interface node is used to generate a PDDL problem instance by querying the current state in a Knowledge Base. The PDDL problem instance is published as a string and written to a file.</p>

## Contents:

- Launch
- Services
- Publishers

## Launching a Problem Interface

An example launch file showing all the parameters is shown below:

```xml
<launch>

	<!-- arguments -->
	<arg name="node_name" 	     default="rosplan_problem_interface" />
	<arg name="knowledge_base"   default="rosplan_knowledge_base" />
	<arg name="domain_path"      default="$(find rosplan_demos)/common/domain_turtlebot_demo.pddl" />
	<arg name="problem_path"     default="$(find rosplan_demos)/common/problem.pddl" />
	<arg name="problem_topic"    default="problem_instance" />


	<!-- problem generation -->
	<node name="$(arg node_name)" pkg="rosplan_planning_system" type="problemInterface" respawn="false" output="screen">
		<param name="knowledge_base" value="$(arg knowledge_base)" />
		<param name="domain_path"    value="$(arg domain_path)" />
		<param name="problem_path"   value="$(arg problem_path)" />
		<param name="problem_topic"  value="$(arg problem_topic)" />
	</node>

</launch>
```

## Services

Topic: **problem_generation_server**  
Type: *std_srvs/Empty*  
The planning_server is called, overriding the node parameters with the arguments of the service.  
Generates a PDDL problem by querying the current state from a Knowledge Base, saving the problem to file with the *problem_path* parameter, and publishing the problem as a string on the topic specified by the *problem_topic* parameter.



Topic: **problem_generation_server_params**  
Type: *rosplan_dispatch_msgs/ProblemService*  
Generates a PDDL problem by querying the current state from a Knowledge Base, overriding the node parameters.

|**Field**               |**Description**|
|problem_path            |The generated problem will be saved to this path.|
|problem_string_response |If true, then the generated problem will also be returned in the *problem_string* response field.|
|problem_generated       |True if the problem was generated successfuly.|
|problem_string          |Contains the generated problem if the *problem_string_response* field is true.|


## Publishers

Topic: **problem_instance**  
Type: *std_msgs/String*  
The PDDL problem instance is published on this topic.

