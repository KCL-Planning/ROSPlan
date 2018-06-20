---
title: Tutorial 06 Knowledge Base I
layout: documentation
permalink: /tutorials/tutorial_06
---

	
## 1. Description

This tutorial will cover the ROSPlan **Knowledge Base** node services for fetching PDDL domain information.

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

## 4.1 Fetching the Domain Information

Open a second terminal and source the workspace.

Use the following command to see a list of the domain services:

```
rosservice list | grep domain
```

## 4.2 Domain Name

Try fetching the domain name for the current domain:

```
rosservice call /rosplan_knowledge_base/domain/name
```

## 4.3 Types

The domain type service looks like this:

```
---
string[] types
string[] super_types
```

There is no request data. The response data is an array of types and array of corresponding super types. Call the Knowledge Base to fetch the list of types for the current domain:

```
rosservice call /rosplan_knowledge_base/domain/types
```

## 4.4 Predicates and Functions

The domain predicate (and function) service looks like this:

```
---
rosplan_knowledge_msgs/DomainFormula[] items
  string name
  diagnostic_msgs/KeyValue[] typed_parameters
    string key
    string value
```

There is no request data. The response data is an array of *DomainFormula* messages, each describing one PDDL predicate or function. For example, in the turtlebot domain we have the PDDL predicate *(robot_at ?v - robot ?wp waypoint)*, which will be represented as one domain formula.

Call the predicate service with the following command:

```
rosservice call /rosplan_knowledge_base/domain/predicates
```

You can see that the *robot_at* predicate is part of the response list:

```
items: 
  - 
    name: robot_at
    typed_parameters: 
      - 
        key: v
        value: robot
      - 
        key: wp
        value: waypoint
  - 
    name: visited
    typed_parameters: 
      - 
        key: wp
        value: waypoint
```

## 4.5 Operators

There is a service to fetch the list of operators, and another service to fetch the details of a single operator. Call the first using the following command to see the list of operators:

```
rosservice call /rosplan_knowledge_base/domain/operators
```

Take a look at the service information to discover the service type, and then look at the service definition using the following commands:

```
rosservice info /rosplan_knowledge_base/domain/operators
rossrv show rosplan_knowledge_msgs/GetDomainOperatorService
```

The response is a list of domain formula. In this case the operator name and parameters make up the formula. For example, you should have seen the output representing *(goto_waypoint ?v - robot ?from ?to - waypoint)*:

```
operators: 
  - 
    name: goto_waypoint
    typed_parameters: 
      - 
        key: v
        value: robot
      - 
        key: from
        value: waypoint
      - 
        key: to
        value: waypoint
```

## 4.6 Operator Details

Calling the second service will show the conditions, duration, and effects of the operator. You can view the structure of this message with the following command:

```
rossrv show rosplan_knowledge_msgs/GetDomainOperatorDetailsService
```

You can see that it is much more complicated. Let's break it down line-by-line.

```
string name
---
```

The operator name should be supplied in the request data.

```
rosplan_knowledge_msgs/DomainOperator op
```

The operator details are contained within a DomainOperator message, broken down below:

```
  rosplan_knowledge_msgs/DomainFormula formula
    string name
    diagnostic_msgs/KeyValue[] typed_parameters
      string key
      string value
```

The first part is the domain formaula showing the operator name and parameters.

```
  rosplan_knowledge_msgs/DomainFormula[] at_start_add_effects
    string name
    diagnostic_msgs/KeyValue[] typed_parameters
      string key
      string value
```

Then there are *at start add effects*. These correspond to the *at start* propositional add effects of the operator. Just like the proposition which is added, the effect is represented as a domain formula, however note that the key value of the typed parameters will be operator parameter label and not the predicate label -- we will see an example of this soon.

```
  rosplan_knowledge_msgs/DomainFormula[] at_start_del_effects
  rosplan_knowledge_msgs/DomainFormula[] at_end_add_effects
  rosplan_knowledge_msgs/DomainFormula[] at_end_del_effects
```

The start and end add and delete effects are represented in the same way.

The start (and end) assignment effects, such as *(= (function_name param-1 ... param-n) A)*, are represented using a *rosplan_knowledge_msgs/DomainAssignment* message:

```
  rosplan_knowledge_msgs/DomainAssignment[] at_start_assign_effects
    uint8 ASSIGN=0
    uint8 INCREASE=1
    uint8 DECREASE=2
    uint8 SCALE_UP=3
    uint8 SCALE_DOWN=4
    uint8 ASSIGN_CTS=5
    uint8 assign_type
    rosplan_knowledge_msgs/DomainFormula LHS
    rosplan_knowledge_msgs/ExprComposite RHS
    bool grounded
```

- The type of assignment (increase, descrease, assign, etc.) is represented with a short int.
- The left-hand side is a domain formula describing the function.
- The right-hand side is the expression that is assigned to the function, represented using the *rosplan_knowledge_msgs/ExprComposite* message type. This message is described in the [documentation](http://kcl-planning.github.io/ROSPlan/documentation/).

The conditions of the operator come next: 

```
  rosplan_knowledge_msgs/DomainFormula[] at_start_simple_condition
    string name
    diagnostic_msgs/KeyValue[] typed_parameters
      string key
      string value
```

The simple propositional conditions are represented in the same way as the simple effects.

Call this service now for *goto_waypoint*.

```
rosservice call /rosplan_knowledge_base/domain/operator_details "name: 'goto_waypoint'" 
```

The output should look like this:

```
op: 
  formula: 
    name: goto_waypoint
    typed_parameters: 
      - 
        key: v
        value: robot
      - 
        key: from
        value: waypoint
      - 
        key: to
        value: waypoint
  at_start_add_effects: []
  at_start_del_effects: 
    - 
      name: robot_at
      typed_parameters: 
        - 
          key: v
          value: robot
        - 
          key: from
          value: waypoint
  at_end_add_effects: 
    - 
      name: robot_at
      typed_parameters: 
        - 
          key: v
          value: robot
        - 
          key: to
          value: waypoint
    - 
      name: visited
      typed_parameters: 
        - 
          key: to
          value: waypoint
  at_end_del_effects: []
  at_start_assign_effects: []
  at_end_assign_effects: []
  at_start_simple_condition: 
    - 
      name: robot_at
      typed_parameters: 
        - 
          key: v
          value: robot
        - 
          key: from
          value: waypoint
    - 
      name: localised
      typed_parameters: 
        - 
          key: v
          value: robot
  over_all_simple_condition: 
    - 
      name: undocked
      typed_parameters: 
        - 
          key: v
          value: robot
  at_end_simple_condition: []
  at_start_neg_condition: []
  over_all_neg_condition: []
  at_end_neg_condition: []
```

Compare this to the operator definition in the domain file:

```
;; Move to any waypoint, avoiding terrain
(:durative-action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:duration ( = ?duration 60)
	:condition (and
		(at start (robot_at ?v ?from))
		(at start (localised ?v))
		(over all (undocked ?v)))
	:effect (and
		(at end (visited ?to))
		(at end (robot_at ?v ?to))
		(at start (not (robot_at ?v ?from))))
)
```
	
## 5. What's Next?

[Tutorial 07: Knowledge Base II](tutorial_07) describes how to use the Knowledge Base services to fetch state information.

[Tutorial 08: Knowledge Base III](tutorial_08) describes how to use the Knowledge Base services to update the current state.

[Tutorial 09: Knowledge Base IV](tutorial_09) describes how to perform queries on the current state in the knowledge base.
