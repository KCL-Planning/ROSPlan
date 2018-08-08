---
title: Tutorial 09 Knowledge Base IV
layout: documentation
permalink: /tutorials/tutorial_09
---

## 1. Description

This tutorial will cover the ROSPlan **Knowledge Base** node service for querying the current state.

## 2. Prior Setup

This tutorial assumes that you have have already followed [Tutorial 01: Problem Generation](../01_problem_generation/tutorial_01.html), and will use the same launch files and scripts.

Change directory to the  ROSPlan workspace.

We will use a new domain and problem file that includes some functions. Create a new file in the workspace called *tutorial_09_domain.pddl* and paste in the following code:

```
(define (domain turtlebot_energy)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
	waypoint 
	robot
)

(:predicates
	(robot_at ?v - robot ?wp - waypoint)
	(visited ?wp - waypoint)
	(undocked ?v - robot)
	(docked ?v - robot)
	(localised ?v - robot)
	(dock_at ?wp - waypoint)
)

(:functions
	(energy ?v - robot)
)

;; Move to any waypoint, avoiding terrain
(:durative-action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:duration ( = ?duration 60)
	:condition (and
		(at start (robot_at ?v ?from))
		(at start (localised ?v))
		(over all (> (energy ?v) 0))
		(over all (undocked ?v)))
	:effect (and
		(at end (visited ?to))
		(at end (robot_at ?v ?to))
		(at end (decrease (energy ?v) 60))
		(at start (not (robot_at ?v ?from))))
)

;; Localise
(:durative-action localise
	:parameters (?v - robot)
	:duration ( = ?duration 60)
	:condition (over all (undocked ?v))
	:effect (at end (localised ?v))
)

;; Dock to charge
(:durative-action dock
	:parameters (?v - robot ?wp - waypoint)
	:duration ( = ?duration 30)
	:condition (and
		(over all (dock_at ?wp))
		(at start (robot_at ?v ?wp))
		(at start (undocked ?v)))
	:effect (and
		(at end (docked ?v))
		(at start (not (undocked ?v))))
)

(:durative-action undock
	:parameters (?v - robot ?wp - waypoint)
	:duration ( = ?duration 10)
	:condition (and
		(over all (dock_at ?wp))
		(at start (docked ?v)))
	:effect (and
		(at start (not (docked ?v)))
		(at end (undocked ?v))
		(at end (assign (energy ?v) 120)))
)
)
```
This adds the function *(energy ?v - robot)* to the turtlebot domain.

Create another new file in the workspace called *tutorial_09_problem.pddl* and paste in the following code:

```
(define (problem task)
(:domain turtlebot_energy)
(:objects
    wp0 wp1 wp2 wp3 wp4 - waypoint
    kenny - robot
)
(:init
    (robot_at kenny wp0)
    (docked kenny)
    (dock_at wp0)

	(= (energy kenny) 120)

)
(:goal (and
    (visited wp0)
    (visited wp1)
    (visited wp2)
    (visited wp3)
    (visited wp4)
)))
```

## 3. Launching

Launch the **Knowledge Base** node (adding the proper paths to your workspace):

```
roslaunch tutorial_01.launch domain_path:=[WORKSPACE PATH]/tutorial_09_domain.pddl problem_path:=[WORKSPACE PATH]/tutorial_09_problem.pddl
```

You know that the node is ready when you see the output:

```
KCL: (KB) Ready to receive
```

## 4.1 The Query Service

Open a second terminal and source the workspace.

The service we want to call is */rosplan_knowledge_base/query_state*.  The service uses the type *rosplan_knowledge_msgs/KnowledgeQueryService*, which looks like this:

```
rosplan_knowledge_msgs/KnowledgeItem[] knowledge
---
bool all_true
bool[] results
rosplan_knowledge_msgs/KnowledgeItem[] false_knowledge
```

An array of *KnowledgeItem* messages are included as request data, and the response data will return a corresponding Boolean array describing which KnowledgeItems are true in the current state. The KnowledgeItem message type is described fully in the documentation, and represents either:

- the existence of an object instance
- a PDDL proposition, e.g. *(robot_at kenny wp0)*
- an equality on a PDDL function, e.g. *(= (energy kenny) 120)*
- an inequality between two expressions, e.g. *(> 10 (+ 5 (energy kenny)))*

## 4.2 Writing a Query Service Client (Python)

Writing queries using a terminal is laborious, so instead we will write a short python program that will call the query service. Create a new file in the workspace, called *query_client.py* and paste in the following code:

```python
#!/usr/bin/env python

import sys
import rospy
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *

query = []

def call_service():
    print "Waiting for service"
    rospy.wait_for_service('/rosplan_knowledge_base/query_state')
    try:
        print "Calling Service"
        query_proxy = rospy.ServiceProxy('rosplan_knowledge_base/query_state', KnowledgeQueryService)
        resp1 = query_proxy(query)
        print "Response is:", resp1.results
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":

    # QUERY 1 (robot_at kenny wp0)
    query1 = KnowledgeItem()
    query1.knowledge_type = KnowledgeItem.FACT
    query1.attribute_name = "robot_at"
    query1.values.append(diagnostic_msgs.msg.KeyValue("v", "kenny"))
    query1.values.append(diagnostic_msgs.msg.KeyValue("wp", "wp0"))
    query.append(query1)

    call_service()
    sys.exit(1)
```

In the second terminal, run the program using the command:

```
python query_client.py
```

You should see the output like this:

```
Waiting for service
Calling Service
Response is: [True]
```

## 4.2 Adding Another Query

Add the following query to the *query_client.py* by pasting the following code before the line `call_service()`.

```python
    # QUERY 2 (robot_at kenny wp3)
    query2 = KnowledgeItem()
    query2.knowledge_type = KnowledgeItem.FACT
    query2.attribute_name = "robot_at"
    query2.values.append(diagnostic_msgs.msg.KeyValue("v", "kenny"))
    query2.values.append(diagnostic_msgs.msg.KeyValue("wp", "wp3"))
    query.append(query2)
```

Call the service again, and you should see:

```
Waiting for service
Calling Service
Response is: [True, False]
```

## 4.3 Creating an Inequality Query

Inequalities are represented in the KnowledgeItem using the message type *rosplan_knowledge_base/DomainInequality*. The message looks like this:

```
uint8 GREATER=0
uint8 GREATEREQ=1
uint8 LESS=2
uint8 LESSEQ=3
uint8 EQUALS=4

uint8 comparison_type
rosplan_knowledge_msgs/ExprComposite LHS
rosplan_knowledge_msgs/ExprComposite RHS
bool grounded
```

Where comparison_type determines the type of inequality (i.e., greater than, equal to, etc.) and the left hand and right hand side of the inequality are represented by the message type *ExprComposite*.

The message type *ExprComposite* represents an expression in prefix notation. The message is an ordered list of operators and operands, each represented using the *ExprBase* message type.

```
  rosplan_knowledge_msgs/ExprBase[] tokens
```

The *ExprBase* message type looks like this:

```
# expression types
uint8 CONSTANT = 0
uint8 FUNCTION = 1
uint8 OPERATOR = 2
uint8 SPECIAL  = 3

# operators
uint8 ADD    = 0
uint8 SUB    = 1
uint8 MUL    = 2
uint8 DIV    = 3
uint8 UMINUS = 4

# special types
uint8 HASHT      = 0
uint8 TOTAL_TIME = 1
uint8 DURATION   = 2

# expression base type
uint8 expr_type

# constant value
float64 constant

# function
rosplan_knowledge_msgs/DomainFormula function

# operator
uint8 op

# special
uint8 special_type
```

We will investigate these message types using a new query. We will query the inequality *(> 125 (+ 5 (energy kenny)))*. First paste the new query below into *query_client.py*, and then we will break down the code, line by line.

```python
    # QUERY 3 (robot_at kenny wp3)
    query3 = KnowledgeItem()
    query3.knowledge_type = KnowledgeItem.INEQUALITY
    query3.ineq.comparison_type = DomainInequality.GREATER
    query3.ineq.grounded = True

    token1 = ExprBase()
    token1.expr_type = ExprBase.CONSTANT
    token1.constant = 125
    query3.ineq.LHS.tokens.append(token1)

    token2 = ExprBase()
    token2.expr_type = ExprBase.OPERATOR
    token2.op = ExprBase.ADD

    token3 = ExprBase()
    token3.expr_type = ExprBase.CONSTANT
    token3.constant = 5

    token4 = ExprBase()
    token4.expr_type = ExprBase.FUNCTION
    token4.function.name = "energy"
    token4.function.typed_parameters.append(diagnostic_msgs.msg.KeyValue("v", "kenny"))

    query3.ineq.RHS.tokens.append(token2)
    query3.ineq.RHS.tokens.append(token3)
    query3.ineq.RHS.tokens.append(token4)

    query.append(query3)
```

The *KnowlegeItem* for this query is set to be an inequality, using the *greater than* comparison type, and is grounded:

```
    query3 = KnowledgeItem()
    query3.knowledge_type = KnowledgeItem.INEQUALITY
    query3.ineq.comparison_type = DomainInequality.GREATER
    query3.ineq.grounded = True
```

The left hand side of the inequality is a single constant expression (125). This is set using a single *ExprBase* message of the `CONSTANT` type.

```
    token1 = ExprBase()
    token1.expr_type = ExprBase.CONSTANT
    token1.constant = 125
```

The constant (125) is pushed into the token list for the left hand side:

```
    query3.ineq.LHS.tokens.append(token1)
```

The right hand side of the inequality is an expression with 3 tokens. In prefix notation they will be: *+*, *5*, *(energy kenny)*. The first *ExprBase* represents the addition token:

```
    token2 = ExprBase()
    token2.expr_type = ExprBase.OPERATOR
    token2.op = ExprBase.ADD
```

The constant token is created in the same way as the constant on the left hand side:

```
    token3 = ExprBase()
    token3.expr_type = ExprBase.CONSTANT
    token3.constant = 5
```

The final function token is created using the *DomainFormula* message type seen in [Tutorial 06: Knowledge Base I](tutorial_06):

```
    token4 = ExprBase()
    token4.expr_type = ExprBase.FUNCTION
    token4.function.name = "energy"
    token4.function.typed_parameters.append(diagnostic_msgs.msg.KeyValue("v", "kenny"))
```

The tokens are pushed into the right hand side, in correct prefix order:

```
    query3.ineq.RHS.tokens.append(token2)
    query3.ineq.RHS.tokens.append(token3)
    query3.ineq.RHS.tokens.append(token4)
```

The inequality is finally pushed into the list of queries in our *KnowledgeQueryService* request:

```
    query.append(query3)
```

## 4.3 Testing the Inequality Query

Call the query by running the program again:

```
python query_client.py
```

You should see the output:

```
Waiting for service
Calling Service
Response is: [True, False, False]
```

The inequality is `False`. A quick check to make sure: the initial state assigns *(= (energy kenny) 120)*, therefore we are checking: *(125 > 5 + 120)* which should indeed be False.

Try modifying the value of the left hand side in the inequality query, so that the left hand side is greater than the right hand side:

```
    token1.expr_type = ExprBase.CONSTANT
    token1.constant = 126
```

You should see the output:

```
Waiting for service
Calling Service
Response is: [True, False, True]
```

## 5. What's Next?

[Tutorial 10: Action Interface](tutorial_10) describes how to write a PDDL action interface connecting the ROSPlan action dispatch to its implementation.
