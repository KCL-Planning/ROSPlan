---
layout: documentation
title: Message 02 KnowledgeItem
---

The KnowledgeItem message is used to represent a part of the current state in the Knowledge Base. It could represent:

- a grounded object instance,
- a proposition,
- a function,
- a metric optimization with a grounded numeric expression, or
- an inequality between two grounded numeric expressions.

The message body is shown below. Afterwards it is described in more detail.

```
# A knowledge item used to represent a piece of the state in ROSPlan

uint8 INSTANCE = 0
uint8 FACT = 1
uint8 FUNCTION = 2
uint8 EXPRESSION = 3
uint8 INEQUALITY = 4

uint8 knowledge_type

# time at which this knowledge becomes true
time initial_time

# knowledge is explicitly false
bool is_negative

#---------
# INSTANCE
#---------

# instance knowledge_type
string instance_type
string instance_name

#----------------------
# PREDICATE OR FUNCTION
#----------------------

# attribute knowledge_type
string attribute_name
diagnostic_msgs/KeyValue[] values

#---------
# FUNCTION
#---------

# function value
float64 function_value

#-----------
# EXPRESSION
#-----------

string optimization
rosplan_knowledge_msgs/ExprComposite expr

#-----------
# INEQUALITY
#-----------

rosplan_knowledge_msgs/DomainInequality ineq
```

## Message Description

```
# A knowledge item used to represent a piece of the state in ROSPlan

uint8 INSTANCE = 0
uint8 FACT = 1
uint8 FUNCTION = 2
uint8 EXPRESSION = 3
uint8 INEQUALITY = 4

uint8 knowledge_type
```

The knowledge_type parameter specifies what the message describes.

```
# time at which this knowledge becomes true
time initial_time
```

If initialised as 0, or set to a value in the past, the initial_time is set to the current time (according to the ROS clock) when the item is added to the Knowledge Base. If set to a value in the future, then the knowledge represents *timed-initial knowledge* that will become true at that time.

```
# knowledge is explicitly false
bool is_negative
```

If true, the knowledge is explicitly false. This can be used to make things explicitly false for planning under uncertainty, and can also be used to model the negative effects of actions and sensing. However, the latter case is optional, as the services provided by the Knowledge Base allow *removal* of positive knowledge, as well as *adding* negative knowledge.

```
#---------
# INSTANCE
#---------

# instance knowledge_type
string instance_type
string instance_name
```

If the knowledge_type is **INSTANCE**, then these strings specify the PDDL type and name of the object instance.

```
#----------------------
# PREDICATE OR FUNCTION
#----------------------

# attribute knowledge_type
string attribute_name
diagnostic_msgs/KeyValue[] values
```

If the knowledge_type is **FACT** or **FUNCTION**, then the string specifies the PDDL predicate symbol for the fact or function. The *KeyValue* array describes the parameters of the action. For example, given the PDDL predicate *(robot_at ?r - robot ?wp waypoint)* and corresponding grounded proposition *(robot_at robot01 waypoint01)*, the *KnowledgeItem* message might be:

```
knowledge_type: rosplan_knowledge_msgs:KnowledgeItem:FACT  
attribute_name: "robot_at"  
values: [ "r": "robot01", "wp": "waypoint01" ]
```

In addition, a PDDL function uses the following field:

```
#---------
# FUNCTION
#---------

# function value
float64 function_value
```

The function_value field stores the constant numeric value assigned to the function.

```
#-----------
# EXPRESSION
#-----------

string optimization
rosplan_knowledge_msgs/ExprComposite expr
```

If the knowledge_type is **EXPRESSION** then these fields can be used to describe the current metric. The string optimization may take the values "maximize" or "minimize". The expression is the numeric expression to be optimized.

```
#-----------
# INEQUALITY
#-----------

rosplan_knowledge_msgs/DomainInequality ineq
```

If the knowledge_type is **INEQUALITY**, this field holds that inequality, as two numeric expressions and one comparison type.
