---
layout: documentation
title: Message 01 Domain Messages
---

The domain is desribed using the following message types:

- DomainFormula
- DomainAssignment
- DomainOperator
- DomainInequality

## DomainFormula

```
# A message used to represent an atomic formula from the domain.
# typed_parameters matches label -> type
string name
diagnostic_msgs/KeyValue[] typed_parameters
```

The domain formula message describes one PDDL predicate. For example, the PDDL predicate *(robot_at ?r - robot ?wp waypoint)* would be represented by the DomainFormula:

```
name: "robot_at"
typed_parameters: [ "r": "robot", "wp": "waypoint" ]
```

## DomainAssignment

```
# A message used to store the numeric effects of an action
# Can be grounded or ungrounded

uint8 ASSIGN   = 0
uint8 INCREASE  = 1
uint8 DECREASE = 2
uint8 SCALE_UP = 3
uint8 SCALE_DOWN = 4
uint8 ASSIGN_CTS = 5

uint8 assign_type

rosplan_knowledge_msgs/DomainFormula LHS
rosplan_knowledge_msgs/ExprComposite RHS

bool grounded
```

The domain assignment message describes a PDDL assignment effect on a numeric function. The assign_type field describes the type of assignment. An assignment modifies a PDDL function's value according to a numeric expression. The LHS (left-hand side) and RHS (right-hand side) of the assignment specify the PDDL function and the numeric expression respectively. Both are described using the *ExprComposite* message. This is descibed fully in the [Expressions] page.

The grounded field, if true, specifies that the domain formula and numeric expression are grounded.

## DomainOperator

```
# A message used to represent an ungrounded operator in the domain.

# (1) name and parameters
rosplan_knowledge_msgs/DomainFormula formula

# (2) duration constraint


# (3) effect lists
rosplan_knowledge_msgs/DomainFormula[] at_start_add_effects
rosplan_knowledge_msgs/DomainFormula[] at_start_del_effects
rosplan_knowledge_msgs/DomainFormula[] at_end_add_effects
rosplan_knowledge_msgs/DomainFormula[] at_end_del_effects
rosplan_knowledge_msgs/DomainAssignment[] at_start_assign_effects
rosplan_knowledge_msgs/DomainAssignment[] at_end_assign_effects

# (4) conditions
rosplan_knowledge_msgs/DomainFormula[] at_start_simple_condition
rosplan_knowledge_msgs/DomainFormula[] over_all_simple_condition
rosplan_knowledge_msgs/DomainFormula[] at_end_simple_condition
rosplan_knowledge_msgs/DomainFormula[] at_start_neg_condition
rosplan_knowledge_msgs/DomainFormula[] over_all_neg_condition
rosplan_knowledge_msgs/DomainFormula[] at_end_neg_condition
```

The domain operator message describes a PDDL durative operator. The message contains arrays of *DomainFormula* and *DomainAssignment* messages to represent the conditions and effects of the operator.

The formula field contains the name and parameters of the operator. For example, the operator *(goto_wayoint ?r - robot ?from ?to - waypoint)* would be represented by the formula field as:

```
name: "goto_waypoint"
typed_parameters: [ "r": "robot", "from": "waypoint", "to": "waypoint" ]
```

## DomainInequality

```
# A message used to store the numeric effects of an action
# Can be grounded or ungrounded

uint8 GREATER   = 0
uint8 GREATEREQ = 1
uint8 LESS      = 2
uint8 LESSEQ    = 3
uint8 EQUALS    = 4

uint8 comparison_type

rosplan_knowledge_msgs/ExprComposite LHS
rosplan_knowledge_msgs/ExprComposite RHS

bool grounded
```

The domain inequality message describes a comparison between two numeric expressions. For example:  
*(> (energy ?r - robot) (+ (minimum-energy) 32))*.

The comaprison_type field describes the type of comparison. The LHS (left-hand side) and RHS (right-hand side) of the comparison are both described using the *ExprComposite* message. This is descibed fully in the [Expressions] page.

The grounded field, if true, specifies that the both numeric expressions are grounded.
