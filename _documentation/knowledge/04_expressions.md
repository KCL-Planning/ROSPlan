---
layout: documentation
title: Message 03 Numeric Expressions
---

The message type *ExprComposite* represents an expression in prefix notation. The message is an ordered list of operators and operands, each represented using the *ExprBase* message type.

```
# A message used to represent a numerical expression; composite type (2/2)
# stores an array of ExprBase as prefix notation

# components
ExprBase[] tokens
```

The *ExprBase* message type looks like this:

```
# A message used to represent a numerical expression; base types (1/2)

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

For example, the consider the following ungrounded numeric expression:  
*(\* ?duration (+ (energy ?r) 10))*

It is described by the *ExprComposite* message in the following way:

```
tokens:
- expr_type: 2
  op: 2
- expr_type: 3
  special_type: 2
- expr_type: 2
  special_type: 0
- expr_type: 1
  function:
    name: energy
    typed_parameters: 
      - 
        key: r
        value: robot
- expr_type: 0
  constant: 10
```

in which the tokens are a prefix array of *ExprBase* messages:  
\[ \*, ?duration, +, (energy ?r), 10 \]
