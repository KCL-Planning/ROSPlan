---
layout: page
title: SQUIRREL Demo
---

## Contents:

- Video
- PDDL domains

This project is funded by the EU FP7 Project SQUIRREL. SQUIRREL addresses clutter in an open world by actively controlling the environment, and incrementally learning to extend the robot’s capabilities while doing so. You can find more details about this application on the project website:

[http://www.squirrel-project.eu](http://www.squirrel-project.eu)

## Video

The SQUIRREL robot explores by visiting viewcones, discovering and recognising objects.

<div class="media"><iframe src="https://www.youtube.com/embed/PMI7Yx778gw" frameborder="0" allowfullscreen></iframe></div>

## PDDL domain

Shown here are two domains used in the SQUIRREL scenario. The first domain is the most general domain which describes moving between areas, exploring areas, and tidying them. Each of these actions is dispatched from a ROSPlan planning system as a PDDL action. The execution of these actions is carried out by a separate ROSPlan planning system node. Each action (explore, examine, tidy) has a corresponding planning system and domain. Show here is the domain of the exploration action.

In this way, the problem is heirarchical. The actions of one plan become the goals for the planner in the next level. Using ROSPlan, this behaviour is easy to achieve.

Domain for tidying:

```
(define (domain squirrel_tidy_room)
(:requirements :strips :typing :disjunctive-preconditions :negative-preconditions)

(:types
	robot
	area
)

(:predicates
	(explored ?a - area)
	(examined ?a - area)
	(robot_in ?v - robot ?a - area)
	(connected ?from ?to - area)
	(tidy ?a - area)
	(accessible ?from ?to - area)
)

(:action move
	:parameters (?v - robot ?from ?to - area)
	:precondition (and
		(robot_in ?v ?from)
		(accessible ?from ?to)
	)
	:effect (and
		(not (robot_in ?v ?from))
		(robot_in ?v ?to)
	)
)

(:action clear_connection
	:parameters (?v - robot ?from ?to - area)
	:precondition (and
		(robot_in ?v ?from)
		(connected ?from ?to)
	)
	:effect (and
		(accessible ?from ?to)
	)
)

(:action explore_area
	:parameters (?v - robot ?a - area)
	:precondition (and
		(robot_in ?v ?a)
	)
	:effect (and
		(explored ?a)
	)
)

(:action examine_area
	:parameters (?v - robot ?a - area)
	:precondition (and
		(robot_in ?v ?a)
		(explored ?a)
	)
	:effect (and
		(examined ?a)
	)
)

(:action tidy_area
	:parameters (?v - robot ?a - area)
	:precondition (and
		(robot_in ?v ?a)
		(examined ?a)
	)
	:effect (and
		(tidy ?a)
	)
)
)
```

Domain for exploring:  

```xml
(define (domain squirrel_explore)
(:requirements :strips :typing  :disjunctive-preconditions :negative-preconditions)

(:types
	waypoint
	robot
)

(:predicates
	(explored ?wp - waypoint)
	(robot_at ?v - robot ?wp - waypoint)
)

;; Use perception actions to search for objects at the current waypoint
(:action explore_waypoint
	:parameters (?v - robot ?wp - waypoint)
	:precondition (and
		(robot_at ?v ?wp)
	)
	:effect (and
		(explored ?wp)
	)
)

;; Move between any two waypoints, avoiding terrain
(:action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:precondition (and
		(robot_at ?v ?from))
	:effect (and
		(not (robot_at ?v ?from))
		(robot_at ?v ?to)
	)
)
)
```
