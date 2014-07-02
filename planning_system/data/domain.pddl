(define (domain simple_blocks)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
	gripper
	block
)

(:predicates
	(onfloor ?b - block)
	(on ?b1 - block ?b2 - block)
	(clear ?b - block)
	(holding ?b - block ?g - gripper)
	(empty ?g - gripper)
)

(:functions
	(weight ?b - block) 
)

(:durative-action pick_up
	:parameters (?b - block ?g - gripper)
	:duration ( = ?duration 30)
	:condition (and
		(at start (onfloor ?b))
		(at start (clear ?b))
		(at start (empty ?g)))
	:effect (and
		(at start (not (empty ?g)))
		(at start (not (clear ?b)))
		(at start (not (onfloor ?b)))
		(at end (holding ?b ?g)))
)

(:durative-action put_down
	:parameters (?b - block ?g - gripper)
	:duration ( = ?duration 30)
	:condition (at start (holding ?b ?g))
	:effect (and
		(at start (not (holding ?b ?g)))
		(at end (onfloor ?b))
		(at end (clear ?b))
		(at end (empty ?g)))
)

(:durative-action unstack
	:parameters (?b1 - block ?b2 - block ?g - gripper)
	:duration ( = ?duration 30)
	:condition (and
		(at start (on ?b1 ?b2))
		(at start (clear ?b1))
		(at start (empty ?g)))
	:effect (and
		(at start (not (empty ?g)))
		(at start (not (clear ?b1)))
		(at start (not (on ?b1 ?b2)))
		(at end (clear ?b2))
		(at end (holding ?b1 ?g)))
)

(:durative-action stack
	:parameters (?b1 - block ?b2 - block ?g - gripper)
	:duration ( = ?duration 30)
	:condition (and
		(at start (holding ?b1 ?g))
		(at start (clear ?b2)))
	:effect (and
		(at start (not (clear ?b2)))
		(at start (not (holding ?b1 ?g)))
		(at end (on ?b1 ?b2))
		(at end (clear ?b1))
		(at end (empty ?g)))
)
)
