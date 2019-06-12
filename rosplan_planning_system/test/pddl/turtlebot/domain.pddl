(define (domain turtlebot)

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
		(at end (undocked ?v)))
)

)
