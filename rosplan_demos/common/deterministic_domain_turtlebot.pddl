(define (domain turtlebot)

(:requirements :strips :typing :disjunctive-preconditions );:probabilistic-effects)

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
(:action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:precondition (and (robot_at ?v ?from) (localised ?v) (undocked ?v))
	:effect (and (visited ?to) (robot_at ?v ?to) (not (robot_at ?v ?from)))
)

;; Localise
(:action localise
	:parameters (?v - robot)
	:precondition (undocked ?v)
	:effect (localised ?v)
)

;; Dock to charge
(:action dock
	:parameters (?v - robot ?wp - waypoint)
	:precondition (and
		(dock_at ?wp)
		(robot_at ?v ?wp)
		(undocked ?v))
	:effect (and (docked ?v) (not (undocked ?v)))
)

(:action undock
	:parameters (?v - robot ?wp - waypoint)
	:precondition (and
		(dock_at ?wp)
		(docked ?v))
	:effect (and
		(not (docked ?v))
		(undocked ?v))
)

)
