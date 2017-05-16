(define (domain turtlebot_example)

(:requirements :strips :typing :disjunctive-preconditions)

(:types
	robot
	waypoint
	floor
)

(:predicates

	;;robot predicates

	(person_greeted ?v - robot)
	(person_guided ?v - robot)
	(robot_at ?v - robot ?wp - waypoint)
	(undocked ?v - robot)
	(docked ?v - robot)
	(localised ?v - robot)
	(in_elevator ?v - robot)
	(outside_elevator ?v - robot)
	(allowed_goto_waypoint ?v - robot)

	;;waypoint & floor predicates

	(dock_at ?wp - waypoint)
	(waypoint_at_floor ?wp - waypoint ?fl - floor )
	(elevator_access_waypoint ?wp - waypoint)
	(visited ?wp - waypoint)
	(current_elevator_waypoint ?wp - waypoint)
	(elevator_waypoint ?wp - waypoint)
	(hallway_waypoint ?wp - waypoint)
	(greeting_waypoint ?wp - waypoint)
	(destination_waypoint ?wp - waypoint)
)


;; Move to any waypoint, avoiding terrain
(:action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint ?fl - floor)
	:precondition (and
		(undocked ?v)
		(outside_elevator ?v)
		(robot_at ?v ?from)
		(waypoint_at_floor ?from ?fl)
		(waypoint_at_floor ?to ?fl)
		(hallway_waypoint ?from)
		(hallway_waypoint ?to)
		(allowed_goto_waypoint ?v)
		)
	:effect (and
		(robot_at ?v ?to)
		(not (robot_at ?v ?from))
		(not (allowed_goto_waypoint ?v))
		)
)


(:action enter_elevator
	:parameters (?v - robot ?fromWp ?toWp - waypoint ?floor - floor)
	:precondition (and
		(undocked ?v)
		(robot_at ?v ?fromWp)
		(elevator_access_waypoint ?fromWp)
		(elevator_waypoint ?toWp)
		(waypoint_at_floor ?fromWp ?floor)
		(waypoint_at_floor ?toWp ?floor)
		)
	:effect (and
		(in_elevator ?v)
		(not (outside_elevator ?v))
		(robot_at ?v ?toWp)
		(not (robot_at ?v ?fromWp))
		)
)

(:action change_map
	:parameters (?v - robot ?fromWp ?toWp - waypoint ?fromFloor ?toFloor - floor)
	:precondition (and
		(undocked ?v)
		(in_elevator ?v)
		(robot_at ?v ?fromWp)
		(current_elevator_waypoint ?fromWp)
		(elevator_waypoint ?toWp)
		(waypoint_at_floor ?fromWp ?fromFloor)
		(waypoint_at_floor ?toWp ?toFloor)
		)
	:effect (and
		(robot_at ?v ?toWp)
		(not (robot_at ?v ?fromWp))
		(current_elevator_waypoint ?toWp)
		(not (current_elevator_waypoint ?fromWp))
		)
)


(:action exit_elevator
	:parameters (?v - robot ?fromWp ?toWp - waypoint ?floor - floor)
	:precondition (and
		(undocked ?v)
		(robot_at ?v ?fromWp)
		(current_elevator_waypoint ?fromWp)
		(elevator_access_waypoint ?toWp)
		(waypoint_at_floor ?fromWp ?floor)
		(waypoint_at_floor ?toWp ?floor)
		)
	:effect (and
		(outside_elevator ?v)
		(not (in_elevator ?v))
		(robot_at ?v ?toWp)
		(allowed_goto_waypoint ?v)
		(not (robot_at ?v ?fromWp)))
)




(:action greet_person
	:parameters (?v - robot ?wp - waypoint)
	:precondition 
		(and
		(undocked ?v)
		(outside_elevator ?v)
		(robot_at ?v ?wp)
		(greeting_waypoint ?wp)
		)
	:effect (and
		(person_greeted ?v)
		(allowed_goto_waypoint ?v)
		)
)	


(:action guide_person
	:parameters (?v - robot ?wp - waypoint)
	:precondition 
		(and
		(undocked ?v)
		(outside_elevator ?v)
		(robot_at ?v ?wp)
		(person_greeted ?v)
		(destination_waypoint ?wp)
		)
	:effect (and
		(person_guided ?v)
		(allowed_goto_waypoint ?v)
		)
)	


;; Dock to charge
(:action dock
	:parameters (?v - robot ?wp - waypoint)
	:precondition 
		(and
		(dock_at ?wp)
		(robot_at ?v ?wp)
		(undocked ?v)
		(outside_elevator ?v)
		)
	:effect (and
		(docked ?v)
		(not (undocked ?v))
		)
)

(:action undock
	:parameters (?v - robot ?wp - waypoint)
	:precondition 
		(and
		(dock_at ?wp)
		(outside_elevator ?v)
		(docked ?v)
		)
	:effect (and
		(not (docked ?v))
		(undocked ?v)
		)
)


;; Localise
(:action localise
	:parameters (?v - robot)
	:precondition (undocked ?v)
	:effect (localised ?v)
)








)
