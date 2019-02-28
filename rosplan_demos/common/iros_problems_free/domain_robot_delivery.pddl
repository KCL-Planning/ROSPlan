(define (domain robot_delivery)

(:requirements :strips :typing :fluents :durative-actions :timed-initial-literals)

(:types
	waypoint robot - object
	machine - waypoint
)

(:functions 
	(distance ?a ?b - waypoint)
)

(:predicates

	(robot_at ?v - robot ?wp - waypoint)
	(undocked ?v - robot)
	(docked ?v - robot)
	(localised ?v - robot)
	(dock_at ?wp - waypoint)

	;; Printing
	(carrying_order ?r - robot)
	(nocarrying_order ?r - robot)
	(asked_unload ?r - robot)
	(order_delivered ?w - waypoint)
	(delivery_destination ?w - waypoint)

	(machine_on ?m - machine)
	(machine_off ?m - machine)
)

;; Move to any waypoint, avoiding terrain
(:durative-action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:duration ( = ?duration (distance ?from ?to))
	:condition (and
		(at start (robot_at ?v ?from))
		(at start (localised ?v))
		(over all (undocked ?v))
		)
	:effect (and
		(at start (not (robot_at ?v ?from)))
		(at start (not (asked_unload ?v)))
		(at end (robot_at ?v ?to))
		)
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


(:durative-action wait_load_at_machine
        :parameters (?r ?h - robot ?m - machine)
        :duration ( = ?duration 15)
        :condition (and
                (at start (machine_on ?m))
                (at start (nocarrying_order ?r))
                (over all (nocarrying_order ?h))
                (over all (robot_at ?r ?m))
                (over all (robot_at ?h ?m))
                )
        :effect (and
                (at end (carrying_order ?r))
                (at start (not (nocarrying_order ?r)))
                )
)

(:durative-action switch_machine_on
        :parameters (?r - robot ?m - machine)
        :duration ( = ?duration 5)
        :condition (and
                (over all (robot_at ?r ?m))
            	(at start (machine_off ?m))
                )
        :effect (and
            	(at start (not (machine_off ?m)))
                (at end (machine_on ?m))
                )
)


(:durative-action wait_unload
	:parameters (?r - robot ?w - waypoint)
	:duration ( = ?duration 15)
	:condition (and
		(at start (carrying_order ?r))
		(at start (delivery_destination ?w))
		(over all (robot_at ?r ?w))
		) 
	:effect (and
		(at start (not (carrying_order ?r)))
		(at end (nocarrying_order ?r))
		(at end (order_delivered ?w))
		)
)

(:durative-action ditch
	:parameters (?r - robot ?w - waypoint)
	:duration (= ?duration 3)
	:condition (and
		(over all (robot_at ?r ?w))
		) 
	:effect (and
		(at start (not (carrying_order ?r)))
		(at start (nocarrying_order ?r))
		)
)
)
