(define (domain driverlog-simple)
(:requirements :typing) 

(:types
    place locatable - object
    driver truck item - locatable
)

(:predicates 
    (at ?item - locatable ?loc - place)
    (in ?item - locatable ?t - truck)
    (link ?x ?y - place)
    (path ?x ?y - place)		
)

(:action load-truck
  :parameters
    (?item - item ?truck - truck ?loc - place)
  :precondition
    (and (at ?truck ?loc)
         (at ?item ?loc)
    )
  :effect
   (and (not (at ?item ?loc))
        (in ?item ?truck)
   )
)

(:action unload-truck
  :parameters
   (?item - item ?truck - truck ?loc - place)
  :precondition
   (and (at ?truck ?loc)
        (in ?item ?truck)
   )
  :effect
   (and (not (in ?item ?truck))
        (at ?item ?loc)
   )
)

(:action board-truck
  :parameters
   (?driver - driver ?truck - truck ?loc - place)
  :precondition
   (and (at ?truck ?loc)
        (at ?driver ?loc)
   )
  :effect
   (and (not (at ?driver ?loc))
        (in ?driver ?truck)
   )
)

(:action get-out
  :parameters
   (?driver - driver ?truck - truck ?loc - place)
  :precondition
   (and (at ?truck ?loc)
        (in ?driver ?truck)
   )
  :effect
   (and (not (in ?driver ?truck))
        (at ?driver ?loc)
   )
)

(:action drive-truck
  :parameters
   (?truck - truck ?loc-from - place ?loc-to - place ?driver - driver)
  :precondition
   (and (at ?truck ?loc-from)
        (in ?driver ?truck)
        (link ?loc-from ?loc-to)
   )
  :effect
   (and (not (at ?truck ?loc-from))
        (at ?truck ?loc-to)
   )
)

(:action walk
  :parameters
   (?driver - driver  ?loc-from - place  ?loc-to - place)
  :precondition
   (and (at ?driver ?loc-from)
        (path ?loc-from ?loc-to)
   )
  :effect
   (and (not (at ?driver ?loc-from))
        (at ?driver ?loc-to)
   )
)
)
