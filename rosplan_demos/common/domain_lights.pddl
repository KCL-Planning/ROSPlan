(define (domain lights)
  (:requirements :typing :negative-preconditions)
  (:types light room)
  (:constants R1 R2 R3 R4 R5 R6 - room)
  (:predicates (lighton ?r - room) (in ?r - room) (in_C) (shed))

  (:action enter
     :parameters (?r - room)
     :precondition (in_C)
     :effect (and (not (in_C)) (in ?r))
  )

  (:action exit
     :parameters (?r - room)
     :precondition (in ?r)
     :effect (and (in_C) (not (in ?r)))
  )

  (:action turn_off
     :parameters (?r - room)
     :precondition (and (in ?r) (lighton ?r))
     :effect (not (lighton ?r))
  )

  (:action observe-lighton
      :parameters (?r - room)
      :precondition (in ?r)
      :effect (lighton ?r)
  )

  (:action shed_knowledge
     :parameters ()
     :precondition ()
     :effect (shed)
  )

)
  
