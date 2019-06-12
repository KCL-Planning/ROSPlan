(define (domain test_domain)
 (:predicates
 (inrooma ?x)
 (inroomb ?x))
 (:action movetob
   :parameters (?x)
   :precondition (inrooma ?x)
   :effect (and (not (inrooma ?x))
         (inroomb ?x)
        )
 )
)
