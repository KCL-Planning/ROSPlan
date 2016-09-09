(define (domain coaches)
  (:types
         person
         robot
         location
  )
  (:constants shop1 shop2 shop3 shop4 shop5 shop6 entr1 entr2 - location
          r - robot
          h2 - person
  )
  (:predicates
              ;robot and person properties
              (at-robot ?x - location) (at-person ?x - location)
              (close-to ?x - person)(bags-loaded ?x -robot)

              ;needs represents the person necessity
              (needs-help ?x - person) (needs-WC ?x - person) (needs-obj ?x - person)
              (needs-food ?x - person) (needs-clothes ?x - person)

              ;desire predicates used to disambiguate which food/cloth
              (desire-italian ?x - person) (desire-chinese ?x - person)
              (desire-sportive ?x - person) (desire-fashion ?x - person)

              ;locations predicates
              (sportive-clothes ?x - location) (fashion-clothes ?x - location)
              (chinese-restaurant ?x - location) (italian-restaurant ?x - location)
              (market ?x - location)(WC ?x - location)

              ; 'sensing' actions
              (ask-italian ?p - person) (ask-chinese ?p - person)
              (ask-sportive ?p - person) (ask-fashion ?p - person)
              (ask-obj ?p - person) (satisfied ?p - person)

              (sensing ?r - robot)

  )

  (:action wait
          :parameters (?r - robot ?x - location)
          :precondition (at-robot ?x)
          :effect (sensing ?r)
  )

  (:action sense_person
                       :parameters ( ?r - robot ?x -location)
                       :precondition (sensing ?r)
;;                       :sense (at-person ?x)
  )


  (:action move
    :parameters ( ?from - location ?to - location ?r - robot)
    :precondition (at-robot ?from)
    :effect (at-robot ?to)
    :non-inertial ((at-robot ?from))
  )

  (:action bringto
    :parameters( ?p - person ?from - location ?to - location )
    :precondition (and(close-to ?p)(at-person ?from))
    :effect (and(at-person ?to)(at-robot ?to))
    :non-inertial ((at-person ?from)(at-robot ?from))

  )

  (:action bringto_WC
    :parameters( ?p - person ?from - location ?to - location )
    :precondition (and
                      (close-to ?p)
                      (at-person ?from)
                      (needs-WC ?p)
                  )
    :effect (and
                (at-person ?to)
                (at-robot ?to)
                (WC ?to)
            )
    :non-inertial ((at-person ?from)(at-robot ?from))

  )

  (:action load-bags
      :parameters ( ?p - person ?r - robot)
      :precondition (and
                        (needs-help ?p)
                        (close-to ?p)
                    )
      :effect (bags-loaded ?r)
  )

  (:action carry-bags
               :parameters ( ?p - person ?r - robot ?to - location)
               :precondition (bags-loaded ?r)
               :effect (and
                            (not (needs-help ?p))
                            (not (bags-loaded ?r))
                            (at-robot ?to)
                       )
  )

  (:action approach-person
               :parameters ( ?p - person ?x - location)
               :precondition (and (at-robot ?x)(at-person ?x))
               :effect (close-to ?p)
  )

  (:action ask_object
           :parameters ( ?p - person )
           :precondition (close-to ?p)
;;           :sense (needs-obj ?p)
 )

 (:action ask_clothes
          :parameters ( ?p - person )
          :precondition (close-to ?p)
;;          :sense (needs-clothes ?p)
 )

 (:action ask_food
          :parameters ( ?p - person )
          :precondition (close-to ?p)
;;          :sense (needs-food ?p)
 )

 (:action advertise-sportive
             :parameters ( ?p - person ?x - location)
             :precondition (and
                               (needs-clothes ?p)
                               (desire-sportive ?p)
                               (at-person ?x)
                               (close-to ?p)
                               (sportive-clothes ?x)
                           )
             :effect (not (needs-clothes ?p))
 )

 (:action advertise-fashion
             :parameters ( ?p - person ?x - location)
             :precondition (and
                               (needs-clothes ?p)
                               (desire-fashion ?p)
                               (at-person ?x)
                               (close-to ?p)
                               (fashion-clothes ?x)
                           )
             :effect (not (needs-clothes ?p))
)

(:action advertise-italian
             :parameters ( ?p - person ?x - location)
             :precondition (and
                               (needs-food ?p)
                               (desire-italian ?p)
                               (at-person ?x)
                               (close-to ?p)
                               (italian-restaurant ?x)
                           )
             :effect (not (needs-food ?p))
)

(:action advertise-chinese
             :parameters ( ?x - location ?p - person)
             :precondition (and
                               (needs-food ?p)
                               (desire-chinese ?p)
                               (at-person ?x)
                               (close-to ?p)
                               (chinese-restaurant ?x)

                           )
             :effect (not (needs-food ?p))
)

(:action advertise-objects
             :parameters ( ?x - location ?p - person )
             :precondition (and
                               (needs-obj ?p)
                               (at-person ?x)
                               (close-to ?p)
                               (market ?x)
                           )
             :effect (not (needs-obj ?p))
)

(:action bye
            :parameters (?p - person)
            :precondition (;and
;                              (not (needs-obj ?p))
                              (not (needs-food ?p))
;                              (not (needs-clothes ?p))
;                              (not (needs-WC ?p))
;                              (not (needs-help ?p))
                          )
            :effect (satisfied ?p)
)

)

(define
  (problem h2-plan)
  (:domain coaches)
  (:formula-init (and
                     (at-robot shop4)
                     (at-person shop2)

                     (market shop1)
                     (sportive-clothes shop2)
                     (WC shop3)
                     (fashion-clothes shop4)
                     (chinese-restaurant shop5)
                     (italian-restaurant shop6)

                     (needs-help h2)
                     (desire-italian h2)
                 )
  )

  (:goal
      (satisfied h2)
  )
)
