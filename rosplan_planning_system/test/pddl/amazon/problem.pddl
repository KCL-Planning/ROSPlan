(define (problem amazon)
(:domain driverlog-simple)

(:objects
    driver - driver
    truck - truck
    mydvd - item
    home amazon london myhouse - place
)

(:init
    (at driver home)
    (at truck amazon)
    (at mydvd amazon)

    (path home amazon)    
    (path amazon home)
    
    (link amazon london)
    (link london amazon)
    
    (link london myhouse)
    (link myhouse london)
)

(:goal
   (and (at mydvd myhouse) )
)
)
