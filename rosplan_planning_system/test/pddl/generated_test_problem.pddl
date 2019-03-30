(define (problem task)
(:domain driverlog-simple)
(:objects
    home amazon london myhouse - place
    driver - driver
    truck - truck
    mydvd - item
)
(:init
    (at driver home)
    (at truck amazon)
    (at mydvd amazon)


    (link amazon london)
    (link london amazon)
    (link london myhouse)
    (link myhouse london)

    (path home amazon)
    (path amazon home)

)
(:goal (and
    (at mydvd myhouse)
))
)
