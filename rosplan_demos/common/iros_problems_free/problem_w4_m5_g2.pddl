(define (problem task)
(:domain robot_delivery)
(:objects
wp0 wp1 wp2 wp3 - waypoint
machine0 machine1 machine2 machine3 machine4 - machine
robot0 robot1 robot2 - robot
)
(:init
    (robot_at robot0 wp0)
    (nocarrying_order robot0)
    (undocked robot0)
    (localised robot0)

    (robot_at robot1 wp0)
    (nocarrying_order robot1)
    (undocked robot1)
    (localised robot1)

    (robot_at robot2 wp0)
    (nocarrying_order robot2)
    (undocked robot2)
    (localised robot2)

    (machine_off machine0)
    (machine_off machine1)
    (machine_off machine2)
    (machine_off machine3)
    (machine_off machine4)

    (delivery_destination wp1)
    (delivery_destination wp2)

    (= (distance wp0 wp1) 22)
    (= (distance wp1 wp0) 22)
    (= (distance wp0 wp2) 7)
    (= (distance wp2 wp0) 7)
    (= (distance wp0 wp3) 10)
    (= (distance wp3 wp0) 10)
    (= (distance wp1 wp2) 28)
    (= (distance wp2 wp1) 28)
    (= (distance wp1 wp3) 13)
    (= (distance wp3 wp1) 13)
    (= (distance wp2 wp3) 16)
    (= (distance wp3 wp2) 16)
    (= (distance wp0 machine0) 11)
    (= (distance machine0 wp0) 11)
    (= (distance wp0 machine1) 4)
    (= (distance machine1 wp0) 4)
    (= (distance wp0 machine2) 16)
    (= (distance machine2 wp0) 16)
    (= (distance wp0 machine3) 10)
    (= (distance machine3 wp0) 10)
    (= (distance wp0 machine4) 21)
    (= (distance machine4 wp0) 21)
    (= (distance wp1 machine0) 32)
    (= (distance machine0 wp1) 32)
    (= (distance wp1 machine1) 19)
    (= (distance machine1 wp1) 19)
    (= (distance wp1 machine2) 21)
    (= (distance machine2 wp1) 21)
    (= (distance wp1 machine3) 15)
    (= (distance machine3 wp1) 15)
    (= (distance wp1 machine4) 16)
    (= (distance machine4 wp1) 16)
    (= (distance wp2 machine0) 5)
    (= (distance machine0 wp2) 5)
    (= (distance wp2 machine1) 10)
    (= (distance machine1 wp2) 10)
    (= (distance wp2 machine2) 12)
    (= (distance machine2 wp2) 12)
    (= (distance wp2 machine3) 16)
    (= (distance machine3 wp2) 16)
    (= (distance wp2 machine4) 17)
    (= (distance machine4 wp2) 17)
    (= (distance wp3 machine0) 20)
    (= (distance machine0 wp3) 20)
    (= (distance wp3 machine1) 7)
    (= (distance machine1 wp3) 7)
    (= (distance wp3 machine2) 15)
    (= (distance machine2 wp3) 15)
    (= (distance wp3 machine3) 3)
    (= (distance machine3 wp3) 3)
    (= (distance wp3 machine4) 20)
    (= (distance machine4 wp3) 20)
    (= (distance machine0 machine1) 14)
    (= (distance machine1 machine0) 14)
    (= (distance machine0 machine2) 12)
    (= (distance machine2 machine0) 12)
    (= (distance machine0 machine3) 20)
    (= (distance machine3 machine0) 20)
    (= (distance machine0 machine4) 17)
    (= (distance machine4 machine0) 17)
    (= (distance machine1 machine2) 17)
    (= (distance machine2 machine1) 17)
    (= (distance machine1 machine3) 7)
    (= (distance machine3 machine1) 7)
    (= (distance machine1 machine4) 22)
    (= (distance machine4 machine1) 22)
    (= (distance machine2 machine3) 17)
    (= (distance machine3 machine2) 17)
    (= (distance machine2 machine4) 6)
    (= (distance machine4 machine2) 6)
    (= (distance machine3 machine4) 22)
    (= (distance machine4 machine3) 22)
)
(:goal (and
    (order_delivered wp1)
    (order_delivered wp2)
))
)
