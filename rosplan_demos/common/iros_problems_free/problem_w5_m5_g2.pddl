(define (problem task)
(:domain robot_delivery)
(:objects
wp0 wp1 wp2 wp3 wp4 - waypoint
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

    (delivery_destination wp0)
    (delivery_destination wp1)

    (= (distance wp0 wp1) 8)
    (= (distance wp1 wp0) 8)
    (= (distance wp0 wp2) 6)
    (= (distance wp2 wp0) 6)
    (= (distance wp0 wp3) 7)
    (= (distance wp3 wp0) 7)
    (= (distance wp0 wp4) 7)
    (= (distance wp4 wp0) 7)
    (= (distance wp1 wp2) 9)
    (= (distance wp2 wp1) 9)
    (= (distance wp1 wp3) 10)
    (= (distance wp3 wp1) 10)
    (= (distance wp1 wp4) 4)
    (= (distance wp4 wp1) 4)
    (= (distance wp2 wp3) 4)
    (= (distance wp3 wp2) 4)
    (= (distance wp2 wp4) 12)
    (= (distance wp4 wp2) 12)
    (= (distance wp3 wp4) 13)
    (= (distance wp4 wp3) 13)
    (= (distance wp0 machine0) 12)
    (= (distance machine0 wp0) 12)
    (= (distance wp0 machine1) 9)
    (= (distance machine1 wp0) 9)
    (= (distance wp0 machine2) 11)
    (= (distance machine2 wp0) 11)
    (= (distance wp0 machine3) 6)
    (= (distance machine3 wp0) 6)
    (= (distance wp0 machine4) 3)
    (= (distance machine4 wp0) 3)
    (= (distance wp1 machine0) 17)
    (= (distance machine0 wp1) 17)
    (= (distance wp1 machine1) 4)
    (= (distance machine1 wp1) 4)
    (= (distance wp1 machine2) 18)
    (= (distance machine2 wp1) 18)
    (= (distance wp1 machine3) 13)
    (= (distance machine3 wp1) 13)
    (= (distance wp1 machine4) 10)
    (= (distance machine4 wp1) 10)
    (= (distance wp2 machine0) 9)
    (= (distance machine0 wp2) 9)
    (= (distance wp2 machine1) 10)
    (= (distance machine1 wp2) 10)
    (= (distance wp2 machine2) 10)
    (= (distance machine2 wp2) 10)
    (= (distance wp2 machine3) 5)
    (= (distance machine3 wp2) 5)
    (= (distance wp2 machine4) 8)
    (= (distance machine4 wp2) 8)
    (= (distance wp3 machine0) 12)
    (= (distance machine0 wp3) 12)
    (= (distance wp3 machine1) 7)
    (= (distance machine1 wp3) 7)
    (= (distance wp3 machine2) 13)
    (= (distance machine2 wp3) 13)
    (= (distance wp3 machine3) 8)
    (= (distance machine3 wp3) 8)
    (= (distance wp3 machine4) 9)
    (= (distance machine4 wp3) 9)
    (= (distance wp4 machine0) 18)
    (= (distance machine0 wp4) 18)
    (= (distance wp4 machine1) 7)
    (= (distance machine1 wp4) 7)
    (= (distance wp4 machine2) 15)
    (= (distance machine2 wp4) 15)
    (= (distance wp4 machine3) 10)
    (= (distance machine3 wp4) 10)
    (= (distance wp4 machine4) 7)
    (= (distance machine4 wp4) 7)
    (= (distance machine0 machine1) 18)
    (= (distance machine1 machine0) 18)
    (= (distance machine0 machine2) 16)
    (= (distance machine2 machine0) 16)
    (= (distance machine0 machine3) 11)
    (= (distance machine3 machine0) 11)
    (= (distance machine0 machine4) 14)
    (= (distance machine4 machine0) 14)
    (= (distance machine1 machine2) 19)
    (= (distance machine2 machine1) 19)
    (= (distance machine1 machine3) 14)
    (= (distance machine3 machine1) 14)
    (= (distance machine1 machine4) 11)
    (= (distance machine4 machine1) 11)
    (= (distance machine2 machine3) 6)
    (= (distance machine3 machine2) 6)
    (= (distance machine2 machine4) 9)
    (= (distance machine4 machine2) 9)
    (= (distance machine3 machine4) 4)
    (= (distance machine4 machine3) 4)
)
(:goal (and
    (order_delivered wp0)
    (order_delivered wp1)
))
)