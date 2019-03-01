(define (problem task)
(:domain robot_delivery)
(:objects
wp0 wp1 wp2 wp3 wp4 wp5 wp6 wp7 - waypoint
machine0 machine1 machine2 - machine
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

    (delivery_destination wp5)
    (delivery_destination wp7)
    (delivery_destination wp6)

    (= (distance wp0 wp1) 5)
    (= (distance wp1 wp0) 5)
    (= (distance wp0 wp2) 2)
    (= (distance wp2 wp0) 2)
    (= (distance wp0 wp3) 25)
    (= (distance wp3 wp0) 25)
    (= (distance wp0 wp4) 11)
    (= (distance wp4 wp0) 11)
    (= (distance wp0 wp5) 15)
    (= (distance wp5 wp0) 15)
    (= (distance wp0 wp6) 15)
    (= (distance wp6 wp0) 15)
    (= (distance wp0 wp7) 21)
    (= (distance wp7 wp0) 21)
    (= (distance wp1 wp2) 4)
    (= (distance wp2 wp1) 4)
    (= (distance wp1 wp3) 25)
    (= (distance wp3 wp1) 25)
    (= (distance wp1 wp4) 9)
    (= (distance wp4 wp1) 9)
    (= (distance wp1 wp5) 19)
    (= (distance wp5 wp1) 19)
    (= (distance wp1 wp6) 19)
    (= (distance wp6 wp1) 19)
    (= (distance wp1 wp7) 25)
    (= (distance wp7 wp1) 25)
    (= (distance wp2 wp3) 24)
    (= (distance wp3 wp2) 24)
    (= (distance wp2 wp4) 10)
    (= (distance wp4 wp2) 10)
    (= (distance wp2 wp5) 16)
    (= (distance wp5 wp2) 16)
    (= (distance wp2 wp6) 16)
    (= (distance wp6 wp2) 16)
    (= (distance wp2 wp7) 22)
    (= (distance wp7 wp2) 22)
    (= (distance wp3 wp4) 17)
    (= (distance wp4 wp3) 17)
    (= (distance wp3 wp5) 25)
    (= (distance wp5 wp3) 25)
    (= (distance wp3 wp6) 17)
    (= (distance wp6 wp3) 17)
    (= (distance wp3 wp7) 17)
    (= (distance wp7 wp3) 17)
    (= (distance wp4 wp5) 25)
    (= (distance wp5 wp4) 25)
    (= (distance wp4 wp6) 25)
    (= (distance wp6 wp4) 25)
    (= (distance wp4 wp7) 31)
    (= (distance wp7 wp4) 31)
    (= (distance wp5 wp6) 9)
    (= (distance wp6 wp5) 9)
    (= (distance wp5 wp7) 11)
    (= (distance wp7 wp5) 11)
    (= (distance wp6 wp7) 7)
    (= (distance wp7 wp6) 7)
    (= (distance wp0 machine0) 13)
    (= (distance machine0 wp0) 13)
    (= (distance wp0 machine1) 3)
    (= (distance machine1 wp0) 3)
    (= (distance wp0 machine2) 9)
    (= (distance machine2 wp0) 9)
    (= (distance wp1 machine0) 9)
    (= (distance machine0 wp1) 9)
    (= (distance wp1 machine1) 3)
    (= (distance machine1 wp1) 3)
    (= (distance wp1 machine2) 13)
    (= (distance machine2 wp1) 13)
    (= (distance wp2 machine0) 12)
    (= (distance machine0 wp2) 12)
    (= (distance wp2 machine1) 2)
    (= (distance machine1 wp2) 2)
    (= (distance wp2 machine2) 10)
    (= (distance machine2 wp2) 10)
    (= (distance wp3 machine0) 23)
    (= (distance machine0 wp3) 23)
    (= (distance wp3 machine1) 25)
    (= (distance machine1 wp3) 25)
    (= (distance wp3 machine2) 17)
    (= (distance machine2 wp3) 17)
    (= (distance wp4 machine0) 7)
    (= (distance machine0 wp4) 7)
    (= (distance wp4 machine1) 9)
    (= (distance machine1 wp4) 9)
    (= (distance wp4 machine2) 19)
    (= (distance machine2 wp4) 19)
    (= (distance wp5 machine0) 27)
    (= (distance machine0 wp5) 27)
    (= (distance wp5 machine1) 17)
    (= (distance machine1 wp5) 17)
    (= (distance wp5 machine2) 9)
    (= (distance machine2 wp5) 9)
    (= (distance wp6 machine0) 27)
    (= (distance machine0 wp6) 27)
    (= (distance wp6 machine1) 17)
    (= (distance machine1 wp6) 17)
    (= (distance wp6 machine2) 7)
    (= (distance machine2 wp6) 7)
    (= (distance wp7 machine0) 33)
    (= (distance machine0 wp7) 33)
    (= (distance wp7 machine1) 23)
    (= (distance machine1 wp7) 23)
    (= (distance wp7 machine2) 13)
    (= (distance machine2 wp7) 13)
    (= (distance machine0 machine1) 11)
    (= (distance machine1 machine0) 11)
    (= (distance machine0 machine2) 21)
    (= (distance machine2 machine0) 21)
    (= (distance machine1 machine2) 11)
    (= (distance machine2 machine1) 11)
)
(:goal (and
    (order_delivered wp5)
    (order_delivered wp7)
    (order_delivered wp6)
))
)