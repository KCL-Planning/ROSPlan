(define (problem task)
(:domain robot_delivery)
(:objects
wp0 wp1 wp2 wp3 wp4 wp5 wp6 - waypoint
machine0 machine1 machine2 machine3 - machine
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

    (delivery_destination wp6)
    (delivery_destination wp4)

    (= (distance wp0 wp1) 13)
    (= (distance wp1 wp0) 13)
    (= (distance wp0 wp2) 19)
    (= (distance wp2 wp0) 19)
    (= (distance wp0 wp3) 18)
    (= (distance wp3 wp0) 18)
    (= (distance wp0 wp4) 19)
    (= (distance wp4 wp0) 19)
    (= (distance wp0 wp5) 28)
    (= (distance wp5 wp0) 28)
    (= (distance wp0 wp6) 27)
    (= (distance wp6 wp0) 27)
    (= (distance wp1 wp2) 11)
    (= (distance wp2 wp1) 11)
    (= (distance wp1 wp3) 6)
    (= (distance wp3 wp1) 6)
    (= (distance wp1 wp4) 23)
    (= (distance wp4 wp1) 23)
    (= (distance wp1 wp5) 16)
    (= (distance wp5 wp1) 16)
    (= (distance wp1 wp6) 15)
    (= (distance wp6 wp1) 15)
    (= (distance wp2 wp3) 16)
    (= (distance wp3 wp2) 16)
    (= (distance wp2 wp4) 13)
    (= (distance wp4 wp2) 13)
    (= (distance wp2 wp5) 10)
    (= (distance wp5 wp2) 10)
    (= (distance wp2 wp6) 9)
    (= (distance wp6 wp2) 9)
    (= (distance wp3 wp4) 28)
    (= (distance wp4 wp3) 28)
    (= (distance wp3 wp5) 21)
    (= (distance wp5 wp3) 21)
    (= (distance wp3 wp6) 20)
    (= (distance wp6 wp3) 20)
    (= (distance wp4 wp5) 16)
    (= (distance wp5 wp4) 16)
    (= (distance wp4 wp6) 13)
    (= (distance wp6 wp4) 13)
    (= (distance wp5 wp6) 4)
    (= (distance wp6 wp5) 4)
    (= (distance wp0 machine0) 29)
    (= (distance machine0 wp0) 29)
    (= (distance wp0 machine1) 24)
    (= (distance machine1 wp0) 24)
    (= (distance wp0 machine2) 18)
    (= (distance machine2 wp0) 18)
    (= (distance wp0 machine3) 15)
    (= (distance machine3 wp0) 15)
    (= (distance wp1 machine0) 17)
    (= (distance machine0 wp1) 17)
    (= (distance wp1 machine1) 12)
    (= (distance machine1 wp1) 12)
    (= (distance wp1 machine2) 16)
    (= (distance machine2 wp1) 16)
    (= (distance wp1 machine3) 27)
    (= (distance machine3 wp1) 27)
    (= (distance wp2 machine0) 11)
    (= (distance machine0 wp2) 11)
    (= (distance wp2 machine1) 12)
    (= (distance machine1 wp2) 12)
    (= (distance wp2 machine2) 6)
    (= (distance machine2 wp2) 6)
    (= (distance wp2 machine3) 17)
    (= (distance machine3 wp2) 17)
    (= (distance wp3 machine0) 22)
    (= (distance machine0 wp3) 22)
    (= (distance wp3 machine1) 17)
    (= (distance machine1 wp3) 17)
    (= (distance wp3 machine2) 21)
    (= (distance machine2 wp3) 21)
    (= (distance wp3 machine3) 32)
    (= (distance machine3 wp3) 32)
    (= (distance wp4 machine0) 11)
    (= (distance machine0 wp4) 11)
    (= (distance wp4 machine1) 24)
    (= (distance machine1 wp4) 24)
    (= (distance wp4 machine2) 8)
    (= (distance machine2 wp4) 8)
    (= (distance wp4 machine3) 7)
    (= (distance machine3 wp4) 7)
    (= (distance wp5 machine0) 6)
    (= (distance machine0 wp5) 6)
    (= (distance wp5 machine1) 9)
    (= (distance machine1 wp5) 9)
    (= (distance wp5 machine2) 11)
    (= (distance machine2 wp5) 11)
    (= (distance wp5 machine3) 20)
    (= (distance machine3 wp5) 20)
    (= (distance wp6 machine0) 3)
    (= (distance machine0 wp6) 3)
    (= (distance wp6 machine1) 12)
    (= (distance machine1 wp6) 12)
    (= (distance wp6 machine2) 10)
    (= (distance machine2 wp6) 10)
    (= (distance wp6 machine3) 17)
    (= (distance machine3 wp6) 17)
    (= (distance machine0 machine1) 14)
    (= (distance machine1 machine0) 14)
    (= (distance machine0 machine2) 12)
    (= (distance machine2 machine0) 12)
    (= (distance machine0 machine3) 17)
    (= (distance machine3 machine0) 17)
    (= (distance machine1 machine2) 17)
    (= (distance machine2 machine1) 17)
    (= (distance machine1 machine3) 28)
    (= (distance machine3 machine1) 28)
    (= (distance machine2 machine3) 12)
    (= (distance machine3 machine2) 12)
)
(:goal (and
    (order_delivered wp6)
    (order_delivered wp4)
))
)
