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

    (delivery_destination wp0)
    (delivery_destination wp2)
    (delivery_destination wp3)

    (= (distance wp0 wp1) 5)
    (= (distance wp1 wp0) 5)
    (= (distance wp0 wp2) 3)
    (= (distance wp2 wp0) 3)
    (= (distance wp0 wp3) 5)
    (= (distance wp3 wp0) 5)
    (= (distance wp0 wp4) 13)
    (= (distance wp4 wp0) 13)
    (= (distance wp0 wp5) 15)
    (= (distance wp5 wp0) 15)
    (= (distance wp0 wp6) 12)
    (= (distance wp6 wp0) 12)
    (= (distance wp1 wp2) 7)
    (= (distance wp2 wp1) 7)
    (= (distance wp1 wp3) 7)
    (= (distance wp3 wp1) 7)
    (= (distance wp1 wp4) 11)
    (= (distance wp4 wp1) 11)
    (= (distance wp1 wp5) 19)
    (= (distance wp5 wp1) 19)
    (= (distance wp1 wp6) 16)
    (= (distance wp6 wp1) 16)
    (= (distance wp2 wp3) 5)
    (= (distance wp3 wp2) 5)
    (= (distance wp2 wp4) 13)
    (= (distance wp4 wp2) 13)
    (= (distance wp2 wp5) 13)
    (= (distance wp5 wp2) 13)
    (= (distance wp2 wp6) 12)
    (= (distance wp6 wp2) 12)
    (= (distance wp3 wp4) 9)
    (= (distance wp4 wp3) 9)
    (= (distance wp3 wp5) 13)
    (= (distance wp5 wp3) 13)
    (= (distance wp3 wp6) 16)
    (= (distance wp6 wp3) 16)
    (= (distance wp4 wp5) 19)
    (= (distance wp5 wp4) 19)
    (= (distance wp4 wp6) 24)
    (= (distance wp6 wp4) 24)
    (= (distance wp5 wp6) 6)
    (= (distance wp6 wp5) 6)
    (= (distance wp0 machine0) 15)
    (= (distance machine0 wp0) 15)
    (= (distance wp0 machine1) 22)
    (= (distance machine1 wp0) 22)
    (= (distance wp0 machine2) 3)
    (= (distance machine2 wp0) 3)
    (= (distance wp0 machine3) 18)
    (= (distance machine3 wp0) 18)
    (= (distance wp1 machine0) 19)
    (= (distance machine0 wp1) 19)
    (= (distance wp1 machine1) 26)
    (= (distance machine1 wp1) 26)
    (= (distance wp1 machine2) 5)
    (= (distance machine2 wp1) 5)
    (= (distance wp1 machine3) 22)
    (= (distance machine3 wp1) 22)
    (= (distance wp2 machine0) 13)
    (= (distance machine0 wp2) 13)
    (= (distance wp2 machine1) 20)
    (= (distance machine1 wp2) 20)
    (= (distance wp2 machine2) 5)
    (= (distance machine2 wp2) 5)
    (= (distance wp2 machine3) 16)
    (= (distance machine3 wp2) 16)
    (= (distance wp3 machine0) 13)
    (= (distance machine0 wp3) 13)
    (= (distance wp3 machine1) 20)
    (= (distance machine1 wp3) 20)
    (= (distance wp3 machine2) 7)
    (= (distance machine2 wp3) 7)
    (= (distance wp3 machine3) 16)
    (= (distance machine3 wp3) 16)
    (= (distance wp4 machine0) 11)
    (= (distance machine0 wp4) 11)
    (= (distance wp4 machine1) 18)
    (= (distance machine1 wp4) 18)
    (= (distance wp4 machine2) 15)
    (= (distance machine2 wp4) 15)
    (= (distance wp4 machine3) 14)
    (= (distance machine3 wp4) 14)
    (= (distance wp5 machine0) 15)
    (= (distance machine0 wp5) 15)
    (= (distance wp5 machine1) 16)
    (= (distance machine1 wp5) 16)
    (= (distance wp5 machine2) 17)
    (= (distance machine2 wp5) 17)
    (= (distance wp5 machine3) 6)
    (= (distance machine3 wp5) 6)
    (= (distance wp6 machine0) 20)
    (= (distance machine0 wp6) 20)
    (= (distance wp6 machine1) 21)
    (= (distance machine1 wp6) 21)
    (= (distance wp6 machine2) 14)
    (= (distance machine2 wp6) 14)
    (= (distance wp6 machine3) 11)
    (= (distance machine3 wp6) 11)
    (= (distance machine0 machine1) 8)
    (= (distance machine1 machine0) 8)
    (= (distance machine0 machine2) 17)
    (= (distance machine2 machine0) 17)
    (= (distance machine0 machine3) 10)
    (= (distance machine3 machine0) 10)
    (= (distance machine1 machine2) 24)
    (= (distance machine2 machine1) 24)
    (= (distance machine1 machine3) 11)
    (= (distance machine3 machine1) 11)
    (= (distance machine2 machine3) 20)
    (= (distance machine3 machine2) 20)
)
(:goal (and
    (order_delivered wp0)
    (order_delivered wp2)
    (order_delivered wp3)
))
)