Number of literals: 57
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 10.000
b (9.000 | 2.000)b (8.000 | 7.001)b (7.000 | 22.002)b (6.000 | 39.002)b (5.000 | 44.002)b (4.000 | 90.002)b (3.000 | 90.002)b (2.000 | 143.003)b (1.000 | 148.003);;;; Solution Found
; States evaluated: 193
; Cost: 163.004
; Time 0.16
0.000: (goto_waypoint robot0 wp0 machine2)  [2.000]
0.000: (goto_waypoint robot1 wp0 machine2)  [2.000]
0.000: (goto_waypoint robot2 wp0 wp6)  [24.000]
2.001: (switch_machine_on robot0 machine2)  [5.000]
7.002: (wait_load_at_machine robot1 robot0 machine2)  [15.000]
22.002: (goto_waypoint robot1 machine2 wp1)  [17.000]
24.001: (goto_waypoint robot2 wp6 wp0)  [24.000]
39.002: (ask_unload robot1 wp1)  [5.000]
44.003: (wait_unload robot1 wp1)  [15.000]
48.002: (goto_waypoint robot2 wp0 machine2)  [2.000]
50.002: (wait_load_at_machine robot2 robot0 machine2)  [15.000]
59.003: (goto_waypoint robot1 wp1 wp6)  [9.000]
65.002: (goto_waypoint robot0 machine2 wp6)  [25.000]
68.004: (goto_waypoint robot1 wp6 machine0)  [19.000]
87.004: (switch_machine_on robot1 machine0)  [5.000]
90.003: (goto_waypoint robot0 wp6 machine0)  [19.000]
109.003: (wait_load_at_machine robot1 robot0 machine0)  [15.000]
117.003: (goto_waypoint robot2 machine2 wp0)  [2.000]
119.003: (ask_unload robot2 wp0)  [5.000]
124.003: (goto_waypoint robot1 machine0 wp6)  [19.000]
124.004: (wait_unload robot2 wp0)  [15.000]
139.004: (goto_waypoint robot2 wp0 wp6)  [24.000]
143.003: (ask_unload robot1 wp6)  [5.000]
148.004: (wait_unload robot1 wp6)  [15.000]
