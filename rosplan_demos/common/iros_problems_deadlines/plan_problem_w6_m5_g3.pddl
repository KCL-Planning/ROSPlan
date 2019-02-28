Number of literals: 64
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 11.000
b (10.000 | 20.000)b (9.000 | 25.001)b (8.000 | 25.001)b (7.000 | 40.002)b (6.000 | 63.002)b (5.000 | 63.002)b (4.000 | 103.003)b (2.000 | 131.003)b (1.000 | 131.003);;;; Solution Found
; States evaluated: 155
; Cost: 145.004
; Time 0.14
0.000: (goto_waypoint robot0 wp0 machine1)  [20.000]
0.000: (goto_waypoint robot1 wp0 machine1)  [20.000]
0.000: (goto_waypoint robot2 wp0 wp2)  [36.000]
20.001: (switch_machine_on robot0 machine1)  [5.000]
25.002: (wait_load_at_machine robot1 robot0 machine1)  [15.000]
36.001: (goto_waypoint robot2 wp2 machine0)  [5.000]
40.002: (goto_waypoint robot1 machine1 wp4)  [13.000]
40.002: (goto_waypoint robot0 machine1 machine0)  [23.000]
41.001: (switch_machine_on robot2 machine0)  [5.000]
53.002: (ask_unload robot1 wp4)  [5.000]
58.003: (wait_unload robot1 wp4)  [15.000]
63.002: (wait_load_at_machine robot2 robot0 machine0)  [15.000]
73.003: (goto_waypoint robot1 wp4 machine0)  [11.000]
84.003: (wait_load_at_machine robot0 robot1 machine0)  [15.000]
92.003: (goto_waypoint robot2 machine0 wp2)  [5.000]
97.003: (ask_unload robot2 wp2)  [5.000]
99.003: (goto_waypoint robot0 machine0 wp3)  [26.000]
102.004: (wait_unload robot2 wp2)  [15.000]
117.004: (goto_waypoint robot2 wp2 wp3)  [28.000]
125.003: (ask_unload robot0 wp3)  [5.000]
130.004: (wait_unload robot0 wp3)  [15.000]
