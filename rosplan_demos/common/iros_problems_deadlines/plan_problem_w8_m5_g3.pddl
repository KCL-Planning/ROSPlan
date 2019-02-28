Number of literals: 70
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 10.000
b (9.000 | 5.000)b (8.000 | 10.001)b (7.000 | 25.002)b (6.000 | 30.002)b (5.000 | 35.002)b (4.000 | 50.003)b (3.000 | 54.002)b (2.000 | 93.004)b (1.000 | 98.004);;;; Solution Found
; States evaluated: 77
; Cost: 113.005
; Time 0.08
0.000: (goto_waypoint robot0 wp0 machine1)  [5.000]
0.000: (goto_waypoint robot1 wp0 machine1)  [5.000]
0.000: (goto_waypoint robot2 wp0 wp3)  [7.000]
5.001: (switch_machine_on robot0 machine1)  [5.000]
7.001: (goto_waypoint robot2 wp3 wp7)  [6.000]
10.002: (wait_load_at_machine robot1 robot0 machine1)  [15.000]
13.002: (goto_waypoint robot2 wp7 machine1)  [10.000]
23.002: (wait_load_at_machine robot2 robot0 machine1)  [15.000]
25.002: (goto_waypoint robot1 machine1 wp0)  [5.000]
30.002: (ask_unload robot1 wp0)  [5.000]
35.003: (wait_unload robot1 wp0)  [15.000]
50.003: (goto_waypoint robot1 wp0 wp7)  [8.000]
58.004: (goto_waypoint robot1 wp7 machine1)  [10.000]
68.004: (wait_load_at_machine robot1 robot0 machine1)  [15.000]
76.004: (goto_waypoint robot2 machine1 wp3)  [11.000]
83.004: (goto_waypoint robot1 machine1 wp7)  [10.000]
87.004: (ask_unload robot2 wp3)  [5.000]
92.005: (wait_unload robot2 wp3)  [15.000]
93.004: (ask_unload robot1 wp7)  [5.000]
98.005: (wait_unload robot1 wp7)  [15.000]
107.005: (goto_waypoint robot2 wp3 wp7)  [6.000]
