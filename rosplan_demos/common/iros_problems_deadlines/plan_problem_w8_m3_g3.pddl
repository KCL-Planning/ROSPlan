Number of literals: 60
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 11.000
b (10.000 | 3.000)b (9.000 | 8.001)b (8.000 | 8.001)b (7.000 | 23.002)b (6.000 | 40.002)b (5.000 | 45.002)b (4.000 | 63.002)b (3.000 | 68.002)b (2.000 | 97.003)b (1.000 | 102.003);;;; Solution Found
; States evaluated: 141
; Cost: 117.004
; Time 0.10
0.000: (goto_waypoint robot0 wp0 machine1)  [3.000]
0.000: (goto_waypoint robot1 wp0 machine1)  [3.000]
0.000: (goto_waypoint robot2 wp0 wp7)  [21.000]
3.001: (switch_machine_on robot0 machine1)  [5.000]
8.002: (wait_load_at_machine robot1 robot0 machine1)  [15.000]
21.001: (goto_waypoint robot2 wp7 wp5)  [11.000]
23.002: (goto_waypoint robot1 machine1 wp5)  [17.000]
23.002: (goto_waypoint robot0 machine1 machine2)  [11.000]
32.002: (goto_waypoint robot2 wp5 machine2)  [9.000]
34.002: (switch_machine_on robot0 machine2)  [5.000]
40.002: (ask_unload robot1 wp5)  [5.000]
45.003: (wait_unload robot1 wp5)  [15.000]
60.003: (goto_waypoint robot1 wp5 machine2)  [9.000]
68.003: (wait_load_at_machine robot2 robot0 machine2)  [15.000]
69.003: (wait_load_at_machine robot1 robot0 machine2)  [15.000]
83.003: (goto_waypoint robot2 machine2 wp6)  [7.000]
84.003: (goto_waypoint robot1 machine2 wp7)  [13.000]
90.003: (ask_unload robot2 wp6)  [5.000]
95.004: (wait_unload robot2 wp6)  [15.000]
97.003: (ask_unload robot1 wp7)  [5.000]
102.004: (wait_unload robot1 wp7)  [15.000]
110.004: (goto_waypoint robot2 wp6 machine2)  [7.000]
