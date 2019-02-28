Number of literals: 62
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 10.000
b (9.000 | 3.000)b (8.000 | 8.001)b (7.000 | 23.002)b (6.000 | 26.002)b (5.000 | 32.001)b (4.000 | 46.003)b (3.000 | 46.003)b (2.000 | 86.003)b (1.000 | 91.003);;;; Solution Found
; States evaluated: 126
; Cost: 106.004
; Time 0.10
0.000: (goto_waypoint robot0 wp0 machine2)  [3.000]
0.000: (goto_waypoint robot1 wp0 machine2)  [3.000]
0.000: (goto_waypoint robot2 wp0 wp3)  [5.000]
3.001: (switch_machine_on robot0 machine2)  [5.000]
5.001: (goto_waypoint robot2 wp3 machine2)  [7.000]
8.002: (wait_load_at_machine robot1 robot0 machine2)  [15.000]
12.001: (wait_load_at_machine robot2 robot0 machine2)  [15.000]
23.002: (goto_waypoint robot1 machine2 wp0)  [3.000]
27.001: (goto_waypoint robot2 machine2 wp2)  [5.000]
32.001: (ask_unload robot2 wp2)  [5.000]
37.002: (wait_unload robot2 wp2)  [15.000]
52.002: (goto_waypoint robot2 wp2 wp3)  [5.000]
57.003: (goto_waypoint robot2 wp3 machine2)  [7.000]
58.002: (ask_unload robot1 wp0)  [5.000]
63.003: (wait_unload robot1 wp0)  [15.000]
64.003: (wait_load_at_machine robot2 robot0 machine2)  [15.000]
78.003: (goto_waypoint robot1 wp0 machine0)  [15.000]
79.003: (goto_waypoint robot2 machine2 wp3)  [7.000]
86.003: (ask_unload robot2 wp3)  [5.000]
91.004: (wait_unload robot2 wp3)  [15.000]
93.004: (goto_waypoint robot1 machine0 wp2)  [13.000]
