Number of literals: 54
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 11.000
b (10.000 | 7.000)b (9.000 | 12.001)b (8.000 | 12.001)b (7.000 | 27.002)b (6.000 | 31.002)b (5.000 | 36.002)b (4.000 | 59.003)b (3.000 | 61.002)b (2.000 | 87.004)b (1.000 | 92.004);;;; Solution Found
; States evaluated: 105
; Cost: 107.005
; Time 0.08
0.000: (goto_waypoint robot0 wp0 machine1)  [7.000]
0.000: (goto_waypoint robot1 wp0 machine1)  [7.000]
0.000: (goto_waypoint robot2 wp0 wp3)  [18.000]
7.001: (switch_machine_on robot0 machine1)  [5.000]
12.002: (wait_load_at_machine robot1 robot0 machine1)  [15.000]
18.001: (goto_waypoint robot2 wp3 wp1)  [15.000]
27.002: (goto_waypoint robot1 machine1 wp1)  [4.000]
31.002: (ask_unload robot1 wp1)  [5.000]
33.002: (goto_waypoint robot2 wp1 machine1)  [4.000]
36.003: (wait_unload robot1 wp1)  [15.000]
37.002: (wait_load_at_machine robot2 robot0 machine1)  [15.000]
51.003: (goto_waypoint robot1 wp1 machine0)  [8.000]
52.002: (goto_waypoint robot0 machine1 machine0)  [11.000]
59.003: (switch_machine_on robot1 machine0)  [5.000]
64.004: (wait_load_at_machine robot1 robot0 machine0)  [15.000]
75.004: (goto_waypoint robot2 machine1 wp5)  [4.000]
79.004: (ask_unload robot2 wp5)  [5.000]
79.004: (goto_waypoint robot1 machine0 wp3)  [8.000]
84.005: (wait_unload robot2 wp5)  [15.000]
87.004: (ask_unload robot1 wp3)  [5.000]
92.005: (wait_unload robot1 wp3)  [15.000]
99.005: (goto_waypoint robot2 wp5 machine0)  [8.000]
