Number of literals: 59
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 10.000
b (9.000 | 4.000)b (8.000 | 9.001)b (7.000 | 24.002)b (6.000 | 50.002)b (5.000 | 50.002)b (4.000 | 101.003)b (3.000 | 101.003)b (2.000 | 152.004)b (1.000 | 157.004);;;; Solution Found
; States evaluated: 186
; Cost: 172.005
; Time 0.14
0.000: (goto_waypoint robot0 wp0 machine0)  [4.000]
0.000: (goto_waypoint robot1 wp0 machine0)  [4.000]
0.000: (goto_waypoint robot2 wp0 machine0)  [4.000]
4.001: (switch_machine_on robot0 machine0)  [5.000]
4.001: (goto_waypoint robot2 machine0 machine2)  [11.000]
9.002: (wait_load_at_machine robot0 robot1 machine0)  [15.000]
15.001: (switch_machine_on robot2 machine2)  [5.000]
24.002: (goto_waypoint robot0 machine0 wp2)  [18.000]
24.002: (goto_waypoint robot1 machine0 wp1)  [26.000]
42.002: (ask_unload robot0 wp2)  [5.000]
47.003: (wait_unload robot0 wp2)  [15.000]
50.003: (goto_waypoint robot1 wp1 machine2)  [18.000]
62.003: (goto_waypoint robot0 wp2 wp1)  [17.000]
68.003: (wait_load_at_machine robot2 robot1 machine2)  [15.000]
79.004: (goto_waypoint robot0 wp1 machine2)  [18.000]
83.003: (goto_waypoint robot1 machine2 wp1)  [18.000]
101.004: (goto_waypoint robot1 wp1 machine2)  [18.000]
115.004: (goto_waypoint robot2 machine2 wp0)  [10.000]
119.004: (wait_load_at_machine robot1 robot0 machine2)  [15.000]
125.004: (ask_unload robot2 wp0)  [5.000]
130.005: (wait_unload robot2 wp0)  [15.000]
134.004: (goto_waypoint robot1 machine2 wp1)  [18.000]
145.005: (goto_waypoint robot2 wp0 wp1)  [27.000]
152.004: (ask_unload robot1 wp1)  [5.000]
157.005: (wait_unload robot1 wp1)  [15.000]
