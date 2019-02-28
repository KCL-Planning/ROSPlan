Number of literals: 65
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 10.000
b (9.000 | 10.000)b (8.000 | 15.001)b (7.000 | 30.002)b (6.000 | 39.002)b (5.000 | 44.002)b (4.000 | 105.003)b (3.000 | 110.003)b (2.000 | 160.003)b (1.000 | 165.003);;;; Solution Found
; States evaluated: 221
; Cost: 180.004
; Time 0.24
0.000: (goto_waypoint robot0 wp0 machine0)  [10.000]
0.000: (goto_waypoint robot1 wp0 machine0)  [10.000]
0.000: (goto_waypoint robot2 wp0 wp1)  [31.000]
10.001: (switch_machine_on robot0 machine0)  [5.000]
15.002: (wait_load_at_machine robot1 robot0 machine0)  [15.000]
30.002: (goto_waypoint robot1 machine0 wp4)  [9.000]
31.001: (goto_waypoint robot2 wp1 wp0)  [31.000]
39.002: (ask_unload robot1 wp4)  [5.000]
44.003: (wait_unload robot1 wp4)  [15.000]
59.003: (goto_waypoint robot1 wp4 machine0)  [9.000]
62.002: (goto_waypoint robot2 wp0 machine1)  [20.000]
68.003: (wait_load_at_machine robot1 robot0 machine0)  [15.000]
82.002: (switch_machine_on robot2 machine1)  [5.000]
83.003: (goto_waypoint robot0 machine0 wp1)  [22.000]
87.002: (goto_waypoint robot2 machine1 wp0)  [20.000]
105.004: (goto_waypoint robot0 wp1 machine3)  [13.000]
107.003: (goto_waypoint robot1 machine0 wp1)  [22.000]
107.003: (goto_waypoint robot2 wp0 machine3)  [19.000]
118.004: (switch_machine_on robot0 machine3)  [5.000]
126.003: (wait_load_at_machine robot2 robot0 machine3)  [15.000]
129.003: (ask_unload robot1 wp1)  [5.000]
134.004: (wait_unload robot1 wp1)  [15.000]
141.003: (goto_waypoint robot2 machine3 wp0)  [19.000]
149.004: (goto_waypoint robot1 wp1 wp0)  [31.000]
160.003: (ask_unload robot2 wp0)  [5.000]
165.004: (wait_unload robot2 wp0)  [15.000]
