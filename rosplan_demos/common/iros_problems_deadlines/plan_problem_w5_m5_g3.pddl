Number of literals: 61
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 11.000
b (10.000 | 10.000)b (9.000 | 15.001)b (8.000 | 15.001)b (7.000 | 30.002)b (6.000 | 44.002)b (5.000 | 46.002)b (4.000 | 63.002)b (3.000 | 68.002)b (2.000 | 106.002)b (1.000 | 111.002);;;; Solution Found
; States evaluated: 102
; Cost: 126.003
; Time 0.10
0.000: (goto_waypoint robot0 wp0 machine2)  [10.000]
0.000: (goto_waypoint robot1 wp0 machine2)  [10.000]
0.000: (goto_waypoint robot2 wp0 wp2)  [10.000]
10.001: (switch_machine_on robot0 machine2)  [5.000]
10.001: (goto_waypoint robot2 wp2 machine4)  [4.000]
14.001: (switch_machine_on robot2 machine4)  [5.000]
15.002: (wait_load_at_machine robot1 robot0 machine2)  [15.000]
30.002: (goto_waypoint robot1 machine2 wp4)  [11.000]
30.002: (goto_waypoint robot0 machine2 machine4)  [14.000]
41.002: (ask_unload robot1 wp4)  [5.000]
44.002: (wait_load_at_machine robot2 robot0 machine4)  [15.000]
46.003: (wait_unload robot1 wp4)  [15.000]
59.002: (goto_waypoint robot0 machine4 machine2)  [14.000]
61.003: (goto_waypoint robot1 wp4 machine2)  [11.000]
73.002: (wait_load_at_machine robot1 robot0 machine2)  [15.000]
85.002: (goto_waypoint robot2 machine4 wp2)  [4.000]
88.002: (goto_waypoint robot1 machine2 wp3)  [18.000]
89.002: (ask_unload robot2 wp2)  [5.000]
94.003: (wait_unload robot2 wp2)  [15.000]
106.002: (ask_unload robot1 wp3)  [5.000]
109.003: (goto_waypoint robot2 wp2 machine2)  [17.000]
111.003: (wait_unload robot1 wp3)  [15.000]
