Number of literals: 12
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%] [130%] [140%] [150%] [160%] [170%] [180%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%] [130%] [140%] [150%] [160%] [170%] [180%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
Initial heuristic = 6.000
b (5.000 | 0.000)b (4.000 | 0.000)b (3.000 | 0.001)b (2.000 | 0.002)b (1.000 | 0.003)
; Plan found with metric 0.004
; States evaluated so far: 7
; Time 0.00
0.000: (load-truck mydvd truck amazon)  [0.001]
0.000: (walk driver home amazon)  [0.001]
0.001: (board-truck driver truck amazon)  [0.001]
0.002: (drive-truck truck amazon london driver)  [0.001]
0.003: (drive-truck truck london myhouse driver)  [0.001]
0.004: (unload-truck mydvd truck myhouse)  [0.001]

 * All goal deadlines now no later than 0.004

Resorting to best-first search
b (5.000 | 0.000)b (4.000 | 0.000)b (3.000 | 0.001)b (2.000 | 0.002)b (1.000 | 0.003)
Problem Unsolvable
;;;; Solution Found
; States evaluated: 944
; Cost: 0.004
; Time 0.20
0.000: (load-truck mydvd truck amazon)  [0.001]
0.000: (walk driver home amazon)  [0.001]
0.001: (board-truck driver truck amazon)  [0.001]
0.002: (drive-truck truck amazon london driver)  [0.001]
0.003: (drive-truck truck london myhouse driver)  [0.001]
0.004: (unload-truck mydvd truck myhouse)  [0.001]
