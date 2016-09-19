CLG Planner run information
  by Alexandre Albore

In the current directory are preesent the following files:
cf2cs: Translator from Contingent Problem P to X(P) and X+(P)
clg: Closed Loop Greedy Planner
run-clg.sh: Script to run both the translator and the planner
README: this file

The script is easy to use and to modify.

Syntax: 
run-clg.sh -0/1 {-q N} domain.pddl problem.pddl
  -0   off-line mode
  -1   on-line mode
  -q   sensing probability, an int between 0--100 (on-line mode only)

NB: -q is an optional parameter for on-line mode.
  When performing a set of tests it is good to change it from different values between 0 and 100. 
  It indicates the probability to take the first option in a (boolean) sensing action.

Example:
(off-line mode, i.e. generating the full plan for domain d.pddl and problem p.pddl)
./run-clg.sh -1 d.pddl p.pddl

(on-line mode, i.e. generatin a single execution for domain d.pddl and problem p.pddl)
./run-clg.sh -0 d.pddl p.pddl

(on-line mode with even probability on branching for domain d.pddl and problem p.pddl)
./run-clg.sh -0 -q 50 d.pddl p.pddl

NB: Domain and problem description have to be in the binaries directory. 
   Otherwise change the path for CLG option -p into the script.


This is a joint work with Hector Palacios and Hector Geffner.
Last update 19-07-2009
