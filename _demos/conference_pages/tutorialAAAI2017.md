---
layout: demos
title: AAAI 2017 Tutorial on AI Planning for Robotics
---
						
The main goal of this tutorial is to show how state-of-the-art formalisms in planning and scheduling (P&S) are used to model robotics domains; some of the challenges and solutions involved in dispatching plans on-board robotic platforms; and to provide an overview of the ROSPlan framework. By so doing, we hope to encourage the rapid development of P&S techniques for robotics. Moreover, the tutorial aims to use the ICRA forum as a means to discuss the main challenges related to planning for autonomous robots (deliberative, reactive, continuous planning and execution etc.).
			
## Tutorial Content
			
### Part 1: AI Planning in Hybrid Systems
			
Hybrid systems are systems with both continuous control variables and discrete logical modes. Many interesting robotics problems are hybrid systems. Planning in these domains requires rich models to capture the interaction between discrete and continuous change, and methods for reasoning with temporal, spatial and continuous constraints. PDDL+ is the extension of PDDL for modelling hybrid systems, through continuous processes and events.
			
This part provides an overview of planning for hybrid systems, showing some concrete examples on how to model hybrid domains using PDDL+ and an overview of existing techniques for PDDL+ planning. We focus on modelling and solving P&S problems for robotics systems, including some examples of P&S solutions for robotic platforms, taken from different teams working on various domains.
			
### Part 2: Integrating AI Planning in ROS
			
The second part of the tutorial will focus on integrating AI Planning and Scheduling into a ROS system, and cover the challenges that arise, such as dealing with incomplete knowledge, state estimation, plan execution, and error detection and recovery. This will cover some case studies in ROSPlan (using mobile bases, AUVs, quadcopters).
			
### Part 3: Open discussion on AI Planning for Robot Control
			
The final part of the tutorial will discuss open issues and new opportunities. We expect to report on the recent news from the Dagstuhl Workshop on Planning and Robotics, to be held on January 10-15, 2017.
			
## Resources
			
- Tutorial slides [AAAI17_PlanningAndRobotics.pdf](slides/AAAI17_PlanningAndRobotics.pdf)
- ROS Indigo [http://wiki.ros.org/indigo](http://wiki.ros.org/indigo)
- ROSPlan [http://kcl-planning.github.io/ROSPlan](http://kcl-planning.github.io/ROSPlan)
			
## Organisers
			
**Michael Cashmore**  
Website: <a href="http://www.inf.kcl.ac.uk/staff/cashmore/">http://www.inf.kcl.ac.uk/staff/cashmore/</a>  
Email: michael.cashmore at kcl.ac.uk

**Daniele Magazzeni**  
Website: <a href="http://www.inf.kcl.ac.uk/staff/danmag/">http://www.inf.kcl.ac.uk/staff/danmag/</a>  
Email: daniele.magazzeni at kcl.ac.uk
