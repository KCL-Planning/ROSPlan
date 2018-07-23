---
layout: demos
title: ICRA 2017 Tutorial on Planning and Robotics
---
			
The main goal of this tutorial is to show how state-of-the-art formalisms in planning and scheduling (P&S) are used to model robotics domains; some of the challenges and solutions involved in dispatching plans on-board robotic platforms; and to provide an overview of the ROSPlan framework. By so doing, we hope to encourage the rapid development of P&S techniques for robotics. Moreover, the tutorial aims to use the ICRA forum as a means to discuss the main challenges related to planning for autonomous robots (deliberative, reactive, continuous planning and execution etc.).

## Tutorial Content
			
### Part 1: AI Planning for Long-Term Autonomy
			
AI planning for long-term autonomy means an autonomous agent planning for unsupervised periods of days or months. Many interesting robotics problems are problems for long-term autonomy. For example, in service environments, or seabed inspection and maintenance. Planning for these scenarios requires rich models to capture the uncertain and evolving environment, and robust methods of execution. There are many open problems, including the handling of temporal constraints, how to exploit opportunities, and how to handle failure and anticipate it in the future.

This part provides an overview of planning for long-term autonomy. We focus on modelling and solving P&S problems for robotics systems, including some examples of P&S solutions taken from different teams working on various domains.
			
### Part 2: Integrating AI Planning in ROS
			
The second part of the tutorial will focus on integrating AI Planning and Scheduling into a ROS system, and cover the challenges that arise in plan execution, such as dealing with incomplete knowledge, state estimation, error detection and recovery, and long-term learning. This will cover some case studies in ROSPlan (using mobile bases, AUVs, quadcopters).
						
### Part 3: Open discussion on AI Planning for Robot Control
			
The final part of the tutorial will discuss open issues and new opportunities. We will report on the recent news from the Dagstuhl Workshop on Planning and Robotics, held on January 10-15, 2017.
			
## Resources

- ROSPlan is a generic framework for task planning in a ROS system. (<a href="http://kcl-planning.github.io/ROSPlan">http://kcl-planning.github.io/ROSPlan</a>)
- SMTPlan+ is a PDDL+ planner for hybrid systems with continuous non-linear dynamics. (<a href="http://kcl-planning.github.io/SMTPlan">http://kcl-planning.github.io/SMTPlan</a>)

## Prerequisite knowledge
				
A general knowledge of Artificial Intelligence concepts is assumed.
				
## Organisers
			
**Michael Cashmore**
<br>Website: <a href="http://www.inf.kcl.ac.uk/staff/cashmore/">http://www.inf.kcl.ac.uk/staff/cashmore/</a>
<br>Email: michael.cashmore at kcl.ac.uk

**Daniele Magazzeni**
<br>Website: <a href="http://www.inf.kcl.ac.uk/staff/danmag/">http://www.inf.kcl.ac.uk/staff/danmag/</a>
<br>Email: daniele.magazzeni at kcl.ac.uk
