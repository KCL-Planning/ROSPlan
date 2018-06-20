---
title: Girona 500 AUV Valve Turning
layout: page
---

## Contents:

- Description
- Video
- PDDL domain

## Description

This project was funded by the EU FP7 Project PANDORA. University of Girona, King's College London, Italian Institute of Technology, and the National Technical University of Athens combine visual-based detection and localization systems, robust control schemes, learning-by-demonstration techniques for intervention, and temporal planning. The Girona 500 AUV operates a valve panel many times in several disturbances: water currents, blocked valves, unknown panel position, and panel occlusion. The positive results encourage the use of, in the short-term, autonomous robots operating in subsea facilities performing interventions with a cost benefit, in comparison with teleoperated vehicles.

The long-term mission was carried out in a water tank of 16x8x5 meters using the Girona 500 AUV equipped with a 4-Degrees of Freedom electrical manipulator, a Stereo Camera and a specifically designed end-effector with an in-hand camera and force-torque sensor. To make the environment more realistic, two propellers, able to generate up to 14Kg of thrust each, were placed close to the panel in order to generate lateral water currents. Experiments were performed in a completely autonomous mode, the vehicle ran on its own batteries and all required processing was performed with the on-board computers.

In the experiment, the vehicle had to locate the intervention panel among different locations and modify the valve handles to achieve different panel configurations. During more than 3 hours, the AUV was required changed the panel to 9 different configurations in various time windows, which required the turning of 29 valves.

You can find more details about this application in the following papers:

*ROSPlan: Planning in the Robot Operating System*  
M. Cashmore, M. Fox, D. Long, D. Magazzeni, B. Ridder, A. Carrera, N. Palomeras, N. Hurtos, M. Carreras.  
Proceedings of the 25th International Conference on Automated Planning and Scheduling (ICAPS-15). June 2015.

*Toward persistent autonomous intervention in a subsea panel*  
N. Palomeras, A. Carrera, N. Hurts, G. C. Karras, C. P. Bechlioulis, M. Cashmore, D. Magazzeni, D. Long, M. Fox, K. J. Kyriakopoulos, P. Kormushev, J. Salvi and M. Carreras<br>
Autonomous Robots, 2015

## Video

The Girona 500 AUV from the University of Girona [Underwater Vision and Robotics Team](http://cirs.udg.edu) searches for and examines valve panels, and turns valves within time windows.

<div class="media"><iframe src="https://www.youtube.com/embed/klHBFiwgOZY" frameborder="0" allowfullscreen></iframe></div>

## PDDL domain

```
(define (domain pandora-domain-persistent)
(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
		waypoint 
		inspectionpoint
		panel
		valve
		vehicle)

(:predicates
	(connected ?wp1 ?wp2 - waypoint)
	(at ?v - vehicle ?wp - waypoint)
	(near ?v - vehicle ?wp - waypoint)
	(cansee ?v - vehicle ?wp - waypoint ?ip - inspectionpoint)
	(canexamine ?v - vehicle ?p - panel ?wp - waypoint)
	(canreach ?v - vehicle ?p - panel ?wp - waypoint)
	(examined ?p - panel)
	(on ?a - valve ?p - panel)
	(turned ?a - valve)
	(chainat ?wp - waypoint)
)

(:functions
		(observed ?ip - inspectionpoint)
		(obs ?ip - inspectionpoint ?wp - waypoint)
		(distance ?wp1 ?wp2 - waypoint) 
)

(:durative-action do_hover
	:parameters (?v - vehicle ?from ?to - waypoint)
	:duration ( = ?duration (* (distance ?from ?to) 10))
	:condition (and (at start (at ?v ?from)) (at start (connected ?from ?to)))
	:effect (and (at start (not (at ?v ?from))) (at end (at ?v ?to)))
)

(:durative-action do_hover_fast
	:parameters (?v - vehicle ?from ?to - waypoint)
	:duration ( = ?duration (* (distance ?from ?to) 5))
	:condition (and (at start (at ?v ?from)) (at start (connected ?from ?to)))
	:effect (and (at start (not (at ?v ?from))) (at end (near ?v ?to)))
)

(:durative-action correct_position
	:parameters (?v - vehicle ?target - waypoint)
	:duration ( = ?duration 10)
	:condition (at start (near ?v ?target))
	:effect (and (at start (not (near ?v ?target))) (at end (at ?v ?target)))
)

(:durative-action valve_state
	:parameters (?v - vehicle ?wp - waypoint ?p - panel)
	:duration ( = ?duration 10)
	:condition (and (at start (at ?v ?wp)) (at start (canexamine ?v ?p ?wp)))
	:effect (and (at start (not (canexamine ?v ?p ?wp))) (at end (examined ?p)))
)

(:durative-action observe
	:parameters (?v - vehicle ?wp - waypoint ?ip - inspectionpoint)
	:duration ( = ?duration 10)
	:condition (and (at start (at ?v ?wp)) (at start (cansee ?v ?wp ?ip)))
	:effect (and 
			(and (at start (not (cansee ?v ?wp ?ip))) (at end (increase (observed ?ip) (obs ?ip ?wp))))
			(and (at start (not (at ?v ?wp))) (at end (near ?v ?wp))))
)

(:durative-action turn_valve
	:parameters (?v - vehicle ?wp - waypoint ?p - panel ?a - valve)
	:duration ( = ?duration 30)
	:condition (and (and (at start (at ?v ?wp)) (at start (canreach ?v ?p ?wp))) (at start (on ?a ?p)))
	:effect (and (at end (turned ?a)) (and (at start (not (at ?v ?wp))) (at end (near ?v ?wp))))
)

(:durative-action follow_chain
	:parameters (?v - vehicle ?from - waypoint)
	:duration ( = ?duration 120)
	:condition (and (at start (at ?v ?from)) (at start (chainat ?from)))
	:effect (and (at start (not (at ?v ?from))) (at end (near ?v ?from)))
)
)
```
