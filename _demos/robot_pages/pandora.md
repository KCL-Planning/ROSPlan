---
layout: page
title: Nessie AUV Inspection task
---

## Contents:

- Video
- PDDL domain

This project was funded by the EU FP7 Project PANDORA. PANDORA was a three-year project to develop plan-based intelligent control of underwater vehicles operating for long periods without human intervention.

You can find more details about this application in the following paper:  
*Planning Inspection Tasks for AUVs*  
M. Cashmore, M. Fox, T. Larkworthy, D. Long, D. Magazzeni.  
Proceedings of OCEANS'13 MTS/IEEE.

## Video

The Nessie AUV from the [Ocean Systems Lab](http://osl.eps.hw.ac.uk) of [Heriot-Watt University](http://www.hw.ac.uk) inspects pipes and pillars in the diver centre at Fort William, Scotland.

<div class="media"><iframe src="https://www.youtube.com/embed/PrJOBmA6UPU" frameborder="0" allowfullscreen></iframe></div>

## PDDL domain

```
(define (domain pandora-domain-inspection)
(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
waypoint 
inspectionpoint
vehicle)

(:predicates
(connected ?wp1 ?wp2 - waypoint)
(at ?v - vehicle ?wp - waypoint)
(near ?v - vehicle ?wp - waypoint)
(cansee ?v - vehicle ?wp - waypoint ?ip - inspectionpoint)
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

(:durative-action observe
:parameters (?v - vehicle ?wp - waypoint ?ip - inspectionpoint)
:duration ( = ?duration 10)
:condition (and (at start (at ?v ?wp)) (at start (cansee ?v ?wp ?ip)))
:effect (and 
	(and (at start (not (cansee ?v ?wp ?ip))) (at end (increase (observed ?ip) (obs ?ip ?wp))))
	(and (at start (not (at ?v ?wp))) (at end (near ?v ?wp))))
)
)
```
