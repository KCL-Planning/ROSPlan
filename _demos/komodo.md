---
layout: page
title: Demonstration with Blocks
---

## Contents:

- Video
- PDDL domain
- PDDL problem instance

Built as a final robotics project in a graduate course, the Komodo BlocksWorld project is an attempt to bring the well known blocksworld domain into the real world, combining planning techniques with robotics.  
By Maor Ashkenazi and Lior-Zur Lotan, Ben-Gurion University, Israel  
maorash (at) cs.bgu.ac.il  
llutan (at) cs.bgu.ac.il

## Video

<div class="media"><iframe src="https://www.youtube.com/embed/JC8jvRs1yVE" frameborder="0" allowfullscreen></iframe></div>

## PDDL domain

```
(define (domain blocks-domain)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types block_t)

(:predicates 
	(inhand ?block - block_t) 
	(emptyhand) 
	(not_emptyhand) 
	(on ?block ?on_block - block_t) 
	(clear ?block - block_t)
)

(:durative-action pick_up
    :parameters (?block ?from_block - block_t)
	:duration ( = ?duration 15)
    :condition (and 
		(at start (emptyhand))
		(over all (clear ?block))
		(at start (on ?block ?from_block)))
    :effect (and 
		(at end (inhand ?block))
		(at end (clear ?from_block))
		(at start (not (emptyhand)))
		(at start (not_emptyhand))
		(at start (not (on ?block ?from_block))))
)

(:durative-action put_down
    :parameters (?block ?on_block - block_t)
	:duration ( = ?duration 17)
    :condition (and 
		(at start (not_emptyhand))
		(over all (clear ?on_block))
		(at start (inhand ?block)))
    :effect (and 
		(at end (on ?block ?on_block))
		(at end (emptyhand))
		(at end (not (not_emptyhand)))
		(at start (not (inhand ?block)))
		(at start (not (clear ?on_block))))
  )
)
```

## PDDL problem instance

```
(define (problem blocks-domiain_task)
(:domain blocks-domain)
(:objects
	a b c d t1 t2 t3 t4 - block_t
)
(:init 
	(clear b)
	(clear t3)
	(clear t2)
	(clear c)
	(emptyhand)
	(on a t1)
	(on b a)
	(on d t4)
	(on c d)
)
(:goal (and
	(on d t2)
	(on a d)
	(on c a)
	(on b t4)
)))
```
