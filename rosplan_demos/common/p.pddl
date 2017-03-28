(define (problem rai2) 
(:domain hri_contingent) 
(:objects
t_init t_name t_welcome t_intro t_news t_news1 t_news2 t_news3 img_news1 img_news2 img_news3 t_joke t_goodbye img_coaches img_activity - statement
q_activity q_news - question)


		
(:init 
(r_HRInotreceived)
(available_s t_init)
(follows_s_s t_name t_init)
(follows_s_s t_intro t_name)
(pauses t_intro)
(follows_s_s img_activity t_intro)
(follows_s_q q_activity img_activity)
	(unknown (available_s t_news))
	(unknown (available_s t_joke))
	(oneof
		(available_s t_news)
		(available_s t_joke)
	)		
	(follows_q_s t_news q_activity)
	(follows_s_q q_news t_news)
		(unknown (available_s t_news1))
		(unknown (available_s t_news2))
		(unknown (available_s t_news3))
		(oneof
			(available_s t_news1)
			(available_s t_news2)
			(available_s t_news3)
		)
		(follows_q_s t_news1 q_news)
		(follows_q_s t_news2 q_news)
		(follows_q_s t_news3 q_news)
		(follows_s_s img_news1 t_news1)
		(follows_s_s img_news2 t_news2)
		(follows_s_s img_news3 t_news3)
		(pauses img_news1)
		(pauses img_news2)
		(pauses img_news3)
		(follows_s_s img_coaches img_news1)
		(follows_s_s img_coaches img_news2)
		(follows_s_s img_coaches img_news3)
	(follows_q_s t_joke q_activity)
	(follows_s_s img_coaches t_joke)
(follows_s_s t_goodbye img_coaches)
)

(:goal (and
(given t_goodbye)
)))
