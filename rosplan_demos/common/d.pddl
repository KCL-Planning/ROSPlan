(define (domain hri_contingent)
	(:requirements :strips :typing :disjunctive-preconditions)


	(:types
		;; text displayed to the user
		statement question
	)


	(:predicates
	
		;; the following text can be given to the user
		(available_q ?t - question)
		(available_s ?t - statement)

		;; the statement has been given
		(given ?t - statement)

		;; once a is given b is available
		(follows_s_s ?b - statement ?a - statement)
		(follows_q_s ?b - statement ?a - question)
		(follows_s_q ?b - question ?a - statement)
		(follows_q_q ?b - question ?a - question)

		;; the token initiates a wait
		(pauses ?t - statement)

		;; we are ready to give statements
		(r_HRIreceived)
		(r_HRInotreceived)

		;; the token might be available
		(available_to_check_s ?t - statement)
		(available_to_check_q ?t - question)
	)
	
	;; Display text; from actions:
	;; display_init
	;; display_text_welcome
	;; display_text_@N
	;; display_text_greet
	;; display_text_goodbye
	;; display_image_@N
	(:action display_text
		:parameters (?t - statement)
		:precondition (and
				(r_HRIreceived)
				(available_s ?t)
			)
		:effect (and
			(given ?t)
			(not (available_s ?t))
			(when (pauses ?t) (not (r_HRIreceived)))
			(when (pauses ?t) (r_HRInotreceived))
			(forall (?u - statement) (when (follows_s_s ?u ?t) (available_s ?u)))
			(forall (?u - question) (when (follows_s_q ?u ?t) (available_q ?u)))
			)
	)

	;; Ask for something; from actions:
	;; ask_whichcontinent; / waitfor_continent_@C;
	;; ask_whichcountry_@C; / waitfor_country_@N;
	;; ask_wantphoto; / waitfor_no;
	(:action ask_question
		:parameters (?q - question)
		:precondition (and
			(r_HRIreceived)
			(available_q ?q)
			)
		:effect (and
			(not (available_q ?q))
			(forall (?u - statement) (when (follows_q_s ?u ?q) (available_to_check_s ?u)))
			(forall (?u - question) (when (follows_q_q ?u ?q) (available_to_check_q ?u)))
			)
	)

	;; Check user response
	(:action check_response_q
		:parameters (?q - question)
		:precondition (available_to_check_q ?q)
		:observe (available_q ?q)
	)

	;; Check user response
	(:action check_response_s
		:parameters (?q - statement)
		:precondition (available_to_check_s ?q)
		:observe (available_s ?q)
	)

	;; Wait for something (not question answers); from actions:
	;; waitfor_HRIreceived;
	;; waitfor_screentouched;
	;; waitfor_personhere
	(:action wait_for_hri
		:parameters ()
		:precondition (r_HRInotreceived)
		:effect (and
			(r_HRIreceived)
			(not (r_HRInotreceived))
			)
	)
)



