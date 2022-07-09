(define (domain robocluedo)

(:requirements 
	:strips 
	:typing 
	:equality 
	:universal-preconditions 
	:durative-actions
)

(:types
	hypothesis-ID
	who where what
)

(:predicates
	;; initialization of the system
	(kb-init )
	
	;; predicates characterization
	(hyp-active ?id - hypothesis-ID)
	(hyp-discarded ?id - hypothesis-ID)
	(hyp-complete ?id - hypothesis-ID)
)

(:functions
	;; LOOP MAX -- max number of hints to get from the environment
	(max-loop )
	
	;; predicates characterization
	(count-who-for-ID ?id - hypothesis-ID)
	(count-what-for-ID ?id - hypothesis-ID)
	(count-where-for-ID ?id - hypothesis-ID)
)



;; === INIT SYSTEM === ;;

;; set up the ontology with the actual state
(:durative-action init-planning-system
	:parameters ( )
	
	:duration (= ?duration 1)
	
	:condition (not (kb-init))
	
	:effect (and
		(kb-init )
	)
)




)
