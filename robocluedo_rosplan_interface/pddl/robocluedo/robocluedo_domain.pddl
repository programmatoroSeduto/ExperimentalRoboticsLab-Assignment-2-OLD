(define (domain robocluedo)

(:requirements 
	:strips 
	:typing 
	:equality 
	:universal-preconditions 
	:durative-actions
	:negative-preconditions
	:fluents
)

(:types
	;; reasoning system
	hypothesisID
	; who where what
	
	;; movement system
	waypoint
)

(:predicates
	;; workig phases
	(investigating )
	(ready-to-acquire )
	(acquiring )
	
	;; initialization of the system
	(kb-init )
	
	;; predicates characterization
	(hyp-active ?id - hypothesisID)
	(hyp-discarded ?id - hypothesisID)
	(hyp-complete ?id - hypothesisID)
	
	;; hypothesis and its elements
	(hyp-who ?id - hypothesisID ?who - who)
	(hyp-where ?id - hypothesisID ?where - where)
	(hyp-what ?id - hypothesisID ?what - what)
	
	;; movement system
	(robot-position ?p - waypoint)
	(last-place-visited ?p - waypoint)
	(is-center ?p - waypoint)
)

(:functions
	;; LOOP MAX -- max number of hints to get from the environment
	(max-loop )
	
	;; predicates characterization
	(count-who-for-ID ?id - hypothesisID)
	(count-what-for-ID ?id - hypothesisID)
	(count-where-for-ID ?id - hypothesisID)
)



;; === INIT SYSTEM === ;;

;; set up the ontology with the actual state
(:durative-action init-planning-system
	:parameters ( )
	
	:duration (= ?duration 1)
	
	:condition (at start (and
		(not (kb-init))
		(not (investigating))
	))
	
	:effect (at end (and
		(kb-init )
		(investigating )
	))
)



;; === MOVEMENT SYSTEM === ;;

(:durative-action move-to
	:parameters ( ?from ?to - waypoint )
	
	:duration (= ?duration 1)
	
	:condition (at start (and
		(investigating )
		(not (ready-to-acquire ))
		(not (acquiring ))
		
		(robot-position ?from)
		(last-place-visited ?from)
		(not (last-place-visited ?to))
		(not (is-center ?to))
		
		(> (max-loop ) 0)
	))
	
	:effect (at end (and
		(not (last-place-visited ?from))
		(last-place-visited ?to)
		
		(not (robot-position ?from))
		(robot-position ?to)
		
		(decrease (max-loop ) 1)
		
		(ready-to-acquire )
	))
)




)
