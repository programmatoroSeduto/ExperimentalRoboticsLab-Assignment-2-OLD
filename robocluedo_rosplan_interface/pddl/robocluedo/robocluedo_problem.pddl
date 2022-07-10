(define (problem task)

(:domain robocluedo)

(:objects
	;; ontology 
	ID1 ID2 ID3 ID4 ID5 ID6 - hypID
	
	;; environment
	w1 w2 w3 w4 - waypoint
)

(:init
	;; problem metadata
	(= (number-of-ids-in-the-problem ) 6)
		
	;; init state of the system
	(pre-init )
	
	;; moves
	(= (remaining-moves ) 3)
	
	;; quality of the solution
	(= (solution-quality ) 0)
	
	;; environment data
	(passage w1 w2 )
		(passage w1 w3 )
		(passage w1 w4 )
		(passage w1 center )
	(passage w2 w1 )
		(passage w2 w3 )
		(passage w2 w4 )
		(passage w2 center )
	(passage w3 w1 )
		(passage w3 w2 )
		(passage w3 w4 )
		(passage w3 center )
	(passage w4 w1 )
		(passage w4 w2 )
		(passage w4 w3 )
		(passage w4 center )
	(passage center w1 )
		(passage center w2 )
		(passage center w3 )
		(passage center w4 )
	
	;; navigation system
	(robot-position center )
	(not-explored w1 )
	(not-explored w2 )
	(not-explored w3 )
	(not-explored w4 )
	
	;; manipulator
	(manipulator-off )
	
	;; hints gathering system
	(can-give-hint w1 )
		(not-acquired-hint w1 )
	(can-give-hint w2 )
		(not-acquired-hint w2 )
	(can-give-hint w3 )
		(not-acquired-hint w3 )
	(can-give-hint w4 )
		(not-acquired-hint w1 )
	
	;; hypotheses classification
	(h-open ID1 )
		(= (h-count-who ID1 ) 0)
		(= (h-count-where ID1 ) 0)
		(= (h-count-what ID1 ) 0)
	(h-open ID2 )
		(= (h-count-who ID2 ) 0)
		(= (h-count-where ID2 ) 0)
		(= (h-count-what ID2 ) 0)
	(h-open ID3 )
		(= (h-count-who ID3 ) 0)
		(= (h-count-where ID3 ) 0)
		(= (h-count-what ID3 ) 0)
	(h-open ID4 )
		(= (h-count-who ID4 ) 0)
		(= (h-count-where ID4 ) 0)
		(= (h-count-what ID4 ) 0)
	(h-open ID5 )
		(= (h-count-who ID5 ) 0)
		(= (h-count-where ID5 ) 0)
		(= (h-count-what ID5 ) 0)
	(h-open ID6 )
		(= (h-count-who ID6 ) 0)
		(= (h-count-where ID6 ) 0)
		(= (h-count-what ID6 ) 0)
	
	;; hypothesis counting
	(= (h-count-open ) 6)
	(= (h-count-complete ) 0)
	(= (h-count-discard ) 0)
)

(:goal (elementary-whatson ))

(:metric maximize (solution-quality ))

)
