(define (problem task)

(:domain robocluedo)

(:objects
	;; hypothesis IDs
	ID1 ID2 ID3 ID4 ID5 ID6 - hypothesisID
	
	;; hypothesis elements
	; missScarlett colonelMustard mrsWhite mrGreen mrsPeacock profPlum - who
	; candlestick dagger leadPipe revolver rope spanner - what
	; conservatory lounge kitchen library hall study bathroom diningRoom billiardRoom - where
	
	;; navigation system
	wp1 wp2 wp3 wp4 center - waypoint
	
)

(:init
	;; the system must be initalized
	; (not (kb-init ))
	
	;; by default max loop is 3
	(= (max-loop ) 3)
	
	;; hypothesis allocation
	;(hyp-active ID1)
	;; not (hyp-discarded ID1)
	;; not (hyp-complete ID1)
	;(= (count-who-for-ID ID1) 0)
	;(= (count-what-for-ID ID1) 0)
	;(= (count-where-for-ID ID1) 0)
	;(hyp-active ID2)
	;; not (hyp-discarded ID2)
	;; not (hyp-complete ID2)
	;(= (count-who-for-ID ID2) 0)
	;(= (count-what-for-ID ID2) 0)
	;(= (count-where-for-ID ID2) 0)
	;(hyp-active ID3)
	;; not (hyp-discarded ID3)
	;; not (hyp-complete ID3)
	;(= (count-who-for-ID ID3) 0)
	;(= (count-what-for-ID ID3) 0)
	;(= (count-where-for-ID ID3) 0)
	;(hyp-active ID4)
	;; not (hyp-discarded ID4)
	;; not (hyp-complete ID4)
	;(= (count-who-for-ID ID4) 0)
	;(= (count-what-for-ID ID4) 0)
	;(= (count-where-for-ID ID4) 0)
	;(hyp-active ID5)
	;; not (hyp-discarded ID5)
	;; not (hyp-complete ID5)
	;(= (count-who-for-ID ID5) 0)
	;(= (count-what-for-ID ID5) 0)
	;(= (count-where-for-ID ID5) 0)
	;(hyp-active ID6)
	;; not (hyp-discarded ID6)
	;; not (hyp-complete ID6)
	;(= (count-who-for-ID ID6) 0)
	;(= (count-what-for-ID ID6) 0)
	;(= (count-where-for-ID ID6) 0)
	
	;; navigation system
	(is-center center)
	(robot-position center)
	(last-place-visited center)
)

(:goal
	(robot-position wp3)
)

)
