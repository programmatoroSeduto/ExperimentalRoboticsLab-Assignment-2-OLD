# Simple Test 

A PDDL2.1 domain to test the ROSplan in a very simple case. 

## First version -- very simple

### Domanin

```lisp

(define (domain dom)

(:requirements 
	:typing
	:adl
	:durative-actions
	:disjunctive-preconditions
	:universal-preconditions
	:fluents
)

(:types 
	boolobj
)

(:predicates
	(b-not-true ?b - boolobj)
	(b-true ?b - boolobj)
)

(:functions
	
)

(:durative-action make-true
	:parameters ( ?b - boolobj )

	:duration (= ?duration 1)

	:condition (and
		(at start (b-not-true ?b))
	)
	
	:effect (and
		(at end (not (b-not-true ?b)))
		(at end (b-true ?b))
	)
)

(:durative-action make-false
	:parameters ( ?b - boolobj )

	:duration (= ?duration 1)

	:condition (and
		(at start (b-true ?b))
	)
	
	:effect (and
		(at end (not (b-true ?b)))
		(at end (b-not-true ?b))
	)
)

)

```

### Problem

```lisp

(define (problem task)

(:domain dom)

(:objects
	b1 b2 b3 - boolobj
)

(:init
	(b-true b1)
	(b-not-true b2)
)

(:goal (and
	(b-true b1) (not (b-not-true b1))
	(b-true b2) (not (b-not-true b2))
	(b-not-true b3) (not (b-true b3))
)
)

)

```

## A more sophisticated version of the problem

### Domain file

```lisp

(define (domain dom)

(:requirements 
	:typing
	:adl
	:durative-actions
	:disjunctive-preconditions
	:universal-preconditions
	:fluents
)

(:types 
	boolobj
	key
)

(:predicates
	(b-not-true ?b - boolobj)
	(b-true ?b - boolobj)
	(start ) (stop ) (signal-stop )
)

(:functions
	
)

(:durative-action set-start
	:parameters ( )

	:duration (= ?duration 1)

	:condition (at start (stop))
	
	:effect (and
		(at end (not (stop)))
		(at end (start))
	)
)

(:durative-action make-true
	:parameters (?b - boolobj)

	:duration (= ?duration 1)

	:condition (and
		(at start (b-not-true ?b))
		(at start (start))
	)
	
	:effect (and
		(at end (not (b-not-true ?b)))
		(at end (b-true ?b))
	)
)

(:durative-action make-false
	:parameters ( ?b - boolobj )

	:duration (= ?duration 1)

	:condition (and
		(at start (b-true ?b))
		(at start (start))
	)
	
	:effect (and
		(at end (not (b-true ?b)))
		(at end (b-not-true ?b))
	)
)

(:durative-action set-signal-stop
	:parameters ( )
	
	:duration (= ?duration 1)
	
	:condition (at start (start ))
	
	:effect (and
		(at end (signal-stop ))
	)
)

(:durative-action set-stop
	:parameters ( )

	:duration (= ?duration 1)

	:condition (and 
		(at start (start ))
		(at start (signal-stop ))
	)
	
	:effect (and
		(at end (not (start )))
		(at end (stop ))
		(at end (not (signal-stop )))
	)
)

)

```

### Problem file

```lisp

(define (problem task)

(:domain dom)

(:objects
	b1 b2 b3 - boolobj
)

(:init
	(b-true b1)
	(b-not-true b2)
	(stop)
)

(:goal (and
	(b-true b1) (not (b-not-true b1))
	(b-true b2) (not (b-not-true b2))
	(b-not-true b3) (not (b-true b3))
	(stop) (not (start))
)
)

)

```

