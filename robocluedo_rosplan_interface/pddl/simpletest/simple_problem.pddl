(define (problem task)
(:domain dom)
(:objects
    b1 b2 b3 - boolobj
)
(:init
    (b-not-true b1)
    (b-not-true b2)
    (b-not-true b3)
    (stop)
)
(:goal (and
    (b-true b1)
    (b-true b2)
    (b-true b3)
    (stop)
))
)
