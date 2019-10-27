(define (domain feeding)
    (:requirements :strips)
    
    (:predicates (SPOON ?x) (GRIPPER ?x) (POTATO ?x) (MOUTH ?x) (INIT ?x)
                 (in-spoon ?x ?y)
                 (engaged ?x ?y)
    )
    
    (:action engage
        :parameters (?x ?y ?z)
        :precondition (and (SPOON ?x) (or (POTATO ?y) (MOUTH ?y) (INIT ?y)) (or (POTATO ?z) (MOUTH ?z) (INIT ?z)) (not (engaged ?x ?z)) (engaged ?x ?y))
        :effect (and (engaged ?x ?z) (not (engaged ?x ?y)))
    )
    
    (:action load-spoon
        :parameters (?x ?y)
        :precondition (and (SPOON ?x) (POTATO ?y) (not (in-spoon ?y ?x)) (engaged ?x ?y))
        :effect (in-spoon ?y ?x)
    )
)