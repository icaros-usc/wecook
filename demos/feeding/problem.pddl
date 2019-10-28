(define (problem feeding_pb)
    (:domain feeding)
    
    (:objects spoon1 potato1 gripper1 mouth1 init1)
    
    (:init (SPOON spoon1) (POTATO potato1) (GRIPPER gripper1) (MOUTH mouth1) (INIT init1)
           (not (in-spoon potato1 spoon1))
           (not (engaged spoon1 mouth1))
           (not (engaged spoon1 potato1))
           (engaged spoon1 init1)
    )
    
    (:goal (and (in-spoon potato1 spoon1) (engaged spoon1 mouth1)))
)