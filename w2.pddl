(define (problem warehouse_problem-problem)
 (:domain warehouse_problem-domain)
 (:objects
 )
 (:init (at_ start_location) (= (total-cost) 0))
 (:goal (and (delivered target0) (delivered target1) (delivered target2) (delivered target3) (delivered target4)))
 (:metric minimize (total-cost))
)
