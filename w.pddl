(define (domain warehouse_problem-domain)
 (:requirements :strips :typing :action-costs)
 (:types location)
 (:constants
   target0 target2 start_location target1 target3 target4 - location
 )
 (:predicates (at_ ?l - location) (delivered ?t - location))
 (:functions (total-cost))
 (:action deliver_0
  :parameters ()
  :precondition (and (at_ start_location))
  :effect (and (delivered target0) (not (at_ start_location)) (at_ target0) (increase (total-cost) 7931)))
 (:action deliver_1_0
  :parameters ()
  :precondition (and (at_ target1))
  :effect (and (delivered target0) (not (at_ target1)) (at_ target0) (increase (total-cost) 7871)))
 (:action deliver_2_0
  :parameters ()
  :precondition (and (at_ target2))
  :effect (and (delivered target0) (not (at_ target2)) (at_ target0) (increase (total-cost) 5363)))
 (:action deliver_3_0
  :parameters ()
  :precondition (and (at_ target3))
  :effect (and (delivered target0) (not (at_ target3)) (at_ target0) (increase (total-cost) 8193)))
 (:action deliver_4_0
  :parameters ()
  :precondition (and (at_ target4))
  :effect (and (delivered target0) (not (at_ target4)) (at_ target0) (increase (total-cost) 7871)))
 (:action deliver_1
  :parameters ()
  :precondition (and (at_ start_location))
  :effect (and (delivered target1) (not (at_ start_location)) (at_ target1) (increase (total-cost) 8570)))
 (:action deliver_0_1
  :parameters ()
  :precondition (and (at_ target0))
  :effect (and (delivered target1) (not (at_ target0)) (at_ target1) (increase (total-cost) 7121)))
 (:action deliver_2_1
  :parameters ()
  :precondition (and (at_ target2))
  :effect (and (delivered target1) (not (at_ target2)) (at_ target1) (increase (total-cost) 5732)))
 (:action deliver_3_1
  :parameters ()
  :precondition (and (at_ target3))
  :effect (and (delivered target1) (not (at_ target3)) (at_ target1) (increase (total-cost) 7963)))
 (:action deliver_4_1
  :parameters ()
  :precondition (and (at_ target4))
  :effect (and (delivered target1) (not (at_ target4)) (at_ target1) (increase (total-cost) 7121)))
 (:action deliver_2
  :parameters ()
  :precondition (and (at_ start_location))
  :effect (and (delivered target2) (not (at_ start_location)) (at_ target2) (increase (total-cost) 9970)))
 (:action deliver_0_2
  :parameters ()
  :precondition (and (at_ target0))
  :effect (and (delivered target2) (not (at_ target0)) (at_ target2) (increase (total-cost) 7709)))
 (:action deliver_1_2
  :parameters ()
  :precondition (and (at_ target1))
  :effect (and (delivered target2) (not (at_ target1)) (at_ target2) (increase (total-cost) 7709)))
 (:action deliver_3_2
  :parameters ()
  :precondition (and (at_ target3))
  :effect (and (delivered target2) (not (at_ target3)) (at_ target2) (increase (total-cost) 7424)))
 (:action deliver_4_2
  :parameters ()
  :precondition (and (at_ target4))
  :effect (and (delivered target2) (not (at_ target4)) (at_ target2) (increase (total-cost) 7709)))
 (:action deliver_3
  :parameters ()
  :precondition (and (at_ start_location))
  :effect (and (delivered target3) (not (at_ start_location)) (at_ target3) (increase (total-cost) 6850)))
 (:action deliver_0_3
  :parameters ()
  :precondition (and (at_ target0))
  :effect (and (delivered target3) (not (at_ target0)) (at_ target3) (increase (total-cost) 8665)))
 (:action deliver_1_3
  :parameters ()
  :precondition (and (at_ target1))
  :effect (and (delivered target3) (not (at_ target1)) (at_ target3) (increase (total-cost) 8665)))
 (:action deliver_2_3
  :parameters ()
  :precondition (and (at_ target2))
  :effect (and (delivered target3) (not (at_ target2)) (at_ target3) (increase (total-cost) 6082)))
 (:action deliver_4_3
  :parameters ()
  :precondition (and (at_ target4))
  :effect (and (delivered target3) (not (at_ target4)) (at_ target3) (increase (total-cost) 8665)))
 (:action deliver_4
  :parameters ()
  :precondition (and (at_ start_location))
  :effect (and (delivered target4) (not (at_ start_location)) (at_ target4) (increase (total-cost) 8018)))
 (:action deliver_0_4
  :parameters ()
  :precondition (and (at_ target0))
  :effect (and (delivered target4) (not (at_ target0)) (at_ target4) (increase (total-cost) 12636)))
 (:action deliver_1_4
  :parameters ()
  :precondition (and (at_ target1))
  :effect (and (delivered target4) (not (at_ target1)) (at_ target4) (increase (total-cost) 12636)))
 (:action deliver_2_4
  :parameters ()
  :precondition (and (at_ target2))
  :effect (and (delivered target4) (not (at_ target2)) (at_ target4) (increase (total-cost) 10713)))
 (:action deliver_3_4
  :parameters ()
  :precondition (and (at_ target3))
  :effect (and (delivered target4) (not (at_ target3)) (at_ target4) (increase (total-cost) 10091)))
)
