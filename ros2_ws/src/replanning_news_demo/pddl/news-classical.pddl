;; REPLANNING ON NEWS for a classical planner: a door may be crossed while it is
;; assumed open. Written by tools/make_pddl.py.
(define (domain replanning-news)
(:requirements :strips :typing :durative-actions)
(:types robot)
(:predicates
  (at_st ?r - robot)
  (at_vw ?r - robot)
  (at_vm ?r - robot)
  (at_ve ?r - robot)
  (at_r ?r - robot)
  (at_nw ?r - robot)
  (at_nm ?r - robot)
  (at_ne ?r - robot)
  (assumed_open_w)
  (assumed_open_m)
  (assumed_open_e)
)

(:durative-action goto_vw_from_st
  :parameters (?r - robot)
  :duration (= ?duration 115)
  :condition (and (at start (at_st ?r)))
  :effect (and (at start (not (at_st ?r))) (at end (at_vw ?r))))

(:durative-action goto_vw_from_vm
  :parameters (?r - robot)
  :duration (= ?duration 54)
  :condition (and (at start (at_vm ?r)))
  :effect (and (at start (not (at_vm ?r))) (at end (at_vw ?r))))

(:durative-action goto_vw_from_ve
  :parameters (?r - robot)
  :duration (= ?duration 95)
  :condition (and (at start (at_ve ?r)))
  :effect (and (at start (not (at_ve ?r))) (at end (at_vw ?r))))

(:durative-action goto_vm_from_st
  :parameters (?r - robot)
  :duration (= ?duration 68)
  :condition (and (at start (at_st ?r)))
  :effect (and (at start (not (at_st ?r))) (at end (at_vm ?r))))

(:durative-action goto_vm_from_vw
  :parameters (?r - robot)
  :duration (= ?duration 54)
  :condition (and (at start (at_vw ?r)))
  :effect (and (at start (not (at_vw ?r))) (at end (at_vm ?r))))

(:durative-action goto_vm_from_ve
  :parameters (?r - robot)
  :duration (= ?duration 48)
  :condition (and (at start (at_ve ?r)))
  :effect (and (at start (not (at_ve ?r))) (at end (at_vm ?r))))

(:durative-action goto_ve_from_st
  :parameters (?r - robot)
  :duration (= ?duration 49)
  :condition (and (at start (at_st ?r)))
  :effect (and (at start (not (at_st ?r))) (at end (at_ve ?r))))

(:durative-action goto_ve_from_vw
  :parameters (?r - robot)
  :duration (= ?duration 95)
  :condition (and (at start (at_vw ?r)))
  :effect (and (at start (not (at_vw ?r))) (at end (at_ve ?r))))

(:durative-action goto_ve_from_vm
  :parameters (?r - robot)
  :duration (= ?duration 48)
  :condition (and (at start (at_vm ?r)))
  :effect (and (at start (not (at_vm ?r))) (at end (at_ve ?r))))

(:durative-action cross_w
  :parameters (?r - robot)
  :duration (= ?duration 24)
  :condition (and (at start (at_vw ?r)) (over all (assumed_open_w)))
  :effect (and (at start (not (at_vw ?r))) (at end (at_nw ?r))))

(:durative-action deliver_from_w
  :parameters (?r - robot)
  :duration (= ?duration 20)
  :condition (and (at start (at_nw ?r)))
  :effect (and (at start (not (at_nw ?r))) (at end (at_r ?r))))

(:durative-action cross_m
  :parameters (?r - robot)
  :duration (= ?duration 34)
  :condition (and (at start (at_vm ?r)) (over all (assumed_open_m)))
  :effect (and (at start (not (at_vm ?r))) (at end (at_nm ?r))))

(:durative-action deliver_from_m
  :parameters (?r - robot)
  :duration (= ?duration 78)
  :condition (and (at start (at_nm ?r)))
  :effect (and (at start (not (at_nm ?r))) (at end (at_r ?r))))

(:durative-action cross_e
  :parameters (?r - robot)
  :duration (= ?duration 24)
  :condition (and (at start (at_ve ?r)) (over all (assumed_open_e)))
  :effect (and (at start (not (at_ve ?r))) (at end (at_ne ?r))))

(:durative-action deliver_from_e
  :parameters (?r - robot)
  :duration (= ?duration 108)
  :condition (and (at start (at_ne ?r)))
  :effect (and (at start (not (at_ne ?r))) (at end (at_r ?r))))
)
