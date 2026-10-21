;; THE COLLAPSED WING for a classical planner, every way assumed clear. Written by tools/make_pddl.py.
(define (domain collapsed-wing)
(:requirements :strips :typing :durative-actions)
(:types robot)
(:predicates
  (at_hall ?r - robot)
  (with_victim ?r - robot)
  (assumed_clear_w)
  (assumed_clear_m)
  (assumed_clear_e)
  (in_way_w ?r - robot)
  (in_way_m ?r - robot)
  (in_way_e ?r - robot)
)

(:durative-action cross_w
  :parameters (?r - robot)
  :duration (= ?duration 144)
  :condition (and (at start (at_hall ?r)) (over all (assumed_clear_w)))
  :effect (and (at start (not (at_hall ?r))) (at end (with_victim ?r))))

(:durative-action retreat_w
  :parameters (?r - robot)
  :duration (= ?duration 72)
  :condition (and (at start (in_way_w ?r)))
  :effect (and (at start (not (in_way_w ?r))) (at end (at_hall ?r))))

(:durative-action cross_m
  :parameters (?r - robot)
  :duration (= ?duration 197)
  :condition (and (at start (at_hall ?r)) (over all (assumed_clear_m)))
  :effect (and (at start (not (at_hall ?r))) (at end (with_victim ?r))))

(:durative-action retreat_m
  :parameters (?r - robot)
  :duration (= ?duration 70)
  :condition (and (at start (in_way_m ?r)))
  :effect (and (at start (not (in_way_m ?r))) (at end (at_hall ?r))))

(:durative-action cross_e
  :parameters (?r - robot)
  :duration (= ?duration 144)
  :condition (and (at start (at_hall ?r)) (over all (assumed_clear_e)))
  :effect (and (at start (not (at_hall ?r))) (at end (with_victim ?r))))

(:durative-action retreat_e
  :parameters (?r - robot)
  :duration (= ?duration 72)
  :condition (and (at start (in_way_e ?r)))
  :effect (and (at start (not (in_way_e ?r))) (at end (at_hall ?r))))
)
