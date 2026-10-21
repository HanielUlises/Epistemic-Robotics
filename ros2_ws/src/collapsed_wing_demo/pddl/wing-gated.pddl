;; THE COLLAPSED WING for a classical planner, gated on a scout's check. Written by tools/make_pddl.py.
(define (domain collapsed-wing)
(:requirements :strips :typing :durative-actions)
(:types robot)
(:predicates
  (at_hall ?r - robot)
  (with_victim ?r - robot)
  (checked_clear_w)
  (checked_clear_m)
  (checked_clear_e)
  (in_way_w ?r - robot)
  (in_way_m ?r - robot)
  (in_way_e ?r - robot)
  (at_entry_e ?r - robot)
  (at_entry_w ?r - robot)
  (at_vantage_e ?r - robot)
  (at_vantage_w ?r - robot)
  (unchecked_e)
  (unchecked_w)
)

(:durative-action goto_e
  :parameters (?r - robot)
  :duration (= ?duration 48)
  :condition (and (at start (at_entry_e ?r)))
  :effect (and (at start (not (at_entry_e ?r))) (at end (at_vantage_e ?r))))

(:durative-action check_e
  :parameters (?r - robot)
  :duration (= ?duration 5)
  :condition (and (at start (unchecked_e)) (over all (at_vantage_e ?r)))
  :effect (and (at start (not (unchecked_e))) (at end (checked_clear_e))))

(:durative-action goto_w
  :parameters (?r - robot)
  :duration (= ?duration 48)
  :condition (and (at start (at_entry_w ?r)))
  :effect (and (at start (not (at_entry_w ?r))) (at end (at_vantage_w ?r))))

(:durative-action check_w
  :parameters (?r - robot)
  :duration (= ?duration 5)
  :condition (and (at start (unchecked_w)) (over all (at_vantage_w ?r)))
  :effect (and (at start (not (unchecked_w))) (at end (checked_clear_w))))

(:durative-action cross_w
  :parameters (?r - robot)
  :duration (= ?duration 144)
  :condition (and (at start (at_hall ?r)) (over all (checked_clear_w)))
  :effect (and (at start (not (at_hall ?r))) (at end (with_victim ?r))))

(:durative-action retreat_w
  :parameters (?r - robot)
  :duration (= ?duration 72)
  :condition (and (at start (in_way_w ?r)))
  :effect (and (at start (not (in_way_w ?r))) (at end (at_hall ?r))))

(:durative-action cross_m
  :parameters (?r - robot)
  :duration (= ?duration 197)
  :condition (and (at start (at_hall ?r)) (over all (checked_clear_m)))
  :effect (and (at start (not (at_hall ?r))) (at end (with_victim ?r))))

(:durative-action retreat_m
  :parameters (?r - robot)
  :duration (= ?duration 70)
  :condition (and (at start (in_way_m ?r)))
  :effect (and (at start (not (in_way_m ?r))) (at end (at_hall ?r))))

(:durative-action cross_e
  :parameters (?r - robot)
  :duration (= ?duration 144)
  :condition (and (at start (at_hall ?r)) (over all (checked_clear_e)))
  :effect (and (at start (not (at_hall ?r))) (at end (with_victim ?r))))

(:durative-action retreat_e
  :parameters (?r - robot)
  :duration (= ?duration 72)
  :condition (and (at start (in_way_e ?r)))
  :effect (and (at start (not (in_way_e ?r))) (at end (at_hall ?r))))
)
