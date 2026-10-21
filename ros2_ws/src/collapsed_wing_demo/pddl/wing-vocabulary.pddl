;; THE COLLAPSED WING: the executor's vocabulary under an epistemic policy.
;; Written by tools/make_pddl.py. What decides the policy is in the EPDDL.
(define (domain collapsed-wing)
(:requirements :strips :typing :durative-actions)
(:types robot)
(:predicates (robot_ok ?r - robot) (radio))

(:durative-action goto_e
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (robot_ok ?r)))
  :effect (and (at end (robot_ok ?r))))

(:durative-action goto_w
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (robot_ok ?r)))
  :effect (and (at end (robot_ok ?r))))

(:durative-action inspect_e
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (robot_ok ?r)))
  :effect (and (at end (robot_ok ?r))))

(:durative-action inspect_w
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (robot_ok ?r)))
  :effect (and (at end (robot_ok ?r))))

(:durative-action tell_w
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (robot_ok ?r)) (at start (radio)))
  :effect (and (at end (robot_ok ?r)) (at end (radio))))

(:durative-action tell_m
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (robot_ok ?r)) (at start (radio)))
  :effect (and (at end (robot_ok ?r)) (at end (radio))))

(:durative-action tell_e
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (robot_ok ?r)) (at start (radio)))
  :effect (and (at end (robot_ok ?r)) (at end (radio))))

(:durative-action cross_w
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (robot_ok ?r)) (at start (radio)))
  :effect (and (at end (robot_ok ?r))))

(:durative-action cross_m
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (robot_ok ?r)) (at start (radio)))
  :effect (and (at end (robot_ok ?r))))

(:durative-action cross_e
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (robot_ok ?r)) (at start (radio)))
  :effect (and (at end (robot_ok ?r))))
)
