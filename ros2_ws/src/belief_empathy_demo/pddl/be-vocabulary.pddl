;; BELIEF AND EMPATHY PARTICLES: the executor's vocabulary under an epistemic policy.
;; Written by tools/make_pddl.py. What decides the policy is in the EPDDL.
(define (domain belief-empathy)
(:requirements :strips :typing :durative-actions)
(:types robot)
(:predicates (ok ?r - robot) (radio))

(:durative-action fly_p1
  :parameters (?u ?g - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?u)))
  :effect (and (at end (ok ?u))))

(:durative-action back_p2
  :parameters (?u ?g - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?u)))
  :effect (and (at end (ok ?u))))

(:durative-action back_p3
  :parameters (?u ?g - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?u)))
  :effect (and (at end (ok ?u))))

(:durative-action hail_p1
  :parameters (?u ?g - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?u)) (at start (radio)))
  :effect (and (at end (ok ?u)) (at end (radio))))

(:durative-action hail_p2
  :parameters (?u ?g - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?u)) (at start (radio)))
  :effect (and (at end (ok ?u)) (at end (radio))))

(:durative-action hail_p3
  :parameters (?u ?g - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?u)) (at start (radio)))
  :effect (and (at end (ok ?u)) (at end (radio))))

(:durative-action cross_west
  :parameters (?u - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?u)))
  :effect (and (at end (ok ?u))))

(:durative-action cross_east
  :parameters (?u - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?u)))
  :effect (and (at end (ok ?u))))

(:durative-action fly_site
  :parameters (?u - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?u)))
  :effect (and (at end (ok ?u))))

(:durative-action go_site
  :parameters (?g - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?g)) (at start (radio)))
  :effect (and (at end (ok ?g)) (at end (radio))))

(:durative-action perform
  :parameters (?u ?g - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?u)) (at start (ok ?g)))
  :effect (and (at end (ok ?u)) (at end (ok ?g))))
)
