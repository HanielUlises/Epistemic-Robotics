;; BELIEF REVISION IN A CHANGING WARD: the executor's vocabulary under an epistemic policy.
;; Written by tools/make_pddl.py. What decides the policy is in the EPDDL.
(define (domain belief-revision)
(:requirements :strips :typing :durative-actions)
(:types robot place lobby view)
(:predicates (robot_ok ?r - robot) (radio) (lobby_free ?l - lobby))

(:durative-action drive
  :parameters (?r - robot ?a - place ?b - place)
  :duration (= ?duration 10)
  :condition (and (at start (robot_ok ?r)) (at start (radio)))
  :effect (and (at end (robot_ok ?r))))

(:durative-action lobby
  :parameters (?r - robot ?l - lobby ?a - place ?b - place)
  :duration (= ?duration 10)
  :condition (and (at start (robot_ok ?r)) (at start (radio)) (at start (lobby_free ?l)))
  :effect (and (at end (robot_ok ?r)) (at end (lobby_free ?l))))

(:durative-action look
  :parameters (?r - robot ?l - lobby ?v - view)
  :duration (= ?duration 10)
  :condition (and (at start (robot_ok ?r)) (at start (radio)) (at start (lobby_free ?l)))
  :effect (and (at end (robot_ok ?r)) (at end (lobby_free ?l))))

(:durative-action tell
  :parameters (?r - robot ?l - lobby)
  :duration (= ?duration 10)
  :condition (and (at start (robot_ok ?r)) (at start (radio)))
  :effect (and (at end (robot_ok ?r)) (at end (radio))))

(:durative-action round
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (robot_ok ?r)) (at start (radio)))
  :effect (and (at end (robot_ok ?r)) (at end (radio))))
)
