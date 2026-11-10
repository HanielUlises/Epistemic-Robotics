;; A FAULTY SENSOR IN THE FLEET: the executor's vocabulary under an epistemic policy.
;; Written by tools/make_pddl.py. What decides the policy is in the EPDDL.
(define (domain faulty-sensor)
(:requirements :strips :typing :durative-actions)
(:types robot place passage view word)
(:predicates (robot_ok ?r - robot) (radio) (passage_free ?p - passage))

(:durative-action drive
  :parameters (?r - robot ?a - place ?b - place)
  :duration (= ?duration 10)
  :condition (and (at start (robot_ok ?r)) (at start (radio)))
  :effect (and (at end (robot_ok ?r))))

(:durative-action passage
  :parameters (?r - robot ?p - passage ?a - place ?b - place)
  :duration (= ?duration 10)
  :condition (and (at start (robot_ok ?r)) (at start (radio)) (at start (passage_free ?p)))
  :effect (and (at end (robot_ok ?r)) (at end (passage_free ?p))))

(:durative-action backout
  :parameters (?r - robot ?p - passage ?a - place ?b - place)
  :duration (= ?duration 10)
  :condition (and (at start (robot_ok ?r)) (at start (radio)) (at start (passage_free ?p)))
  :effect (and (at end (robot_ok ?r)) (at end (passage_free ?p))))

(:durative-action look
  :parameters (?r - robot ?p - passage ?v - view)
  :duration (= ?duration 10)
  :condition (and (at start (robot_ok ?r)) (at start (radio)) (at start (passage_free ?p)))
  :effect (and (at end (robot_ok ?r)) (at end (passage_free ?p))))

(:durative-action say
  :parameters (?r - robot ?p - passage ?w - word)
  :duration (= ?duration 10)
  :condition (and (at start (robot_ok ?r)) (at start (radio)))
  :effect (and (at end (robot_ok ?r)) (at end (radio))))
)
