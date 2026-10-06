;; The classical half of the pass-through mission.
;;
;; Deliberately blind, as the survey domains' classical halves are: nothing
;; here knows which bay is open, and nothing here could say that the carrier
;; must know it before crossing. That condition is in the EPDDL, where the
;; executor checks it against the model, and in the route the carrier drives,
;; where the least fixed point enforces it. This file exists so the executor
;; has actions to dispatch.

(define (domain pass-through)
(:requirements :strips :typing :adl :durative-actions)

(:types
  robot
  tunnel
)

(:predicates
  (ready ?r - robot)
  (surveyed ?r - robot ?t - tunnel)
  (told ?from ?to - robot ?t - tunnel)
  (delivered)
)

(:durative-action survey
  :parameters (?r - robot ?t - tunnel)
  :duration (= ?duration 5)
  :condition (and (at start (ready ?r)))
  :effect (and (at end (surveyed ?r ?t)))
)

;; A map exchange, announcing that the sender knows the bay is open.
(:durative-action tell_open
  :parameters (?from ?to - robot ?t - tunnel)
  :duration (= ?duration 2)
  :condition (and (at start (ready ?from)))
  :effect (and (at end (told ?from ?to ?t)))
)

;; The same exchange, announcing that the sender knows the bay is shut. Two
;; actions and not one because the performer checks the receiver's map against
;; what was announced, and has to know which announcement it is carrying.
(:durative-action tell_shut
  :parameters (?from ?to - robot ?t - tunnel)
  :duration (= ?duration 2)
  :condition (and (at start (ready ?from)))
  :effect (and (at end (told ?from ?to ?t)))
)

(:durative-action cross
  :parameters (?r - robot ?t - tunnel)
  :duration (= ?duration 30)
  :condition (and (at start (ready ?r)))
  :effect (and (at end (delivered)))
)
)
