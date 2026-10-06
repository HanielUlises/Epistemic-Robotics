;; The classical half of the false-belief demonstration.
;;
;; Deliberately blind: nothing here knows where the crate is or what either
;; robot believes about it, and nothing here could say that fetching it needs
;; the picker to believe it is there. Those conditions are in the EPDDL, where
;; the executor checks them against the model before dispatching. This file
;; exists so the executor has actions to dispatch.

(define (domain false-belief)
(:requirements :strips :typing :adl :durative-actions)

(:types
  robot
  bay
)

(:predicates
  (ready ?r - robot)
  (docked ?r - robot)
  (moved ?from ?to - bay)
  (reported ?from ?to - bay)
  (looked ?r - robot ?b - bay)
  (fetched)
)

(:durative-action go_dock
  :parameters (?r - robot)
  :duration (= ?duration 60)
  :condition (and (at start (ready ?r)))
  :effect (and (at end (docked ?r)))
)

(:durative-action relocate
  :parameters (?r - robot ?from ?to - bay)
  :duration (= ?duration 180)
  :condition (and (at start (ready ?r)))
  :effect (and (at end (moved ?from ?to)))
)

(:durative-action report
  :parameters (?from-robot ?to-robot - robot ?from ?to - bay)
  :duration (= ?duration 4)
  :condition (and (at start (ready ?from-robot)))
  :effect (and (at end (reported ?from ?to)))
)

(:durative-action look
  :parameters (?r - robot ?b - bay)
  :duration (= ?duration 60)
  :condition (and (at start (ready ?r)))
  :effect (and (at end (looked ?r ?b)))
)

(:durative-action fetch
  :parameters (?r - robot ?b - bay)
  :duration (= ?duration 120)
  :condition (and (at start (ready ?r)))
  :effect (and (at end (fetched)))
)
)
