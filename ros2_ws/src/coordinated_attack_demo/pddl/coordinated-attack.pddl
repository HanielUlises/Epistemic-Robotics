;; The classical half of the coordinated attack.
;;
;; Deliberately blind: nothing here knows which stand the work order names,
;; and nothing here could say that a lift needs common knowledge of it. That
;; condition is in the EPDDL, where the executor checks it against the model
;; before dispatching lift. This file exists so the executor has actions to
;; dispatch.

(define (domain coordinated-attack)
(:requirements :strips :typing :adl :durative-actions)

(:types
  robot
  stand
)

(:predicates
  (ready ?r - robot)
  (read ?r - robot ?s - stand)
  (said ?from ?to - robot ?s - stand)
  (at-view ?r - robot)
  (signalled ?s - stand)
  (sighted)
  (lifted)
)

(:durative-action read_order
  :parameters (?r - robot ?s - stand)
  :duration (= ?duration 60)
  :condition (and (at start (ready ?r)))
  :effect (and (at end (read ?r ?s)))
)

;; The four radio messages, one action each, because the performer has to
;; know which level of nesting it is carrying.
(:durative-action radio_tell
  :parameters (?from ?to - robot ?s - stand)
  :duration (= ?duration 4)
  :condition (and (at start (ready ?from)))
  :effect (and (at end (said ?from ?to ?s)))
)

(:durative-action radio_ack
  :parameters (?from ?to - robot ?s - stand)
  :duration (= ?duration 4)
  :condition (and (at start (ready ?from)))
  :effect (and (at end (said ?from ?to ?s)))
)

(:durative-action radio_ack2
  :parameters (?from ?to - robot ?s - stand)
  :duration (= ?duration 4)
  :condition (and (at start (ready ?from)))
  :effect (and (at end (said ?from ?to ?s)))
)

(:durative-action radio_ack3
  :parameters (?from ?to - robot ?s - stand)
  :duration (= ?duration 4)
  :condition (and (at start (ready ?from)))
  :effect (and (at end (said ?from ?to ?s)))
)

(:durative-action go_view
  :parameters (?r - robot)
  :duration (= ?duration 60)
  :condition (and (at start (ready ?r)))
  :effect (and (at end (at-view ?r)))
)

(:durative-action signal
  :parameters (?r - robot ?s - stand)
  :duration (= ?duration 6)
  :condition (and (at start (ready ?r)))
  :effect (and (at end (signalled ?s)))
)

;; The signal of the positions domain, which names its listener: whether the
;; listener saw it is an outcome there, decided by where the listener stands.
(:durative-action signal_to
  :parameters (?from ?to - robot ?s - stand)
  :duration (= ?duration 6)
  :condition (and (at start (ready ?from)))
  :effect (and (at end (signalled ?s)))
)

;; The two robots see each other through t2, each from its viewpoint.
(:durative-action sight
  :parameters (?a ?b - robot)
  :duration (= ?duration 6)
  :condition (and (at start (ready ?a)) (at start (ready ?b)))
  :effect (and (at end (sighted)))
)

;; Joint: one action for both robots, since neither half may run alone.
(:durative-action lift
  :parameters (?a ?b - robot ?s - stand)
  :duration (= ?duration 90)
  :condition (and (at start (ready ?a)) (at start (ready ?b)))
  :effect (and (at end (lifted)))
)
)
