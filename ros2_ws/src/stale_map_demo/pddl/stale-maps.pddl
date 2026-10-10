;; The classical half of the stale-maps mission.
;;
;; Blind on purpose. Nothing here knows which bays hold a load, which robots
;; saw the forklift, or what any robot's map says; a hauler that crosses a
;; bay it does not believe open is refused by the executor, which checks the
;; EPDDL precondition against the model, and not by anything in this file.
;; It exists so the executor has actions to dispatch.

(define (domain stale-maps)
(:requirements :strips :typing :adl :durative-actions)

(:types
  robot
  bay
)

(:predicates
  (ready ?r - robot)
  (changed ?t - bay)
  (sent ?from ?to - robot ?t - bay)
  (delivered ?r - robot)
  (shift_done)
  (forklift_on_shift)
)

(:durative-action stage
  :parameters (?t - bay)
  :duration (= ?duration 20)
  :condition (and (at start (forklift_on_shift)))
  :effect (and (at end (changed ?t)))
)

(:durative-action clear
  :parameters (?t - bay)
  :duration (= ?duration 20)
  :condition (and (at start (forklift_on_shift)))
  :effect (and (at end (changed ?t)))
)

;; A map of a bay sent, which must leave the receiver reading the bay clear.
(:durative-action send_open
  :parameters (?from ?to - robot ?t - bay)
  :duration (= ?duration 3)
  :condition (and (at start (ready ?from)))
  :effect (and (at end (sent ?from ?to ?t)))
)

;; The same, which must leave it reading the bay blocked. Two actions and not
;; one because the performer checks the receiver's map against the reading
;; sent, and has to know which reading it is carrying.
(:durative-action send_blocked
  :parameters (?from ?to - robot ?t - bay)
  :duration (= ?duration 3)
  :condition (and (at start (ready ?from)))
  :effect (and (at end (sent ?from ?to ?t)))
)

(:durative-action cross
  :parameters (?r - robot ?t - bay)
  :duration (= ?duration 90)
  :condition (and (at start (ready ?r)))
  :effect (and (at end (delivered ?r)) (at end (shift_done)))
)
)
