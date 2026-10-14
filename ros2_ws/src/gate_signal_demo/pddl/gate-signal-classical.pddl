;; THE GATE SIGNAL, CLASSICAL. Written by tools/make_pddl.py.
;;
;; Each robot plans with this on its own. r1 knows its order and has two routes
;; to its bay, the back route and the route through its gate, each lasting the
;; time it takes, and reads the order board first; r2 can drive from its booth into a bay, look over a bay from
;; its side door, cross from one bay to the other, and unload. Nothing here
;; says what either robot knows of the other.
(define (domain gate-signal)
(:requirements :strips :typing :durative-actions :fluents)
(:types robot bay)
(:predicates
  (at_start ?r - robot) (at_gate ?r - robot ?b - bay) (at_station ?r - robot)
  (in ?r - robot ?b - bay) (ordered ?r - robot ?b - bay) (free ?b - bay)
  (looked ?r - robot ?b - bay) (delivered ?r - robot) (read ?r - robot))
(:functions (d_back ?b - bay) (d_gate ?b - bay) (d_enter ?b - bay) (d_go ?b - bay) (d_cross))
(:durative-action read_order
  :parameters (?r - robot)
  :duration ( = ?duration 3)
  :condition (and (at start (at_start ?r)))
  :effect (and (at end (read ?r))))
(:durative-action back_route
  :parameters (?r - robot ?b - bay)
  :duration ( = ?duration (d_back ?b))
  :condition (and (at start (at_start ?r)) (at start (ordered ?r ?b)))
  :effect (and (at start (not (at_start ?r))) (at end (in ?r ?b))))
(:durative-action to_gate
  :parameters (?r - robot ?b - bay)
  :duration ( = ?duration (d_gate ?b))
  :condition (and (at start (at_start ?r)) (at start (ordered ?r ?b)))
  :effect (and (at start (not (at_start ?r))) (at end (at_gate ?r ?b))))
(:durative-action enter_bay
  :parameters (?r - robot ?b - bay)
  :duration ( = ?duration (d_enter ?b))
  :condition (and (at start (at_gate ?r ?b)))
  :effect (and (at start (not (at_gate ?r ?b))) (at end (in ?r ?b))))
(:durative-action go
  :parameters (?r - robot ?b - bay)
  :duration ( = ?duration (d_go ?b))
  :condition (and (at start (at_station ?r)))
  :effect (and (at start (not (at_station ?r))) (at end (in ?r ?b))))
(:durative-action look_bay
  :parameters (?r - robot ?b - bay)
  :duration ( = ?duration 3)
  :condition (and (over all (in ?r ?b)))
  :effect (and (at end (looked ?r ?b))))
(:durative-action cross
  :parameters (?r - robot ?from ?to - bay)
  :duration ( = ?duration (d_cross))
  :condition (and (at start (in ?r ?from)))
  :effect (and (at start (not (in ?r ?from))) (at end (in ?r ?to))))
(:durative-action unload
  :parameters (?r - robot ?b - bay)
  :duration ( = ?duration 2)
  :condition (and (over all (in ?r ?b)) (at start (free ?b)))
  :effect (and (at end (delivered ?r))))
)
