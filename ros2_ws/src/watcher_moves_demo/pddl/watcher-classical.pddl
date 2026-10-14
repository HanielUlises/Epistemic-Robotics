;; THE WATCHER, CLASSICAL. Written by tools/make_pddl.py.
;;
;; The carrier's model. It knows its order once it has read the board, and has
;; every way the epistemic domain has of reaching its bay and of informing the
;; ally: the gate, which the ally's booth sees, or the lamp at a window. It can
;; read the duty board too. What it cannot say is that someone else is
;; watching: a goal over facts has no place for what another robot knows.
(define (domain watcher)
(:requirements :strips :typing :durative-actions :fluents)
(:types robot bay side)
(:predicates
  (at_start ?r - robot) (at_window ?r - robot ?s - side) (in ?r - robot ?b - bay)
  (ordered ?r - robot ?b - bay) (read ?r - robot) (post_read ?r - robot)
  (informed) (gates_allowed))
(:functions (d_gate ?b - bay) (d_window ?s - side) (d_back ?s - side ?b - bay))
(:durative-action read_order
  :parameters (?r - robot)
  :duration ( = ?duration 3)
  :condition (and (at start (at_start ?r)))
  :effect (and (at end (read ?r))))
(:durative-action read_post
  :parameters (?r - robot)
  :duration ( = ?duration 3)
  :condition (and (at start (at_start ?r)))
  :effect (and (at end (post_read ?r))))
(:durative-action to_gate
  :parameters (?r - robot ?b - bay)
  :duration ( = ?duration (d_gate ?b))
  :condition (and (at start (at_start ?r)) (at start (ordered ?r ?b)) (at start (gates_allowed)))
  :effect (and (at start (not (at_start ?r))) (at end (in ?r ?b)) (at end (informed))))
(:durative-action to_window
  :parameters (?r - robot ?s - side)
  :duration ( = ?duration (d_window ?s))
  :condition (and (at start (at_start ?r)))
  :effect (and (at start (not (at_start ?r))) (at end (at_window ?r ?s))))
(:durative-action signal
  :parameters (?r - robot ?s - side ?b - bay)
  :duration ( = ?duration 4)
  :condition (and (over all (at_window ?r ?s)) (at start (ordered ?r ?b)))
  :effect (and (at end (informed))))
(:durative-action back_route
  :parameters (?r - robot ?s - side ?b - bay)
  :duration ( = ?duration (d_back ?s ?b))
  :condition (and (at start (at_window ?r ?s)) (at start (ordered ?r ?b)) (at start (informed)))
  :effect (and (at start (not (at_window ?r ?s))) (at end (in ?r ?b))))
)
