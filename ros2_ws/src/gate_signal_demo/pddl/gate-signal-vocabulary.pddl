;; THE GATE SIGNAL: the executor's vocabulary under the epistemic policy.
;; Written by tools/make_pddl.py. What decides the policy is in the EPDDL.
(define (domain gate-signal)
(:requirements :strips :typing :durative-actions)
(:types robot bay)
(:predicates (robot_ok ?r - robot))

(:durative-action read_order
    :parameters (?r - robot)
    :duration ( = ?duration 10)
    :condition (and (at start (robot_ok ?r)))
    :effect (and (at end (robot_ok ?r))))

(:durative-action to_gate
    :parameters (?r - robot)
    :duration ( = ?duration 10)
    :condition (and (at start (robot_ok ?r)))
    :effect (and (at end (robot_ok ?r))))

(:durative-action back_route
    :parameters (?r - robot)
    :duration ( = ?duration 10)
    :condition (and (at start (robot_ok ?r)))
    :effect (and (at end (robot_ok ?r))))

(:durative-action enter_bay
    :parameters (?r - robot)
    :duration ( = ?duration 10)
    :condition (and (at start (robot_ok ?r)))
    :effect (and (at end (robot_ok ?r))))

(:durative-action watch
    :parameters (?r ?o - robot)
    :duration ( = ?duration 10)
    :condition (and (at start (robot_ok ?r)))
    :effect (and (at end (robot_ok ?r))))

(:durative-action go
    :parameters (?r - robot ?b - bay)
    :duration ( = ?duration 10)
    :condition (and (at start (robot_ok ?r)))
    :effect (and (at end (robot_ok ?r))))

(:durative-action unload
    :parameters (?r - robot)
    :duration ( = ?duration 10)
    :condition (and (at start (robot_ok ?r)))
    :effect (and (at end (robot_ok ?r))))
)
