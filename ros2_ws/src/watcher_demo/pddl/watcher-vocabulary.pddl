;; THE WATCHER: the executor's vocabulary under the epistemic policy.
;; Written by tools/make_pddl.py. What decides the policy is in the EPDDL.
(define (domain watcher)
(:requirements :strips :typing :durative-actions)
(:types robot)
(:predicates (robot_ok ?r - robot))

(:durative-action read_order
    :parameters (?r - robot)
    :duration ( = ?duration 10)
    :condition (and (at start (robot_ok ?r)))
    :effect (and (at end (robot_ok ?r))))

(:durative-action read_post
    :parameters (?r - robot)
    :duration ( = ?duration 10)
    :condition (and (at start (robot_ok ?r)))
    :effect (and (at end (robot_ok ?r))))

(:durative-action to_gate
    :parameters (?r - robot)
    :duration ( = ?duration 10)
    :condition (and (at start (robot_ok ?r)))
    :effect (and (at end (robot_ok ?r))))

(:durative-action to_window_w
    :parameters (?r - robot)
    :duration ( = ?duration 10)
    :condition (and (at start (robot_ok ?r)))
    :effect (and (at end (robot_ok ?r))))

(:durative-action to_window_e
    :parameters (?r - robot)
    :duration ( = ?duration 10)
    :condition (and (at start (robot_ok ?r)))
    :effect (and (at end (robot_ok ?r))))

(:durative-action signal
    :parameters (?r - robot)
    :duration ( = ?duration 10)
    :condition (and (at start (robot_ok ?r)))
    :effect (and (at end (robot_ok ?r))))

(:durative-action back_route
    :parameters (?r - robot)
    :duration ( = ?duration 10)
    :condition (and (at start (robot_ok ?r)))
    :effect (and (at end (robot_ok ?r))))
)
