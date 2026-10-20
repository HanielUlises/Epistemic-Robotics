;; THE LINK GRADES: the executor's vocabulary under an epistemic policy.
;; Written by tools/make_pddl.py. What decides the policy is in the EPDDL.
(define (domain link-grades)
(:requirements :strips :typing :durative-actions)
(:types robot)
(:predicates (robot_ok ?r - robot))

(:durative-action drive_dock
    :parameters (?r - robot)
    :duration ( = ?duration 10)
    :condition (and (at start (robot_ok ?r)))
    :effect (and (at end (robot_ok ?r))))

(:durative-action drive_station
    :parameters (?r - robot)
    :duration ( = ?duration 10)
    :condition (and (at start (robot_ok ?r)))
    :effect (and (at end (robot_ok ?r))))

(:durative-action inspect
    :parameters (?r - robot)
    :duration ( = ?duration 10)
    :condition (and (at start (robot_ok ?r)))
    :effect (and (at end (robot_ok ?r))))

(:durative-action share
    :parameters (?a ?b - robot)
    :duration ( = ?duration 10)
    :condition (and (at start (robot_ok ?a)) (at start (robot_ok ?b)))
    :effect (and (at end (robot_ok ?a)) (at end (robot_ok ?b))))
)
