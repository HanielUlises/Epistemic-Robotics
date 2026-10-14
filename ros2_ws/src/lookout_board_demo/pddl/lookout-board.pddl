;; THE LOOKOUT BOARD, CLASSICAL. Written by tools/make_pddl.py.
;;
;; What the robot physically does. Under the epistemic policy this is only the
;; executor's vocabulary; under the classical formulations it is the whole
;; model. A look needs no knowledge, only a lookout not yet found screened, and
;; makes the zone known: the classical planner assumes it will see. Nothing
;; here can say what the robot will know after reading the board, which is the
;; one thing that would make reading it worth a detour.

(define (domain lookout-board)
(:requirements :strips :typing :durative-actions :fluents)

(:types
robot
place
slot
)

(:predicates
(robot_at ?r - robot ?p - place)
(is_board ?p - place)
(is_lookout ?p - place)
(open_view ?p - place)       ; not yet found screened
(read ?r - robot ?s - slot)
(checked ?r - robot ?p - place)
(looked ?r - robot ?p - place)
(zone_known)
)

(:functions
(route ?a ?b - place)
)

(:durative-action goto
    :parameters (?r - robot ?from ?to - place)
    :duration ( = ?duration (route ?from ?to))
    :condition (and
        (at start (robot_at ?r ?from))
    )
    :effect (and
        (at start (not (robot_at ?r ?from)))
        (at end (robot_at ?r ?to))
    )
)

(:durative-action read_slot
    :parameters (?r - robot ?b - place ?s - slot)
    :duration ( = ?duration 2)
    :condition (and
        (over all (robot_at ?r ?b))
        (at start (is_board ?b))
    )
    :effect (and
        (at end (read ?r ?s))
    )
)

(:durative-action check_view
    :parameters (?r - robot ?l - place)
    :duration ( = ?duration 2)
    :condition (and
        (over all (robot_at ?r ?l))
        (at start (is_lookout ?l))
    )
    :effect (and
        (at end (checked ?r ?l))
    )
)

(:durative-action look_zone
    :parameters (?r - robot ?l - place)
    :duration ( = ?duration 3)
    :condition (and
        (over all (robot_at ?r ?l))
        (at start (is_lookout ?l))
        (at start (open_view ?l))
    )
    :effect (and
        (at end (looked ?r ?l))
        (at end (zone_known))
    )
)
)
