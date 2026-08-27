(define (domain building-rooms)

;; The classical half of the same mission.
;;
;; The EPDDL domain in epddl-workspace/building-rooms says what the robot comes
;; to know; this says what it physically does. The executor never reads the
;; EPDDL — it takes the action name out of a plan item, looks it up here, and
;; runs the behavior tree registered for it. Both halves have to exist, and the
;; action mapping next to this file is what joins them: `inspect3_r1` on the
;; epistemic side is `(look_into r1 door3)` here.
;;
;; The conditions below are the mechanical ones: where the robot is, whether it
;; has looked. That the robot does not yet know whether the room is blocked —
;; the condition that actually decides the plan — cannot be written here at
;; all, which is the reason the other file exists.

(:requirements :strips :typing :adl :durative-actions)

(:types
robot
waypoint
)

(:predicates

(robot_at ?r - robot ?wp - waypoint)
(is_door ?wp - waypoint)

;; True once ?r has pointed its laser through the doorway. What the map then
;; said is not recorded here — no predicate can hold "r1 knows whether" — so
;; this only says the looking happened.
(looked ?r - robot ?wp - waypoint)

)

(:functions
)

(:durative-action goto_door
    :parameters (?r - robot ?from ?to - waypoint)
    :duration ( = ?duration 60)
    :condition (and
        (at start(robot_at ?r ?from))
        (at start(is_door ?to))
    )
    :effect (and
        (at start(not(robot_at ?r ?from)))
        (at end(robot_at ?r ?to))
    )
)

(:durative-action look_into
    :parameters (?r - robot ?wp - waypoint)
    :duration ( = ?duration 8)
    :condition (and
        (over all(robot_at ?r ?wp))
        (at start(is_door ?wp))
    )
    :effect (and
        (at end(looked ?r ?wp))
    )
)

)
