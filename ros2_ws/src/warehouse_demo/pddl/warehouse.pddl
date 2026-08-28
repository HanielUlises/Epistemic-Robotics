(define (domain warehouse)

;; The classical half of the warehouse mission.
;;
;; The EPDDL domain in epddl-workspace/robot-warehouse says what the robot
;; comes to know; this says what it physically does. The executor never reads
;; the EPDDL -- it takes the action name out of a plan item, looks it up here,
;; and runs the behaviour tree registered for it. The action mapping beside
;; this file is what joins the two halves: `inspect_r1_bay2` on the epistemic
;; side is `(look_into r1 bay2)` here.
;;
;; The conditions below are the mechanical ones: where the robot is, what it is
;; holding. That the robot does not yet know which bay holds the pallet -- the
;; condition that actually decides the plan -- cannot be written here at all,
;; which is the reason the other file exists.

(:requirements :strips :typing :adl :durative-actions)

(:types
robot
zone
)

(:predicates

(robot_at ?r - robot ?z - zone)
(is_bay ?z - zone)
(is_dock ?z - zone)

;; True once ?r has pointed its laser into the bay. What the map then said is
;; not recorded here -- no predicate can hold "r1 knows whether" -- so this
;; only says the looking happened.
(looked ?r - robot ?z - zone)

;; Likewise for a radio call: that ?r said something about ?z, not what it
;; said. The content is the announcement's epistemic effect and lives in the
;; model; this only says the fleet was on the channel to hear it.
(announced ?r - robot ?z - zone)

(holding ?r - robot)
(delivered ?r - robot)

)

(:functions
)

(:durative-action goto_zone
    :parameters (?r - robot ?from ?to - zone)
    :duration ( = ?duration 120)
    :condition (and
        (at start(robot_at ?r ?from))
    )
    :effect (and
        (at start(not(robot_at ?r ?from)))
        (at end(robot_at ?r ?to))
    )
)

(:durative-action look_into
    :parameters (?r - robot ?z - zone)
    :duration ( = ?duration 30)
    :condition (and
        (over all(robot_at ?r ?z))
        (at start(is_bay ?z))
    )
    :effect (and
        (at end(looked ?r ?z))
    )
)

(:durative-action pick_up
    :parameters (?r - robot ?z - zone)
    :duration ( = ?duration 6)
    :condition (and
        (over all(robot_at ?r ?z))
        (at start(is_bay ?z))
    )
    :effect (and
        (at end(holding ?r))
    )
)

;; Radioing the fleet what this robot knows about a bay.
;;
;; Only reachable with three robots or more. With two, `pick_up` is already
;; public -- the fleet watching r1 lift the pallet out of a bay learns which
;; bay it was -- and the planner declines to spend an action on saying so. A
;; third robot that must also know, on the branch where r1 found bay 2 *empty*,
;; is the case where watching a lift settles the wrong bay and the fleet has to
;; be told.
;;
;; There is no classical condition on what may be said, because there is no
;; classical way to say it. That a robot may only announce what it knows is
;; `([?i] (pallet-at ?z))` on the epistemic side, and it is enforced there.
(:durative-action announce
    :parameters (?r - robot ?z - zone)
    :duration ( = ?duration 4)
    :condition (and
        (at start(is_bay ?z))
    )
    :effect (and
        (at end(announced ?r ?z))
    )
)

(:durative-action drop_off
    :parameters (?r - robot ?z - zone)
    :duration ( = ?duration 6)
    :condition (and
        (over all(robot_at ?r ?z))
        (at start(is_dock ?z))
        (at start(holding ?r))
    )
    :effect (and
        (at end(not(holding ?r)))
        (at end(delivered ?r))
    )
)

)
