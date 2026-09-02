(define (domain hotel)

;; The mechanical half of the hotel incident.
;;
;; The EPDDL domain in epddl-workspace/hotel-incident says what the fleet comes
;; to know and who is allowed to know it. This says what a robot physically
;; does. The executor never reads the EPDDL: it takes the action name out of a
;; policy node, looks it up here, and runs the performer registered for it. The
;; action mapping beside this file joins the two halves, so that
;; `inspect_inspector_l2_suite` on the epistemic side is
;; `(look_into inspector l2_suite)` here.
;;
;; None of the conditions that decide the plan can be written below. That a
;; robot may only shut off a leak it *knows* is there, that the guest must not
;; overhear, that containing the leak in a closed suite leaves everyone
;; downstairs believing something false -- none of it is expressible in this
;; file, and all of it is what the mission is about.
;;
;; Nor is a lift. `goto_zone inspector lobby l3_suite` is one action here and
;; one action in the epistemic model; the lift call, the door, the floor
;; transition and the negotiation with the other robot for the same lift are
;; Open-RMF's, on the far side of the bridge.

(:requirements :strips :typing :adl :durative-actions)

(:types
robot
zone
)

(:predicates

(robot_at ?r - robot ?z - zone)
(is_suite ?z - zone)
(is_desk ?z - zone)

;; True once ?r has looked into the suite. What it saw is not recorded here --
;; no predicate can hold "r knows whether" -- so this says only that the
;; looking happened.
(looked ?r - robot ?z - zone)

;; The valve is shut. That the leak was in this suite rather than the other is
;; a fact about the building; that the fleet came to believe it is not, and
;; lives in the model.
(shut_off ?r - robot ?z - zone)

;; Likewise for the radio: that ?r said something to ?j, not what it said.
(radioed ?r ?j - robot ?z - zone)

;; And for the public address system.
(paged ?r - robot ?z - zone)

)

(:functions
)

;; Long, because it may be a lift ride between floors and the robot may have to
;; wait for the other one to come out. RMF decides how long it really takes and
;; the bridge waits for it; this number only has to be generous.
(:durative-action goto_zone
    :parameters (?r - robot ?from ?to - zone)
    :duration ( = ?duration 180)
    :condition (and
        (at start(robot_at ?r ?from))
    )
    :effect (and
        (at start(not(robot_at ?r ?from)))
        (at end(robot_at ?r ?to))
    )
)

; Two robots sent at once, as one action.
;
; A policy is a chain of product updates and the executor runs it strictly in
; order, so two consecutive goto_zone nodes are two robots moving one after the
; other. Robots move together only when the togetherness is in the model: one
; event, applied once, that relocates both. The bridge submits an RMF task per
; robot and holds the action open until every one of them is done.
(:durative-action deploy
    :parameters (?r1 ?r2 - robot ?from1 ?to1 ?from2 ?to2 - zone)
    :duration ( = ?duration 180)
    :condition (and
        (at start(robot_at ?r1 ?from1))
        (at start(robot_at ?r2 ?from2))
    )
    :effect (and
        (at start(not(robot_at ?r1 ?from1)))
        (at start(not(robot_at ?r2 ?from2)))
        (at end(robot_at ?r1 ?to1))
        (at end(robot_at ?r2 ?to2))
    )
)

(:durative-action look_into
    :parameters (?r - robot ?z - zone)
    :duration ( = ?duration 30)
    :condition (and
        (over all(robot_at ?r ?z))
        (at start(is_suite ?z))
    )
    :effect (and
        (at end(looked ?r ?z))
    )
)

;; The one action that changes the building rather than what is believed about
;; it, and the reason the mission has to repair beliefs afterwards.
(:durative-action shut_valve
    :parameters (?r - robot ?z - zone)
    :duration ( = ?duration 20)
    :condition (and
        (over all(robot_at ?r ?z))
        (at start(is_suite ?z))
    )
    :effect (and
        (at end(shut_off ?r ?z))
    )
)

;; Handheld radio, one robot to another. There is no classical condition on
;; what may be said, because there is no classical way to say it: that a robot
;; may only pass on what it knows is `([?i] (safe))` on the epistemic side, and
;; it is enforced there.
(:durative-action radio
    :parameters (?r ?j - robot ?z - zone)
    :duration ( = ?duration 4)
    :condition (and
        (at start(is_suite ?z))
    )
    :effect (and
        (at end(radioed ?r ?j ?z))
    )
)

;; The public address system. Physically the cheapest way to inform the fleet,
;; and the goal is what makes it inadmissible.
(:durative-action page
    :parameters (?r - robot ?z - zone)
    :duration ( = ?duration 4)
    :condition (and
        (at start(is_suite ?z))
    )
    :effect (and
        (at end(paged ?r ?z))
    )
)

)
