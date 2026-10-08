;; The classical half of the hotel's distributed leak, written by
;; tools/hotel_domain.py. Nothing here knows where the leak is, who knows the
;; floor and who the column, or that the guest is listening. That is in the
;; EPDDL, and the executor checks it against the model.

(define (domain hotel-distributed)
(:requirements :strips :typing :adl :durative-actions)

(:types
  robot
  zone
)

(:predicates
  (robot_at ?r - robot ?z - zone)
  (looked ?r - robot ?z - zone)
  (shut_off ?r - robot ?z - zone)
  (said ?r - robot ?z - zone)
  (paged ?r - robot)
)

;; Long, because a ride between floors is a lift call and a wait as well.
;; Open-RMF decides how long it takes and the bridge waits for it.
(:durative-action goto_zone
  :parameters (?r - robot ?from ?to - zone)
  :duration (= ?duration 240)
  :condition (and (at start (robot_at ?r ?from)))
  :effect (and (at start (not (robot_at ?r ?from))) (at end (robot_at ?r ?to)))
)

(:durative-action look_into
  :parameters (?r - robot ?z - zone)
  :duration (= ?duration 10)
  :condition (and (over all (robot_at ?r ?z)))
  :effect (and (at end (looked ?r ?z)))
)

(:durative-action shut_valve
  :parameters (?r - robot ?z - zone)
  :duration (= ?duration 15)
  :condition (and (over all (robot_at ?r ?z)))
  :effect (and (at end (shut_off ?r ?z)))
)

(:durative-action say_floor
  :parameters (?r - robot ?z - zone)
  :duration (= ?duration 6)
  :condition (and (at start (robot_at ?r ?z)))
  :effect (and (at end (said ?r ?z)))
)

(:durative-action say_column
  :parameters (?r - robot ?z - zone)
  :duration (= ?duration 6)
  :condition (and (at start (robot_at ?r ?z)))
  :effect (and (at end (said ?r ?z)))
)

(:durative-action say_safe
  :parameters (?r - robot ?z - zone)
  :duration (= ?duration 6)
  :condition (and (at start (robot_at ?r ?z)))
  :effect (and (at end (said ?r ?z)))
)


(:durative-action page_floor
  :parameters (?r - robot)
  :duration (= ?duration 7)
  :condition (and)
  :effect (and (at end (paged ?r)))
)

(:durative-action page_column
  :parameters (?r - robot)
  :duration (= ?duration 7)
  :condition (and)
  :effect (and (at end (paged ?r)))
)

(:durative-action page_safe
  :parameters (?r - robot)
  :duration (= ?duration 7)
  :condition (and)
  :effect (and (at end (paged ?r)))
)

)
