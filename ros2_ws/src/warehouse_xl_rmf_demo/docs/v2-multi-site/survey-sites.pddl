;; The classical half of the multi-site survey.
;;
;; As in the single-site model this is deliberately blind: nothing here can
;; tell a broadcast from an encrypted relay, and nothing here knows which site
;; is contaminated. It exists so the executor has something to drive.

(define (domain survey-sites)
(:requirements :strips :typing :adl :durative-actions)

(:types
  robot
  site
)

(:predicates
  (at_depot ?r - robot)
  (on_site ?r - robot ?s - site)
  (scanned ?r - robot ?s - site)
  (told ?r - robot ?s - site)
)

(:durative-action goto
  :parameters (?r - robot ?s - site)
  :duration (= ?duration 4)
  :condition (and
    (at start (at_depot ?r)))
  :effect (and
    (at start (not (at_depot ?r)))
    (at end (on_site ?r ?s)))
)

(:durative-action scan
  :parameters (?r - robot ?s - site)
  :duration (= ?duration 3)
  :condition (and
    (over all (on_site ?r ?s)))
  :effect (and
    (at end (scanned ?r ?s)))
)

(:durative-action relay
  :parameters (?from ?to - robot ?s - site)
  :duration (= ?duration 2)
  :condition (and
    (at start (scanned ?from ?s)))
  :effect (and
    (at end (told ?from ?s)))
)
)
