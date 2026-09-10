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
  ;; That this robot has scanned something, anywhere. Distinct from
  ;; `(scanned ?r ?s)` and load-bearing: see the note on `relay` below.
  (has_scanned ?r - robot)
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
    (at end (scanned ?r ?s))
    (at end (has_scanned ?r)))
)

;; The speaker must have scanned *something*, and not this site.
;;
;; `(at start (scanned ?from ?s))` is the obvious condition and it is wrong,
;; in a way that is worth spelling out because it is the whole domain in one
;; line. An agent here reports on a site it has never been to: with exactly
;; one of three contaminated, ruling two out settles the third, and the
;; announcement that carries the finding names a site the speaker only knows
;; about by inference. Requiring it to have scanned that site encodes the
;; assumption the epistemic half exists to deny.
;;
;; It is also the kind of wrong that only one branch shows. The branch where
;; the first scan comes back clean relays about the site just scanned and
;; passes; the branch where it comes back dirty relays about a different site
;; and the executor sits on
;;
;;     Error checking at start reqs: (and (scanned scout a06))
;;
;; for ever. Two of the three worlds ran green with this condition in place.
(:durative-action relay
  :parameters (?from ?to - robot ?s - site)
  :duration (= ?duration 2)
  :condition (and
    (at start (has_scanned ?from)))
  :effect (and
    (at end (told ?from ?s)))
)
)
