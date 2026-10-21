;; THE LINK GRADES: the corridor on a link that may fall, for a classical planner.
;; Written by tools/make_pddl.py.
;;
;; What a robot knows is recorded as a fact, (knows ?r): it has decided the
;; corridor. A robot decides it by looking from the corridor's mouth, or is
;; told by one that has, which takes the link: report needs (link). Whether
;; (link) is in the problem is the formulation's.
(define (domain link-grades)
(:requirements :strips :typing :durative-actions)
(:types robot)
(:predicates
  (at_dock ?r - robot)
  (at_station ?r - robot)
  (at_spot ?r - robot)
  (knows ?r - robot)
  (link)
)

(:durative-action drive_dock
  :parameters (?r - robot)
  :duration (= ?duration 39)
  :condition (and (at start (at_dock ?r)))
  :effect (and (at start (not (at_dock ?r))) (at end (at_spot ?r))))

(:durative-action drive_station
  :parameters (?r - robot)
  :duration (= ?duration 82)
  :condition (and (at start (at_station ?r)))
  :effect (and (at start (not (at_station ?r))) (at end (at_spot ?r))))

(:durative-action inspect
  :parameters (?r - robot)
  :duration (= ?duration 5)
  :condition (and (over all (at_spot ?r)))
  :effect (and (at end (knows ?r))))

(:durative-action report
  :parameters (?from ?to - robot)
  :duration (= ?duration 5)
  :condition (and (at start (knows ?from)) (over all (link)))
  :effect (and (at end (knows ?to))))
)
