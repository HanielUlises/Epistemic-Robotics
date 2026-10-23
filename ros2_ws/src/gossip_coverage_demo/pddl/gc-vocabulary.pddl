;; GOSSIP OUT OF CONTACT: the executor's vocabulary under an epistemic policy.
;; Written by tools/make_pddl.py. What decides the policy is in the EPDDL.
(define (domain gossip-coverage)
(:requirements :strips :typing :durative-actions)
(:types robot)
(:predicates (ok ?r - robot) (count_1) (count_2) (count_3) (count_4) (count_5) (count_6) (count_7) (count_8))
(:durative-action call
  :parameters (?a ?b - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?a)) (at start (ok ?b)))
  :effect (and (at end (ok ?a)) (at end (ok ?b))))

(:durative-action await
  :parameters (?a ?b - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?a)) (at start (ok ?b)))
  :effect (and (at end (ok ?a)) (at end (ok ?b))))

(:durative-action leave
  :parameters (?a ?b - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?a)) (at start (ok ?b)))
  :effect (and (at end (ok ?a)) (at end (ok ?b))))

(:durative-action go_site_0
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?r)))
  :effect (and (at end (ok ?r)) (at end (count_1))))

(:durative-action go_site_1
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?r)))
  :effect (and (at end (ok ?r)) (at end (count_2))))

(:durative-action go_site_2
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?r)))
  :effect (and (at end (ok ?r)) (at end (count_3))))

(:durative-action go_site_3
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?r)))
  :effect (and (at end (ok ?r)) (at end (count_4))))

(:durative-action go_site_4
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?r)))
  :effect (and (at end (ok ?r)) (at end (count_5))))

(:durative-action go_site_5
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?r)))
  :effect (and (at end (ok ?r)) (at end (count_6))))

(:durative-action go_site_6
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?r)))
  :effect (and (at end (ok ?r)) (at end (count_7))))

(:durative-action go_site_7
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?r)))
  :effect (and (at end (ok ?r)) (at end (count_8))))

(:durative-action do_task_1
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?r)))
  :effect (and (at end (ok ?r))))

(:durative-action do_task_2
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?r)) (at start (count_1)))
  :effect (and (at end (ok ?r))))

(:durative-action do_task_3
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?r)) (at start (count_1)) (at start (count_2)))
  :effect (and (at end (ok ?r))))

(:durative-action do_task_4
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?r)) (at start (count_1)) (at start (count_2)) (at start (count_3)))
  :effect (and (at end (ok ?r))))

(:durative-action do_task_5
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?r)) (at start (count_1)) (at start (count_2)) (at start (count_3)) (at start (count_4)))
  :effect (and (at end (ok ?r))))

(:durative-action do_task_6
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?r)) (at start (count_1)) (at start (count_2)) (at start (count_3)) (at start (count_4)) (at start (count_5)))
  :effect (and (at end (ok ?r))))

(:durative-action do_task_7
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?r)) (at start (count_1)) (at start (count_2)) (at start (count_3)) (at start (count_4)) (at start (count_5)) (at start (count_6)))
  :effect (and (at end (ok ?r))))

(:durative-action do_task_8
  :parameters (?r - robot)
  :duration (= ?duration 10)
  :condition (and (at start (ok ?r)) (at start (count_1)) (at start (count_2)) (at start (count_3)) (at start (count_4)) (at start (count_5)) (at start (count_6)) (at start (count_7)))
  :effect (and (at end (ok ?r))))
)
