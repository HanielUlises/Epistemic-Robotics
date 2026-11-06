#!/usr/bin/env python3
# Copyright 2026 Haniel Vásquez Morales
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
The executor's vocabulary, the action mapping, and the classical model.

    make_pddl.py --out pddl/

news-vocabulary.pddl  the executor's vocabulary under the epistemic policy: one
                      action per thing a robot does, each touching only its
                      own robot's fact, so that the executor may dispatch two
                      robots' actions together. A message writes (radio) and a
                      crossing reads it, so that no crossing is dispatched
                      beside the message it waits for. What decides the policy
                      is in the EPDDL.
news-mapping.json     the EPDDL's grounded actions as vocabulary actions.
news-classical.pddl   the classical model: the courier crosses a door while
                      (assumed_open_<d>) holds. The mission writes the facts of
                      one valuation, the most convenient of those consistent
                      with what is known, and plans again when that valuation
                      is ruled out.

No condition is negative: POPF through PlanSys2 does not take them. Durations
are the routes' lengths at layout.SPEED; the executor only reports overruns.
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402

LOOK = 4
TELL = 3
DOORS = ('w', 'm', 'e')
VOCAB = ['goto_vw', 'goto_vm', 'goto_ve', 'look_w', 'look_m', 'look_e', 'tell_w', 'tell_m', 'tell_e',
         'cross_w', 'cross_m', 'cross_e', 'deliver']
SOUTH = ('st', 'vw', 'vm', 've')


def goto_trip(a, b):
    nodes, south, _ = L.roadmap()
    return L.trip(L.route(south, a, b))


def cross_trip(d):
    return L.trip(L.CROSS[d])


def deliver_trip(d):
    _, _, north = L.roadmap()
    return L.trip(L.route(north, L.CROSS[d][-1], 'r'))


def durative(name, params, d, cond, eff):
    return (f'\n(:durative-action {name}\n  :parameters ({params})\n  :duration (= ?duration {d})\n'
            f'  :condition (and {cond})\n  :effect (and {eff}))\n')


def vocabulary():
    acts = ''
    for n in VOCAB:
        cond = '(at start (robot_ok ?r))' + (' (at start (radio))' if n.startswith(('tell_', 'cross_')) else '')
        eff = '(at end (robot_ok ?r))' + (' (at end (radio))' if n.startswith('tell_') else '')
        acts += durative(n, '?r - robot', 10, cond, eff)
    return (';; REPLANNING ON NEWS: the executor\'s vocabulary under an epistemic policy.\n'
            ';; Written by tools/make_pddl.py. What decides the policy is in the EPDDL.\n'
            '(define (domain replanning-news)\n(:requirements :strips :typing :durative-actions)\n'
            f'(:types robot)\n(:predicates (robot_ok ?r - robot) (radio))\n{acts})\n')


def classical():
    preds = ['(at_st ?r - robot)', '(at_vw ?r - robot)', '(at_vm ?r - robot)', '(at_ve ?r - robot)',
             '(at_r ?r - robot)'] + [f'(at_n{d} ?r - robot)' for d in DOORS] + [f'(assumed_open_{d})' for d in DOORS]
    acts = ''
    for v in ('vw', 'vm', 've'):
        for a in SOUTH:
            if a == v:
                continue
            acts += durative(f'goto_{v}_from_{a}', '?r - robot', goto_trip(a, v), f'(at start (at_{a} ?r))',
                             f'(at start (not (at_{a} ?r))) (at end (at_{v} ?r))')
    for d in DOORS:
        acts += durative(f'cross_{d}', '?r - robot', cross_trip(d),
                         f'(at start (at_v{d} ?r)) (over all (assumed_open_{d}))',
                         f'(at start (not (at_v{d} ?r))) (at end (at_n{d} ?r))')
        acts += durative(f'deliver_from_{d}', '?r - robot', deliver_trip(d), f'(at start (at_n{d} ?r))',
                         f'(at start (not (at_n{d} ?r))) (at end (at_r ?r))')
    plist = '\n  '.join(preds)
    return (';; REPLANNING ON NEWS for a classical planner: a door may be crossed while it is\n'
            ';; assumed open. Written by tools/make_pddl.py.\n'
            '(define (domain replanning-news)\n(:requirements :strips :typing :durative-actions)\n'
            f'(:types robot)\n(:predicates\n  {plist}\n)\n{acts})\n')


def mapping():
    out = {}
    for r in ('c', 's'):
        for v in ('vw', 'vm', 've'):
            out[f'goto-{v}_{r}'] = {'action': f'(goto_{v} {r})', 'duration': 60.0}
    for r in ('c', 's'):
        for d in DOORS:
            out[f'look-{d}_{r}'] = {'action': f'(look_{d} {r})', 'duration': float(LOOK)}
    for r in ('s', 'k'):
        for d in DOORS:
            out[f'tell-{d}_{r}'] = {'action': f'(tell_{d} {r})', 'duration': float(TELL)}
    for d in DOORS:
        out[f'cross-{d}_c'] = {'action': f'(cross_{d} c)', 'duration': float(cross_trip(d))}
    out['deliver_c'] = {'action': '(deliver c)', 'duration': float(deliver_trip('e'))}
    return out


def problem(robots, facts, goal):
    return (f"(define (problem replanning-news) (:domain replanning-news)\n"
            f"(:objects {' '.join(robots)} - robot)\n(:init {' '.join(facts)})\n(:goal (and {' '.join(goal)})))\n")


def problem_vocabulary(robots):
    return problem(robots, [f'(robot_ok {r})' for r in robots] + ['(radio)'], ['(robot_ok c)'])


def cost(place, d):
    """What the courier's trip to the resident through door d takes, from place, in seconds."""
    return (goto_trip(place, f'v{d}') if place != f'v{d}' else 0) + cross_trip(d) + deliver_trip(d)


def problem_classical(place, assumed_open):
    """The courier at place (st, vw, vm, ve), with these doors assumed open."""
    return problem(['c'], [f'(at_{place} c)'] + [f'(assumed_open_{d})' for d in assumed_open], ['(at_r c)'])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    open(os.path.join(a.out, 'news-vocabulary.pddl'), 'w').write(vocabulary())
    open(os.path.join(a.out, 'news-classical.pddl'), 'w').write(classical())
    json.dump(mapping(), open(os.path.join(a.out, 'news-mapping.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
