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
The executor's vocabulary for the epistemic policy, and the mapping into it.

    make_pddl.py --out pddl/

be-vocabulary.pddl  one action per thing a robot does, over two kinds of fact:
                    (ok ?r), which an action of robot ?r reads and writes, and
                    (radio), which every hail writes and every drive to the
                    site reads. ePlanSys's executor dispatches together the
                    consecutive actions of a policy of which neither writes what
                    the other reads: the aerial vehicle's flight to the site and
                    the ground vehicle's drive run together, and the drive never
                    starts before the hail that tells the vehicle of the task.
                    What decides the policy is in the EPDDL.
be-mapping.json     the EPDDL's grounded actions as vocabulary actions.
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402

# name: (parameters, facts read and written)
VOCAB = {
    'fly_p1': ('?u ?g', ['(ok ?u)']), 'back_p2': ('?u ?g', ['(ok ?u)']), 'back_p3': ('?u ?g', ['(ok ?u)']),
    'hail_p1': ('?u ?g', ['(ok ?u)', '(radio)']), 'hail_p2': ('?u ?g', ['(ok ?u)', '(radio)']),
    'hail_p3': ('?u ?g', ['(ok ?u)', '(radio)']),
    'cross_west': ('?u', ['(ok ?u)']), 'cross_east': ('?u', ['(ok ?u)']),
    'fly_site': ('?u', ['(ok ?u)']),
    'go_site': ('?g', ['(ok ?g)', '(radio)']),
    'perform': ('?u ?g', ['(ok ?u)', '(ok ?g)']),
}
DURATION = {'fly_p1': 40, 'back_p2': 15, 'back_p3': 15, 'hail_p1': 8, 'hail_p2': 8, 'hail_p3': 8,
            'cross_west': 1, 'cross_east': 1, 'fly_site': 40, 'go_site': 120, 'perform': 10}


def vocabulary():
    acts = ''
    for name, (params, facts) in VOCAB.items():
        typed = ' '.join(params.split()) + ' - robot'
        cond = ' '.join(f'(at start {f})' for f in facts)
        eff = ' '.join(f'(at end {f})' for f in facts)
        acts += (f'\n(:durative-action {name}\n  :parameters ({typed})\n  :duration (= ?duration 10)\n'
                 f'  :condition (and {cond})\n  :effect (and {eff}))\n')
    return (';; BELIEF AND EMPATHY PARTICLES: the executor\'s vocabulary under an epistemic policy.\n'
            ';; Written by tools/make_pddl.py. What decides the policy is in the EPDDL.\n'
            '(define (domain belief-empathy)\n(:requirements :strips :typing :durative-actions)\n'
            f'(:types robot)\n(:predicates (ok ?r - robot) (radio))\n{acts})\n')


def mapping():
    out = {}
    for g in L.UGVS:
        for b in (1, 2, 3):
            out[f'hail-p{b}_u_{g}'] = {'action': f'(hail_p{b} u {g})', 'duration': float(DURATION[f'hail_p{b}'])}
        out[f'fly-p1_u_{g}'] = {'action': f'(fly_p1 u {g})', 'duration': float(DURATION['fly_p1'])}
        for b in (2, 3):
            out[f'back-p{b}_u_{g}'] = {'action': f'(back_p{b} u {g})', 'duration': float(DURATION[f'back_p{b}'])}
        out[f'go-site_{g}'] = {'action': f'(go_site {g})', 'duration': float(DURATION['go_site'])}
        out[f'perform_u_{g}'] = {'action': f'(perform u {g})', 'duration': float(DURATION['perform'])}
    for n in ('cross-west', 'cross-east', 'fly-site'):
        v = n.replace('-', '_')
        out[f'{n}_u'] = {'action': f'({v} u)', 'duration': float(DURATION[v])}
    return out


def problem():
    return ('(define (problem belief-empathy) (:domain belief-empathy)\n(:objects u g1 g2 - robot)\n'
            '(:init (ok u) (ok g1) (ok g2) (radio))\n(:goal (and (ok u))))\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    open(os.path.join(a.out, 'be-vocabulary.pddl'), 'w').write(vocabulary())
    json.dump(mapping(), open(os.path.join(a.out, 'be-mapping.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
