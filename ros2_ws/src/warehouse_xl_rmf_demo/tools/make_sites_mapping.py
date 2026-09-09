#!/usr/bin/env python3
# Copyright 2026 Haniel Ulises
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
Writes the action mapping for the multi-site survey, and writes all of it.

`draft_epistemic_mapping` resolves what it can resolve and refuses the rest.
For this domain it resolves 18 of 72 entries -- the `goto`s and the `scan`s,
whose names it finds in the PDDL domain -- and leaves 54, every grounding of
`relay-dirty` and `relay-clean`, with a note saying the correspondence is a
modelling decision and has to be written by hand.

It is right that it refuses. There is no `relay-dirty` in the PDDL domain and
there cannot be: the classical half of this mission is deliberately blind, and
a planner that could see the difference between reporting contamination and
reporting its absence would be reasoning about the thing the epistemic half
exists to reason about. So the tool cannot infer the correspondence.

But the decision, once taken, is one line and not fifty-four. Both epistemic
announcements carry the same classical content -- an agent said something to
another agent about a site -- and both therefore map onto the single classical
`(relay ?i ?j ?s)`. What separates them is which world the speaker was in when
they spoke, which is exactly what the epistemic model holds and the PDDL does
not. Writing that rule down here, once, is the honest form of the decision;
writing it out fifty-four times by hand is the same decision plus fifty-three
opportunities to mistype an agent name.

The 54 are not decoration. They are every (speaker, listener, site) triple for
each of the two announcements: 2 x 3 x 3 x 3. The policy uses four of them.

    make_sites_mapping.py --draft draft.json --out pddl/survey-sites-mapping.json
"""

import argparse
import json
import sys


# What a speech act costs the executor, in seconds. RMF is not asked to do
# anything for these -- nobody moves to say something -- so the duration is
# the pause the bridge holds before reporting the action done.
RELAY_DURATION = 2.0

# The two announcements of the domain, and the classical action both collapse
# onto. Keyed on the epistemic action name, which is the part of a grounded
# name before the first underscore.
COLLAPSE = {
    'relay-dirty': 'relay',
    'relay-clean': 'relay',
}


def complete(draft):
    """Return (mapping, decided, untouched) with every `_check` resolved."""
    mapping, decided, untouched = {}, [], []

    for name, entry in sorted(draft.items()):
        spec = dict(entry)
        if '_check' not in spec:
            mapping[name] = spec
            untouched.append(name)
            continue

        head, _, rest = name.partition('_')
        if head not in COLLAPSE:
            # Something in the domain changed and this tool has not been told
            # about it. Guessing here would produce a mapping that runs and
            # dispatches the wrong action, which is the failure this whole
            # file exists to avoid.
            raise SystemExit(
                f'{name}: no rule for the epistemic action {head!r}. '
                f'The rules cover {sorted(COLLAPSE)}; either the domain grew '
                f'an announcement or a site name has an underscore in it, '
                f'which would have split the arguments wrongly.')

        args = rest.split('_')
        spec['action'] = f'({COLLAPSE[head]} ' + ' '.join(args) + ')'
        spec['duration'] = RELAY_DURATION
        del spec['_check']
        mapping[name] = spec
        decided.append(name)

    return mapping, decided, untouched


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--draft', required=True,
                    help="what draft_epistemic_mapping wrote, refusals and all")
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    with open(args.draft) as handle:
        draft = json.load(handle)

    mapping, decided, untouched = complete(draft)

    with open(args.out, 'w') as handle:
        json.dump(mapping, handle, indent=2, sort_keys=True)
        handle.write('\n')

    print(f'{args.out}: {len(mapping)} entries')
    print(f'  {len(untouched)} resolved by the drafter, {len(decided)} by the '
          f'rule above')
    for rule, target in sorted(COLLAPSE.items()):
        count = sum(1 for n in decided if n.startswith(rule + '_'))
        print(f'    {rule:>12} -> ({target} ...)   {count} groundings')
    return 0


if __name__ == '__main__':
    sys.exit(main())
