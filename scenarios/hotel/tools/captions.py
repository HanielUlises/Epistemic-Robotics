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
Turns a run log into the captions burnt over the film.

The film is usually sped up, because most of a run is two robots waiting for a
lift. `--speed` divides the positions by that factor and leaves the hold alone,
so a caption over a four-times film is still on screen long enough to read.

A recording of this mission without them is a video of two robots taking lifts.
Everything that makes it worth watching -- which suite the fleet is uncertain
about, what the inspection settled, that shutting the valve left the porter
believing something false -- happens in the model and is invisible on the
floor. These are the lines that say so, placed at the second the log says they
happened.

The log carries ROS timestamps, which for these nodes are wall clock. The
recorder writes down the instant ffmpeg started, and the difference is the
position in the film.

    captions.py --log run.log --start 1788385302.29 --out segments.txt

Each output line is

    start end | headline | detail

read back by record_demo.sh into one drawtext filter per line.
"""

import argparse
import re

# The bridge saying it has handed an action to RMF.
DISPATCH = re.compile(
    r'\[(?P<t>\d+\.\d+)\].*eplansys_rmf_bridge\]: '
    r'(?P<action>goto_zone|deploy|look_into|shut_valve): '
    r'(?P<agent>\w+) -> (?P<fleet>[\w-]+)/(?P<robot>[\w-]+) heading for (?P<place>\w+)')

# One action, several robots. The bridge says so after the last of them.
TOGETHER = re.compile(
    r'\[(?P<t>\d+\.\d+)\].*eplansys_rmf_bridge\]: '
    r'(?P<action>\w+): (?P<count>\d+) robots moving on one action')

LOCAL = re.compile(
    r'\[(?P<t>\d+\.\d+)\].*eplansys_rmf_bridge\]: '
    r'(?P<action>radio|page): speech act')

# The epistemic state after a product update. The world counts are the point:
# sensing takes them down, a private action puts them up.
APPLIED = re.compile(
    r'\[(?P<t>\d+\.\d+)\].*epistemic_state\] applied (?P<action>\S+?)'
    r'(?: -> (?P<outcome>\S+?))?: (?P<worlds>\d+) worlds, (?P<designated>\d+) designated')

PLANNED = re.compile(
    r'\[(?P<t>\d+\.\d+)\].*policy with (?P<nodes>\d+) nodes, (?P<shape>\w+)')

DONE = re.compile(r'\[(?P<t>\d+\.\d+)\].*(?P<verdict>mission complete|mission failed)')

ZONE = {
    'L2_master_suite': 'the L2 master suite',
    'L3_master_suite': 'the L3 master suite',
    'lobby': 'the lobby',
    'l2_suite': 'the L2 master suite',
    'l3_suite': 'the L3 master suite',
}

DISPATCH_CAPTION = {
    'goto_zone': (
        'Driving to {place}',
        '{agent} is {robot}; Open-RMF has the lift, the door and the traffic'),
    'look_into': (
        'Inspecting {place}',
        'Only a robot standing in the suite can tell, and the rest of the '
        'fleet will see that it looked without seeing what it found'),
    'shut_valve': (
        'Shutting the valve in {place}',
        'A closed suite, so only whoever is inside will know it happened'),
    'deploy': (
        'Sending {agent} to {place}',
        'One epistemic action, both robots: the policy commits to covering '
        'both floors before it knows which one matters'),
}


def zone(name):
    return ZONE.get(name, name)


def applied_caption(action, outcome, worlds, designated):
    """What the product update did, in words rather than in event names."""
    worlds, designated = int(worlds), int(designated)
    model = f'{worlds} worlds, {designated} designated'

    if action.startswith('inspect'):
        found = 'flooded' if outcome and outcome.endswith('wet') else 'dry'
        return (f'Inspected: {found}',
                f'Sensing narrowed the model to {model}')

    if action.startswith('contain'):
        return ('Valve shut, and only that suite saw it happen',
                f'A private change widened the model to {model}: whoever was '
                f'elsewhere still believes the suite is flooded')

    if action.startswith('brief-safe'):
        return ('Told the porter over the radio, not the public address',
                f'The false belief is repaired, and the guest never heard: {model}')

    if action.startswith('brief-leak'):
        return ('Told the porter which suite, over the radio',
                f'Private, so the guest learns nothing: {model}')

    if action.startswith('page'):
        return ('Paged the whole hotel', f'Everyone hears, guest included: {model}')

    if action.startswith('deploy'):
        return ('Both robots in position, one on each floor',
                f'Positions stay common knowledge: {model}')

    if action.startswith('go'):
        return ('Arrived', f'Positions stay common knowledge: {model}')

    return (action, model)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--log', required=True)
    parser.add_argument('--start', required=True, type=float,
                        help='epoch seconds at which the recording began')
    parser.add_argument('--out', required=True)
    parser.add_argument('--hold', type=float, default=6.0,
                        help='seconds a caption stays up, in the final film')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='how much the film was sped up, so that positions '
                             'are divided by it while the hold is not')
    args = parser.parse_args()

    events = []
    with open(args.log, errors='replace') as handle:
        for line in handle:
            for pattern, kind in ((PLANNED, 'planned'), (TOGETHER, 'together'),
                                  (DISPATCH, 'dispatch'), (LOCAL, 'local'),
                                  (APPLIED, 'applied'), (DONE, 'done')):
                match = pattern.search(line)
                if not match:
                    continue
                events.append((float(match.group('t')), kind, match))
                break

    events.sort(key=lambda e: e[0])

    lines = []
    for stamp, kind, match in events:
        at = (stamp - args.start) / args.speed
        if at < 0:
            at = 0.0

        if kind == 'planned':
            # The planner and the mission node both announce the policy; one
            # caption is enough.
            if any('policy' in h for _, _, h, _ in lines):
                continue
            headline = f"A {match.group('shape')} policy, {match.group('nodes')} nodes"
            detail = ('Two suites, one leak, and the plan splits the robots '
                      'across both lifts before it knows which')
        elif kind == 'together':
            headline = f"{match.group('count')} robots, one action, both lifts"
            detail = ('The executor runs a policy strictly in order, so robots '
                      'move together only when one event moves both of them')
        elif kind == 'dispatch':
            template, detail_template = DISPATCH_CAPTION[match.group('action')]
            fields = {
                'agent': match.group('agent'),
                'robot': match.group('robot'),
                'place': zone(match.group('place')),
            }
            headline = template.format(**fields)
            detail = detail_template.format(**fields)
        elif kind == 'local':
            headline = 'Speaking'
            detail = 'No robot moves to say something, so RMF is not asked'
            continue
        elif kind == 'applied':
            headline, detail = applied_caption(
                match.group('action'), match.group('outcome'),
                match.group('worlds'), match.group('designated'))
        else:
            headline = match.group('verdict').capitalize()
            detail = 'The porter knows, and the guest never learned which suite'

        lines.append((at, kind, headline, detail))

    # A joint action logs one line per robot and then one line saying how many
    # moved together. Those land in the same instant and would be drawn on top
    # of each other, so a run of dispatches followed by a `together` collapses
    # into the `together`. Nothing else is dropped: a caption that says what
    # the model did is never redundant.
    WINDOW = 0.75
    kept = []
    index = 0
    while index < len(lines):
        at, kind, headline, detail = lines[index]

        if kind == 'dispatch':
            run_end = index
            while (run_end + 1 < len(lines) and
                   lines[run_end + 1][1] in ('dispatch', 'together') and
                   lines[run_end + 1][0] - at < WINDOW):
                run_end += 1
            if lines[run_end][1] == 'together':
                kept.append(lines[run_end])       # the one that says "2 robots"
            else:
                kept.append(lines[index])         # a lone dispatch
            index = run_end + 1
            continue

        kept.append(lines[index])
        index += 1

    # What is left may still be closer together than it can be read. Each
    # caption is pushed to start no earlier than the previous one ends, which
    # queues a burst of them instead of flashing it.
    MIN_SHOWN = 2.6
    placed = []
    previous_end = 0.0
    for at, kind, headline, detail in kept:
        start = max(at, previous_end)
        end = start + MIN_SHOWN
        placed.append((start, end, headline, detail))
        previous_end = end

    with open(args.out, 'w') as handle:
        handle.write('# start end | headline | detail\n')
        for i, (start, end, headline, detail) in enumerate(placed):
            # A caption holds until the next one is due, up to --hold, so a long
            # lift ride does not leave the frame bare.
            following = placed[i + 1][0] if i + 1 < len(placed) else end + args.hold
            handle.write(
                f'{start:.2f} {min(start + args.hold, max(end, following)):.2f} '
                f'| {headline} | {detail}\n')

    print(f'{len(placed)} captions -> {args.out}')


if __name__ == '__main__':
    main()
