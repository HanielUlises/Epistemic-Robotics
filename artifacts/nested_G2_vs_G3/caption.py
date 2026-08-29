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
Burns the three formulas into a recording of a nested run.

`scenarios/warehouse/tools/caption_video.py` narrates the fetch mission: what
the robot is doing, and the one moment its own question is settled. This one
narrates something a camera cannot see at all.

The result of a nested run is not an event in the warehouse. It is an instant
at which one formula over the epistemic model becomes true while another over
the same model does not, and on the screen that instant looks like a small
robot finishing a turn. So the panel is the picture: three rows, one per
formula, each reading UNDECIDED or HOLDS, changing when and only when the run's
own probe log says the epistemic state changed its answer.

Nothing here is timed by hand. Every transition comes from a `formula_probe`
line in the log, every step from a `warehouse_mission` line in the same log,
and both carry the epoch stamp ROS wrote on them -- so the caption cannot drift
from the run, and two runs captioned this way can be read against each other.

  python3 caption.py --video media/G2-raw.mp4 --log logs/G2.log \\
      --start "$(cat logs/G2.start)" --goal G2
"""

import argparse
import os
import re
import subprocess
import sys

# Everything below is the demo's own output. None of it is inferred.
PROBE = re.compile(
    r'\[(\d+\.\d+)\].*\[formula_probe\]: \[t=\s*[\d.]+\] (\S+)\s+(TRUE|FALSE)')
EXECUTING = re.compile(r'\[(\d+\.\d+)\].*\[warehouse_mission\]: executing (\S+)')
PHASE = re.compile(
    r'\[(\d+\.\d+)\].*\[warehouse_mission\]: '
    r'(problem seeded|policy: .*|mission complete.*)')
OUTCOME = re.compile(r'\[(\d+\.\d+)\].*\[epistemic_bt\].*observed (e-inspect-\S+)')

# The three formulas, in the order they are stacked. The middle one is the
# nested one and is drawn in the accent colour: it is the row the run is for.
# The site's palette, carried onto a dark band: near-white for the ordinary
# rows, a light steel for the nested one, and the accent red for the verdict
# that does not hold. Status greens and ambers were tried and read as though
# they came from somewhere else.
ROWS = [
    ('Kw_r1_P', '[Kw r1] pallet-at_bay2', 0xDCDCDC),
    ('K_r2_Kw_r1_P', '[K r2] [Kw r1] pallet-at_bay2', 0x7FA8DC),
    ('Kw_r2_P', '[Kw r2] pallet-at_bay2', 0xDCDCDC),
]

HOLDS, UNDECIDED = 0xFFFFFF, 0xE2445F

# Which of them the planner was actually given. The other two are asked of the
# same state in the same run and were nobody's goal, which is what makes the
# panel evidence rather than an illustration of the goal.
GOAL_ROW = {'G1': 'Kw_r1_P', 'G2': 'K_r2_Kw_r1_P', 'G3': 'Kw_r2_P'}

STEP_TEXT = {
    'goto_zone': 'DRIVING',
    'look_into': 'LOOKING INTO THE BAY',
    'pick_up': 'PICKING UP THE PALLET',
    'drop_off': 'UNLOADING AT RECEIVING',
    'announce': 'RADIOING THE FLEET',
}
WHY = {
    'goto_zone': 'crossing the warehouse to the aisle nobody has looked into',
    'look_into': 'semi-private: the fleet sees THAT r1 looks, not what it sees',
    'pick_up': 'allowed only where r1 KNOWS the pallet is',
    'announce': 'the only action that tells r2 the answer rather than the question',
}
OUTCOME_TEXT = {
    'e-inspect-found': ('THE PALLET IS HERE', 'the branch was not chosen when '
                        'the plan was made'),
    'e-inspect-empty': ('THE BAY IS EMPTY', 'so it is in the other aisle -- and '
                        'r1 now knows which'),
}


def escape(text):
    """ffmpeg's drawtext eats colons, backslashes, quotes and percent signs."""
    return (text.replace('\\', r'\\\\')
                .replace(':', r'\:')
                .replace("'", r"\'")
                .replace('%', r'\%')
                .replace('[', r'\[')
                .replace(']', r'\]'))


def read(log_path):
    with open(log_path, errors='ignore') as handle:
        return handle.read()


def transitions(text):
    """When each formula changed its answer, in epoch seconds.

    Only changes are recorded, because only changes were logged: the probe
    prints a line when a formula's answer differs from the last one it got, so
    this is the sequence of instants the model moved.
    """
    out = {label: [] for label, _title, _colour in ROWS}
    for match in PROBE.finditer(text):
        when, label, value = float(match.group(1)), match.group(2), match.group(3)
        if label in out:
            out[label].append((when, value == 'TRUE'))
    return out


def steps(text, start, speed):
    """(video seconds, headline, subtitle) for the band across the top."""
    rows = [(0.0, 'STARTING UP', 'Gazebo, SLAM, Nav2 and ePlanSys')]

    for match in PHASE.finditer(text):
        when, what = float(match.group(1)), match.group(2)
        if what.startswith('problem seeded'):
            rows.append((when, 'PLANNING', 'ALETHEIA is solving the instance'))
        elif what.startswith('policy:'):
            rows.append((when, 'POLICY: ' + what[len('policy:'):].strip(),
                         'a branching policy, not a sequence'))
        elif what.startswith('mission complete'):
            rows.append((when, 'DONE', 'the goal formula holds'))

    for match in OUTCOME.finditer(text):
        when, event = float(match.group(1)), match.group(2)
        if event in OUTCOME_TEXT:
            head, why = OUTCOME_TEXT[event]
            rows.append((when, head, why))

    count = 0
    for match in EXECUTING.finditer(text):
        when, verb = float(match.group(1)), match.group(2)
        count += 1
        rows.append((when, '%d.  %s' % (count, STEP_TEXT.get(verb, verb.upper())),
                     WHY.get(verb, '')))

    timed = []
    for when, head, why in sorted(rows, key=lambda row: row[0]):
        timed.append((max(0.0, (when - start) / speed if when else 0.0), head, why))
    return timed


def panel(changes, start, speed, font, goal, width):
    """The three formulas, stacked, each row saying what it said at that frame."""
    top, height, gap = 130, 132, 34
    parts = [
        "drawbox=x=0:y=%d:w=760:h=%d:color=black@0.88:t=fill" % (top, height),
        "drawtext=fontfile=%s:text='%s':fontcolor=0x969696:fontsize=19"
        ":x=28:y=%d" % (font, escape('THE EPISTEMIC STATE, ASKED TWICE A SECOND'),
                        top + 9),
    ]

    for index, (label, title, colour) in enumerate(ROWS):
        y = top + 36 + index * gap
        goal_mark = '  <- the goal' if GOAL_ROW.get(goal) == label else ''
        parts.append(
            "drawtext=fontfile=%s:text='%s':fontcolor=0x%06X:fontsize=21"
            ":x=28:y=%d" % (font, escape(title + goal_mark), colour, y))

        # The verdict, drawn at a fixed column so the three rows line up and a
        # viewer can read the difference between them at a glance rather than
        # by comparing two lengths of text.
        marks = changes.get(label) or []
        spans, previous_at, previous_value = [], 0.0, None
        for when, holds in marks:
            at = max(0.0, (when - start) / speed)
            if previous_value is not None:
                spans.append((previous_at, at, previous_value))
            previous_at, previous_value = at, holds
        if previous_value is None:
            spans.append((0.0, 1e6, False))
        else:
            spans.append((previous_at, 1e6, previous_value))

        for begin, end, holds in spans:
            if end <= begin:
                continue
            parts.append(
                "drawtext=fontfile=%s:text='%s':fontcolor=0x%06X:fontsize=21"
                ":x=560:y=%d:enable='between(t,%.2f,%.2f)'"
                % (font, 'HOLDS' if holds else 'UNDECIDED',
                   HOLDS if holds else UNDECIDED, y, begin, end))
    return parts


def find_font():
    for path in ('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
                 '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
                 '/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf'):
        if os.path.exists(path):
            return path
    raise SystemExit('no usable font found for the caption')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video', required=True)
    parser.add_argument('--log', required=True)
    parser.add_argument('--start', type=float, required=True,
                        help='epoch seconds when the capture began')
    parser.add_argument('--goal', required=True, choices=sorted(GOAL_ROW))
    parser.add_argument('--speed', type=float, default=1.0)
    parser.add_argument('--width', type=int, default=3840)
    parser.add_argument('--trim', default='auto',
                        help='seconds of the head to drop, or "auto" to begin '
                             'shortly before the problem is seeded, or "none"')
    parser.add_argument('--out')
    args = parser.parse_args()

    font = find_font()
    text = read(args.log)

    # Where to begin.
    #
    # A fleet of three takes three minutes to come up -- Gazebo, three mappers,
    # three Nav2 stacks -- and none of it is the demonstration. `auto` starts
    # the clip shortly before the mission seeds the problem, which is the first
    # frame in which anything is being decided. The captions are unaffected:
    # they are placed from the log's own epoch stamps against the epoch the
    # clip now starts at, so trimming the head moves the picture and the words
    # together or not at all.
    lead = 8.0
    if args.trim == 'auto':
        seeded = PHASE.search(text)
        first = float(seeded.group(1)) - lead if seeded else args.start
        trim = max(0.0, first - args.start)
    elif args.trim in ('none', ''):
        trim = 0.0
    else:
        trim = float(args.trim)
    args.start += trim
    timed = steps(text, args.start, args.speed)
    changes = transitions(text)

    parts = []
    if args.speed != 1.0:
        parts.append("setpts=PTS/%g" % args.speed)
    parts.append("drawbox=x=0:y=0:w=%d:h=118:color=black@0.85:t=fill" % args.width)
    for index, (at, head, why) in enumerate(timed):
        end = timed[index + 1][0] if index + 1 < len(timed) else 1e6
        if end <= at:
            continue
        window = "between(t,%.2f,%.2f)" % (at, end)
        parts.append(
            "drawtext=fontfile=%s:text='%s':fontcolor=white:fontsize=40"
            ":x=40:y=16:enable='%s'" % (font, escape(head), window))
        if why:
            parts.append(
                "drawtext=fontfile=%s:text='%s':fontcolor=0xC8D2DC:fontsize=26"
                ":x=40:y=68:enable='%s'" % (font, escape(why), window))

    # Which run this is, always on screen. Two recordings of the same warehouse
    # that differ in one formula are worth nothing to a viewer who cannot tell
    # which one they are looking at.
    parts.append(
        "drawtext=fontfile=%s:text='%s':fontcolor=0xFFFFFF:fontsize=30"
        ":x=w-tw-40:y=40" % (font, escape('GOAL ' + args.goal)))
    parts += panel(changes, args.start, args.speed, font, args.goal, args.width)

    out = args.out or args.video.replace('-raw.mp4', '.mp4')
    command = ['ffmpeg', '-y', '-hide_banner', '-loglevel', 'error']
    if trim > 0.0:
        command += ['-ss', '%.2f' % trim]
    command += ['-i', args.video, '-vf', ','.join(parts),
               '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '20',
               '-pix_fmt', 'yuv420p', '-movflags', '+faststart', out]
    print('captioning %s: %d steps, %d formula transitions, '
          'head trimmed %.0f s, %g x'
          % (args.goal, len(timed), sum(len(v) for v in changes.values()),
             trim, args.speed))
    subprocess.run(command, check=True)
    print('wrote', out)


if __name__ == '__main__':
    sys.exit(main())
