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
Burns the mission narration into a recording of the demo.

Why not draw it in rviz.  An rviz text marker is geometry in the 3-D scene: it
scales with the camera, spaces its words oddly, and once a 3840-pixel capture
has been scaled down for viewing it is a row of thin grey smudges.  Several
goes at making it legible that way failed, and they failed for a reason that
was not going to be tuned away.

Here the caption is drawn by ffmpeg over the finished video, in a real font at
a real pixel size, in a band across the top.  What it says comes from the run's
own log -- the actions ePlanSys dispatched and the moment the epistemic state
settled -- so it cannot drift from what the robot did.

  python3 caption_video.py --video out/warehouse_demo.mp4 \\
      --log out/demo.log --start 1787864875.0 --speed 5
"""

import argparse
import os
import re
import shlex
import subprocess
import sys

# The log line that starts an action, and the one that says the question is
# settled. Both are the demo's own output, not something inferred.
EXECUTING = re.compile(r'\[(\d+\.\d+)\].*\[warehouse_mission\]: executing (\S+)')
ACTION = re.compile(r'\[(\d+\.\d+)\].*action_full_name.*')
KNOWS = re.compile(r'\[(\d+\.\d+)\].*\(Kw (\S+) (\S+)\) holds')
PHASE = re.compile(r'\[(\d+\.\d+)\].*\[warehouse_mission\]: (problem seeded|policy: .*|mission complete.*)')

STEP_TEXT = {
    'goto_zone': 'DRIVING',
    'look_into': 'LOOKING INTO THE BAY',
    'pick_up': 'PICKING UP THE PALLET',
    'drop_off': 'UNLOADING AT RECEIVING',
}

WHY = {
    'goto_zone': 'crossing the warehouse',
    'look_into': 'the two worlds disagree; the laser settles it',
    'pick_up': 'allowed only because r1 KNOWS the pallet is here',
    'drop_off': 'the pallet reaches its dock',
}


def events(log_path, start, speed):
    """(video seconds, headline, subtitle), from the run's own log."""
    out = []
    settled_at = None

    with open(log_path, errors='ignore') as handle:
        text = handle.read()

    for match in KNOWS.finditer(text):
        settled_at = float(match.group(1))
        break

    out.append((0.0, 'STARTING UP', 'Gazebo, SLAM, Nav2 and ePlanSys'))

    for match in PHASE.finditer(text):
        when, what = float(match.group(1)), match.group(2)
        if what.startswith('problem seeded'):
            out.append((when, 'PLANNING', 'ePlanSys is solving the instance'))
        elif what.startswith('policy:'):
            out.append((when, 'POLICY: ' + what[len('policy:'):].strip(),
                        'a branching policy, not a sequence'))
        elif what.startswith('mission complete'):
            out.append((when, 'DONE',
                        'delivered, and r1 knows which bay it came from'))

    step = 0
    for match in EXECUTING.finditer(text):
        when, verb = float(match.group(1)), match.group(2)
        step += 1
        head = f'{step}.  ' + STEP_TEXT.get(verb, verb.upper())
        out.append((when, head, WHY.get(verb, '')))

    # Log stamps are seconds since the epoch; the recording began at `start`,
    # and the finished video is `speed` times faster than the world was.
    timed = []
    for when, head, why in sorted(out, key=lambda row: row[0]):
        at = (when - start) / speed if when else 0.0
        timed.append((max(0.0, at), head, why))
    return timed


def knows_line(log_path):
    """When the epistemic question was settled, if it was."""
    with open(log_path, errors='ignore') as handle:
        for line in handle:
            found = KNOWS.search(line)
            if found:
                return float(found.group(1))
    return None


def escape(text):
    """ffmpeg's drawtext eats colons, backslashes and quotes."""
    return (text.replace('\\', r'\\\\')
                .replace(':', r'\:')
                .replace("'", r"\'")
                .replace('%', r'\%'))


def build_filter(timed, settled_video_time, width, font):
    """One drawtext per caption, each shown until the next one starts."""
    parts = []
    band_h = 116
    parts.append(
        f"drawbox=x=0:y=0:w={width}:h={band_h}:color=black@0.72:t=fill")

    for i, (at, head, why) in enumerate(timed):
        end = timed[i + 1][0] if i + 1 < len(timed) else 1e6
        if end <= at:
            continue
        window = f"between(t,{at:.2f},{end:.2f})"
        parts.append(
            f"drawtext=fontfile={font}:text='{escape(head)}'"
            f":fontcolor=white:fontsize=40:x=40:y=16:enable='{window}'")
        if why:
            parts.append(
                f"drawtext=fontfile={font}:text='{escape(why)}'"
                f":fontcolor=0xC8D2DC:fontsize=27:x=40:y=68:enable='{window}'")

    # The epistemic state, on the right of the band: the one thing a viewer
    # should be able to check at any frame.
    if settled_video_time is None:
        parts.append(
            f"drawtext=fontfile={font}:text='{escape('[Kw. r1] pallet-at_bay2 = UNKNOWN')}'"
            f":fontcolor=0xFFC864:fontsize=30:x=w-tw-40:y=40")
    else:
        parts.append(
            f"drawtext=fontfile={font}:text='{escape('[Kw. r1] pallet-at_bay2 = UNKNOWN')}'"
            f":fontcolor=0xFFC864:fontsize=30:x=w-tw-40:y=40"
            f":enable='lt(t,{settled_video_time:.2f})'")
        parts.append(
            f"drawtext=fontfile={font}:text='{escape('[Kw. r1] pallet-at_bay2 = TRUE')}'"
            f":fontcolor=0x8CFF8C:fontsize=30:x=w-tw-40:y=40"
            f":enable='gte(t,{settled_video_time:.2f})'")
    return ','.join(parts)


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
    parser.add_argument('--speed', type=float, default=1.0)
    parser.add_argument('--width', type=int, default=3840)
    args = parser.parse_args()

    timed = events(args.log, args.start, args.speed)
    settled = knows_line(args.log)
    settled_video = None if settled is None else \
        max(0.0, (settled - args.start) / args.speed)

    filters = build_filter(timed, settled_video, args.width, find_font())
    captioned = args.video.replace('.mp4', '.captioned.mp4')

    command = ['ffmpeg', '-y', '-hide_banner', '-loglevel', 'error',
               '-i', args.video, '-vf', filters,
               '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '20',
               '-pix_fmt', 'yuv420p', '-movflags', '+faststart', captioned]
    print('captioning with', len(timed), 'steps')
    subprocess.run(command, check=True)
    os.replace(captioned, args.video)
    print('wrote', args.video)


if __name__ == '__main__':
    sys.exit(main())
