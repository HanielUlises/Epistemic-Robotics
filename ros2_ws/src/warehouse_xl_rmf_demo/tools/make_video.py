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
Composes the screen capture into the demonstration video.

The capture is a 3840x1080 grab of two monitors, Gazebo on the left and RViz on
the right. Each half is scaled to half of a 1080p frame, so the pair sits side
by side at 16:9 with nothing cropped away.

Captions are written from the mission's own log rather than by hand: the times
are taken from the log lines the bridge and the executor emit, offset against
the moment recording began and divided by the playback speed. A caption that
says the robot is sensing therefore appears when it sensed.

    make_video.py --raw raw.mkv --log survey.log --t0 <unix time> --out out.mp4
"""

import argparse
import os
import subprocess
import tempfile

FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
MONO = '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf'


def esc(text):
    """Escape a literal for drawtext inside a filter graph."""
    # Apostrophes are removed rather than escaped: a quote inside a filter
    # literal terminates it whatever is done to it, and a caption that leaks
    # its own drawtext parameters onto the screen is worse than one without a
    # possessive.
    # Quotes and commas are removed rather than escaped. Both terminate a
    # filter literal, and escaping them inside a filter-complex script does not
    # reliably prevent that: the first attempt put a caption's own drawtext
    # parameters on screen, and the second split the graph on a comma in the
    # word "warehouse, 42 x 63 m". A caption without a possessive or a comma is
    # a smaller loss than either.
    out = text.replace("'", '').replace('"', '').replace(',', ' ')
    for ch in (':', '[', ']', ';', '\\'):
        out = out.replace(ch, '\\' + ch)
    return out


def drawtext(text, font, size, x, y, start=None, end=None, alpha='0.82'):
    parts = [
        f'fontfile={font}',
        f'text={esc(text)}',
        'fontcolor=white',
        f'fontsize={size}',
        'box=1',
        f'boxcolor=0x101418@{alpha}',
        'boxborderw=14',
        f'x={x}',
        f'y={y}',
    ]
    if start is not None:
        # Commas inside the enable expression belong to the expression, not to
        # the filter graph, so they are escaped rather than left to split it.
        parts.append(f"enable=between(t\\,{start}\\,{end})")
    return 'drawtext=' + ':'.join(parts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--raw', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--speed', type=float, default=2.0)
    ap.add_argument('--trim', type=float, default=60.0,
                    help='seconds of capture to keep, from the start')
    ap.add_argument('--precomposed', action='store_true',
                    help='the capture is already a stacked 1920x1080 frame')
    ap.add_argument('--gazebo-x', type=int, default=1920)
    ap.add_argument('--rviz-x', type=int, default=0)
    ap.add_argument('--captions', required=True,
                    help='file of "seconds<TAB>text" lines, in capture time')
    args = ap.parse_args()

    caps = []
    with open(args.captions) as fh:
        for line in fh:
            line = line.rstrip('\n')
            if not line.strip() or line.startswith('#'):
                continue
            t, text = line.split('\t', 1)
            caps.append((float(t), text))
    caps.sort()

    # Which half is which is a property of the desktop, not of the capture:
    # on this machine Gazebo is on the second monitor and RViz on the first, so
    # the right half of the grab is Gazebo. Swapping them here puts Gazebo on
    # the left of the finished frame, where the labels say it is.
    if args.precomposed:
        # The capture already grabbed the two windows separately and stacked
        # them, which is how the desktop is kept out of frame: grabbing the
        # whole screen and cropping afterwards films whatever else is open.
        filters = f'[0:v]setpts=PTS/{args.speed}[fast];[fast]'
    else:
        chain = [
            f'crop=1920:1080:{args.gazebo_x}:0,scale=960:1080[gz]',
            f'crop=1920:1080:{args.rviz_x}:0,scale=960:1080[rv]',
        ]
        filters = (f'[0:v]{chain[0]};[0:v]{chain[1]};[gz][rv]hstack=inputs=2[stacked];'
                   f'[stacked]setpts=PTS/{args.speed}[fast];[fast]')

    overlays = [
        drawtext('GAZEBO — dynamic logistics warehouse, 42 x 63 m',
                 MONO, 22, 24, 24, alpha='0.70'),
        drawtext('RVIZ — Open-RMF traffic schedule',
                 MONO, 22, 984, 24, alpha='0.70'),
    ]
    for i, (t, text) in enumerate(caps):
        start = round(t / args.speed, 2)
        end = round((caps[i + 1][0] if i + 1 < len(caps) else args.trim)
                    / args.speed, 2)
        overlays.append(drawtext(text, FONT, 28, '(w-text_w)/2', 'h-92',
                                 start, end))
    overlays.append(drawtext(
        'three TurtleBot3 Waffles under Open-RMF  ·  epistemic policy by ePlanSys',
        FONT, 19, '(w-text_w)/2', 'h-40', alpha='0.55'))

    graph = filters + ','.join(overlays) + '[v]'

    with tempfile.NamedTemporaryFile('w', suffix='.txt', delete=False) as fh:
        fh.write(graph)
        script = fh.name

    cmd = ['ffmpeg', '-y', '-hide_banner', '-loglevel', 'error',
           '-t', str(args.trim), '-i', args.raw,
           '-filter_complex_script', script, '-map', '[v]',
           '-c:v', 'libx264', '-preset', 'slow', '-crf', '23',
           '-pix_fmt', 'yuv420p', '-movflags', '+faststart', args.out]
    subprocess.run(cmd, check=True)
    os.unlink(script)

    dur = subprocess.run(
        ['ffprobe', '-v', 'error', '-show_entries', 'format=duration',
         '-of', 'csv=p=0', args.out],
        capture_output=True, text=True).stdout.strip()
    print(f'{args.out}: {float(dur):.1f}s, {len(caps)} captions, {args.speed}x')


if __name__ == '__main__':
    main()
