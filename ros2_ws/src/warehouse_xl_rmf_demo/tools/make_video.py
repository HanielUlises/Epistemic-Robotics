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

The capture is a 3840x1080 grab of a virtual display carrying Gazebo on the
left and RViz on the right. Each pane is cropped to its own 3D canvas, so the
menus, docks and status bars are dropped and the two views are what remains.
Both crops are taken at 16:9 and scaled to 1280x720, which puts them side by
side without squeezing a landscape viewport into a portrait half-frame.

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
    # A semicolon goes the same way. ffmpeg splits a filter-complex script
    # into filters on semicolons before it unescapes anything, so a caption
    # containing one is read as the start of a filter and the graph fails to
    # build with "No such filter" naming the rest of the sentence.
    out = (text.replace("'", '').replace('"', '')
               .replace(',', ' ').replace(';', ' —'))
    for ch in (':', '[', ']', '\\'):
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
    ap.add_argument('--start', type=float, default=0.0,
                    help='seconds of capture to drop from the front')
    ap.add_argument('--trim', type=float, default=60.0,
                    help='seconds of capture to keep, from --start')
    ap.add_argument('--precomposed', action='store_true',
                    help='the capture is already a stacked 1920x1080 frame')
    ap.add_argument('--gazebo-crop', default='1640:922:265:70',
                    help='W:H:X:Y of the Gazebo 3D canvas in the capture')
    ap.add_argument('--rviz-crop', default='810:1016:2494:30',
                    help='W:H:X:Y of the RViz 3D canvas in the capture')
    ap.add_argument('--pane', default='1280:720',
                    help='W:H the Gazebo pane is scaled to')
    # The roadmap is 38.8 m across and 60.0 m deep, so the RViz canvas holding
    # all of it is taller than it is wide. Giving it the same width as the
    # Gazebo pane would be half a pane of empty floor.
    ap.add_argument('--rviz-pane', default='576:720',
                    help='W:H the RViz pane is scaled to')
    ap.add_argument('--captions', required=True,
                    help='file of "seconds<TAB>text" lines, in capture time')
    # Which pane is on the left of the finished frame. The single-site
    # recording puts Gazebo there; the multi-site one puts RViz there, because
    # that mission is about which of three named places a robot was sent to and
    # the roadmap is where that is legible.
    ap.add_argument('--order', default='gazebo,rviz',
                    choices=['gazebo,rviz', 'rviz,gazebo'],
                    help='left pane first')
    ap.add_argument('--gazebo-label',
                    default='GAZEBO — aisle_07 of the dynamic logistics warehouse')
    ap.add_argument('--rviz-label',
                    default='RVIZ — roadmap and traffic schedule')
    ap.add_argument('--fps', type=int, default=30,
                    help='output frame rate; shared with the cards so the '
                         'segments concatenate')
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
    # Captions are timed against the capture. Dropping seconds off the front
    # moves every event that much earlier in the finished video. The last
    # caption before the cut is kept and pinned to zero: the video opens in the
    # middle of a policy node, and the frame should say which one.
    before = [c for c in caps if c[0] < args.start]
    caps = [(t - args.start, text) for t, text in caps if t >= args.start]
    if before:
        caps.insert(0, (0.0, before[-1][1]))

    # The capture is of a virtual display which never carries anything but
    # these two windows, so a full-screen grab cannot pick up the desktop and
    # the panes are cut out of it afterwards.
    if args.precomposed:
        filters = f'[0:v]setpts=PTS/{args.speed}[fast];[fast]'
    else:
        chain = [
            f'crop={args.gazebo_crop},scale={args.pane}[gz]',
            f'crop={args.rviz_crop},scale={args.rviz_pane}[rv]',
        ]
        first, second = args.order.split(',')
        tag = {'gazebo': '[gz]', 'rviz': '[rv]'}
        filters = (f'[0:v]{chain[0]};[0:v]{chain[1]};'
                   f'{tag[first]}{tag[second]}hstack=inputs=2[stacked];'
                   f'[stacked]setpts=PTS/{args.speed}[fast];[fast]')

    # Each label sits over its own pane, so the offsets follow the order rather
    # than assuming it. A label naming the wrong window is worse than none.
    width = {'gazebo': int(args.pane.split(':')[0]),
             'rviz': int(args.rviz_pane.split(':')[0])}
    label = {'gazebo': args.gazebo_label, 'rviz': args.rviz_label}
    if args.precomposed:
        order = ['gazebo', 'rviz']
    else:
        order = args.order.split(',')
    overlays, offset = [], 0
    for pane in order:
        overlays.append(drawtext(label[pane], MONO, 20, offset + 20, 18,
                                 alpha='0.70'))
        offset += width[pane]
    for i, (t, text) in enumerate(caps):
        start = round(t / args.speed, 2)
        end = round((caps[i + 1][0] if i + 1 < len(caps) else args.trim)
                    / args.speed, 2)
        overlays.append(drawtext(text, FONT, 26, '(w-text_w)/2', 'h-84',
                                 start, end))
    overlays.append(drawtext(
        'three TurtleBot3 Waffles under Open-RMF  ·  epistemic policy by ePlanSys',
        FONT, 18, '(w-text_w)/2', 'h-38', alpha='0.55'))

    graph = filters + ','.join(overlays) + '[v]'

    with tempfile.NamedTemporaryFile('w', suffix='.txt', delete=False) as fh:
        fh.write(graph)
        script = fh.name

    # A constant output rate, and not the capture's. Segments are concatenated
    # with the cards, and the concat demuxer joins streams by copying them: a
    # variable-rate segment produced by setpts and a card produced from a still
    # do not share a timebase, and the join then plays the second stream on the
    # first one's clock.
    cmd = ['ffmpeg', '-y', '-hide_banner', '-loglevel', 'error',
           '-ss', str(args.start), '-t', str(args.trim), '-i', args.raw,
           '-filter_complex_script', script, '-map', '[v]',
           '-r', str(args.fps),
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
