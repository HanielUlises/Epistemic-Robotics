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
Composes a recorded pass-through run into the finished video.

    make_video.py --raw raw.mkv --log run.log --policy raw_policy.json --out film.mp4

The frame is 1920 x 1080. Gazebo is the left pane and RViz the right, each
cut to its 3D canvas and set at 960 x 660; the 420 pixels under them carry,
from left to right, the policy with the node being executed marked, the step
in formal notation, and the knowledge table.

The panel is drawn and composited frame by frame instead of burned in with
drawtext. drawtext has no subscript, no column and no notion of a table, and
its escaping turns a comma or a colon into a hazard; a caption whose whole
content is notation cannot be set in it.

The playback speed follows the timeline's segments, and the current speed is
shown in the corner of the Gazebo pane whenever it is not real time, so the
transits being sampled is never mistaken for the robots being fast.

An opening card states the premise and a closing card the result. The closing
card's figures are the run's: which bays were read as what, how many cells
each exchange carried, how long the route was.
"""

import argparse
import json
import os
import subprocess
import sys

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_captions  # noqa: E402

DEJAVU = '/usr/share/fonts/truetype/dejavu'
LIB = '/usr/share/fonts/truetype/liberation'

BLACK = (17, 17, 17)
RED = (200, 16, 46)
GRAY = (110, 110, 110)
LIGHT = (175, 175, 175)
BORDER = (214, 214, 214)
PANEL = (250, 250, 250)
OPEN = (26, 140, 70)
SHUT = (200, 40, 30)
AGENT = {'west': (51, 115, 204), 'east': (230, 128, 38), 'carrier': (46, 153, 89)}

W, H = 1920, 1080
PANE_W, PANE_H = 960, 660
PANEL_H = H - PANE_H


def font(name, size):
    for path in (f'{DEJAVU}/{name}.ttf', f'{LIB}/{name}.ttf'):
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    return ImageFont.truetype(f'{DEJAVU}/DejaVuSans.ttf', size)


class Faces(dict):
    """Font by size, loaded on first use."""

    def __init__(self, name):
        super().__init__()
        self.name = name

    def __missing__(self, size):
        self[size] = font(self.name, size)
        return self[size]


SANS = Faces('DejaVuSans')
BOLD = Faces('DejaVuSans-Bold')
MONO = Faces('DejaVuSansMono')
MONOB = Faces('DejaVuSansMono-Bold')


# ─── Notation ───────────────────────────────────────────────────────────────

def runs(text):
    """Split `a_{b}c` into [(a, False), (b, True), (c, False)]."""
    out, i, buf = [], 0, ''
    while i < len(text):
        if text.startswith('_{', i):
            end = text.find('}', i)
            if end < 0:
                break
            if buf:
                out.append((buf, False))
                buf = ''
            out.append((text[i + 2:end], True))
            i = end + 1
        else:
            buf += text[i]
            i += 1
    if buf:
        out.append((buf, False))
    return out


def formula_width(draw, text, size, bold=False):
    base = (BOLD if bold else SANS)[size]
    small = SANS[max(12, int(size * 0.68))]
    return sum(draw.textlength(t, font=small if sub else base) for t, sub in runs(text))


def formula(draw, x, y, text, size, fill, bold=False):
    """Draw text with subscripts; return the x after it."""
    base = (BOLD if bold else SANS)[size]
    small = SANS[max(12, int(size * 0.68))]
    for t, sub in runs(text):
        if sub:
            draw.text((x, y + int(size * 0.42)), t, font=small, fill=fill)
            x += draw.textlength(t, font=small)
        else:
            draw.text((x, y), t, font=base, fill=fill)
            x += draw.textlength(t, font=base)
    return x


def wrap_formula(draw, text, size, width):
    """Break a line at spaces so it fits; subscripts are never split."""
    words = text.split(' ')
    lines, cur = [], ''
    for w in words:
        trial = (cur + ' ' + w) if cur else w
        if formula_width(draw, trial, size) <= width or not cur:
            cur = trial
        else:
            lines.append(cur)
            cur = w
    if cur:
        lines.append(cur)
    return lines


def pretty(action):
    """survey_east_t3 -> survey(east, t3)."""
    parts = action.split('_')
    return f'{parts[0]}({", ".join(parts[1:])})'


# ─── The panel ──────────────────────────────────────────────────────────────

def policy_lines(policy):
    """The policy as indented lines: (depth, text, node action, outcome label)."""
    items = policy['items']
    out = []

    def walk(i, depth, label):
        item = items[i]
        action = item['epistemic_action']
        out.append((depth, label, action))
        kids = item['children']
        for k, child in enumerate(kids):
            if child < 0 or child >= len(items):
                continue
            outcome = item['outcomes'][k] if k < len(item['outcomes']) else ''
            walk(child, depth + (1 if len(kids) > 1 else 0), outcome if len(kids) > 1 else '')
    walk(0, 0, '')
    return out


def draw_policy(draw, x0, y0, width, lines, current, done, stats=''):
    draw.text((x0, y0), 'POLICY  π' + (f'  ·  {stats}' if stats else ''), font=MONOB[15], fill=RED)
    y = y0 + 30
    for depth, label, action in lines:
        indent = x0 + 18 * depth
        text = (f'{label} → ' if label else '') + pretty(action)
        if action == current:
            fill, fnt = RED, MONOB[15]
            draw.text((indent - 16, y), '▶', font=MONO[15], fill=RED)
        elif action in done:
            fill, fnt = BLACK, MONO[15]
        else:
            fill, fnt = LIGHT, MONO[15]
        draw.text((indent, y), text, font=fnt, fill=fill)
        y += 24


def draw_table(draw, x0, y0, width, table):
    draw.text((x0, y0), 'WHAT EACH AGENT KNOWS', font=MONOB[15], fill=RED)
    formula(draw, x0, y0 + 30,
            f'M:  {table.get("worlds", "?")} worlds,  {table.get("designated", "?")} designated',
            19, BLACK)
    bays = table.get('bays', ['t1', 't2', 't3'])
    col0, colw = x0 + 150, 96
    y = y0 + 74
    for k, t in enumerate(bays):
        formula(draw, col0 + k * colw + 20, y, f't_{{{t[1:]}}}', 21, BLACK, bold=True)
    y += 36
    draw.line([(x0, y - 6), (x0 + width - 10, y - 6)], fill=BORDER, width=1)
    rows = table.get('rows', {})
    order = ['west', 'east', 'carrier']
    groups = [k for k in rows if k.startswith('D{')] or ['D{west,east}']
    for who in order + groups:
        if who.startswith('D{'):
            formula(draw, x0, y, f'D_{{{who[2:-1]}}}', 21, BLACK)
        else:
            x = formula(draw, x0, y, f'K_{{{who}}}', 21, AGENT.get(who, BLACK), bold=True)
        cells = rows.get(who, {})
        for k, t in enumerate(bays):
            s = cells.get(t, '?')
            fill = OPEN if s == 'open' else SHUT if s == 'shut' else LIGHT
            text = 'open' if s == 'open' else '¬open' if s == 'shut' else '?'
            draw.text((col0 + k * colw, y), text, font=BOLD[19] if s != '?' else SANS[19], fill=fill)
        y += 38


def draw_step(draw, x0, y0, width, step):
    draw.text((x0, y0), step.get('label', ''), font=MONOB[15], fill=RED)
    y = y0 + 28
    formula(draw, x0, y, step.get('title', ''), 28, BLACK, bold=True)
    y += 46
    gloss = step.get('gloss', '')
    if gloss:
        for line in wrap_formula(draw, gloss, 19, width):
            formula(draw, x0, y, line, 19, GRAY)
            y += 28
        y += 6
    lines = step.get('lines', [])
    # The most recent lines are kept when there are more than fit.
    budget = (y0 + PANEL_H - 40) - y
    rendered = []
    for line in lines:
        rendered.append(wrap_formula(draw, line, 21, width))
    while rendered and sum(len(r) for r in rendered) * 31 > budget:
        rendered.pop(0)
    for chunk in rendered:
        for k, line in enumerate(chunk):
            if k == 0:
                draw.text((x0 - 18, y + 2), '·', font=SANS[21], fill=RED)
            formula(draw, x0, y, line, 21, BLACK)
            y += 31


def panel(state, policy_rows, done, stats=''):
    img = Image.new('RGB', (W, PANEL_H), PANEL)
    d = ImageDraw.Draw(img)
    d.line([(0, 0), (W, 0)], fill=BORDER, width=2)
    col = [(34, 540), (620, 820), (1480, 420)]
    for x, _ in col[1:]:
        d.line([(x - 28, 24), (x - 28, PANEL_H - 24)], fill=BORDER, width=1)
    draw_policy(d, col[0][0], 22, col[0][1], policy_rows, state.get('node'), done, stats)
    draw_step(d, col[1][0], 22, col[1][1], state['step'])
    draw_table(d, col[2][0], 22, col[2][1], state['table'])
    return img


# ─── Cards ──────────────────────────────────────────────────────────────────

def centred(d, y, text, fnt, fill):
    w = d.textlength(text, font=fnt)
    d.text(((W - w) / 2, y), text, font=fnt, fill=fill)


def centred_formula(d, y, text, size, fill, bold=False):
    w = formula_width(d, text, size, bold)
    formula(d, (W - w) / 2, y, text, size, fill, bold)


def opening_card(still=None):
    img = Image.new('RGB', (W, H), PANEL)
    d = ImageDraw.Draw(img)
    top = 110
    centred(d, top + 150, 'EPISTEMIC ROBOTICS  ·  MULTI-AGENT EPISTEMIC PLANNING OVER SLAM MAPS',
            MONOB[17], RED)
    centred(d, top + 200, 'The pass-through problem', BOLD[56], BLACK)
    centred(d, top + 290, 'A racking block crosses a 30 × 50 m hall. Three pass-through bays, '
                    't1, t2 and t3, are the only ways through;', SANS[24], GRAY)
    centred(d, top + 326, 'exactly one of them is open, and no agent knows which.', SANS[24], GRAY)
    d.line([(560, top + 392), (1360, top + 392)], fill=BORDER, width=1)
    rows = [
        ('agents', 'two scouts, west and east, with 12 m mapping lasers; a carrier with a 3.5 m safety scanner'),
        ('sensing', 'survey(i, t): scout i reads bay t from its own SLAM map, semi-privately'),
        ('announcement', 'share(i, j, t): i\'s knowledge map fused into j\'s, a semi-private announcement'),
        ('ontic', 'cross(carrier, t) requires K_{carrier} open(t)'),
        ('route', 'W = µZ.(dock ∨ (Safe ∧ ◇Z)),   Safe = ⟦free ∨ ⋁_{t} (t ∧ K_{carrier} open(t))⟧'),
    ]
    y = top + 420
    for key, text in rows:
        d.text((430, y + 3), key.upper(), font=MONOB[15], fill=RED)
        formula(d, 610, y, text, 22, BLACK)
        y += 46
    d.line([(560, y + 16), (1360, y + 16)], fill=BORDER, width=1)
    centred(d, y + 40, 'ePlanSys on ROS 2 Humble  ·  plank and Aletheia  ·  Gazebo 11  ·  '
                       'slam_toolbox  ·  recorded on an Xvfb display', SANS[17], GRAY)
    centred(d, y + 72, 'left: Gazebo, the world as it is.   right: RViz, what the agents have '
                       'observed and what they know.', SANS[17], GRAY)
    return img


def closing_card(facts):
    img = Image.new('RGB', (W, H), PANEL)
    d = ImageDraw.Draw(img)
    route = facts.get('route', {})
    bay_ = route.get('bay', 't2')
    top = 90
    centred(d, top + 150, 'RESULT', MONOB[17], RED)
    centred_formula(d, top + 196, f'The carrier crossed {bay_}, which no robot surveyed', 44, BLACK, bold=True)
    y = top + 290
    for s in facts.get('surveys', []):
        centred_formula(
            d, y, f'survey({s["agent"]}, {s["bay"]}):  {s["seen"]} of {s["cells"]} cells observed, '
                  f'{s["occupied"]} occupied  ⇒  {s["event"]}', 24, BLACK)
        y += 40
    for s in facts.get('shares', []):
        centred_formula(
            d, y, f'map to {s["to"]}:  {make_captions.num(s["cells"])} cells newly known, '
                  f'{s["in_bay"]} of them in {s["bay"]}', 24, BLACK)
        y += 40
    y += 14
    d.line([(560, y), (1360, y)], fill=BORDER, width=1)
    y += 26
    dist = facts.get('distributed', {})
    group = ','.join(dist.get('group', ['west', 'east']))
    centred_formula(d, y, f'after the two surveys:   D_{{{group}}} open({bay_})   while   '
                          f'¬K_{{west}} open({bay_})  and  ¬K_{{east}} open({bay_})', 26, BLACK)
    y += 48
    carried = facts.get('elimination', {}).get('cells_carried', '?')
    centred_formula(d, y, f'after the two exchanges:   K_{{carrier}} open({bay_}),  by elimination; '
                          f'the exchanges carried {carried} cells of {bay_}', 26, BLACK)
    y += 48
    if route:
        centred_formula(d, y, f'Safe_{{carrier}} lifted {bay_};  route {route.get("length")} m through '
                              f'{route.get("via")},  {facts.get("crossing_seconds", "?")} s to the dock',
                        26, BLACK)
        y += 48
    if 'collapsed' in facts:
        centred_formula(d, y, 'cross is public:  M collapses to one world,  s ⊨ C_{all} open('
                              + bay_ + ')', 26, BLACK)
        y += 48
    y += 20
    d.line([(560, y), (1360, y)], fill=BORDER, width=1)
    centred(d, y + 26, 'The knowledge precondition held twice, and by two independent roads: the '
                       'executor checked it against the model,', SANS[19], GRAY)
    centred(d, y + 56, 'and the route existed only because the safe set of the fixed point '
                       'contains the bays the carrier knows to be open.', SANS[19], GRAY)
    return img


def compose(panes, state_panel, speed):
    frame = Image.new('RGB', (W, H))
    frame.paste(panes, (0, 0))
    frame.paste(state_panel, (0, PANE_H))
    d = ImageDraw.Draw(frame, 'RGBA')
    for x, text in ((0, 'GAZEBO  ·  the world, loads included'),
                    (PANE_W, 'RVIZ  ·  what each agent has observed, and knows')):
        wtxt = d.textlength(text, font=MONOB[16])
        d.rectangle([x + 12, 12, x + 12 + wtxt + 20, 42], fill=(16, 20, 24, 190))
        d.text((x + 22, 17), text, font=MONOB[16], fill=(255, 255, 255))
    if abs(speed - 1.0) > 1e-3:
        badge = f'×{speed:g}'
        wtxt = d.textlength(badge, font=BOLD[24])
        d.rectangle([PANE_W - wtxt - 36, 12, PANE_W - 12, 48], fill=(200, 16, 46, 210))
        d.text((PANE_W - wtxt - 24, 14), badge, font=BOLD[24], fill=(255, 255, 255))
    d.line([(PANE_W, 0), (PANE_W, PANE_H)], fill=(40, 40, 40), width=2)
    return frame


def preview(args, states, rows, stats, gw, gh, gx, gy):
    rw, rh, rx, ry = (int(v) for v in args.rviz_crop.split(':'))
    tmp = args.out + '.panes.png'
    subprocess.run(
        ['ffmpeg', '-y', '-hide_banner', '-loglevel', 'error', '-ss', str(args.preview), '-i', args.raw,
         '-filter_complex',
         f'[0:v]split[a][b];[a]crop={gw}:{gh}:{gx}:{gy},scale={PANE_W}:{PANE_H}[g];'
         f'[b]crop={rw}:{rh}:{rx}:{ry},scale={PANE_W}:{PANE_H}[r];[g][r]hstack=inputs=2[v]',
         '-map', '[v]', '-frames:v', '1', tmp], check=True)
    si = max(i for i, s in enumerate(states) if s['t'] <= args.preview)
    done = {s['node'] for s in states[:si + 1] if s.get('node')}
    compose(Image.open(tmp).convert('RGB'), panel(states[si], rows, done, stats), 1.0).save(args.out)
    os.unlink(tmp)
    print(args.out)


# ─── Composition ────────────────────────────────────────────────────────────

def out_to_cap(segments):
    """Output time -> capture time, and the speed there, as a function."""
    table = []
    t_out = 0.0
    for s in segments:
        start = s['start']
        end = s['end'] if s['end'] is not None else 1e9
        table.append((t_out, start, end, s['speed']))
        t_out += (end - start) / s['speed'] if end < 1e8 else 0.0

    def f(t):
        for (o, a, b, v) in reversed(table):
            if t >= o:
                return a + (t - o) * v, v
        return 0.0, 1.0
    return f, table


def encoder(path, fps):
    return subprocess.Popen(
        ['ffmpeg', '-y', '-hide_banner', '-loglevel', 'error', '-f', 'rawvideo',
         '-pix_fmt', 'rgb24', '-s', f'{W}x{H}', '-r', str(fps), '-i', '-',
         '-c:v', 'libx264', '-preset', 'medium', '-crf', '20', '-pix_fmt', 'yuv420p',
         '-movflags', '+faststart', path], stdin=subprocess.PIPE)


def card_frames(enc, img, seconds, fps, fade=0.5):
    base = img.convert('RGB')
    black = Image.new('RGB', (W, H), PANEL)
    n = int(seconds * fps)
    for i in range(n):
        t = i / fps
        a = min(1.0, t / fade, (seconds - t) / fade)
        frame = Image.blend(black, base, max(0.0, a)) if a < 1.0 else base
        enc.stdin.write(frame.tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--raw', required=True)
    ap.add_argument('--log', required=True)
    ap.add_argument('--policy', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--t0', type=float, help='capture start; default <raw>.t0')
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--capture-fps', type=float, default=12.0)
    ap.add_argument('--gazebo-crop', default='1367:940:401:68',
                    help='W:H:X:Y of the Gazebo canvas in the capture')
    ap.add_argument('--rviz-crop', default='1406:967:2414:70',
                    help='W:H:X:Y of the RViz canvas in the capture')
    ap.add_argument('--open-card', type=float, default=7.0)
    ap.add_argument('--close-card', type=float, default=9.0)
    ap.add_argument('--panels', help='also write each distinct panel as a PNG here')
    ap.add_argument('--preview', type=float,
                    help='write one composed frame at this capture time to --out, as a PNG')
    args = ap.parse_args()

    t0 = args.t0 if args.t0 is not None else float(open(os.path.splitext(args.raw)[0] + '.t0').read())
    timeline = make_captions.build(args.log, t0)
    with open(args.policy) as fh:
        policy = json.load(fh)
    rows = policy_lines(policy)
    states = timeline['states']
    pol = timeline['facts'].get('policy', {})
    stats = (f'{pol["nodes"]} nodes, {pol["leaves"]} leaves, {pol["seconds"]} s' if pol else '')
    to_cap, table = out_to_cap(timeline['segments'])

    duration = float(subprocess.run(
        ['ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of', 'csv=p=0', args.raw],
        capture_output=True, text=True).stdout.strip())
    # The last segment runs to the end of the capture.
    last_o, last_a, _, last_v = table[-1]
    total_out = last_o + (duration - last_a) / last_v

    gw, gh, gx, gy = (int(v) for v in args.gazebo_crop.split(':'))
    if args.preview is not None:
        preview(args, states, rows, stats, gw, gh, gx, gy)
        return
    rw, rh, rx, ry = (int(v) for v in args.rviz_crop.split(':'))
    decode = subprocess.Popen(
        ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-i', args.raw, '-filter_complex',
         f'[0:v]split[a][b];[a]crop={gw}:{gh}:{gx}:{gy},scale={PANE_W}:{PANE_H}[g];'
         f'[b]crop={rw}:{rh}:{rx}:{ry},scale={PANE_W}:{PANE_H}[r];[g][r]hstack=inputs=2[v]',
         '-map', '[v]', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-'],
        stdout=subprocess.PIPE)
    frame_bytes = 2 * PANE_W * PANE_H * 3

    enc = encoder(args.out, args.fps)
    card_frames(enc, opening_card(), args.open_card, args.fps)

    cache = {}
    done_by_state = []
    seen = set()
    for s in states:
        if s.get('node'):
            seen.add(s['node'])
        done_by_state.append(set(seen))

    current_index, current = -1, None
    n_out = int(total_out * args.fps)
    si = 0
    for k in range(n_out):
        t_out = k / args.fps
        t_cap, speed = to_cap(t_out)
        want = int(t_cap * args.capture_fps)
        while current_index < want:
            buf = decode.stdout.read(frame_bytes)
            if len(buf) < frame_bytes:
                break
            current = buf
            current_index += 1
        if current is None:
            continue
        while si + 1 < len(states) and states[si + 1]['t'] <= t_cap:
            si += 1
        if si not in cache:
            cache.clear()
            cache[si] = panel(states[si], rows, done_by_state[si], stats)
            if args.panels:
                os.makedirs(args.panels, exist_ok=True)
                cache[si].save(os.path.join(args.panels, f'panel_{si:02d}.png'))
        frame = compose(Image.frombytes('RGB', (2 * PANE_W, PANE_H), current), cache[si], speed)
        enc.stdin.write(frame.tobytes())

    decode.stdout.close()
    decode.wait()
    card_frames(enc, closing_card(timeline['facts']), args.close_card, args.fps)
    enc.stdin.close()
    enc.wait()

    dur = subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of',
                          'csv=p=0', args.out], capture_output=True, text=True).stdout.strip()
    print(f'{args.out}: {float(dur):.1f} s, {len(states)} caption states, '
          f'{len(timeline["segments"])} speed segments')


if __name__ == '__main__':
    main()
