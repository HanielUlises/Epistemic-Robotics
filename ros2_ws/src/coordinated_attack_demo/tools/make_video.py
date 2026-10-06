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
Composes the two recorded floors of the coordinated attack into one film.

    make_video.py --radio  raw_radio.mkv  run_radio.log  raw_radio_policy.json \\
                  --beacon raw_beacon.mkv run_beacon.log raw_beacon_policy.json \\
                  --out coordinated_attack.mp4

The frame is 1920 x 1080. RViz is the left pane and Gazebo the right, each
cut to its 3D canvas and set at 960 x 660. The 420 pixels under them carry,
from left to right: the policy, or on the radio floor the protocol run in its
place, with the node being executed marked; the step in formal notation; and
what each robot knows, with the depth of their mutual knowledge drawn as a
ladder from E^1 to C.

The film is an opening card, the radio floor, the beacon floor and a closing
card. Every figure on the panels and the closing card is a line one of the two
runs wrote; see captions.py.

The panel is drawn and composited frame by frame, as pass_through_demo's is,
because a caption whose content is notation needs subscripts, superscripts
and columns that ffmpeg's drawtext does not have.
"""

import argparse
import json
import os
import subprocess
import sys

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import captions  # noqa: E402

DEJAVU = '/usr/share/fonts/truetype/dejavu'

BLACK = (17, 17, 17)
RED = (200, 16, 46)
GRAY = (110, 110, 110)
LIGHT = (175, 175, 175)
BORDER = (214, 214, 214)
PANEL = (250, 250, 250)
GREEN = (26, 140, 70)
AGENT = {'south': (51, 115, 204), 'north': (230, 128, 38)}

W, H = 1920, 1080
PANE_W, PANE_H = 960, 660
PANEL_H = H - PANE_H


class Faces(dict):

    def __init__(self, name):
        super().__init__()
        self.name = name

    def __missing__(self, size):
        self[size] = ImageFont.truetype(f'{DEJAVU}/{self.name}.ttf', size)
        return self[size]


SANS = Faces('DejaVuSans')
BOLD = Faces('DejaVuSans-Bold')
MONO = Faces('DejaVuSansMono')
MONOB = Faces('DejaVuSansMono-Bold')


# ─── Notation ───────────────────────────────────────────────────────────────

def runs(text):
    """Split `a_{b}c^{d}` into [(a, 0), (b, -1), (c, 0), (d, +1)]."""
    out, i, buf = [], 0, ''
    while i < len(text):
        if text.startswith('_{', i) or text.startswith('^{', i):
            end = text.find('}', i)
            if end < 0:
                break
            if buf:
                out.append((buf, 0))
                buf = ''
            out.append((text[i + 2:end], -1 if text[i] == '_' else 1))
            i = end + 1
        else:
            buf += text[i]
            i += 1
    if buf:
        out.append((buf, 0))
    return out


def formula_width(draw, text, size, bold=False):
    base = (BOLD if bold else SANS)[size]
    small = SANS[max(12, int(size * 0.68))]
    return sum(draw.textlength(t, font=small if level else base) for t, level in runs(text))


def formula(draw, x, y, text, size, fill, bold=False):
    base = (BOLD if bold else SANS)[size]
    small = SANS[max(12, int(size * 0.68))]
    for t, level in runs(text):
        if level:
            dy = int(size * 0.42) if level < 0 else -int(size * 0.12)
            draw.text((x, y + dy), t, font=small, fill=fill)
            x += draw.textlength(t, font=small)
        else:
            draw.text((x, y), t, font=base, fill=fill)
            x += draw.textlength(t, font=base)
    return x


def wrap_formula(draw, text, size, width):
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
    """ack2_north_south_s1 -> ack2(north, south, s1)."""
    parts = action.split('_')
    return f'{parts[0]}({", ".join(parts[1:])})'


# ─── The panel ──────────────────────────────────────────────────────────────

def policy_lines(policy):
    items = policy['items']
    out = []

    def walk(i, depth, label):
        item = items[i]
        out.append((depth, label, item['epistemic_action']))
        kids = item['children']
        for k, child in enumerate(kids):
            if child < 0 or child >= len(items):
                continue
            outcome = item['outcomes'][k] if k < len(item['outcomes']) else ''
            walk(child, depth + (1 if len(kids) > 1 else 0), outcome if len(kids) > 1 else '')
    walk(0, 0, '')
    return out


def draw_policy(draw, x0, y0, lines, current, done, title):
    draw.text((x0, y0), title, font=MONOB[15], fill=RED)
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


def draw_step(draw, x0, y0, width, step):
    draw.text((x0, y0), step.get('label', ''), font=MONOB[15], fill=RED)
    y = y0 + 28
    for line in wrap_formula(draw, step.get('title', ''), 27, width):
        formula(draw, x0, y, line, 27, BLACK, bold=True)
        y += 38
    y += 6
    gloss = step.get('gloss', '')
    if gloss:
        for line in wrap_formula(draw, gloss, 18, width):
            formula(draw, x0, y, line, 18, GRAY)
            y += 26
        y += 6
    budget = (y0 + PANEL_H - 40) - y
    rendered = [wrap_formula(draw, line, 20, width) for line in step.get('lines', [])]
    while rendered and sum(len(r) for r in rendered) * 30 > budget:
        rendered.pop(0)
    for chunk in rendered:
        for k, line in enumerate(chunk):
            if k == 0:
                draw.text((x0 - 18, y + 1), '·', font=SANS[20], fill=RED)
            formula(draw, x0, y, line, 20, BLACK)
            y += 30


def draw_knowledge(draw, x0, y0, width, know):
    draw.text((x0, y0), 'WHAT EACH ROBOT KNOWS', font=MONOB[15], fill=RED)
    formula(draw, x0, y0 + 30,
            f'M:  {know.get("worlds", "?")} worlds,  {know.get("designated", "?")} designated',
            19, BLACK)
    stand = know.get('stand')
    y = y0 + 74
    if not stand:
        formula(draw, x0, y, 'the order has not been read', 19, GRAY)
        return
    atom = f'job(s_{{{stand[1:]}}})'
    for agent in ('south', 'north'):
        formula(draw, x0, y, f'K_{{{agent}}} {atom}', 21, AGENT[agent], bold=True)
        k = know.get('knows', {}).get(agent)
        draw.text((x0 + 250, y), 'yes' if k else 'no', font=BOLD[20] if k else SANS[20],
                  fill=GREEN if k else LIGHT)
        y += 36
    y += 10
    formula(draw, x0, y, f'MUTUAL KNOWLEDGE OF {atom}', 15, RED)
    y += 30
    depth = know.get('depth')
    labels = ['E^{1}', 'E^{2}', 'E^{3}', 'E^{4}', 'E^{5}', 'C']
    box_w, gap = 58, 8
    for i, label in enumerate(labels):
        x = x0 + i * (box_w + gap)
        if label == 'C':
            filled = depth == 'C'
            colour = RED
        else:
            filled = depth == 'C' or (isinstance(depth, int) and depth >= i + 1)
            colour = GREEN
        draw.rectangle([x, y, x + box_w, y + 44], fill=colour if filled else PANEL,
                       outline=colour if filled else BORDER, width=2)
        tw = formula_width(draw, label, 20, bold=True)
        formula(draw, x + (box_w - tw) / 2, y + 9, label, 20, (255, 255, 255) if filled else LIGHT,
                bold=True)
    y += 60
    chain = know.get('chain') or []
    if depth == 'C':
        formula(draw, x0, y, 'no chain of relations reaches ¬' + atom, 17, GRAY)
    elif chain:
        text = ' → '.join(chain) + ' → ¬' + atom
        for line in wrap_formula(draw, 'C blocked by:  ' + text, 17, width - 10):
            formula(draw, x0, y, line, 17, GRAY)
            y += 24


def panel(state, rows, done, title):
    img = Image.new('RGB', (W, PANEL_H), PANEL)
    d = ImageDraw.Draw(img)
    d.line([(0, 0), (W, 0)], fill=BORDER, width=2)
    cols = [(34, 520), (600, 820), (1470, 430)]
    for x, _ in cols[1:]:
        d.line([(x - 28, 24), (x - 28, PANEL_H - 24)], fill=BORDER, width=1)
    draw_policy(d, cols[0][0], 22, rows, state.get('node'), done, title)
    draw_step(d, cols[1][0], 22, cols[1][1], state['step'])
    draw_knowledge(d, cols[2][0], 22, cols[2][1], state['know'])
    return img


# ─── Cards ──────────────────────────────────────────────────────────────────

def centred(d, y, text, fnt, fill):
    w = d.textlength(text, font=fnt)
    d.text(((W - w) / 2, y), text, font=fnt, fill=fill)


def centred_formula(d, y, text, size, fill, bold=False):
    w = formula_width(d, text, size, bold)
    formula(d, (W - w) / 2, y, text, size, fill, bold)


def opening_card():
    img = Image.new('RGB', (W, H), PANEL)
    d = ImageDraw.Draw(img)
    top = 100
    centred(d, top + 120, 'EPISTEMIC ROBOTICS  ·  MULTI-AGENT EPISTEMIC PLANNING', MONOB[17], RED)
    centred(d, top + 170, 'The coordinated attack', BOLD[56], BLACK)
    centred(d, top + 260, 'Two robots must lift one load together, one under each end. A work '
                          'order names which of two stands;', SANS[24], GRAY)
    centred(d, top + 296, 'only south can read it, and the racking block keeps the two out of '
                          'sight of each other.', SANS[24], GRAY)
    d.line([(560, top + 360), (1360, top + 360)], fill=BORDER, width=1)
    rows = [
        ('sensing', 'read-order(south, s): semi-private; north learns only that it was read'),
        ('radio', 'tell, ack, ack2, ack3: a lossy message, delivered, lost or never sent'),
        ('beacon', 'signal(i, s): an announcement seen only from the two mouths of t_{2}'),
        ('ontic', 'lift(s) requires  job(s) ∧ C_{south,north} job(s)'),
        ('why C', 'each raises its end only if it knows the other will: the weakest such '
                  'condition is C'),
    ]
    y = top + 388
    for key, text in rows:
        d.text((400, y + 3), key.upper(), font=MONOB[15], fill=RED)
        formula(d, 560, y, text, 22, BLACK)
        y += 46
    d.line([(560, y + 16), (1360, y + 16)], fill=BORDER, width=1)
    centred(d, y + 40, 'ePlanSys on ROS 2 Humble  ·  plank and Aletheia  ·  Gazebo 11  ·  '
                       'recorded on an Xvfb display', SANS[17], GRAY)
    centred(d, y + 72, 'left: RViz, what the robots know.   right: Gazebo, the world as it is.',
            SANS[17], GRAY)
    return img


def act_card(number, title, lines):
    img = Image.new('RGB', (W, H), PANEL)
    d = ImageDraw.Draw(img)
    centred(d, 330, number, MONOB[20], RED)
    centred(d, 375, title, BOLD[52], BLACK)
    y = 480
    for line in lines:
        centred_formula(d, y, line, 26, GRAY)
        y += 44
    return img


def closing_card(radio, beacon):
    img = Image.new('RGB', (W, H), PANEL)
    d = ImageDraw.Draw(img)
    rf, bf = radio['facts'], beacon['facts']
    s = bf.get('order', 's1')
    atom = f'job(s_{{{s[1:]}}})'
    top = 70
    centred(d, top + 120, 'RESULT', MONOB[17], RED)
    centred_formula(d, top + 166, 'Four delivered messages reach E^{4}. One beacon reaches C.',
                    40, BLACK, bold=True)
    y = top + 260
    d.text((420, y), 'RADIO', font=MONOB[16], fill=RED)
    y += 34
    n = len(rf.get('messages', []))
    lines = [f'the planner returned no policy for  lifted  ({rf.get("no_policy_seconds", "?")} s)',
             f'{n} messages run, {n} delivered:  s ⊨ E^{{{n}}} {atom},   s ⊭ C_{{south,north}} {atom}',
             'the executor refused lift on its precondition; neither robot moved toward a stand']
    for line in lines:
        formula(d, 420, y, line, 24, BLACK)
        y += 40
    y += 16
    d.text((420, y), 'BEACON', font=MONOB[16], fill=RED)
    y += 34
    pol = bf.get('policy', {})
    views = bf.get('views', {})
    lines = [f'policy of {pol.get("nodes", "?")} nodes and {pol.get("leaves", "?")} leaves in '
             f'{pol.get("seconds", "?")} s; it does not use the radio',
             'at the viewpoints, beacon in line of sight:  ' +
             ',  '.join(f'{a} {v["distance"]} m' for a, v in views.items()),
             f'one signal:  s ⊨ C_{{south,north}} {atom}']
    joint = bf.get('joint')
    if joint:
        lines.append(f'joint start under the load:  starts {joint["starts"]} s apart,  '
                     f'arrivals {joint["arrivals"]} s apart')
    raised = bf.get('raised')
    if raised:
        lines.append(f'load_{raised["stand"]} raised {raised["height"]} m,  '
                     f'{raised["seconds"]} s after lift began')
    for line in lines:
        formula(d, 420, y, line, 24, BLACK)
        y += 40
    y += 24
    d.line([(560, y), (1360, y)], fill=BORDER, width=1)
    centred(d, y + 26, 'Delivery that is not known to have happened adds one level of mutual '
                       'knowledge per message and never yields common knowledge.', SANS[19], GRAY)
    centred(d, y + 56, 'An announcement each robot sees, and is known to see, yields it in one '
                       'update (Halpern and Moses, 1990).', SANS[19], GRAY)
    return img


# ─── Composition ────────────────────────────────────────────────────────────

def compose(panes, state_panel, speed):
    frame = Image.new('RGB', (W, H))
    frame.paste(panes, (0, 0))
    frame.paste(state_panel, (0, PANE_H))
    d = ImageDraw.Draw(frame, 'RGBA')
    for x, text in ((0, 'RVIZ  ·  what the robots know'),
                    (PANE_W, 'GAZEBO  ·  the world as it is')):
        wtxt = d.textlength(text, font=MONOB[16])
        d.rectangle([x + 12, 12, x + 12 + wtxt + 20, 42], fill=(16, 20, 24, 190))
        d.text((x + 22, 17), text, font=MONOB[16], fill=(255, 255, 255))
    if abs(speed - 1.0) > 1e-3:
        badge = f'×{speed:g}'
        wtxt = d.textlength(badge, font=BOLD[24])
        d.rectangle([W - wtxt - 36, 12, W - 12, 48], fill=(200, 16, 46, 210))
        d.text((W - wtxt - 24, 14), badge, font=BOLD[24], fill=(255, 255, 255))
    d.line([(PANE_W, 0), (PANE_W, PANE_H)], fill=(40, 40, 40), width=2)
    return frame


def out_to_cap(segments):
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
    blank = Image.new('RGB', (W, H), PANEL)
    n = int(seconds * fps)
    for i in range(n):
        t = i / fps
        a = min(1.0, t / fade, (seconds - t) / fade)
        frame = Image.blend(blank, base, max(0.0, a)) if a < 1.0 else base
        enc.stdin.write(frame.tobytes())


def pane_filter(rviz_crop, gazebo_crop):
    rw, rh, rx, ry = (int(v) for v in rviz_crop.split(':'))
    gw, gh, gx, gy = (int(v) for v in gazebo_crop.split(':'))
    return (f'[0:v]split[a][b];[a]crop={rw}:{rh}:{rx}:{ry},scale={PANE_W}:{PANE_H}[r];'
            f'[b]crop={gw}:{gh}:{gx}:{gy},scale={PANE_W}:{PANE_H}[g];[r][g]hstack=inputs=2[v]')


def load_floor(raw, log, policy_path, floor):
    t0 = float(open(os.path.splitext(raw)[0] + '.t0').read())
    timeline = captions.build(log, t0, floor)
    with open(policy_path) as fh:
        policy = json.load(fh)
    return timeline, policy


def floor_title(timeline, policy):
    if policy.get('planned'):
        pol = timeline['facts'].get('policy', {})
        return f'POLICY  π  ·  {pol.get("nodes", "?")} nodes, {pol.get("leaves", "?")} leaves'
    return 'PROTOCOL  ·  no policy exists; run by hand'


def write_floor(enc, args, raw, timeline, policy, fps):
    rows = policy_lines(policy)
    title = floor_title(timeline, policy)
    states = timeline['states']
    to_cap, table = out_to_cap(timeline['segments'])
    duration = float(subprocess.run(
        ['ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of', 'csv=p=0', raw],
        capture_output=True, text=True).stdout.strip())
    # The capture runs on after the mission ends; a few seconds of the result
    # are kept and the rest is not.
    if 'complete' in timeline['marks']:
        duration = min(duration, timeline['marks']['complete'] + 6.0)
    last_o, last_a, _, last_v = table[-1]
    total_out = last_o + max(0.0, duration - last_a) / last_v

    decode = subprocess.Popen(
        ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-i', raw, '-filter_complex',
         pane_filter(args.rviz_crop, args.gazebo_crop), '-map', '[v]', '-f', 'rawvideo',
         '-pix_fmt', 'rgb24', '-'], stdout=subprocess.PIPE)
    frame_bytes = 2 * PANE_W * PANE_H * 3

    done_by_state, seen = [], set()
    for s in states:
        if s.get('node'):
            seen.add(s['node'])
        done_by_state.append(set(seen))

    cache, current_index, current, si = {}, -1, None, 0
    for k in range(int(total_out * fps)):
        t_cap, speed = to_cap(k / fps)
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
            cache[si] = panel(states[si], rows, done_by_state[si], title)
        frame = compose(Image.frombytes('RGB', (2 * PANE_W, PANE_H), current), cache[si], speed)
        enc.stdin.write(frame.tobytes())
    # A floor trimmed short of its capture stops reading before the decoder is
    # done; it is stopped, not left to report a broken pipe.
    decode.terminate()
    decode.stdout.close()
    decode.wait()


def preview(args, raw, timeline, policy, at, out):
    tmp = out + '.panes.png'
    subprocess.run(
        ['ffmpeg', '-y', '-hide_banner', '-loglevel', 'error', '-ss', str(at), '-i', raw,
         '-filter_complex', pane_filter(args.rviz_crop, args.gazebo_crop), '-map', '[v]',
         '-frames:v', '1', tmp], check=True)
    states = timeline['states']
    si = max(i for i, s in enumerate(states) if s['t'] <= at)
    done = {s['node'] for s in states[:si + 1] if s.get('node')}
    compose(Image.open(tmp).convert('RGB'),
            panel(states[si], policy_lines(policy), done, floor_title(timeline, policy)),
            1.0).save(out)
    os.unlink(tmp)
    print(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--radio', nargs=3, metavar=('RAW', 'LOG', 'POLICY'), required=True)
    ap.add_argument('--beacon', nargs=3, metavar=('RAW', 'LOG', 'POLICY'), required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--capture-fps', type=float, default=12.0)
    ap.add_argument('--rviz-crop', default='1406:967:494:70',
                    help='W:H:X:Y of the RViz canvas in the capture')
    ap.add_argument('--gazebo-crop', default='1367:940:2321:68',
                    help='W:H:X:Y of the Gazebo canvas in the capture')
    ap.add_argument('--preview', nargs=2, metavar=('FLOOR', 'T'),
                    help='write one composed frame of FLOOR at capture time T to --out')
    ap.add_argument('--cards', help='write the four cards as PNGs into this directory')
    args = ap.parse_args()

    radio = load_floor(*args.radio, 'radio')
    beacon = load_floor(*args.beacon, 'beacon')

    if args.preview:
        floor, at = args.preview
        raw = (args.radio if floor == 'radio' else args.beacon)[0]
        timeline, policy = radio if floor == 'radio' else beacon
        preview(args, raw, timeline, policy, float(at), args.out)
        return

    n_msgs = len(radio[0]['facts'].get('messages', []))
    cards = {
        'opening': opening_card(),
        'radio': act_card('I', 'The radio', [
            'No beacon on this floor. The radio can lose a message and nobody would know.',
            'The planner finds no policy. The protocol is run anyway, every message delivered,',
            f'and the executor checks lift against the model after all {n_msgs} of them.']),
        'beacon': act_card('II', 'The beacon', [
            'The same floor with a stack light in t_{2}, seen only from the two mouths of t_{2}.',
            'Where each robot stands is common knowledge, so who sees the light is too.']),
        'closing': closing_card(radio[0], beacon[0]),
    }
    if args.cards:
        os.makedirs(args.cards, exist_ok=True)
        for name, img in cards.items():
            img.save(os.path.join(args.cards, f'card_{name}.png'))

    enc = encoder(args.out, args.fps)
    card_frames(enc, cards['opening'], 9.0, args.fps)
    card_frames(enc, cards['radio'], 6.0, args.fps)
    write_floor(enc, args, args.radio[0], *radio, args.fps)
    card_frames(enc, cards['beacon'], 6.0, args.fps)
    write_floor(enc, args, args.beacon[0], *beacon, args.fps)
    card_frames(enc, cards['closing'], 11.0, args.fps)
    enc.stdin.close()
    enc.wait()

    dur = subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of',
                          'csv=p=0', args.out], capture_output=True, text=True).stdout.strip()
    print(f'{args.out}: {float(dur):.1f} s')


if __name__ == '__main__':
    main()
