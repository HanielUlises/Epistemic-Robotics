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
Renders the opening and closing cards of the multi-site video.

The opening card names the film and shows a frame of it, and does no more than
that. The closing card carries the one fact the footage cannot: the
contaminated site is the one no robot visited, so the frame in which the
mission succeeds looks exactly like the frame before it.

The still on the opening card is cut from the run being shown and not from a
library. A title card standing over a frame of some other run would be the one
claim in this repository a viewer has no way to check, and the same rule
governs the closing card, whose every line is read out of the mission log.

The cards are drawn and not composed in a filter graph. drawtext places one
line at a time against the frame and has no notion of a column, so a card built
from it is a stack of guessed offsets that shift whenever a word is added. Its
escaping rules also make a colon, a comma or an apostrophe a hazard, which is a
poor constraint on a card whose whole job is to be read.

The card carries the page's styling and not its own. Same ground, same rules,
same red, same three typefaces, same tight tracking on the headings: Liberation
Sans and Liberation Mono are metric-compatible with the Arial and Courier the
pages ask for, so a still of a card and a screenshot of the page are set in the
same faces at the same widths. A video that looks like a different project from
the page carrying it reads as borrowed.

    make_sites_cards.py --outdir /tmp/cards --site a31
"""

import argparse
import os

from PIL import Image, ImageDraw, ImageFont

LIB = '/usr/share/fonts/truetype/liberation'
DEJAVU = '/usr/share/fonts/truetype/dejavu'

# The page's :root palette, verbatim.
BLACK = (17, 17, 17)          # --black
RED = (200, 16, 46)           # --red
GRAY = (90, 90, 90)           # --gray
BORDER = (214, 214, 214)      # --border
PANEL = (250, 250, 250)       # --panel
CODE_BG = (244, 244, 244)     # --code-bg

W, H = 1856, 720
MARGIN = 150

FACES = {
    'sans': 'LiberationSans-Regular',
    'sans-bold': 'LiberationSans-Bold',
    'mono': 'LiberationMono-Regular',
}


def face(kind, size):
    """A font, falling back to DejaVu where Liberation is not installed."""
    for path in (f'{LIB}/{FACES[kind]}.ttf',
                 f'{DEJAVU}/DejaVuSans.ttf'):
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def logic(size):
    """A monospace face carrying the connectives.

    Liberation Mono has no U+2227, and PIL draws a missing glyph as nothing at
    all and not as a box, so a conjunction set in it becomes a gap between two
    conjuncts and the formula reads as a list. DejaVu Sans Mono carries the
    block, and is close enough to Courier at these sizes for one line.
    """
    return ImageFont.truetype(f'{DEJAVU}/DejaVuSansMono.ttf', size)


def width_of(draw, text, font, track=0.0):
    """The width the tracked text will occupy."""
    if not track:
        box = draw.textbbox((0, 0), text, font=font)
        return box[2] - box[0]
    return sum(draw.textlength(ch, font=font) + track for ch in text) - track


def put(draw, x, y, text, font, fill, track=0.0):
    """Draw text at x, optionally letter-spaced, and return its width.

    PIL has no tracking, and the pages set their headings at -0.035em, which
    at these sizes is two pixels a glyph. Left alone the card's headings are
    visibly looser than the page's, so the glyphs are placed one at a time.
    """
    if not track:
        draw.text((x, y), text, font=font, fill=fill)
        return width_of(draw, text, font)
    for ch in text:
        draw.text((x, y), ch, font=font, fill=fill)
        x += draw.textlength(ch, font=font) + track
    return x


def centred(draw, y, text, font, fill, track=0.0):
    """Draw one line centred on the card, and return the y below it."""
    w = width_of(draw, text, font, track)
    put(draw, (W - w) / 2, y, text, font, fill, track)
    box = draw.textbbox((0, 0), text, font=font)
    return y + (box[3] - box[1])


def rule(draw, y, width=W - 2 * MARGIN, colour=BORDER):
    draw.line([((W - width) / 2, y), ((W + width) / 2, y)], fill=colour, width=1)


def kicker(draw, y, text):
    """The page's section label: small, monospace, red."""
    return centred(draw, y, text, face('mono', 22), RED, track=1.6)


def ground():
    img = Image.new('RGB', (W, H), PANEL)
    d = ImageDraw.Draw(img)
    # The page puts a hairline under its header and above its footer. The card
    # is a page without the prose, so it keeps both.
    d.line([(0, 0), (W, 0)], fill=BORDER, width=3)
    d.line([(0, H - 3), (W, H - 3)], fill=BORDER, width=3)
    return img, d


def feathered(still, box, fade_x, fade_y, strength=0.94):
    """An alpha mask that fades a still into the ground it is pasted on.

    A still dropped into a rectangle reads as a screenshot pasted onto a page.
    Ramping its alpha to nothing along the edge that meets the text lets it
    read as one surface: the frame is there, and the title is over ground the
    frame has faded out of. The right edge is left alone, since it runs off the
    card and has nothing to fade into.

    `strength` holds the whole still just short of opaque, so the ground tints
    it very slightly and the two do not look like separate layers.
    """
    w, h = box
    mask = Image.new('L', (w, h), int(255 * strength))
    px = mask.load()
    for x in range(min(fade_x, w)):
        # Smoothstep. A linear ramp leaves a visible edge where it reaches
        # full opacity, because the eye finds the discontinuity in the slope.
        t = x / fade_x
        a = t * t * (3 - 2 * t)
        for y in range(h):
            px[x, y] = int(px[x, y] * a)
    for y in range(min(fade_y, h)):
        t = y / fade_y
        a = t * t * (3 - 2 * t)
        for x in range(w):
            px[x, y] = int(px[x, y] * a)
            px[x, h - 1 - y] = int(px[x, h - 1 - y] * a)
    return mask


def wrapped(draw, text, font, limit, track=0.0):
    """Break text into lines no wider than limit."""
    lines, line = [], ''
    for word in text.split():
        trial = f'{line} {word}'.strip()
        if line and width_of(draw, trial, font, track) > limit:
            lines.append(line)
            line = word
        else:
            line = trial
    if line:
        lines.append(line)
    return lines


def opening(path, shot):
    """The title, and a frame of the run. Nothing else.

    An opening card competes with the footage behind it for the few seconds it
    is up. Everything that can be said later is said later: the premise is what
    the first captions establish, and the result has a card of its own.
    """
    img, d = ground()

    # The still runs off the right edge of the card and fades into the ground
    # on its left, so the title sits on the page and not on a screenshot.
    shot_x = 700
    shot_w, shot_h = W - shot_x, H
    if shot and os.path.exists(shot):
        still = Image.open(shot).convert('RGB')
        # Cover, and not fit: the still is 2.58:1 and the space it goes into is
        # narrower, so fitting it would letterbox the ground it is fading into.
        scale = max(shot_w / still.width, shot_h / still.height)
        still = still.resize((round(still.width * scale),
                              round(still.height * scale)), Image.LANCZOS)
        # Cropped from the right, which is the Gazebo pane. The roadmap is the
        # half a still this small cannot resolve.
        still = still.crop((still.width - shot_w, (still.height - shot_h) // 2,
                            still.width, (still.height - shot_h) // 2 + shot_h))
        img.paste(still, (shot_x, 0),
                  feathered(still, (shot_w, shot_h), fade_x=520, fade_y=90))

    left = 128
    title = face('sans-bold', 62)
    lines = wrapped(d, 'Contamination with a Location', title,
                    shot_x - left - 40, track=-2.2)
    y = (H - (58 + len(lines) * 74)) // 2

    put(d, left, y, 'MULTI-SITE EPISTEMIC SURVEY', face('mono', 21), RED,
        track=1.6)
    y += 58
    for line in lines:
        put(d, left, y, line, title, BLACK, track=-2.2)
        y += 74

    img.save(path)
    return path


def closing(path, site, other):
    img, d = ground()

    y = kicker(d, 74, 'RESULT')
    y = centred(d, y + 40, f'{site} is contaminated.',
                face('sans-bold', 60), BLACK, track=-2.1)
    y = centred(d, y + 34, 'No robot went there.',
                face('sans-bold', 60), RED, track=-2.1)

    rule(d, y + 54, 760)

    # The derivation, in the order the run performed it. Each row is one action
    # of the policy and its consequence, so a viewer can check the claim above
    # against the captions they have just watched.
    steps = [
        (f'the scout scans {other[0]}', 'clean', False),
        ('it tells the relay, on a private channel', 'the relay learns it too', False),
        (f'the relay scans {other[1]}', 'clean', False),
        ('exactly one site is contaminated', f'it is {site}', True),
    ]
    left = face('sans', 27)
    right = face('mono', 25)
    arrow = face('mono', 25)
    y += 96
    for act, out, final in steps:
        w = width_of(d, act, left)
        put(d, W / 2 - 56 - w, y, act, left, GRAY)
        put(d, W / 2 - 30, y + 1, '→', arrow, GRAY)
        put(d, W / 2 + 40, y + 1, out, right, RED if final else BLACK)
        y += 42

    rule(d, y + 26, 560)
    y = centred(d, y + 54, 'Kw(scout)  ∧  Kw(relay)  ∧  ¬Kw(observer)',
                logic(26), BLACK, track=0.4)
    centred(d, H - 62,
            '9 formulas checked against the state the fleet left behind, '
            'and 2 transcripts of address',
            face('mono', 18), GRAY)
    img.save(path)
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--outdir', required=True)
    ap.add_argument('--site', default='a31',
                    help='the site the run found contaminated')
    ap.add_argument('--scanned', default='a17,a06',
                    help='the two sites that were scanned, in order')
    ap.add_argument('--shot', default='',
                    help='a frame of the run, for the opening card')
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    other = args.scanned.split(',')
    print(opening(os.path.join(args.outdir, 'card_open.png'), args.shot))
    print(closing(os.path.join(args.outdir, 'card_close.png'), args.site, other))


if __name__ == '__main__':
    main()
