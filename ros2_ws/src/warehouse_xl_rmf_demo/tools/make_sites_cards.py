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

Two things are wanted of a card that the caption track cannot give. The first
is the premise: a viewer who does not already know the domain sees two robots
crossing a warehouse and has no reason to care which aisles they stop at, and
the premise is one sentence, that exactly one of three named sites is
contaminated and nobody knows which. The second is the result, which is the one
fact the film cannot show: the contaminated site is the one no robot visited,
so the frame in which the mission succeeds looks exactly like the frame before
it.

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


def opening(path, policy):
    img, d = ground()

    y = kicker(d, 108, 'MULTI-SITE EPISTEMIC SURVEY')
    y = centred(d, y + 44, 'Contamination with a Location',
                face('sans-bold', 66), BLACK, track=-2.3)
    y = centred(d, y + 40,
                'A three-site epistemic survey executed by two robots '
                'under Open-RMF',
                face('sans', 27), GRAY, track=-0.4)

    rule(d, y + 56, 760)

    y = centred(d, y + 96, 'Exactly one of  a17   a31   a06  is contaminated.',
                face('sans', 33), BLACK, track=-0.6)
    y = centred(d, y + 34, 'Which one is unknown to every agent.',
                face('sans', 33), RED, track=-0.6)
    # The planner runs during bringup, before the screen capture opens, so this
    # fact has no frame of its own to be captioned on.
    if policy:
        centred(d, y + 46, policy, face('mono', 22), GRAY)

    centred(d, H - 92, 'ePlanSys  ·  PlanSys2  ·  Open-RMF  ·  Gazebo Classic',
            face('mono', 21), BLACK, track=0.6)
    centred(d, H - 58,
            'recorded on a virtual display  ·  captions timed from the mission log',
            face('mono', 18), GRAY)
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
    ap.add_argument('--policy', default='',
                    help='one line describing the policy the planner returned')
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    other = args.scanned.split(',')
    print(opening(os.path.join(args.outdir, 'card_open.png'), args.policy))
    print(closing(os.path.join(args.outdir, 'card_close.png'), args.site, other))


if __name__ == '__main__':
    main()
