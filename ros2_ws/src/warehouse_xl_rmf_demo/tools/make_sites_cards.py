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
the premise is one sentence -- exactly one of three named sites is
contaminated, and nobody knows which. The second is the result, which is the
one fact the film cannot show: the contaminated site is the one no robot
visited, so the frame in which the mission succeeds looks exactly like the
frame before it.

The cards are drawn rather than composed in a filter graph. drawtext places
one line at a time against the frame and has no notion of a column, so a card
built from it is a stack of guessed offsets that shift whenever a word is
added. And the escaping rules make a colon, a comma or an apostrophe a hazard,
which is a poor constraint on a card whose whole job is to be read.

Latin Modern is used because the report is set in it. The video and the paper
are the same result and should not look like two projects.

    make_sites_cards.py --outdir /tmp/cards --site a31
"""

import argparse
import os

from PIL import Image, ImageDraw, ImageFont

LM = '/usr/share/texmf/fonts/opentype/public/lm'
DEJAVU = '/usr/share/fonts/truetype/dejavu'

# The report's palette, and for the same reason as the typeface.
INK = (26, 30, 36)
PAPER = (238, 240, 243)
SLATE = (146, 156, 168)
RULE = (72, 80, 90)
SIGNAL = (196, 74, 84)
STEEL = (138, 168, 204)

W, H = 1856, 720


def face(name, size):
    """A font, falling back to DejaVu where Latin Modern is not installed."""
    for path in (f'{LM}/{name}.otf',
                 f'{DEJAVU}/DejaVuSerif.ttf',
                 f'{DEJAVU}/DejaVuSans.ttf'):
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def logic(size):
    """A monospace face that has the connectives.

    Latin Modern Mono is missing U+2227, and PIL draws a missing glyph as
    nothing at all rather than as a box, so a conjunction set in it becomes a
    gap between two conjuncts and the formula reads as a list. DejaVu Sans
    Mono carries the whole block.
    """
    return ImageFont.truetype(f'{DEJAVU}/DejaVuSansMono.ttf', size)


def centred(draw, y, text, font, fill):
    """Draw one line centred on the card, and return the baseline below it."""
    box = draw.textbbox((0, 0), text, font=font)
    draw.text(((W - (box[2] - box[0])) / 2 - box[0], y), text,
              font=font, fill=fill)
    return y + (box[3] - box[1])


def rule(draw, y, width=520, colour=RULE):
    draw.line([((W - width) / 2, y), ((W + width) / 2, y)], fill=colour,
              width=2)


def opening(path, policy):
    img = Image.new('RGB', (W, H), INK)
    d = ImageDraw.Draw(img)

    rule(d, 150, 900)
    y = centred(d, 186, 'CONTAMINATION WITH A LOCATION',
                face('lmroman10-bold', 62), PAPER)
    y = centred(d, y + 46, 'A three-site epistemic survey executed by two robots',
                face('lmroman10-italic', 32), SLATE)
    y = centred(d, y + 26, 'under Open-RMF',
                face('lmroman10-italic', 32), SLATE)
    rule(d, y + 62, 900)

    y = centred(d, y + 104,
                'Exactly one of  a17   a31   a06  is contaminated.',
                face('lmroman10-regular', 36), PAPER)
    y = centred(d, y + 30, 'Which one is unknown to every agent.',
                face('lmroman10-regular', 36), SIGNAL)
    # The planner runs during bringup, before the screen capture opens, so
    # this fact has no frame of its own to be captioned on.
    if policy:
        centred(d, y + 44, policy, face('lmmono10-regular', 24), SLATE)

    centred(d, H - 92,
            'ePlanSys  ·  PlanSys2  ·  Open-RMF  ·  Gazebo Classic',
            face('lmmono10-regular', 24), STEEL)
    centred(d, H - 56,
            'recorded on a virtual display  ·  captions timed from the mission log',
            face('lmmono10-regular', 20), RULE)
    img.save(path)
    return path


def closing(path, site, other):
    img = Image.new('RGB', (W, H), INK)
    d = ImageDraw.Draw(img)

    rule(d, 96, 900)
    y = centred(d, 124, f'{site} is contaminated.',
                face('lmroman10-bold', 58), PAPER)
    y = centred(d, y + 40, 'No robot went there.',
                face('lmroman10-bold', 58), SIGNAL)
    rule(d, y + 62, 900)

    # The derivation, in the order the run performed it. Each line is one
    # action of the policy and its consequence, so a viewer can check the
    # claim above against the captions they have just watched.
    steps = [
        (f'the scout scans {other[0]}', 'clean'),
        ('it tells the relay, on a private channel', 'the relay learns it too'),
        (f'the relay scans {other[1]}', 'clean'),
        ('exactly one site is contaminated', f'it is {site}'),
    ]
    mono = face('lmmono10-regular', 27)
    left = face('lmroman10-regular', 29)
    y += 118
    for act, out in steps:
        box = d.textbbox((0, 0), act, font=left)
        d.text((W / 2 - 40 - (box[2] - box[0]), y), act, font=left, fill=SLATE)
        d.text((W / 2 - 18, y + 1), '→', font=logic(30), fill=STEEL)
        d.text((W / 2 + 40, y), out, font=mono,
               fill=SIGNAL if out.startswith('it is') else PAPER)
        y += 44

    rule(d, y + 26, 640)
    y = centred(d, y + 56,
                'Kw(scout)  ∧  Kw(relay)  ∧  ¬Kw(observer)',
                logic(30), STEEL)
    centred(d, H - 72,
            '9 formulas checked against the state the fleet left behind, '
            'and 2 transcripts of address',
            face('lmmono10-regular', 20), RULE)
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
