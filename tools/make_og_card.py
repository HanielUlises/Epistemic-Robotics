#!/usr/bin/env python3
"""
Draws a link-preview card in the layout the site's other cards already use.

The cards under media/og are what a link to this site unfurls into on Slack,
on Discord, on a timeline. Most of them share one layout, and two do not:
warehouse_xl.jpg and warehouse_xl_sites.jpg are raw screenshots, of a
simulator window and of a policy diagram, and both were readable at the size
they were captured and are not at the size a preview is shown.

This draws the shared layout so the two can join it. Measured off
media/og/nested_run.jpg, which is the cleanest instance of it: a red bar across
the top, the domain group as a red monospace kicker, the title, a short red
rule, the description in two or three lines, the project line at the foot
behind a red diamond, a badge at bottom right naming the frame the domain
reaches, and a still bleeding off the right edge and fading out westward.

    make_og_card.py --out media/og/warehouse_xl_sites.jpg \
        --kicker robot-warehouse \
        --title 'Knowing which, and knowing about a place nobody went' \
        --blurb 'Three survey sites, two robots that look, and a team that
                 ends up knowing the state of the one nobody went to.' \
        --badge 'S5n -> KD45' --shot /tmp/shot.png
"""

import argparse
import os

from PIL import Image, ImageDraw, ImageFont

LIB = '/usr/share/fonts/truetype/liberation'
DEJAVU = '/usr/share/fonts/truetype/dejavu'

# The pages' :root palette.
BLACK = (17, 17, 17)
RED = (200, 16, 46)
GRAY = (90, 90, 90)
BORDER = (214, 214, 214)
WHITE = (255, 255, 255)

W, H = 1200, 630
LEFT = 74
TOPBAR = 10


def face(name, size):
    for path in (f'{LIB}/{name}.ttf', f'{DEJAVU}/DejaVuSans.ttf'):
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def width_of(draw, text, font, track=0.0):
    if not track:
        return draw.textbbox((0, 0), text, font=font)[2]
    return sum(draw.textlength(c, font=font) + track for c in text) - track


def put(draw, x, y, text, font, fill, track=0.0):
    """Draw text, optionally letter-spaced. PIL has no tracking of its own."""
    if not track:
        draw.text((x, y), text, font=font, fill=fill)
        return
    for ch in text:
        draw.text((x, y), ch, font=font, fill=fill)
        x += draw.textlength(ch, font=font) + track


def wrapped(draw, text, font, limit, track=0.0):
    lines, line = [], ''
    for word in text.split():
        trial = f'{line} {word}'.strip()
        if line and width_of(draw, trial, font, track) > limit:
            lines.append(line); line = word
        else:
            line = trial
    if line:
        lines.append(line)
    return lines


def feathered(size, fade_x, strength=0.5):
    """Alpha that ramps to nothing on the left edge of the still.

    Smoothstep and not a straight line: a linear ramp reaches full opacity at
    a point the eye can find, because the slope changes discontinuously there.
    """
    w, h = size
    mask = Image.new('L', (w, h), int(255 * strength))
    px = mask.load()
    for x in range(min(fade_x, w)):
        t = x / fade_x
        a = t * t * (3 - 2 * t)
        for y in range(h):
            px[x, y] = int(px[x, y] * a)
    return mask


def card(args):
    img = Image.new('RGB', (W, H), WHITE)
    d = ImageDraw.Draw(img)

    # The still first, so every glyph is drawn over it.
    if args.shot and os.path.exists(args.shot):
        shot_x = args.shot_x
        box = (W - shot_x, H - TOPBAR)
        still = Image.open(args.shot).convert('RGB')
        scale = max(box[0] / still.width, box[1] / still.height)
        still = still.resize((round(still.width * scale),
                              round(still.height * scale)), Image.LANCZOS)
        still = still.crop((still.width - box[0],
                            (still.height - box[1]) // 2,
                            still.width,
                            (still.height - box[1]) // 2 + box[1]))
        img.paste(still, (shot_x, TOPBAR), feathered(box, args.fade))

    d.rectangle([0, 0, W, TOPBAR - 1], fill=RED)

    put(d, LEFT, 158, args.kicker, face('LiberationMono-Regular', 21), RED,
        track=3.2)

    title = face('LiberationSans-Bold', args.title_size)
    y = 204
    for line in wrapped(d, args.title, title, args.text_width, track=-1.8):
        put(d, LEFT, y, line, title, BLACK, track=-1.8)
        y += args.title_size + 14

    d.rectangle([LEFT, y + 16, LEFT + 75, y + 20], fill=RED)

    blurb = face('LiberationSans-Regular', 27)
    y += 54
    for line in wrapped(d, args.blurb, blurb, args.text_width - 90):
        d.text((LEFT, y), line, font=blurb, fill=GRAY)
        y += 36

    # The foot: a red diamond, the project, and the frame the domain reaches.
    foot = face('LiberationSans-Regular', 23)
    d.polygon([(LEFT + 6, 559), (LEFT + 14, 567), (LEFT + 6, 575),
               (LEFT - 2, 567)], fill=RED)
    d.text((LEFT + 26, 556), 'The Epistemic Robotics Project', font=foot,
           fill=GRAY)

    if args.badge:
        # DejaVu for this one line. Liberation Sans has no subscript n and no
        # black diamond, and PIL draws a glyph it does not have as nothing at
        # all, so a badge reading "S5n to KD45" would silently come out "S5 to
        # KD45" with a gap in it.
        bf = ImageFont.truetype(f'{DEJAVU}/DejaVuSans.ttf', 24)
        bw = width_of(d, args.badge, bf) + 32
        bx = W - 78 - bw
        d.rectangle([bx, 550, bx + bw, 586], fill=WHITE, outline=BORDER)
        d.text((bx + 16, 555), args.badge, font=bf, fill=GRAY)

    img.save(args.out, quality=92)
    print(f'{args.out}: {W}x{H}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--kicker', required=True)
    ap.add_argument('--title', required=True)
    ap.add_argument('--blurb', default='')
    ap.add_argument('--badge', default='')
    ap.add_argument('--shot', default='')
    ap.add_argument('--shot-x', type=int, default=560,
                    help='left edge of the still')
    ap.add_argument('--fade', type=int, default=540,
                    help='px over which the still fades in from the left')
    ap.add_argument('--title-size', type=int, default=62)
    ap.add_argument('--text-width', type=int, default=760)
    card(ap.parse_args())


if __name__ == '__main__':
    main()
