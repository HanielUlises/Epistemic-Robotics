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
Emits an animated version of the accessibility figure as an HTML fragment.

The static figure prints one panel per agent per step, which grows with the
run. The animation holds one panel per agent and steps the run through them, so
a world appearing, an arc being cut and a self-loop disappearing are all changes
the reader watches happen.

Worlds are laid out once, using the union of every world the run mentions, and
each keeps its position for the whole animation. The transitions are CSS, in the
manner of the other animated figures on this site.

    animation.py --analysis out/analysis-l3.json --agents inspector porter guest \\
                 --out fragment.html --id hotel
"""

import argparse
import json
import math

R = 15.0
GAP = 3.5


def collect(analysis, agents):
    rows = [('initial', analysis['initial'])] + [
        (s['action'], s['agents']) for s in analysis['steps']]
    worlds, atoms = [], {}
    for _, snapshot in rows:
        g = snapshot[agents[0]]['structure']
        for w in g.get('worlds', []):
            if w not in worlds:
                worlds.append(w)
        for w, labs in g.get('labels', {}).items():
            atoms.setdefault(w, [])
            for a in g.get('varying_atoms', []):
                if a in labs and a not in atoms[w]:
                    atoms[w].append(a)
    return rows, worlds, atoms


def short(atom):
    return (atom.replace('leak-at_', 'leak@').replace('contained_', 'sealed@')
                .replace('pallet-at_', 'pallet@').replace('_suite', ''))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--analysis', required=True)
    parser.add_argument('--agents', nargs='+', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--id', required=True, help='unique prefix on the page')
    args = parser.parse_args()

    analysis = json.load(open(args.analysis))
    rows, worlds, atoms = collect(analysis, args.agents)
    n = len(worlds)

    pw, ph = 250, 232
    width = pw * len(args.agents)
    height = ph + 6

    place = {}
    radius = min(pw - 40, ph - 70) * 0.34
    for i, w in enumerate(worlds):
        a = -math.pi / 2 + 2 * math.pi * i / max(1, n)
        place[w] = (pw / 2 + radius * math.cos(a), ph / 2 + 8 + radius * math.sin(a))

    uid = args.id
    svg = []
    for c, agent in enumerate(args.agents):
        ox = c * pw
        svg.append(f'<g transform="translate({ox},0)">')
        svg.append(f'<rect class="kpanel" x="6" y="6" width="{pw - 12}" '
                   f'height="{ph - 12}" rx="3"/>')
        svg.append(f'<text class="kagent" x="18" y="26">{agent}</text>')
        svg.append(f'<text class="kframe" id="{uid}-frame-{agent}" x="18" y="42"></text>')

        for w in worlds:
            px, py = place[w]
            # Every edge that could ever be drawn is emitted once and shown or
            # hidden per step, so the browser can animate the change.
            for t in worlds:
                if w == t:
                    # Outward from the middle of the panel: every other arc
                    # runs toward another node, so nothing else is drawn on
                    # that bearing.
                    ccx, ccy = pw / 2, ph / 2 + 8
                    out_ang = (math.atan2(py - ccy, px - ccx)
                               if (px, py) != (ccx, ccy) else -math.pi / 2)
                    a0 = out_ang - math.radians(38)
                    a1 = out_ang + math.radians(38)
                    rim = R + 2.0
                    sx, sy = px + rim * math.cos(a0), py + rim * math.sin(a0)
                    ex, ey = px + rim * math.cos(a1), py + rim * math.sin(a1)
                    bulge = R + 22
                    c1x = px + bulge * math.cos(a0 - math.radians(10))
                    c1y = py + bulge * math.sin(a0 - math.radians(10))
                    c2x = px + bulge * math.cos(a1 + math.radians(10))
                    c2y = py + bulge * math.sin(a1 + math.radians(10))
                    svg.append(
                        f'<path class="kedge kloop cut" data-a="{agent}" '
                        f'marker-end="url(#{uid}-head)" '
                        f'data-from="{w}" data-to="{t}" '
                        f'd="M{sx:.1f},{sy:.1f} C{c1x:.1f},{c1y:.1f} '
                        f'{c2x:.1f},{c2y:.1f} {ex:.1f},{ey:.1f}"/>')
                    continue
                bx, by = place[t]
                dx, dy = bx - px, by - py
                length = math.hypot(dx, dy)
                if length <= 2 * (R + GAP):
                    continue
                ux, uy = dx / length, dy / length
                off = R + GAP
                svg.append(
                    f'<path class="kedge cut" data-a="{agent}" '
                    f'marker-end="url(#{uid}-head)" data-from="{w}" '
                    f'data-to="{t}" d="M{px + ux * off:.1f},{py + uy * off:.1f} '
                    f'L{bx - ux * off:.1f},{by - uy * off:.1f}"/>')

        for w in worlds:
            px, py = place[w]
            svg.append(f'<g class="kworld dead" data-a="{agent}" data-w="{w}">')
            svg.append(f'<circle class="kdot" cx="{px:.0f}" cy="{py:.0f}" r="{R:.0f}"/>')
            svg.append(f'<text class="kwid" x="{px:.0f}" y="{py + 4:.0f}" '
                       f'text-anchor="middle">{w}</text>')
            # The atoms true at a world change from step to step, so the text
            # is filled in per frame. Showing the union over the run would put
            # `sealed@l3` under a world before the valve was closed.
            for k in range(2):
                svg.append(f'<text class="katom" id="{uid}-at-{agent}-{w}-{k}" '
                           f'x="{px:.0f}" y="{py + 29 + k * 11:.0f}" '
                           f'text-anchor="middle"></text>')
            svg.append('</g>')
        svg.append('</g>')

    frames = []
    for label, snapshot in rows:
        frame = {'label': label, 'agents': {}}
        for agent in args.agents:
            e = snapshot[agent]
            g = e['structure']
            labels = {}
            for w, labs in g.get('labels', {}).items():
                labels[w] = [short(a) for a in g.get('varying_atoms', [])
                             if a in labs][:2]
            frame['agents'][agent] = {
                'labels': labels,
                'worlds': g.get('worlds', []),
                'designated': g.get('designated', []),
                'edges': [[w, t] for w, ts in g.get('relations', {}).get(agent, {}).items()
                          for t in ts],
                'reflexive': e['reflexive'],
                'actual': e['includes_actual'],
                'D': e['designated'],
            }
        frames.append(frame)

    fragment = f'''<figure class="demo-figure kfig" id="{uid}-fig">
  <svg viewBox="0 0 {width} {height}" width="100%" role="img"
       aria-label="Each agent's accessibility relation, stepped through the run">
    <defs>
      <marker id="{uid}-head" viewBox="0 0 10 10" refX="9.5" refY="5"
              markerWidth="6" markerHeight="6" orient="auto">
        <path d="M0,0 L10,5 L0,10 z" fill="#1A52A0"/></marker>
    </defs>
    {''.join(svg)}
  </svg>
  <div class="kbar">
    <button type="button" class="kbtn" id="{uid}-prev" aria-label="previous step">&#8592;</button>
    <button type="button" class="kbtn" id="{uid}-play" aria-label="play">&#9654; play</button>
    <button type="button" class="kbtn" id="{uid}-next" aria-label="next step">&#8594;</button>
    <span class="kstep" id="{uid}-label"></span>
  </div>
</figure>
<script>
(function(){{
  var frames = {json.dumps(frames)};
  var root = document.getElementById('{uid}-fig');
  if (!root) return;
  var i = 0, timer = null;

  function draw() {{
    var f = frames[i];
    root.querySelectorAll('.kworld').forEach(function(g) {{
      var st = f.agents[g.dataset.a];
      var live = st.worlds.indexOf(g.dataset.w) >= 0;
      g.classList.toggle('dead', !live);
      g.classList.toggle('des', live && st.designated.indexOf(g.dataset.w) >= 0);
    }});
    Object.keys(f.agents).forEach(function(a) {{
      var labels = f.agents[a].labels || {{}};
      Object.keys(labels).forEach(function(w) {{
        for (var k = 0; k < 2; k++) {{
          var t = document.getElementById('{uid}-at-' + a + '-' + w + '-' + k);
          if (t) t.textContent = labels[w][k] || '';
        }}
      }});
    }});
    root.querySelectorAll('.kedge').forEach(function(p) {{
      var st = f.agents[p.dataset.a];
      var on = st.edges.some(function(e) {{
        return e[0] === p.dataset.from && e[1] === p.dataset.to;
      }});
      p.classList.toggle('cut', !on);
    }});
    Object.keys(f.agents).forEach(function(a) {{
      var st = f.agents[a];
      var el = document.getElementById('{uid}-frame-' + a);
      if (el) el.textContent = '|D| = ' + st.D +
        (st.actual ? '' : '  \\u00b7  excludes the actual world');
      if (el) el.setAttribute('class', st.actual ? 'kframe' : 'kframe wrong');
    }});
    var lab = document.getElementById('{uid}-label');
    if (lab) lab.textContent = (i === 0 ? 'initial state'
      : 'step ' + i + ' \\u00b7 ' + f.label);
  }}

  function go(d) {{ i = (i + d + frames.length) % frames.length; draw(); }}

  document.getElementById('{uid}-prev').onclick = function(){{ stop(); go(-1); }};
  document.getElementById('{uid}-next').onclick = function(){{ stop(); go(1); }};
  var play = document.getElementById('{uid}-play');
  function stop() {{
    if (timer) {{ clearInterval(timer); timer = null; play.innerHTML = '&#9654; play'; }}
  }}
  play.onclick = function() {{
    if (timer) return stop();
    play.innerHTML = '&#10073;&#10073; pause';
    timer = setInterval(function(){{ go(1); }}, 1900);
  }};

  // Start when it is on screen, and stop when it leaves.
  if ('IntersectionObserver' in window) {{
    new IntersectionObserver(function(es) {{
      es.forEach(function(e) {{ if (!e.isIntersecting) stop(); }});
    }}, {{threshold: 0.15}}).observe(root);
  }}
  draw();
}})();
</script>'''

    open(args.out, 'w').write(fragment)
    print(f'{len(frames)} frames, {n} worlds, {len(args.agents)} panels -> {args.out}')


if __name__ == '__main__':
    main()
