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
Draws a recorded epistemic model as a Kripke structure, in TikZ.

`scripts/record_models.py` writes the model after each product update: the
worlds, the atoms true at each, the designated set, and one accessibility
relation per agent. This renders one such file as a labelled graph, and as the
partition each agent's relation induces.

Two renderings are produced because neither serves both sizes. A graph with
every relation drawn is readable at three worlds and unreadable at eight, where
three equivalence relations over eight worlds put up to eighty-four edges on
the page. The partition is exact at any size and shows less: it gives the
blocks of each relation and not their arrangement.

An accessibility relation that is an equivalence is drawn undirected, and its
reflexive edges are omitted; the omission is stated in the caption of whatever
includes the output. A relation that is not symmetric is drawn with arrowheads,
and a world at which it is not reflexive is marked, because neither is a
property a figure may quietly repair. The models of this domain are not all
S5: a private announcement leaves the excluded agent believing no utterance
occurred, which is a relation that is serial, transitive and Euclidean and is
not reflexive at the worlds where the utterance did occur.

    plot_kripke.py --model model_03.json --graph g.tex --partition p.tex
    plot_kripke.py --model model_04.json --frame f.tex
"""

import argparse
import itertools
import json
import math
import os
import re


# The palette of the written reports.
INK, SLATE, MIST = 'ink', 'slate', 'mist'
RULE, STEEL, SIGNAL = 'rulegrey', 'steel', 'signal'

# One letter per agent, for edge labels. Three agents and a full name apiece
# would be wider than the edges they sit on.
INITIAL = {'scout': 's', 'relay': 'r', 'observer': 'o'}


def valuation(atoms):
    """The informative part of a world's valuation.

    Every world of this domain satisfies exactly one `contaminated_*` atom, and
    the `on-site` atoms are a record of where the robots have been and are the
    same at every world of a given model. The contaminated atom is therefore
    the whole of what distinguishes one world from another by valuation, and it
    is what the figure shows; worlds that agree on it are distinguished by
    accessibility and not by what holds at them.
    """
    for atom in atoms:
        found = re.fullmatch(r'contaminated[_-](\w+)', atom)
        if found:
            return found.group(1)
    return r'\ensuremath{\varnothing}'


def frame_properties(model):
    """{agent: {property: bool}} for each agent's relation.

    Reported rather than assumed. The problem declares finitary S5 theories and
    the parser announces an S5 frame, and both statements are about the initial
    model; what the relations satisfy after a sequence of product updates is a
    question about those updates and is answered by measuring them.
    """
    worlds = model['worlds']
    out = {}
    for agent, rows in sorted(model['relations'].items()):
        rel = {w: set(rows.get(w, [])) for w in worlds}
        out[agent] = {
            'reflexive': all(w in rel[w] for w in worlds),
            'symmetric': all(u in rel[v] for u in worlds for v in rel[u]),
            'transitive': all(x in rel[u] for u in worlds
                              for v in rel[u] for x in rel[v]),
            'serial': all(rel[w] for w in worlds),
            'euclidean': all(y in rel[x] for w in worlds
                             for x in rel[w] for y in rel[w]),
        }
    return out


def frame_of(p):
    """The strongest of the two named frames the relation satisfies."""
    if p['reflexive'] and p['symmetric'] and p['transitive']:
        return 'S5'
    if p['serial'] and p['transitive'] and p['euclidean']:
        return 'KD45'
    return '---'


def undirected_edges(model):
    """{(u, v): ([agents], directed)} for u < v, reflexive pairs omitted.

    `directed` is true when some agent relates the pair one way only, in which
    case the edge is drawn with an arrowhead. Symmetrising it would assert a
    property the model does not have.
    """
    worlds = model['worlds']
    index = {w: i for i, w in enumerate(worlds)}
    relations = model['relations']

    edges = {}
    for agent, rows in sorted(relations.items()):
        for source, targets in rows.items():
            for target in targets:
                if source == target:
                    continue
                back = relations[agent].get(target, [])
                one_way = source not in back
                u, v = sorted((source, target), key=lambda w: index[w])
                agents, directed = edges.setdefault((u, v), ([], False))
                if agent not in agents:
                    agents.append(agent)
                edges[(u, v)] = (agents, directed or one_way)
    return edges


def partitions(model):
    """{agent: [block, ...]} where a block is a list of world names.

    Computed as the connected components of the agent's relation. Under S5 the
    components are the equivalence classes.
    """
    worlds = model['worlds']
    out = {}
    for agent, rows in sorted(model['relations'].items()):
        seen, blocks = set(), []
        for start in worlds:
            if start in seen:
                continue
            block, stack = [], [start]
            seen.add(start)
            while stack:
                world = stack.pop()
                block.append(world)
                for nxt in rows.get(world, []):
                    if nxt not in seen:
                        seen.add(nxt)
                        stack.append(nxt)
            blocks.append(sorted(block, key=worlds.index))
        out[agent] = blocks
    return out


def reflexive_worlds(model):
    """{world: [agents whose relation is not reflexive there]}."""
    out = {}
    for world in model['worlds']:
        missing = [agent for agent, rows in sorted(model['relations'].items())
                   if world not in rows.get(world, [])]
        if missing:
            out[world] = missing
    return out


def frame_tex(models):
    """The frame each agent's relation satisfies, at each recorded model."""
    mark = lambda b: r'$\checkmark$' if b else r'---'
    rows = []
    for index, model in models:
        props = frame_properties(model)
        for n, (agent, p) in enumerate(sorted(props.items())):
            first = f'{index}' if n == 0 else ''
            size = f'{len(model["worlds"])}' if n == 0 else ''
            rows.append(
                f'{first} & {size} & \\textit{{{agent}}} & '
                + ' & '.join(mark(p[k]) for k in
                             ('reflexive', 'symmetric', 'transitive',
                              'serial', 'euclidean'))
                + f' & {frame_of(p)} \\\\')
        rows.append(r'\addlinespace')
    return ('\\begin{tabular}{@{}rrlcccccl@{}}\n\\toprule\n'
            '\\textbf{\\#} & $|W|$ & \\textbf{Agent} & refl. & sym. & trans. & '
            'ser. & eucl. & \\textbf{Frame} \\\\\n\\midrule\n'
            + '\n'.join(rows[:-1])
            + '\n\\bottomrule\n\\end{tabular}')


def graph_tex(model, radius=22.0):
    """The model as an undirected labelled graph, on a circle."""
    worlds = model['worlds']
    designated = set(model['designated'])
    labels = model['labels']
    edges = undirected_edges(model)
    reflexive_at = reflexive_worlds(model)

    n = len(worlds)
    lines = [
        r'\begin{tikzpicture}[',
        '  x=1mm, y=1mm,',
        f'  wld/.style={{circle, draw={INK}, fill={MIST}, minimum size=9mm,',
        r'              inner sep=0pt, font=\ttfamily\scriptsize},',
        f'  des/.style={{circle, draw={SIGNAL}, line width=0.6pt,',
        r'              minimum size=11.4mm, inner sep=0pt},',
        f'  rel/.style={{draw={STEEL}, line width=0.5pt}},',
        f'  dir/.style={{draw={STEEL}, line width=0.5pt, -{{Latex[length=1.6mm]}}}},',
        f'  irr/.style={{circle, draw={SIGNAL}, line width=0.5pt, dashed,',
        r'              minimum size=13.4mm, inner sep=0pt},',
        r'  ag/.style={font=\ttfamily\tiny, text=' + STEEL + r', inner sep=1.2pt,',
        f'             fill=white}},',
        r'  val/.style={font=\tiny, text=' + SLATE + r'}]',
    ]

    place = {}
    for i, world in enumerate(worlds):
        # Start at the top and proceed clockwise, so that the reading order of
        # the worlds on the page is the order they carry in the model.
        angle = math.pi / 2 - 2 * math.pi * i / n
        x, y = radius * math.cos(angle), radius * math.sin(angle)
        place[world] = (x, y)

    for (u, v), (agents, directed) in sorted(edges.items()):
        ux, uy = place[u]
        vx, vy = place[v]
        tag = '\\,'.join(INITIAL.get(a, a[0]) for a in agents)
        style = 'dir' if directed else 'rel'
        lines.append(
            f'  \\draw[{style}] ({ux:.2f},{uy:.2f}) -- ({vx:.2f},{vy:.2f})'
            f' node[ag, pos=0.5] {{{tag}}};')

    for world in worlds:
        x, y = place[world]
        short = world.replace('w', '')
        # A world at which some agent's relation is not reflexive is marked:
        # that agent does not consider it possible while standing in it, which
        # under S5 cannot happen and is what a private announcement produces
        # for the agent excluded from it.
        if world in reflexive_at and reflexive_at[world]:
            lines.append(f'  \\node[irr] at ({x:.2f},{y:.2f}) {{}};')
        if world in designated:
            lines.append(f'  \\node[des] at ({x:.2f},{y:.2f}) {{}};')
        lines.append(
            f'  \\node[wld] at ({x:.2f},{y:.2f}) '
            f'{{$w_{{{short}}}$}};')
        # The valuation sits outside the circle, on the ray through it.
        ox = x * 1.42 if abs(x) > 1e-6 else 0.0
        oy = y * 1.42 if abs(y) > 1e-6 else (radius * 1.42)
        lines.append(
            f'  \\node[val] at ({ox:.2f},{oy:.2f}) '
            f'{{{valuation(labels.get(world, []))}}};')

    lines.append(r'\end{tikzpicture}')
    return '\n'.join(lines)


def partition_tex(model):
    """Each agent's relation as a partition, or as its images where it is not.

    A relation that is an equivalence is exhibited by its classes, and the
    classes are what an S5 reading of the model requires. A relation that is
    not an equivalence has no classes, and the connected components of such a
    relation are not the blocks of anything: a component is order-dependent
    under a forward traversal and can place a world in a block none of whose
    other members it is related to. For those agents the images `R(w)` are
    given instead, grouped by the worlds sharing one, since `R(w)` is what
    every modal formula at `w` is evaluated over.
    """
    props = frame_properties(model)
    designated = set(model['designated'])
    worlds = model['worlds']

    def render(block):
        cells = []
        for world in block:
            short = world.replace('w', '')
            body = f'w_{{{short}}}'
            cells.append(f'\\mathbf{{{body}}}' if world in designated
                         else body)
        return r'\{' + ',\\,'.join(cells) + r'\}'

    rows, mixed = [], False
    for agent, parts in partitions(model).items():
        p = props[agent]
        if p['reflexive'] and p['symmetric'] and p['transitive']:
            body = r'\ '.join(render(b) for b in parts)
            rows.append(f'\\textit{{{agent}}} & ${body}$ \\\\')
            continue

        mixed = True
        rel = model['relations'][agent]
        groups = {}
        for world in worlds:
            image = tuple(sorted(rel.get(world, []), key=worlds.index))
            groups.setdefault(image, []).append(world)
        body = r'\quad '.join(
            f'{render(src)} \\mapsto {render(list(image))}'
            for image, src in groups.items())
        rows.append(f'\\textit{{{agent}}}$^{{\\dagger}}$ & ${body}$ \\\\')

    head = (r'\textbf{Equivalence classes of its relation}' if not mixed else
            r'\textbf{Equivalence classes, or $w \mapsto R(w)$ marked }'
            r'$\dagger$')
    return ('\\begin{tabular}{@{}ll@{}}\n\\toprule\n'
            '\\textbf{Agent} & ' + head + ' \\\\\n'
            '\\midrule\n' + '\n'.join(rows)
            + '\n\\bottomrule\n\\end{tabular}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True,
                    help='one file written by scripts/record_models.py')
    ap.add_argument('--graph', help='write the labelled graph here')
    ap.add_argument('--partition', help='write the partition table here')
    ap.add_argument('--frame', help='write the frame-properties table here, '
                                    'over every model given to --frame-models')
    ap.add_argument('--frame-models', nargs='*', default=[],
                    help='the models the frame table covers, in order')
    ap.add_argument('--radius', type=float, default=22.0)
    args = ap.parse_args()

    with open(args.model) as handle:
        model = json.load(handle)

    for agent, p in sorted(frame_properties(model).items()):
        if frame_of(p) != 'S5':
            print(f'{args.model}: {agent} is {frame_of(p)}, not S5 '
                  f'(reflexive={p["reflexive"]}, symmetric={p["symmetric"]})')

    if args.graph:
        with open(args.graph, 'w') as handle:
            handle.write(graph_tex(model, args.radius) + '\n')
        print(f'{args.graph}: {len(model["worlds"])} worlds, '
              f'{len(model["designated"])} designated')
    if args.partition:
        with open(args.partition, 'w') as handle:
            handle.write(partition_tex(model) + '\n')
        print(f'{args.partition}: partitions for '
              f'{len(model["relations"])} agents')
    if args.frame:
        series = []
        for n, path in enumerate(args.frame_models or [args.model]):
            with open(path) as handle:
                series.append((n, json.load(handle)))
        with open(args.frame, 'w') as handle:
            handle.write(frame_tex(series) + '\n')
        print(f'{args.frame}: {len(series)} models')

    if not args.graph and not args.partition and not args.frame:
        print(json.dumps(
            {a: [b for b in blocks] for a, blocks in partitions(model).items()},
            indent=2))


if __name__ == '__main__':
    main()
