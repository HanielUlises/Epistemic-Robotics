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
Decides an epistemic goal against a recorded model, and shows the working.

A run's report says whether a goal held. It does not say where. The semantics
of the modal language is defined by the extension of a formula -- the set of
worlds at which it is true -- and a goal is satisfied when the designated set
is contained in the extension of the goal. The containment is the fact worth
reporting: a formula false at the point and true at four of five worlds is one
world away from holding, and a table of two truth values cannot say so.

Two decision procedures are implemented over the same models, and they are
required to agree with each other and with the verdict the executor recorded.

`extension` is the labelling algorithm. It computes the extension of every
subformula bottom-up over the subformula order, in O(|Sub(g)| * (|W| + |R|)),
and reports the containment.

`encode` is the propositional reduction. One variable x[psi, w] per subformula
and world, Tseitin-style defining clauses for each connective and each
modality, and the negated goal asserted at a designated world. The model fixes
every atom, so the defining clauses are a chain of implications that unit
propagation closes with no decision: the encoding is refuted, or satisfied,
without search. The number of decisions taken is reported and is expected to
be zero, which is the formal statement that model checking is not the search
problem planning is.

    model_check.py extensions --analysis scenarios/hotel/out/analysis-l3.json \\
                              --agents inspector porter guest \\
                              --formula '(and safe (K porter safe)
                                              (not (Kw guest leak-at_l3_suite)))' \\
                              --out docs/img/hotel-extensions.svg

    model_check.py cnf --analysis scenarios/hotel/out/analysis-l3.json --step 4 \\
                       --formula '...' --out docs/img/hotel-sat.svg
"""

import argparse
import json
import sys


# ─────────────────────────────────────────────────────────── the language ──

def tokenize(text):
    return text.replace('(', ' ( ').replace(')', ' ) ').split()


def parse(text):
    """One formula, from the s-expression syntax the analysis files use."""
    tokens = tokenize(text)
    node, rest = _parse(tokens)
    if rest:
        raise ValueError(f'trailing input after the formula: {" ".join(rest)}')
    return node


def _parse(tokens):
    if not tokens:
        raise ValueError('the formula ended early')
    head, rest = tokens[0], tokens[1:]
    if head != '(':
        return ('atom', head), rest
    if not rest:
        raise ValueError('unclosed parenthesis')
    operator, rest = rest[0], rest[1:]
    arguments = []
    while rest and rest[0] != ')':
        argument, rest = _parse(rest)
        arguments.append(argument)
    if not rest:
        raise ValueError('unclosed parenthesis')
    rest = rest[1:]

    key = operator.lower()
    if key in ('k', 'kw', 'b', 'bw', 'm'):
        if len(arguments) != 2 or arguments[0][0] != 'atom':
            raise ValueError(f'{operator} takes an agent and a formula')
        return (key, arguments[0][1], arguments[1]), rest
    if key == 'not':
        return ('not', arguments[0]), rest
    if key in ('and', 'or'):
        return (key, tuple(arguments)), rest
    if key in ('implies', '=>'):
        return ('or', (('not', arguments[0]), arguments[1])), rest
    raise ValueError(f'unknown connective: {operator}')


def show(node):
    """The formula written back out, for a figure label."""
    kind = node[0]
    if kind == 'atom':
        return node[1]
    if kind == 'not':
        return f'¬{show(node[1])}'
    if kind in ('and', 'or'):
        glue = ' ∧ ' if kind == 'and' else ' ∨ '
        return '(' + glue.join(show(x) for x in node[1]) + ')'
    if kind == 'c':
        return f'C_{{{",".join(node[1])}}} {show(node[2])}'
    symbol = {'k': 'K', 'kw': 'Kw', 'b': 'B', 'bw': 'Bw', 'm': '◇'}[kind]
    return f'{symbol}_{node[1]} {show(node[2])}'


def subformulas(node, seen=None):
    """Sub(phi), in an order in which every formula follows its parts."""
    seen = [] if seen is None else seen
    if node in seen:
        return seen
    if node[0] == 'not':
        subformulas(node[1], seen)
    elif node[0] in ('and', 'or'):
        for part in node[1]:
            subformulas(part, seen)
    elif node[0] in ('k', 'kw', 'b', 'bw', 'm', 'c'):
        subformulas(node[2], seen)
        if node[0] in ('kw', 'bw'):
            subformulas(('not', node[2]), seen)
    seen.append(node)
    return seen


# ───────────────────────────────────────────────────────────── the models ──

class Model:
    """A finite pointed Kripke model, as the run recorded it."""

    def __init__(self, worlds, relations, labels, designated):
        self.worlds = list(worlds)
        self.relations = {a: {w: set(r.get(w, ())) for w in self.worlds}
                          for a, r in relations.items()}
        self.labels = {w: set(labels.get(w, ())) for w in self.worlds}
        self.designated = set(designated)

    def image(self, agent, world):
        return self.relations.get(agent, {}).get(world, set())


def model_of(structure, designated=None):
    return Model(structure['worlds'], structure.get('relations', {}),
                 structure.get('labels', {}),
                 designated if designated is not None
                 else structure.get('designated', []))


# ────────────────────────────────────────────────── the labelling algorithm ──

def extensions(model, formula):
    """[[psi]] for every psi in Sub(formula), bottom-up.

    The recurrence is the satisfaction clause of each connective read as an
    operation on sets. The modal case is the only one that consults the
    relation: w is in [[K_i psi]] exactly when the whole of R_i(w) lies in
    [[psi]], so knowledge is a universal quantifier over an agent's image and
    Kw_i is the union of the two ways of settling the question.
    """
    table = {}
    for node in subformulas(formula):
        kind = node[0]
        if kind == 'atom':
            if node[1] == 'true':
                table[node] = set(model.worlds)
            elif node[1] == 'false':
                table[node] = set()
            else:
                table[node] = {w for w in model.worlds
                               if node[1] in model.labels[w]}
        elif kind == 'not':
            table[node] = set(model.worlds) - table[node[1]]
        elif kind == 'and':
            table[node] = set(model.worlds).intersection(
                *[table[p] for p in node[1]])
        elif kind == 'or':
            table[node] = set().union(*[table[p] for p in node[1]])
        elif kind in ('k', 'b'):
            inner = table[node[2]]
            table[node] = {w for w in model.worlds
                           if model.image(node[1], w) <= inner}
        elif kind in ('kw', 'bw'):
            inner, outer = table[node[2]], table[('not', node[2])]
            table[node] = {w for w in model.worlds
                           if model.image(node[1], w) <= inner
                           or model.image(node[1], w) <= outer}
        elif kind == 'm':
            inner = table[node[2]]
            table[node] = {w for w in model.worlds
                           if model.image(node[1], w) & inner}
        elif kind == 'c':
            # Common knowledge quantifies over the transitive closure of the
            # union of the group's relations, so the extension is the greatest
            # set closed under every one of them and contained in [[psi]].
            inner = table[node[2]]
            closed = set(model.worlds)
            while True:
                shrunk = {w for w in closed
                          if w in inner and all(model.image(a, w) <= closed
                                                for a in node[1])}
                if shrunk == closed:
                    break
                closed = shrunk
            table[node] = closed
        else:
            raise ValueError(f'no clause for {kind}')
    return table


def holds(model, formula):
    """M, W* |= phi, which is containment of the designated set."""
    return model.designated <= extensions(model, formula)[formula]


# ────────────────────────────────────────────── the propositional reduction ──

class Encoding:
    """A Tseitin encoding of one model-checking instance, and its refutation.

    x[psi, w] is the truth of psi at w. The atoms are fixed by the valuation
    and enter as units, and every other subformula enters as the defining
    clauses of its connective. Asserting the negation of the goal at a
    designated world gives a formula that is unsatisfiable exactly when the
    goal holds there, so the goal test is a refutation and the refutation is
    the certificate the page reports.
    """

    def __init__(self, model, formula):
        self.model = model
        self.formula = formula
        self.names = {}
        self.clauses = []
        self.families = {}
        self._build()

    def variable(self, node, world):
        key = (node, world)
        if key not in self.names:
            self.names[key] = len(self.names) + 1
        return self.names[key]

    def add(self, family, *clauses):
        for clause in clauses:
            self.clauses.append(clause)
            self.families[family] = self.families.get(family, 0) + 1

    def _build(self):
        model, worlds = self.model, self.model.worlds
        for node in subformulas(self.formula):
            for w in worlds:
                x = self.variable(node, w)
                kind = node[0]
                if kind == 'atom':
                    truth = node[1] in model.labels[w] or node[1] == 'true'
                    self.add('valuation', [x] if truth else [-x])
                elif kind == 'not':
                    y = self.variable(node[1], w)
                    self.add('negation', [x, y], [-x, -y])
                elif kind == 'and':
                    ys = [self.variable(p, w) for p in node[1]]
                    self.add('conjunction', *[[-x, y] for y in ys])
                    self.add('conjunction', [x] + [-y for y in ys])
                elif kind == 'or':
                    ys = [self.variable(p, w) for p in node[1]]
                    self.add('disjunction', *[[x, -y] for y in ys])
                    self.add('disjunction', [-x] + ys)
                elif kind in ('k', 'b'):
                    ys = [self.variable(node[2], v)
                          for v in sorted(model.image(node[1], w))]
                    self.add('modality', *[[-x, y] for y in ys])
                    self.add('modality', [x] + [-y for y in ys])
                elif kind in ('kw', 'bw'):
                    positive = [self.variable(node[2], v)
                                for v in sorted(model.image(node[1], w))]
                    negative = [self.variable(('not', node[2]), v)
                                for v in sorted(model.image(node[1], w))]
                    # x <-> (AND positive) OR (AND negative), expanded through
                    # two auxiliary literals held by the same defining shape.
                    a = self.variable(('aux-all', node), w)
                    b = self.variable(('aux-none', node), w)
                    self.add('modality', *[[-a, y] for y in positive])
                    self.add('modality', [a] + [-y for y in positive])
                    self.add('modality', *[[-b, y] for y in negative])
                    self.add('modality', [b] + [-y for y in negative])
                    self.add('modality', [-a, x], [-b, x], [-x, a, b])
        # The empty conjunction and disjunction leave a stray clause behind.
        self.clauses = [c for c in self.clauses if c]

    def goal_clauses(self):
        """The negated goal, at each designated world."""
        return [[-self.variable(self.formula, w)]
                for w in sorted(self.model.designated)]


def solve(clauses, variables):
    """DPLL, instrumented to report how much of it was needed.

    Unit propagation alone closes an encoding whose atoms are fixed by a
    model, so `decisions` is the quantity of interest: zero decisions is the
    statement that the reduction is decided by propagation and that no search
    took place.
    """
    assignment = {}
    propagations = 0
    decisions = 0

    def propagate():
        nonlocal propagations
        changed = True
        while changed:
            changed = False
            for clause in clauses:
                unassigned, satisfied = [], False
                for literal in clause:
                    value = assignment.get(abs(literal))
                    if value is None:
                        unassigned.append(literal)
                    elif value == (literal > 0):
                        satisfied = True
                        break
                if satisfied:
                    continue
                if not unassigned:
                    return False
                if len(unassigned) == 1:
                    literal = unassigned[0]
                    assignment[abs(literal)] = literal > 0
                    propagations += 1
                    changed = True
        return True

    def search():
        nonlocal decisions
        if not propagate():
            return False
        free = [v for v in variables if v not in assignment]
        if not free:
            return True
        decisions += 1
        trail = dict(assignment)
        for value in (True, False):
            assignment.clear()
            assignment.update(trail)
            assignment[free[0]] = value
            if search():
                return True
        assignment.clear()
        assignment.update(trail)
        return False

    satisfiable = search()
    return {'satisfiable': satisfiable, 'propagations': propagations,
            'decisions': decisions, 'assignment': dict(assignment)}


# ─────────────────────────────────────────────────── reading a recorded run ──

def designated_of(snapshot, agents):
    """The designated set of the shared model, recovered from the perspectives.

    `get_agent_perspective` reports the model with agent i's designated set,
    which is the image R_i[W*] rather than W* itself. No call reports W*. It is
    nonetheless determined: a world w belongs to W* exactly when R_i(w) is the
    set agent i designates, for every agent at once, and on both recorded runs
    that condition selects a set of the size the executor's log reports.
    """
    reference = snapshot[agents[0]]['structure']
    chosen = []
    for w in reference['worlds']:
        if all(set(snapshot[a]['structure']['relations'][a].get(w, ()))
               == set(snapshot[a]['structure']['designated'])
               for a in agents):
            chosen.append(w)
    return chosen


def timeline(analysis, agents):
    """Every recorded snapshot as a model, in order."""
    entries = [('0', 'initial state', analysis['initial'])]
    for index, step in enumerate(analysis['steps']):
        entries.append((str(index + 1), step['action'], step['agents']))
    out = []
    for label, action, snapshot in entries:
        structure = snapshot[agents[0]]['structure']
        model = model_of(structure, designated_of(snapshot, agents))
        out.append({'label': label, 'action': action, 'model': model,
                    'recorded': snapshot['model']})
    return out


def agree(model, recorded):
    """Every verdict the executor recorded, checked against the labelling."""
    disagreements = []
    for text, verdict in recorded.items():
        if holds(model, parse(text)) != verdict:
            disagreements.append(text)
    return disagreements


# ───────────────────────────────────────────────────────────────── figures ──
#
# The pages typeset their mathematics with MathJax, so a figure that carries
# formulas is emitted as an HTML fragment rather than as an SVG: the formulas
# go in as TeX and are set by the same engine that sets the surrounding prose,
# and only the part that is genuinely a picture -- the worlds of each model,
# and which of them the extension contains -- is drawn.

INK, RED, GRAY, BORDER, PANEL = '#111111', '#C8102E', '#5A5A5A', '#D6D6D6', '#FAFAFA'


def escape(text):
    return text.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')


def tex_atom(name):
    # A grounded atom is set in the typewriter face. MathJax renders an escaped
    # underscore inside \texttt as a literal backslash, so the atom is broken at
    # its underscores and they are set between the pieces.
    return '\\_'.join('\\texttt{' + part + '}' for part in name.split('_'))


def tex(node, top=True):
    """One formula as TeX, in the notation the pages already use."""
    kind = node[0]
    if kind == 'atom':
        return tex_atom(node[1])
    if kind == 'not':
        return '\\neg ' + tex(node[1], False)
    if kind in ('and', 'or'):
        glue = ' \\wedge ' if kind == 'and' else ' \\vee '
        body = glue.join(tex(p, False) for p in node[1])
        return body if top else '(' + body + ')'
    if kind == 'c':
        group = ',\\,'.join(tex_atom(a) for a in node[1])
        return f'C_{{{group}}}\\,' + tex(node[2], False)
    symbol = {'k': 'K', 'b': 'B', 'kw': '\\mathit{Kw}', 'bw': '\\mathit{Bw}',
              'm': '\\widehat{K}'}[kind]
    return f'{symbol}_{{{tex_atom(node[1])}}}\\,' + tex(node[2], False)


EXT_STYLE = """<style>
/* Extension matrix. One row per subformula of the goal, one column per step;
   a cell draws the worlds of that step's model. Scoped to this figure. */
.extmat{width:100%;border-collapse:collapse;font-size:.8rem}
.extmat th{font-family:var(--sg);font-size:.68rem;font-weight:500;color:var(--gray);
  letter-spacing:.04em;padding:.3rem .4rem;text-align:center;vertical-align:bottom;
  border-bottom:1px solid var(--border)}
.extmat th:first-child{text-align:left}
.extmat th .st{display:block;font-family:var(--mono);font-size:.66rem;color:var(--black)}
.extmat th .wc{display:block;font-size:.62rem;color:var(--gray);font-weight:400}
.extmat td{padding:.34rem .4rem;text-align:center;border-bottom:1px solid var(--border);
  border-bottom-color:rgba(0,0,0,.07)}
.extmat td:first-child{text-align:left;white-space:nowrap;padding-right:1.1rem}
.extmat tr.goal td{border-top:1.4px solid var(--black);border-bottom:none;
  padding-top:.5rem}
.extmat tr:last-child td{border-bottom:none}
.extmat .wr{display:inline-flex;gap:3px;align-items:center}
.extmat td.q{font-family:var(--mono);font-size:.72rem;color:var(--black)}
.extmat tr.stat td{color:var(--gray)}
.extmat tr.stat:first-of-type td{border-top:1px solid var(--border)}
</style>
"""


R_WORLD = 3.1          # a world
R_RING = 5.8           # the ring that marks it designated
STEP = 14              # centre to centre


def dots(model, inside):
    """The worlds of one model, filled where the formula holds.

    The designated ring is wider than the world it encloses, so the box is
    padded by its radius at both ends: without that the first and last rings
    are clipped by the viewBox.
    """
    n = len(model.worlds)
    pad = R_RING + 1.2
    width = (n - 1) * STEP + 2 * pad
    height = 2 * pad
    out = [f'<svg class="wr" width="{width:.0f}" height="{height:.0f}" '
           f'viewBox="0 0 {width:.1f} {height:.1f}" aria-hidden="true">']
    for k, w in enumerate(model.worlds):
        cx, cy = pad + k * STEP, pad
        fill = INK if w in inside else '#FFFFFF'
        if w in model.designated:
            out.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{R_RING}" '
                       f'fill="none" stroke="{RED}" stroke-width="1.1"/>')
        out.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{R_WORLD}" '
                   f'fill="{fill}" stroke="{INK}" stroke-width="1"/>')
    out.append('</svg>')
    return ''.join(out)


def extension_figure(track, formula, title, caption, extra=(), stats=()):
    """The extension of every subformula of the goal, at every step.

    The recorded run reports a truth value at the point. This reports the set
    of worlds at which each subformula is true, which is what the semantics
    actually assigns it, and draws the goal's verdict as the containment it
    is: the goal holds at a step exactly when every designated world in the
    bottom row is filled.
    """
    subs = list(subformulas(formula))
    for node in extra:
        for part in subformulas(node):
            if part not in subs:
                subs.insert(len(subs) - 1, part)
    for entry in track:
        table = extensions(entry['model'], formula)
        for node in extra:
            table.update(extensions(entry['model'], node))
        entry['table'] = table

    rows = []
    header = ['<tr><th>subformula</th>']
    for entry in track:
        head = escape(entry['action'].split('_')[0])
        header.append(f'<th>{entry["label"]}<span class="st">{head}</span>'
                      f'<span class="wc">|W| = {len(entry["model"].worlds)}</span></th>')
    header.append('</tr>')

    for node in subs:
        final = node is subs[-1]
        name = '\\gamma' if final else tex(node)
        cells = [f'<td>\\({name}\\)</td>']
        for entry in track:
            model = entry['model']
            cells.append(f'<td>{dots(model, entry["table"][node])}</td>')
        klass = ' class="goal"' if final else ''
        rows.append(f'<tr{klass}>' + ''.join(cells) + '</tr>')

    for label, measure in stats:
        cells = [f'<td>\\({label}\\)</td>']
        for entry in track:
            cells.append(f'<td class="q">{measure(entry["model"])}</td>')
        rows.append('<tr class="stat">' + ''.join(cells) + '</tr>')

    return (EXT_STYLE
            + '<div class="fig">\n'
            + '  <div class="fig-head">\n'
            + f'    <span class="fig-title">{title}</span>\n'
            + f'    <span class="note">\\(\\gamma \\equiv {tex(formula)}\\)</span>\n'
            + '  </div>\n'
            + '  <div class="fig-body">\n'
            + '    <table class="extmat">\n'
            + '      <thead>' + ''.join(header) + '</thead>\n'
            + '      <tbody>\n        '
            + '\n        '.join(rows)
            + '\n      </tbody>\n    </table>\n  </div>\n'
            + f'  <div class="fig-cap">{caption}</div>\n'
            + '</div>\n')


CLAUSE_SHAPES = {
    'valuation': (r'x_{p,w} \;\text{or}\; \neg x_{p,w}',
                  'one unit clause per atom and world, read off \\(V\\)'),
    'negation': (r'x_{\neg\psi,w} \leftrightarrow \neg x_{\psi,w}',
                 'two clauses'),
    'conjunction': (r'x_{\psi\wedge\chi,w} \leftrightarrow x_{\psi,w} \wedge x_{\chi,w}',
                    'one clause per conjunct, and one closing'),
    'disjunction': (r'x_{\psi\vee\chi,w} \leftrightarrow x_{\psi,w} \vee x_{\chi,w}',
                    'one clause per disjunct, and one closing'),
    'modality': (r'x_{K_i\psi,w} \leftrightarrow \bigwedge_{v \in R_i(w)} x_{\psi,v}',
                 'one clause per accessible world, and one closing'),
    'goal': (r'\neg x_{\gamma,w} \ \text{ for each } w \in W^{*}',
             'the negated goal, asserted at each designated world'),
}


def cnf_figure(model, formula, report, title, caption):
    """The goal test written as a propositional refutation."""
    families = list(report['families'].items()) + [('goal', report['goal_clauses'])]
    rows = []
    for name, count in families:
        shape, note = CLAUSE_SHAPES[name]
        rows.append(f'<tr><td>{name}</td><td>\\({shape}\\)</td>'
                    f'<td class="n">{count}</td></tr>')

    verdict = 'unsatisfiable' if not report['satisfiable'] else 'satisfiable'
    reading = ('\\(\\gamma\\) is entailed at every designated world, so the goal '
               'holds' if not report['satisfiable'] else
               'the negated goal is consistent with the model, so the goal fails')

    return ('<style>\n'
            '.cnf{width:100%;border-collapse:collapse;font-size:.8rem}\n'
            '.cnf th{font-family:var(--sg);font-size:.68rem;font-weight:500;'
            'color:var(--gray);letter-spacing:.04em;text-align:left;'
            'padding:.3rem .5rem;border-bottom:1px solid var(--border)}\n'
            '.cnf td{padding:.4rem .5rem;border-bottom:1px solid rgba(0,0,0,.07)}\n'
            '.cnf td.n,.cnf th.n{text-align:right;font-family:var(--mono)}\n'
            '.cnf tr:last-child td{border-bottom:none}\n'
            '.verdict{margin-top:1rem;border:1px solid var(--border);background:#fff;'
            'padding:.7rem .9rem;border-radius:2px;font-size:.83rem}\n'
            '.verdict b{color:var(--red);text-transform:uppercase;'
            'letter-spacing:.05em;font-size:.78rem}\n'
            '</style>\n'
            '<div class="fig">\n'
            '  <div class="fig-head">\n'
            f'    <span class="fig-title">{title}</span>\n'
            f'    <span class="note">{report["variables"]} variables, '
            f'{report["clauses"]} clauses</span>\n'
            '  </div>\n'
            '  <div class="fig-body">\n'
            '    <table class="cnf">\n'
            '      <thead><tr><th>clause family</th><th>defining shape</th>'
            '<th class="n">clauses</th></tr></thead>\n'
            '      <tbody>\n        ' + '\n        '.join(rows) + '\n'
            '      </tbody>\n    </table>\n'
            f'    <div class="verdict"><b>{verdict}</b> &#183; '
            f'{report["propagations"]} unit propagations, '
            f'<b style="color:var(--black)">{report["decisions"]} decisions</b>'
            f' &#183; {reading}.</div>\n'
            '  </div>\n'
            f'  <div class="fig-cap">{caption}</div>\n'
            '</div>\n')


def depth(node):
    """Modal depth of a formula."""
    kind = node[0]
    if kind == 'atom':
        return 0
    if kind == 'not':
        return depth(node[1])
    if kind in ('and', 'or'):
        return max(depth(p) for p in node[1])
    return 1 + depth(node[2])


def report_for(model, formula):
    """Encode, refute, and record what the solver needed."""
    encoding = Encoding(model, formula)
    variables = list(range(1, len(encoding.names) + 1))
    goal = encoding.goal_clauses()
    result = solve(encoding.clauses + goal, variables)
    return {'variables': len(encoding.names),
            'clauses': len(encoding.clauses) + len(goal),
            'families': encoding.families, 'goal_clauses': len(goal),
            'satisfiable': result['satisfiable'],
            'propagations': result['propagations'],
            'decisions': result['decisions']}


# ───────────────────────────────────────────────────────────────────── cli ──

DEFAULT_EXT_CAPTION = (
    'A dot is a world of that step&#8217;s model, filled where the formula holds '
    'and ringed where the world is designated. The goal is satisfied exactly '
    'where every ringed dot in the bottom row is filled, which is the '
    'containment \\(W^{*} \\subseteq [\\![\\gamma]\\!]\\).')

DEFAULT_CNF_CAPTION = (
    'The same question decided as a propositional refutation. Every recorded '
    'verdict of the run is reproduced by both procedures.')


def load(args):
    analysis = json.load(open(args.analysis))
    track = timeline(analysis, args.agents)
    for entry in track:
        disagreements = agree(entry['model'], entry['recorded'])
        if disagreements:
            raise SystemExit(f'step {entry["label"]}: the labelling disagrees '
                             f'with the recorded verdict on {disagreements}')
    checked = sum(len(e['recorded']) for e in track)
    print(f'{checked} recorded verdicts reproduced over {len(track)} snapshots')
    return track


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)

    for name in ('extensions', 'cnf', 'report'):
        p = sub.add_parser(name)
        p.add_argument('--analysis', required=True)
        p.add_argument('--agents', nargs='+', required=True)
        p.add_argument('--formula', required=True, help='the goal, as an s-expression')
        p.add_argument('--out')
        p.add_argument('--title', default='')
        p.add_argument('--caption', default='')
        p.add_argument('--extra', action='append', default=[],
                       help='repeatable; a formula tracked beside the goal '
                            'without being part of it')
        if name == 'cnf':
            p.add_argument('--step', type=int, default=-1,
                           help='index into the run; -1 is the final model')

    args = parser.parse_args()
    formula = parse(args.formula)
    track = load(args)

    if args.command == 'extensions':
        fragment = extension_figure(
            track, formula,
            args.title or 'The extension of each subformula, at every step',
            args.caption or DEFAULT_EXT_CAPTION,
            [parse(text) for text in args.extra])
        open(args.out, 'w').write(fragment)
        print(f'{len(subformulas(formula))} subformulas -> {args.out}')

    elif args.command == 'cnf':
        entry = track[args.step]
        report = report_for(entry['model'], formula)
        fragment = cnf_figure(
            entry['model'], formula, report,
            args.title or 'The goal test as a propositional refutation',
            args.caption or DEFAULT_CNF_CAPTION)
        open(args.out, 'w').write(fragment)
        print(json.dumps(report, indent=1))
        print(f'-> {args.out}')

    else:
        for entry in track:
            model = entry['model']
            table = extensions(model, formula)
            print(f'{entry["label"]:>3}  {entry["action"]}')
            print(f'     |W|={len(model.worlds)}  W*={sorted(model.designated)}')
            for node in subformulas(formula):
                print(f'     {show(node):<48} {sorted(table[node])}')
            print(f'     {json.dumps(report_for(model, formula))}')


if __name__ == '__main__':
    main()
