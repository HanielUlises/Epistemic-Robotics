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
Statistics and figures for the communication study.

    analyze.py --csv /tmp/study/study.csv --out /tmp/study/analysis

Success rates with Wilson 95% intervals; the plan against each protocol on
the same instances by an exact McNemar test on the discordant pairs; message
counts on the instances both succeed on, compared by the Wilcoxon signed-rank
test, with the median ratio. Writes summary.json, tables.md and one SVG per
regime, the protocols in a fixed colour each and the merge rule as the line
style.
"""

import argparse
import collections
import csv
import json
import math
import os

import numpy as np
from scipy import stats

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

COLOUR = {'plan': '#2a78d6', 'flood': '#eb6834', 'pull': '#1baf7a', 'gossip': '#eda100'}
LABEL = {'plan': 'plan', 'flood': 'flood', 'pull': 'pull', 'gossip': 'gossip'}
INK, MUTED, GRID = '#0b0b0b', '#52514e', '#e4e3df'


def wilson(k, n, z=1.96):
    if n == 0:
        return (float('nan'), float('nan'), float('nan'))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (p, max(0.0, c - h), min(1.0, c + h))


def mcnemar(a, b):
    """Exact two-sided McNemar test on paired successes: a and b are lists of
    0/1 over the same instances. Returns (plan-only, other-only, p)."""
    only_a = sum(1 for x, y in zip(a, b) if x and not y)
    only_b = sum(1 for x, y in zip(a, b) if y and not x)
    n = only_a + only_b
    p = 1.0 if n == 0 else min(1.0, 2 * stats.binom.cdf(min(only_a, only_b), n, 0.5))
    return only_a, only_b, p


def load(path):
    rows = list(csv.DictReader(open(path)))
    for r in rows:
        r['success'] = int(r['success'])
        r['budget'] = None if r['budget'] in ('', 'None') else int(r['budget'])
        r['sigma'] = None if r['sigma'] in ('', 'None') else float(r['sigma'])
        r['messages'] = None if r['messages'] == '' else int(r['messages'])
        r['infer'] = int(r['infer'])
    return rows


def index(rows):
    """(regime, cell, strategy, rule, infer, budget, sigma) -> {seed: row}"""
    out = collections.defaultdict(dict)
    for r in rows:
        out[(r['regime'], r['cell'], r['strategy'], r['rule'], r['infer'], r['budget'], r['sigma'])][r['seed']] = r
    return out


def series(idx, regime, cell, strategy, rule, infer=1, sigma=None):
    """Success over the budgets, with Wilson intervals."""
    pts = []
    for key, by_seed in idx.items():
        if key[:5] == (regime, cell, strategy, rule, infer) and key[6] == sigma:
            k = sum(r['success'] for r in by_seed.values())
            pts.append((key[5], k, len(by_seed)))
    return sorted(pts, key=lambda p: (p[0] is None, p[0] or 0))


def full(idx, regime, cell, strategy, rule, infer=1, sigma=None):
    return idx.get((regime, cell, strategy, rule, infer, None, sigma), {})


def summarise(idx, cells):
    """Per cell, at unlimited budget: each strategy's success with interval,
    the McNemar comparison with the plan, and messages on common successes."""
    out = {}
    for regime, cell, sigmas in cells:
        for sigma in sigmas:
            plan = full(idx, regime, cell, 'plan', 'update', 1, sigma)
            seeds = sorted(plan)
            entry = {'n': len(seeds), 'strategies': {}}
            p_succ = [plan[s]['success'] for s in seeds]
            for strategy in ('plan', 'flood', 'pull', 'gossip'):
                for rule in (('update',) if strategy == 'plan' else ('recency', 'version', 'overwrite', 'confidence')):
                    for infer in ((1, 0) if regime == 'elimination' and strategy != 'plan' else (1,)):
                        d = full(idx, regime, cell, strategy, rule, infer, sigma)
                        if not d:
                            continue
                        succ = [d[s]['success'] for s in seeds if s in d]
                        k, n = sum(succ), len(succ)
                        rate = wilson(k, n)
                        e = {'k': k, 'n': n, 'rate': rate}
                        msgs = [d[s]['messages'] for s in seeds if s in d and d[s]['success'] and d[s]['messages'] is not None]
                        if msgs:
                            e['messages_median'] = float(np.median(msgs))
                            e['messages_iqr'] = [float(np.percentile(msgs, 25)), float(np.percentile(msgs, 75))]
                        if strategy != 'plan':
                            a = [plan[s]['success'] for s in seeds if s in d]
                            b = succ
                            e['mcnemar'] = mcnemar(a, b)
                            both = [s for s in seeds if s in d and plan[s]['success'] and d[s]['success']]
                            if len(both) >= 5:
                                pm = np.array([plan[s]['messages'] for s in both], float)
                                bm = np.array([d[s]['messages'] for s in both], float)
                                ratio = np.median(bm / np.maximum(pm, 1))
                                try:
                                    w = stats.wilcoxon(bm, pm, alternative='greater').pvalue
                                except ValueError:
                                    w = float('nan')
                                e['paired_messages'] = {'n': len(both), 'median_ratio': float(ratio),
                                                        'wilcoxon_p': float(w)}
                        key = strategy if strategy == 'plan' else f'{strategy}-{rule}' + ('' if infer else '-noinfer')
                        entry['strategies'][key] = e
            secs = [plan[s]['plan_seconds'] for s in seeds if plan[s]['plan_seconds'] not in ('', None)]
            secs = [float(x) for x in secs]
            entry['plan_seconds_median'] = float(np.median(secs)) if secs else None
            entry['plan_seconds_max'] = float(np.max(secs)) if secs else None
            entry['plan_solved'] = sum(1 for s in seeds if plan[s]['reason'] in ('', 'over budget'))
            raw = [int(plan[s]['plan_messages_raw']) for s in seeds
                   if plan[s]['plan_messages_raw'] not in ('', None) and plan[s]['success']]
            kept = [plan[s]['messages'] for s in seeds if plan[s]['success']]
            if raw:
                entry['elimination_removed_median'] = float(np.median(np.array(raw) - np.array(kept)))
            inf = [int(plan[s]['inferred_sends'] or 0) for s in seeds if plan[s]['success']]
            entry['plan_inferred_sends'] = int(sum(inf))
            entry['skipped'] = sum(int(plan[s]['skipped_before']) for s in seeds)
            out[f'{regime}/{cell}' + (f'/sigma={sigma:g}' if sigma is not None else '')] = entry
    return out


# ─── Figures ────────────────────────────────────────────────────────────────

def style(ax, title):
    ax.set_title(title, fontsize=10, color=INK, loc='left')
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=8)
    ax.yaxis.grid(True, color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)


def budget_axis(pts):
    """Budgets on a symmetric-log axis, unlimited drawn one step past the
    largest."""
    finite = [p[0] for p in pts if p[0] is not None]
    top = max(finite) if finite else 1
    xs = [p[0] if p[0] is not None else top * 1.6 for p in pts]
    return xs, top * 1.6


def curve(ax, idx, regime, cell, strategy, rule, infer=1, dashed=False, label=None):
    pts = series(idx, regime, cell, strategy, rule, infer)
    if not pts:
        return None
    xs, unl = budget_axis(pts)
    ys = [wilson(k, n) for _, k, n in pts]
    ax.fill_between(xs, [y[1] for y in ys], [y[2] for y in ys], color=COLOUR[strategy], alpha=0.12, linewidth=0)
    ax.plot(xs, [y[0] for y in ys], color=COLOUR[strategy], linewidth=2, linestyle='--' if dashed else '-',
            label=label or LABEL[strategy])
    return unl


def budget_figure(idx, regime, cells, path, title, infer_variants=False):
    cols = 2
    rows = (len(cells) + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(4.2 * cols, 3.0 * rows), sharey=True, squeeze=False)
    axes = axes.ravel()
    for ax, (cell, sub) in zip(axes, cells):
        unl = None
        for strategy in ('plan', 'flood', 'pull', 'gossip'):
            rule = 'update' if strategy == 'plan' else 'recency'
            u = curve(ax, idx, regime, cell, strategy, rule)
            unl = unl or u
            if infer_variants and strategy == 'flood':
                curve(ax, idx, regime, cell, 'flood', 'recency', infer=0, dashed=True, label='flood, no inference')
        ax.set_xscale('symlog', linthresh=1)
        style(ax, sub)
        ax.set_ylim(-0.02, 1.05)
        ax.set_xlabel('message budget (maps of a bay)', fontsize=8, color=MUTED)
        if unl:
            first = min(p[0] for p in series(idx, regime, cell, 'plan', 'update') if p[0] is not None)
            ticks = [t for t in [0, 1, 4, 16, 64, 256] if first <= t < unl]
            ax.set_xticks(ticks + [unl])
            ax.set_xticklabels([str(t) for t in ticks] + ['∞'])
            ax.set_xlim(first if first > 0 else 0, unl * 1.05)
    for k in range(0, len(axes), cols):
        axes[k].set_ylabel('shifts succeeded', fontsize=8, color=MUTED)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.suptitle(title, fontsize=11, color=INK, x=0.01, ha='left')
    fig.tight_layout()
    fig.legend(handles, labels, loc='center left', bbox_to_anchor=(1.0, 0.5), frameon=False, fontsize=8)
    fig.savefig(path, bbox_inches='tight')
    plt.close(fig)


def bars_figure(summary, keys, path, title, strategies):
    fig, axes = plt.subplots(1, len(keys), figsize=(3.6 * len(keys), 2.8), sharey=True)
    axes = np.atleast_1d(axes)
    for ax, (key, sub) in zip(axes, keys):
        e = summary[key]['strategies']
        names = [s for s in strategies if s in e]
        for k, s in enumerate(names):
            p, lo, hi = e[s]['rate']
            base = s.split('-')[0]
            ax.bar(k, p, color=COLOUR[base], width=0.62)
            ax.errorbar(k, p, yerr=[[p - lo], [hi - p]], color=INK, capsize=3, linewidth=1)
            ax.text(k, min(1.02, hi + 0.03), f'{p:.0%}', ha='center', fontsize=8, color=INK)
        ax.set_xticks(range(len(names)))
        ax.set_xticklabels([n.replace('-recency', '').replace('-', '\n') for n in names], fontsize=8, color=MUTED)
        ax.set_ylim(0, 1.12)
        style(ax, sub)
    axes[0].set_ylabel('shifts succeeded', fontsize=8, color=MUTED)
    fig.suptitle(title, fontsize=11, color=INK, x=0.01, ha='left')
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def skew_figure(idx, path, sigmas):
    fig, ax = plt.subplots(figsize=(6.4, 3.0))
    for strategy, rule, dashed, label in (('plan', 'update', False, 'plan'),
                                          ('flood', 'version', True, 'flood, schedule version'),
                                          ('flood', 'recency', False, 'flood, recency'),
                                          ('pull', 'recency', False, 'pull, recency')):
        ys = []
        for s in sigmas:
            d = full(idx, 'skew', 'n8', strategy, rule, 1, s)
            k, n = sum(r['success'] for r in d.values()), len(d)
            ys.append(wilson(k, n))
        ax.fill_between(sigmas, [y[1] for y in ys], [y[2] for y in ys], color=COLOUR[strategy], alpha=0.12, linewidth=0)
        ax.plot(sigmas, [y[0] for y in ys], color=COLOUR[strategy], linewidth=2,
                linestyle='--' if dashed else '-', label=label)
    style(ax, 'Clock skew: a bay that changed twice, 10 s apart')
    ax.set_xlabel('clock deviation σ (s)', fontsize=8, color=MUTED)
    ax.set_ylabel('shifts succeeded', fontsize=8, color=MUTED)
    ax.set_ylim(-0.02, 1.05)
    ax.set_xlim(0, sigmas[-1])
    ax.legend(loc='lower left', frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def tables(summary):
    lines = []
    for key, e in summary.items():
        lines.append(f'### {key} (n = {e["n"]}; plan solved {e["plan_solved"]}, '
                     f'median {e["plan_seconds_median"]} s, max {e["plan_seconds_max"]} s)\n')
        lines.append('| strategy | success [95% CI] | messages, median [IQR] | plan only / other only, McNemar p | paired messages: median ratio, Wilcoxon p |')
        lines.append('| --- | --- | --- | --- | --- |')
        for s, v in e['strategies'].items():
            p, lo, hi = v['rate']
            m = f"{v['messages_median']:.0f} [{v['messages_iqr'][0]:.0f}, {v['messages_iqr'][1]:.0f}]" if 'messages_median' in v else ''
            mc = f"{v['mcnemar'][0]} / {v['mcnemar'][1]}, {v['mcnemar'][2]:.2g}" if 'mcnemar' in v else ''
            pm = v.get('paired_messages')
            pmt = f"{pm['median_ratio']:.0f}×, {pm['wilcoxon_p']:.2g} (n {pm['n']})" if pm else ''
            lines.append(f'| {s} | {v["k"]}/{v["n"]} = {p:.0%} [{lo:.0%}, {hi:.0%}] | {m} | {mc} | {pmt} |')
        lines.append('')
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--csv', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rows = load(args.csv)
    idx = index(rows)
    present = sorted({(r['regime'], r['cell']) for r in rows})
    sig = sorted({r['sigma'] for r in rows if r['regime'] == 'skew' and r['sigma'] is not None})
    cells = [(g, c, sig if g == 'skew' else [None]) for g, c in present]
    summary = summarise(idx, cells)
    with open(os.path.join(args.out, 'summary.json'), 'w') as fh:
        json.dump(summary, fh, indent=1)
    with open(os.path.join(args.out, 'tables.md'), 'w') as fh:
        fh.write(tables(summary))
    if any(g == 'budget' for g, _ in present):
        budget_figure(idx, 'budget', [('n8-full', '8 robots, every pair linked'), ('n16-full', '16 robots, every pair linked'),
                                      ('n8-radio', '8 robots, radio graph'), ('n16-radio', '16 robots, radio graph')],
                      os.path.join(args.out, 'budget.svg'), 'A message budget: shifts succeeded against messages allowed')
    if any(g == 'elimination' for g, _ in present):
        budget_figure(idx, 'elimination', [('b4', '4 bays'), ('b6', '6 bays')],
                      os.path.join(args.out, 'elimination.svg'),
                      'Knowledge by elimination: the open bay seen by nobody, or by few', infer_variants=True)
    if any(g == 'joint' for g, _ in present):
        budget_figure(idx, 'joint', [('pairs1', 'one pair'), ('pairs2', 'two pairs')],
                      os.path.join(args.out, 'joint.svg'), 'A joint crossing: both haulers of a pair at one open bay')
    if any(g == 'secrecy' for g, _ in present):
        keys = [(k, t) for k, t in (('secrecy/n8', '8 robots'), ('secrecy/n12', '12 robots'),
                                    ('secrecy/n8-hauls', '8 robots, a contractor hauls')) if k in summary]
        bars_figure(summary, keys, os.path.join(args.out, 'secrecy.svg'),
                    'Secrecy: haulers delivered and no contractor holds the secret, unlimited budget',
                    ['plan', 'flood-recency', 'pull-recency', 'gossip-recency'])
    if sig:
        skew_figure(idx, os.path.join(args.out, 'skew.svg'), sig)
    print(os.path.join(args.out, 'tables.md'))


if __name__ == '__main__':
    main()
