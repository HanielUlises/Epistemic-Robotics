"""
The timed policy against Aletheia's and the procedures, in the kinematic model.

    compare.py PRE TIMED

PRE is precheck.py's output directory (the grounded tasks and Aletheia's
policies, per side); TIMED holds the skeletons. Every policy is timed by
estimate.run_policy and replayed through the product update in every world
(trace_policy.replay), which fails on any branch that does not fit. Two sets of
cases: the 24 seeds, and every fault pattern at every site (48).
"""

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import timed_policy as TP  # noqa: E402
import estimate as E  # noqa: E402
import precheck  # noqa: E402
import trace_policy  # noqa: E402
import worlds as W  # noqa: E402

SITES = {'east': [(4.0, 2.0), (4.0, 6.0)], 'west': [(-4.0, 2.0), (-4.0, 6.0)]}


def main():
    pre, timed = sys.argv[1], sys.argv[2]
    tasks, ataska, aletheia, sks = {}, {}, {}, {}
    for side in SITES:
        tasks[side] = json.load(open(os.path.join(timed, side, f'problem_latent_{side}.json')))
        ataska[side] = json.load(open(os.path.join(pre, side, f'problem_latent_{side}.json')))
        aletheia[side] = json.load(open(os.path.join(pre, side, 'policy.json')))
        sks[side] = TP.Skeleton.load(tasks[side], os.path.join(timed, f'{side}.skel'))
    policies, planned = {}, {}
    for side, sites in SITES.items():
        for site in sites:
            u, t = precheck.found(site)
            r = TP.plan(sks[side], site, u, t)
            policies[site], planned[site] = r['policy'], r
            # every designated world: the policy's branch fits and ends in (done)
            for w in tasks[side]['initial-state']['designated']:
                rep = trace_policy.replay(tasks[side], r['policy'], w)
                assert rep['steps'][-1]['holds']['done'], (site, w)
            json.dump(r['policy'], open(os.path.join(timed, f'policy_{site[0]:+.0f}_{site[1]:.0f}.json'), 'w'))

    def row(side, site, p):
        u, t = precheck.found(site)
        out = {}
        out['timed'], steps = E.run_policy(tasks[side], policies[site], p, site, u, t)
        out['timed_steps'] = precheck.short(steps)
        out['aletheia'], _ = E.run_policy(ataska[side], aletheia[side], p, site, u, t)
        for s in ('published', 'strong', 'ideal'):
            out[s], st = E.run_procedure(s, p, site, u, t)
            if s == 'strong':
                out['strong_steps'] = st
        return out

    def summary(rows, title):
        print(title)
        for k in ('timed', 'aletheia', 'published', 'strong', 'ideal'):
            ts = [r[k] for r in rows]
            print(f'  {k:>10}: mean {sum(ts) / len(ts):6.1f} s   worst {max(ts):6.1f} s')
        d = [r['timed'] - r['strong'] for r in rows]
        print(f"  timed vs strong: sooner in {sum(x < -1 for x in d)}, within 1 s in {sum(abs(x) <= 1 for x in d)}, "
              f"later in {sum(x > 1 for x in d)}; largest loss {max(d):+.1f} s, largest gain {min(d):+.1f} s")

    seeds = []
    for seed in range(1, 25):
        sc = W.scenario_latent(seed)
        p, site = tuple(sc['pattern']), tuple(sc['site'])
        r = row(sc['side'], site, p)
        r.update(seed=seed, pattern=p, site=site)
        seeds.append(r)
    summary(seeds, 'the 24 seeds')
    for r in seeds:
        print(f"  seed {r['seed']:>2} {r['pattern']} {r['site']}: timed {r['timed']:6.1f}  aletheia {r['aletheia']:6.1f}"
              f"  strong {r['strong']:6.1f}  ideal {r['ideal']:6.1f}   {' '.join(r['timed_steps'])}")
    full = []
    for side, sites in SITES.items():
        for site in sites:
            for p in E.PATTERNS:
                r = row(side, site, p)
                r.update(pattern=p, site=site)
                full.append(r)
    summary(full, 'every pattern at every site (48)')
    for site, r in planned.items():
        print(f"  plan at {site}: predicted mean {r['expected']:.1f} s, worst {r['worst']:.1f} s, "
              f"{r['expanded']} nodes, {r['seconds']:.2f} s")
    json.dump({'seeds': seeds, 'full': full}, open(os.path.join(timed, 'compare.json'), 'w'), indent=1, default=str)


if __name__ == '__main__':
    main()
