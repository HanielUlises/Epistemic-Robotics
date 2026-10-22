#!/bin/bash
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
#
# Everything about the belief and empathy demonstration that needs no
# simulator.
#
#   1. the area: tools/check_layout.py;
#   2. the policy, for a site in each half: plank grounds the task, Aletheia's
#      AO* plans it with consistent beliefs, and in every world the policy
#      seeks the nearer vehicle at its particles in order, never goes to a
#      third particle, never sends a vehicle to the site that has not been
#      hailed, and has the task done by a capable vehicle;
#   3. what every robot knows after every step, replayed through the product
#      update (tools/trace_policy.py), against the paper's update rules;
#   4. the oblivious reading: the same domain with an observer out of contact
#      believing that nothing happened has no policy with consistent beliefs,
#      and the policy found without that requirement leaves every vehicle met
#      believing a contradiction;
#   5. the executor's vocabulary and mapping cover every action of both
#      policies, and name only actions the task has;
#   6. the variant with latent tool faults: Aletheia's policy reaches the task
#      in all twelve patterns and sends no vehicle before it is met;
#   7. the same of the timed policies in policies/, one per site.
set -e
HERE=$(dirname "$(realpath "$0")")
EPDDL=${EPDDL:-$(realpath "$HERE/../../../../epddl-workspace/belief-empathy")}
export ER_TOOLS=${ER_TOOLS:-$(realpath "$HERE/../../../../tools")}
PLANK=${PLANK:-$HOME/plank/build/plank}
PLANNER=${EPISTEMIC_PLANNER:-$HOME/aletheia-subjective/build/epistemic_planner}
LIB=${PLANK_LIB:-$HOME/plank/benchmarks/libraries/intermediate.epddl}
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

echo "== 1. the area"
python3 "$HERE/check_layout.py" | tail -1

for side in east west; do
  echo "== 2. the policy, the site in the $side"
  "$PLANK" export -d "$EPDDL/belief-empathy.epddl" -p "$EPDDL/instances/problem_$side.epddl" \
      -l "$LIB" "$EPDDL/hailing.epddl" -o "$OUT/$side" > /dev/null 2>&1
  "$PLANNER" --task "$OUT/$side/problem_$side.json" --plan "$OUT/$side/policy.json" \
      --conditional --consistent-beliefs 2>&1 | grep -E 'Loaded|Solution found' | sed 's/^/  /'
  python3 - "$OUT/$side/problem_$side.json" "$OUT/$side/policy.json" "$side" <<'PY'
import json, sys
task, plan, side = json.load(open(sys.argv[1])), json.load(open(sys.argv[2])), sys.argv[3]
events = {a: v['designated'] for a, v in task['actions'].items()}

def walk(n, d=0):
    if n is None:
        return
    br = n['branches']
    print('    ' + '  ' * d + n['action'])
    for b in br:
        walk(b['subtree'], d + (len(br) > 1))
walk(plan)
near, far = ('g1', 'g2') if side == 'west' else ('g2', 'g1')
labels = task['initial-state']['labels']
ok = True
for w in task['initial-state']['designated']:
    p = {g: next(k for k in (1, 2, 3) if f'p{k}_{g}' in labels[w]) for g in ('g1', 'g2')}
    n, steps = plan, []
    while n is not None:
        evs = events[n['action']]
        pick = n['branches'][0]
        for b in n['branches']:
            e = evs[b['event']]
            if e.startswith('e-met-') and p[n['action'].split('_')[-1]] != int(e[-1]):
                continue
            if e.startswith('e-absent-') and p[n['action'].split('_')[-1]] == int(e[-1]):
                continue
            pick = b
            break
        steps.append((n['action'], evs[pick['event']]))
        n = pick['subtree']
    hails = [(a.split('_')[-1], int(a.split('_')[0][-1]), e) for a, e in steps if a.startswith('hail')]
    met = [g for g, b, e in hails if e.startswith('e-met')]
    doer = [a.split('_')[-1] for a, _ in steps if a.startswith('perform')]
    sent = [a.split('_')[1] for a, _ in steps if a.startswith('go-site')]
    good = (len(doer) == 1 and p[doer[0]] < 3 and set(sent) <= set(met)
            and all(b < 3 for _, b, _ in hails) and hails[0][:2] == (near, 1)
            and all(b == k + 1 for k, (g, b, _) in enumerate(h for h in hails if h[0] == near)))
    print(f'  {"ok  " if good else "FAIL"} g1 at {p["g1"]}, g2 at {p["g2"]}: hails '
          + ', '.join(f'{g}@{b} {"met" if "met" in e else "absent"}' for g, b, e in hails)
          + f'; the task done with {doer[0]}')
    ok &= good
sys.exit(0 if ok else 1)
PY
done

echo "== 3. what each robot knows, step by step (east)"
python3 "$HERE/trace_policy.py" "$OUT/east/problem_east.json" "$OUT/east/policy.json" --json "$OUT/trace.json" \
    --quiet
python3 - "$OUT/trace.json" <<'PY'
import json, sys
ok = True
for r in json.load(open(sys.argv[1])):
    st = r['steps']
    tag = f"g1 at {r['particles']['g1']}, g2 at {r['particles']['g2']}"
    first = st[0]['holds']
    checks = [('each vehicle knows its own particle, eq. (13)', all(s['holds']['own'] for s in st)),
              ('the sought vehicle knows u does not know where it is: empathy', first['empathy']),
              ('nobody but u knows of the task at first', not first['K_g1 task'] and not first['K_g2 task']),
              ('the task done at the end', st[-1]['holds']['done']),
              ('no agent ever believes a contradiction', all(min(s['considered'].values()) > 0 for s in st))]
    absent1 = [s for s in st if s['step'].startswith(f"hail-p1_u_{r['sought']}") and 'absent' in s['step']]
    if absent1:
        checks.append(('found absent at p1, u knows the vehicle is not there, eq. (11)',
                       absent1[0]['holds']['ruled-out']))
        checks.append(('and holds both later particles possible, where eq. (12) believes one',
                       absent1[0]['holds']['open']))
    met = [s for s in st if s['step'].startswith('hail') and 'met' in s['step']]
    if met:
        o = r['other']
        checks.append(('after the meeting, u knows that the vehicle knows of the task', met[0]['holds']['higher']
                       or met[0]['step'].endswith(f'u_{o} / e-met-1')))
        checks.append((f'and {o} cannot rule out that it was told', met[0]['holds']['unsure']
                       or met[0]['step'].endswith(f'u_{o} / e-met-1')))
    if r['particles'][r['sought']] == 3:
        absent2 = [s for s in st if 'absent-2' in s['step']][0]
        checks.append(('two hails unanswered, u knows the vehicle is at p3', absent2['holds']['last']))
        checks.append(('and, from the fault budget, that the other has had no fault', absent2['holds']['budget']))
    bad = [c for c, good in checks if not good]
    print(f"  {'ok  ' if not bad else 'FAIL'} {tag}: {len(checks)} checks" + (f'; failed: {bad}' if bad else ''))
    ok &= not bad
sys.exit(0 if ok else 1)
PY

echo "== 4. the oblivious reading"
"$PLANK" export -d "$EPDDL/belief-empathy.epddl" -p "$EPDDL/instances/problem_east.epddl" \
    -l "$LIB" "$EPDDL/variants/hailing-oblivious.epddl" -o "$OUT/obl" > /dev/null 2>&1
if "$PLANNER" --task "$OUT/obl/problem_east.json" --plan "$OUT/obl/c.json" --conditional --consistent-beliefs \
    2>&1 | grep -q 'No solution found'; then
  echo "  ok   no policy with consistent beliefs"
else
  echo "  FAIL a policy with consistent beliefs"; exit 1
fi
"$PLANNER" --task "$OUT/obl/problem_east.json" --plan "$OUT/obl/v.json" --conditional 2>&1 \
    | grep -E 'Solution found' | sed 's/^/  without the requirement: /'
python3 - "$OUT/obl/problem_east.json" "$OUT/obl/v.json" <<'PY'
import json, os, sys
sys.path.insert(0, os.environ['ER_TOOLS'])
from model_check import extensions, model_of
from plan_trace import convert, update
task, plan = json.load(open(sys.argv[1])), json.load(open(sys.argv[2]))
collapsed, stuck = 0, []
for w in task['initial-state']['designated']:
    model, actual, n = model_of(task['initial-state']), w, plan
    while n is not None:
        act = task['actions'][n['action']]
        pick = None
        for b in n['branches']:
            e = act['designated'][b['event']]
            pre = convert(act['preconditions'][e])
            if actual in extensions(model, pre)[pre]:
                pick = (b, e)
                break
        if pick is None:
            stuck.append((w, n['action']))
            break
        model, pairs = update(model, act, ['u', 'g1', 'g2'], [pick[1]])
        actual = next(k for k, (x, e) in pairs.items() if x == actual and e == pick[1])
        if pick[1].startswith('e-met') and not model.image(n['action'].split('_')[-1], actual):
            collapsed += 1
        n = pick[0]['subtree']
print(f'  ok   replayed, {collapsed} hails leave the vehicle met believing a contradiction'
      + ''.join(f'; in {w} {a} is not applicable' for w, a in stuck))
sys.exit(0 if collapsed > 0 else 1)
PY

echo "== 5. the vocabulary"
python3 "$HERE/make_pddl.py" --out "$OUT/pddl"
python3 - "$OUT/east/problem_east.json" "$OUT/pddl/be-mapping.json" "$OUT"/*/policy.json <<'PY'
import json, sys
task, mapping = json.load(open(sys.argv[1])), json.load(open(sys.argv[2]))

def actions(n):
    return set() if n is None else {n['action']}.union(*[actions(b['subtree']) for b in n['branches']])
used = set().union(*[actions(json.load(open(p))) for p in sys.argv[3:]])
missing, unknown = sorted(used - set(mapping)), sorted(set(mapping) - set(task['actions']))
good = not missing and not unknown
print(f'  {"ok  " if good else "FAIL"} {len(used)} actions in the two policies, {len(mapping)} mapped'
      + (f'; unmapped: {missing}' if missing else '') + (f'; not in the task: {unknown}' if unknown else ''))
sys.exit(0 if good else 1)
PY

for side in east west; do
  echo "== 6. the variant with latent tool faults, the site in the $side"
  "$PLANK" export -d "$EPDDL/belief-empathy-latent.epddl" -p "$EPDDL/instances/problem_latent_$side.epddl" \
      -l "$LIB" "$EPDDL/hailing.epddl" -o "$OUT/latent-$side" > /dev/null 2>&1
  "$PLANNER" --task "$OUT/latent-$side/problem_latent_$side.json" --plan "$OUT/latent-$side/policy.json" \
      --conditional --consistent-beliefs 2>&1 | grep -E 'Loaded|Solution found' | sed 's/^/  /'
  python3 - "$OUT/latent-$side/problem_latent_$side.json" "$OUT/latent-$side/policy.json" "$side" "$HERE" <<'PY'
import json, sys
sys.path.insert(0, sys.argv[4])
import estimate as E
import layout as L
task, plan, side = json.load(open(sys.argv[1])), json.load(open(sys.argv[2])), sys.argv[3]
site = L.SITES[1] if side == 'east' else L.SITES[0]
ok, both = True, 0
for p in E.PATTERNS:
    t, steps = E.run_policy(task, plan, p, site, site, 100.0)
    met = {s.split('_')[2].split(':')[0] for s in steps if s.startswith('hail') and 'e-met' in s}
    sent = [s.split('_')[1] for s in steps if s.startswith('go-site')]
    good = t is not None and set(sent) <= met
    both += len(set(sent)) == 2
    if not good:
        print(f'  FAIL pattern {p}: {steps}')
    ok &= good
print(f'  {"ok  " if ok else "FAIL"} the task done in all {len(E.PATTERNS)} patterns, every vehicle sent '
      f'only after it was met; both vehicles recruited in {both}')
sys.exit(0 if ok else 1)
PY
done

echo "== 7. the timed policies, one per site"
python3 - "$OUT" "$HERE" <<'PY'
import json, os, sys
sys.path.insert(0, sys.argv[2])
import estimate as E
import layout as L
ok = True
for site in L.SITES:
    side = L.side(site)
    task = json.load(open(os.path.join(sys.argv[1], f'latent-{side}', f'problem_latent_{side}.json')))
    pol = json.load(open(os.path.join(sys.argv[2], '..', 'policies',
                                      f'policy_latent_site{site[0]:+.0f}_{site[1]:.0f}.json')))
    good = 0
    for p in E.PATTERNS:
        t, steps = E.run_policy(task, pol, p, site, site, 100.0)
        met = {s.split('_')[2].split(':')[0] for s in steps if s.startswith('hail') and 'e-met' in s}
        sent = [s.split('_')[1] for s in steps if s.startswith('go-site')]
        good += t is not None and set(sent) <= met
    print(f'  {"ok  " if good == len(E.PATTERNS) else "FAIL"} site {site}: the task done in {good} of '
          f'{len(E.PATTERNS)} patterns, every vehicle sent only after it was met')
    ok &= good == len(E.PATTERNS)
sys.exit(0 if ok else 1)
PY
