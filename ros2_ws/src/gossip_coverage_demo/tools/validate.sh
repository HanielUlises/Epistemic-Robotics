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
# Everything about the gossip demonstration that needs no Gazebo.
#
#   1. the arena: every seed's crates stand a metre from the walls and clear
#      of the tasks, and every Case II seed has the three-robot task found
#      while the two-robot task is being done;
#   2. the canonical instances (instances.py): plank grounds each, Aletheia
#      plans it as the demonstration does, and tools/check_policy.py replays
#      the policy in every designated world: the paper's update rules hold at
#      every step, the goal is reached, and every robot can follow the policy
#      on what it observes; AO* finds no shallower policy;
#   3. the oblivious reading: no policy with consistent beliefs exists, and
#      the policy of the Unsure reading, replayed there, leaves robots with
#      the actual world out of what they consider possible, knowing falsehoods
#      and then believing a contradiction (tools/oblivious.py);
#   4. the executor's vocabulary and mapping cover every action of the
#      canonical policies;
#   5. the port of ePlanSys's scheduler (gossip/schedule.py) groups the
#      canonical policies so that no task is started before an arrival it
#      needs;
#   6. one kinematic run of every method in each case.
set -e
HERE=$(dirname "$(realpath "$0")")
EPDDL=${EPDDL:-$(realpath "$HERE/../../../../epddl-workspace/gossip-coverage")}
export ER_TOOLS=${ER_TOOLS:-$(realpath "$HERE/../../../../tools")}
PLANK=${PLANK:-$HOME/plank/build/plank}
PLANNER=${EPISTEMIC_PLANNER:-$HOME/aletheia-subjective/build/epistemic_planner}
LIB=${PLANK_LIB:-$HOME/plank/benchmarks/libraries/intermediate.epddl}
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

echo "== 1. the arena"
python3 - "$HERE" <<'PY'
import math, sys
sys.path.insert(0, sys.argv[1])
import arena as A
bad = 0
for case in (1, 2):
    for seed in range(1, 13):
        s = A.scenario(seed, case)
        for (x, y) in s['crates']:
            if min(x, y, A.SPEC.width - x, A.SPEC.height - y) < 1.0:
                bad += 1
            if any(math.dist((x, y), t['pos']) < 1.0 for t in s['tasks']):
                bad += 1
print(f'  24 worlds, {bad} crates too near a wall or a task')
sys.exit(1 if bad else 0)
PY

declare -A CASE=([hardware_case1]=1 [hardware_case2]=2 [sim_case1]=1 [sim_case2]=2)
for i in hardware_case1 hardware_case2 sim_case1 sim_case2; do
  echo "== 2. $i"
  "$PLANK" export -d "$EPDDL/gossip-coverage.epddl" -p "$EPDDL/instances/$i.epddl" -l "$LIB" "$EPDDL/gossip.epddl" \
      -o "$OUT/$i" > /dev/null 2>&1
  "$PLANNER" --task "$OUT/$i/$i.json" --plan "$OUT/$i/policy.json" --strategy replan --subjective auto \
      --consistent-beliefs 2>&1 | grep -E 'Loaded|Solution found' | sed 's/^/  /'
  python3 "$HERE/check_policy.py" "$OUT/$i/$i.json" "$OUT/$i/policy.json" --case "${CASE[$i]}" --quiet | sed 's/^/  /'
  case $i in hardware_*)
    "$PLANNER" --task "$OUT/$i/$i.json" --plan "$OUT/$i/aostar.json" --conditional --subjective auto \
        --consistent-beliefs 2>&1 | grep -E 'Solution found' | sed 's/^/  AO*: /';;
  esac
done

for i in sim_case1 hardware_case2; do
  echo "== 3. the oblivious reading, $i"
  "$PLANK" export -d "$EPDDL/gossip-coverage.epddl" -p "$EPDDL/instances/$i.epddl" -l "$LIB" \
      "$EPDDL/variants/gossip-oblivious.epddl" -o "$OUT/obl-$i" > /dev/null 2>&1
  if "$PLANNER" --task "$OUT/obl-$i/$i.json" --plan "$OUT/obl-$i/policy.json" --strategy replan --subjective auto \
      --consistent-beliefs --timeout 60 2>&1 | grep -q 'No solution exists'; then
    echo "  no policy with consistent beliefs exists"
  else
    echo "  UNEXPECTED: a policy with consistent beliefs"; exit 1
  fi
  python3 "$HERE/oblivious.py" "$EPDDL/instances/$i.epddl" "$OUT/$i/policy.json" | tail -2 | cut -c1-160 | sed 's/^/  /'
done

echo "== 4. the executor's vocabulary and mapping"
python3 - "$HERE" "$OUT" <<'PY'
import json, os, sys
sys.path.insert(0, sys.argv[1])
import make_pddl as V
mapping = V.mapping()
vocab = V.vocabulary()
missing = []
for i in ('hardware_case1', 'hardware_case2', 'sim_case1', 'sim_case2'):
    pol = json.load(open(os.path.join(sys.argv[2], i, 'policy.json')))
    stack = [pol]
    while stack:
        n = stack.pop()
        if n is None:
            continue
        a = n['action']
        # the mapping is written for the Gazebo team of three
        if all(r in ('r1', 'r2', 'r3') for r in a.split('_')[1:]):
            if a not in mapping or f"(:durative-action {mapping[a]['action'][1:].split()[0]}" not in vocab:
                missing.append(a)
        stack += [b['subtree'] for b in n['branches']]
print('  every action of the three-robot policies is mapped' if not missing else f'  UNMAPPED {missing}')
sys.exit(1 if missing else 0)
PY

echo "== 5. the scheduler"
python3 - "$HERE" "$OUT" <<'PY'
import json, os, sys
sys.path.insert(0, sys.argv[1])
from gossip import schedule as SCH
from gossip.epistemic import DURATION, kind, requires
for i in ('hardware_case1', 'hardware_case2', 'sim_case1', 'sim_case2'):
    d = os.path.join(sys.argv[2], i)
    task, pol = json.load(open(os.path.join(d, f'{i}.json'))), json.load(open(os.path.join(d, 'policy.json')))
    root, items = SCH.schedule(pol, task, lambda a: DURATION.get(kind(a), 10.0), requires)
    # walk every branch; a do-task must come after every arrival before it on its branch
    bad, groups = 0, 0

    def walk(node, seen):
        global bad, groups
        if node is None:
            return
        n, chain = node, []
        for _ in range(node.group):
            chain.append(items[n.item].action)
            nxt = n.next[0] if n.next else None
            if len(chain) < node.group:
                n = nxt
        if len(chain) > 1:
            groups += 1
        for a in chain:
            if a.startswith('do-task-'):
                k = int(a.split('_')[0].rsplit('-', 1)[1])
                arrived = sum(1 for x in seen if x.startswith('go-site-'))
                if arrived < k - 1:
                    bad += 1
        last = n
        for nx in last.next:
            walk(nx, seen + chain)
    walk(root, [])
    print(f'  {i}: {groups} groups of more than one action, {bad} tasks started early')
    if bad:
        sys.exit(1)
PY

echo "== 6. one kinematic run of every method in each case"
python3 - "$HERE" <<'PY'
import os, sys
sys.path.insert(0, sys.argv[1])
os.chdir(sys.argv[1])
import study
from gossip import env as E, sim as S
for case in (1, 2, 3):
    area = study.area_for(E.SIM, 1, 3, case)
    times = {m: S.run(area, 3, case, m, seed=1031 + case)['mission'] for m in ('ideal', 'proposed', 'flock', 'epistemic')}
    print(f'  case {case}, three robots: ' + ', '.join(f'{m} {t} s' for m, t in times.items()))
    if None in times.values():
        sys.exit(1)
PY
echo "== all checks passed"
