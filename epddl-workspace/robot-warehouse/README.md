# robot-warehouse

A small epistemic warehouse domain, written to exercise the aletheia planner
end to end: EPDDL → `plank export` → `epistemic_planner`.

Two robots share a three-bay warehouse (`bay1 — bay2 — bay3`). A package sits
in `bay2` or `bay3`, and nobody knows which. Actions:

| Action | Type | Meaning |
|---|---|---|
| `move ?i ?from ?to` | public-ontic | drive between adjacent bays; positions are public |
| `scan ?i ?l` | private-sensing | read the shelf you stand on; the other robot doesn't even see the scan |
| `broadcast ?i ?l` | public-announcement | radio "the package is in `?l`"; becomes common knowledge |

Goal in both instances: it is common knowledge that *both* robots know whether
the package is in `bay2`.

## Instances

- `problem_1.epddl` — the true bay is asserted in `:init`, so only one world is
  designated and the plan collapses to a single branch: `R2` already stands in
  `bay3`, scans, and broadcasts.
- `problem_2.epddl` — the true bay is left open, so both worlds stay designated
  and the plan must succeed either way: `R1` drives to `bay2`, scans, then
  broadcasts `bay2` on a positive reading and `bay3` on a negative one (it can
  deduce the location from the common knowledge that the package is in exactly
  one of the two bays).

## Running

```sh
LIB=~/plank/benchmarks/libraries/intermediate.epddl

~/plank/build/plank export -d domain.epddl -p instances/problem_2.epddl -l $LIB -o export_out

~/Projects/aletheia/build/epistemic_planner \
    --task export_out/problem_2.json \
    --plan export_out/plan_2.json \
    --explain
```

`--explain` prints the task features and which policy rule chose the heuristic
and strategy. Both instances land on `knowledge-spread` + AO\*, and the
planner's own validator re-checks every branch of the emitted plan.
