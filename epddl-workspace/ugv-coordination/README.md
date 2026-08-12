# ugv-coordination

A scaling benchmark of multi-UGV epistemic coordination, written for the
[aletheia](https://github.com/HanielUlises/aletheia) planner and grounded with
[plank](https://github.com/a-burigana/plank). The accompanying report is
`../../ugv_del_coordination.tex`.

## Setting

A corridor of `L` bays, `N` UGVs, and a target known to sit in exactly one of
the last `U` bays, which one, nobody knows. Every instance leaves the true bay
unasserted, so all `U` candidate worlds stay designated and a plan must succeed
whichever way the scans come out.

| Action | Type | Who observes |
|---|---|---|
| `move ?i ?from ?to` | public-ontic | everyone (fleet pose is shared) |
| `scan ?i ?l` | private-sensing | only `?i`, others don't even see the scan |
| `broadcast ?i ?l` (`ugv-public`) | public-announcement | everyone → common knowledge |
| `report ?i ?l` (`ugv-radio`) | private-announcement | only UGVs in the sender's bay or an adjacent one; the rest are oblivious |
| `tell ?i ?j ?l` (`ugv-directed`) | private-announcement | exactly the addressee `?j`; everyone else oblivious |
| `unlock ?i ?from ?to` (`ugv-facility`) | public-ontic | everyone (keyholders only) |

All announcements have the epistemic precondition `([?i] (target-at ?l))`: a
vehicle can only report what it knows.

`ugv-directed` and `ugv-facility` also carry capability facts
(`has-scanner`, `keyholder`), so the fleet is heterogeneous and no single
vehicle can complete the mission alone.

## Instances

144 instances crossing domain (`public`, `radio`) × `N ∈ {2,3,4}` ×
`L ∈ {3,4,5,6}` × `U ∈ {2,3,4}` × start (`depot`, `spread`) × goal:

- `ck`, `([C. All] (and ([Kw. Ri] (target-at b)) ...))`, fleet-wide common knowledge
- `nested`, `([R1] ([Kw. R2] (target-at b)))`, second-order

Names encode the parameters: `radio-a3-b5-u3-spread-ck`.

### Extended families (60 instances, `gen_instances_x.py`)

- `directed-*`, point-to-point radio, one detector in the fleet.
  `nested` goal `([R1] ([Kw. R2] p))`; `private` goal
  `(and ([Kw. R2] p) (not ([Kw. R3] p)))`, inform one teammate, keep another
  ignorant, which a broadcast model cannot express.
- `facility-*`, heterogeneous fleet (`has-scanner`, `keyholder`), a locked
  mid-corridor bay, and `i` items to localise, so `|W| = 2^i`. The keyholder
  must open the way before the scanner can reach the far bays.

## Running

```sh
python3 gen_instances.py          # 144 corridor instances  -> instances/
python3 gen_instances_x.py        #  60 extended instances  -> instances/
python3 run_benchmark.py          # grounds + solves all, writes results.csv
python3 run_benchmark.py directed # or re-run one family, merging into results.csv
python3 make_tables.py            # regenerates ../../tables/*.tex for the report
```

`run_benchmark.py` allows 300 s per grounding and 60 s per search. A full
sweep takes roughly 25 minutes, nearly all of it grounding the `a4-b6-u4`
instances.

## Results (2026-08-12)

196/204 solved: 136/144 corridor, 60/60 extended. All eight failures are
`radio` at `b6/u4` with `N ≥ 3`, and all are search timeouts, not errors.

⚠️ **Four of the returned policies are wrong**, the multi-item facility
instances `facility-a{2,3}-b{4,5}-i2-depot-ck`. See
[`bug-repro/`](bug-repro/README.md) for a minimal reproduction and the
evidence. Their cost figures are valid; their `solved` status is not.

| Parameter | Median expanded nodes |
|---|---|
| `U` = 2 → 3 → 4 (at `N`=2) | 172 → 2,177 → 330,188 |
| `N` = 2 → 3 → 4 (at `L`≤5) | 621 → 2,714 → 5,480 |
| `L` = 3 → 4 → 5 (at `U`=2) | 66 → 214 → 2,801 |

Initial uncertainty dominates search; `N × L` dominates grounding (0.02 s up to
76 s). The range-limited domain costs about 3× the public one at matched size.

Extended families:

| Family | Solved | Median expanded | Median depth | Max ground actions |
|---|---|---|---|---|
| `directed` | 50/50 | 630 | 5 | 97 |
| `facility` | 10/10 | 3,971 | 7 | 72 |

Addressed communication grounds to `N(N-1)L` actions, the largest action sets
in the suite, but does not enlarge the model, so search stays cheap. The
facility family is the costliest per instance: it is the only one that must
change the world (unlock) before it can learn anything, and multi-item
instances start with four worlds.

The planner selected `knowledge-spread` + AO\* on every solved instance.

## A single instance

```sh
plank export -d domains/ugv-radio.epddl -p instances/radio-a3-b4-u2-depot-ck.epddl \
    -l ~/plank/benchmarks/libraries/intermediate.epddl -o export_out

~/Projects/aletheia/build/epistemic_planner \
    --task export_out/radio-a3-b4-u2-depot-ck.json \
    --plan plans/radio-a3-b4-u2-depot-ck.json --explain
```

That one is worth reading: R1 drives to `b3`, scans, and on **both** branches
drives back to `b2` before reporting, the report is worthless out of range, reporting `b3` on a positive reading and `b4` on a negative one, a bay no
vehicle ever scanned.
