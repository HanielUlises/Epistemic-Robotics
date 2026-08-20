# Doxastic Depot: a KD45ₙ (belief) EPDDL domain

A depot of bays connected by an adjacency graph. Robots carry crates between
bays. Every relocation is **private**: only the agents standing in the *source*
bay observe it, so everybody else keeps believing the crate is where it used to
be. Those stale beliefs are *false* beliefs, so the accessibility relations lose
reflexivity and the reachable state space is KD45ₙ, not S5ₙ.

The domain exercises the whole doxastic toolbox of the `intermediate` action
type library:

| action | type | role |
|---|---|---|
| `go` | `public-ontic` | positions stay common knowledge |
| `relocate` | `private-ontic` | **creates** false beliefs |
| `seal` | `public-ontic` | public ontic change, no location info |
| `peek` | `private-sensing` | fully private observation |
| `inspect` | `quasi-private-sensing` | bystanders see *that* it happened |
| `broadcast` | `public-announcement` | belief repair into common knowledge |
| `whisper` | `private-announcement` | truthful, bay-local |
| `brief` | `quasi-private-announcement` | addressee learns, bystanders only see the exchange |

`broadcast`, `whisper` and `brief` are truthful: their event precondition is
`(and (at ?c ?b) ([?i] (at ?c ?b)))`, i.e. the speaker must believe what it says
and it must be true.

## Files

```
doxastic-depot.epddl        domain
instances/problem_1.epddl   plant a false belief                      (plan: 2)
instances/problem_2.epddl   second-order false belief, modal depth 2  (plan: 2)
instances/problem_3.epddl   false belief + repair into CK             (plan: 3)
instances/problem_4.epddl   explicit KD45 initial model (see caveat)  (plan: 1)
instances/problem_5.epddl   announcement turns private knowledge into CK (plan: 1)
out/                        `plank export` output (grounded JSON tasks)
```

All instances ground to **315 actions** over **29 atoms** and 3 agents.

## Reproducing

```sh
L=~/Projects/plank/benchmarks/libraries/intermediate.epddl
plank parse    -d doxastic-depot.epddl -p instances/problem_1.epddl -l $L
plank ground   -d doxastic-depot.epddl -p instances/problem_1.epddl -l $L
plank export   -d doxastic-depot.epddl -p instances/problem_1.epddl -l $L -o out
plank validate -d doxastic-depot.epddl -p instances/problem_1.epddl -l $L \
               -a "go_chief_bay1_bay2" "relocate_rob1_c1_bay1_bay2"     # true
```

The exported tasks are consumed directly by the Aletheia planner:

```sh
aletheia/build/epistemic_planner --task out/problem_1.json --plan plan.json --explain
```

Aletheia reports `Frame: KD45 (belief)` on every instance and solves 1, 2, 3 and
5; the plans it returns are all accepted by `plank validate`.

## Two findings worth keeping

### 1. `problem_4` needs the fork's explicit-init fix

`problem_4.epddl` gives the initial model explicitly (`:worlds / :relations /
:labels / :designated`) so that a relation is non-reflexive from the very first
world. Upstream plank 1.0 parses and grounds it, but the exported JSON labels
then contain only the *facts*, and every non-fact predicate is lost:

```json
"labels": { "w0": ["office_bay3", "adjacent_bay1_bay2", ...] }   /* at_c1_bay2 missing */
```

The cause is in `build_label`
(`src/lib/epddl/grounder/initial_state/explicit_initial_state_grounder.cpp`):
`ground_atoms` is a raw `boost::dynamic_bitset` already sized to the number of
atoms, so `ground_atoms.push_back(p)` appends a bit *past the end* instead of
setting atom `p`'s bit. The fork (`~/Projects/plank`) already carries the fix,
`ground_atoms.set(p)`, together with a matching fix in the label parser.

`out/problem_4.json` was exported with the fork's binary. The four S5-theory
instances export byte-identical with or without the fix, since only the
explicit-init path was affected.

plank's test suite has no grounder-level tests
(`tests/units/epddl/parser/problems/explicit_init_tests.cpp` covers the parser
only), so `problem_4` here doubles as the regression case for that fix. It is
not part of the Aletheia benchmark set; the next section says why.

### 2. plank and Aletheia disagree on announcing a fact an agent believes false

In `problem_3` the plan

```
relocate_rob1_c1_bay1_bay2   ;; the chief, in the office, is oblivious
broadcast_rob1_c1_bay2       ;; public announcement of the new location
```

is accepted by `plank validate` (`true`) but Aletheia never finds it: it proves
depth 2 exhausted and returns a 3-step plan instead. The reason is legitimate on
both sides. After the announcement the chief's accessibility from the designated
world is *empty*: the chief believed `(at c1 bay1)`, and no surviving event
pairs with that belief. plank leaves the model non-serial, where `[chief]φ` is
vacuously true and `C_All (at c1 bay2)` therefore holds. Aletheia enforces
KD45 seriality (`serial_core` in `src/product_update.cpp`), deletes the
non-serial worlds, empties `W*`, and prunes the action as `NonSerial`, i.e. it
treats a public announcement that contradicts an agent's belief as inapplicable
rather than as belief revision.

This is a semantic choice, not a crash, but it changes what "solvable" means for
any KD45 benchmark with belief-contradicting announcements, so plans validated
with plank are not automatically plans Aletheia can find (and vice versa).

`problem_4` is the sharp version of the same divergence: its initial model
*already* contains the chief's false belief, so the only repair is an
announcement the chief believes false. plank validates the one-step plan
`broadcast_rob1_c1_bay2` as `true`; Aletheia reports "No solution within depth 2"
(and keeps deepening past 2 minutes if left unbounded). Any belief-revision
semantics added to Aletheia should use this instance as its first test.
