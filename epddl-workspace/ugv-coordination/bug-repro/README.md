# aletheia: conjunctive goal over two atoms accepted too early

**Status:** open, found 2026-08-12 by the `facility` benchmark family.

## Symptom

With a goal that conjoins knowledge requirements over **two distinct atoms**,
AO\* returns a policy whose leaf is a *private* sensing action, although the
goal requires the other agents to know as well. Only the conjunct for the
most-recently-sensed atom is satisfied at that leaf.

Affected benchmark instances (4 of 204):
`facility-a{2,3}-b{4,5}-i2-depot-ck`.

## Repro

```sh
LIB=~/plank/benchmarks/libraries/intermediate.epddl
D=../domains/ugv-facility.epddl

plank export -d $D -p two-atom-goal.epddl -l $LIB -o /tmp/rp
~/Projects/aletheia/build/epistemic_planner --task /tmp/rp/two-atom-goal.json \
    --plan /tmp/rp/plan.json
```

2 agents, 3 bays, 2 items, 4 worlds. Goal:

```
([C. All] (and ([Kw. R1] (item-at o1 b2)) ([Kw. R2] (item-at o1 b2))
               ([Kw. R1] (item-at o2 b1)) ([Kw. R2] (item-at o2 b1)) ))
```

Returned policy (depth 4, reported `validator] OK`):

```
move_R1_b1_b2 -> scan_R1_o1_b2 -> broadcast_R1_o1_b{2,3} -> scan_R1_o2_b2  [leaf]
```

The final `scan` is private (`(default Oblivious)`), so R2 neither sees the
reading nor knows a scan occurred; its accessibility class still contains
worlds disagreeing on `o2`, and `[Kw. R2] (item-at o2 b1)` must fail.

## Evidence it is a planner fault, not an encoding fault

1. **plank disagrees.** On the same history, plank's validator reports the goal
   unsatisfied:
   ```sh
   plank validate -d $D -p two-atom-goal.epddl -l $LIB \
       -a move_R1_b1_b2 scan_R1_o1_b2 scan_R1_o2_b2      # -> false
   ```
2. **Control instance.** `one-atom-goal-control.epddl` is the identical model
   (same 4 worlds, same actions) with a goal naming only `o2`. It is solved
   correctly — the policy ends in `broadcast`. So neither the 4-world model nor
   the domain is at fault; the conjunction over two atoms is.
3. **Every other family is clean.** All 190 corridor and addressed-radio
   policies end in a communication action, as does every single-item facility
   policy. A scan of all 196 returned plans finds this leaf shape in exactly
   the 4 multi-item instances.

## Where to look

`satisfies()` and the formula evaluator (`src/state.cpp`, `src/formula.cpp`)
read correctly on inspection: `Kw` is `[i]φ ∨ [i]¬φ`, `satisfies` is
`designated ⊆ sat(f)`, and interning is keyed on kind/atom/agent/group/children.
The goal test is the same `state.satisfies(*task.goal)` call everywhere. That
leaves the *state* the AO\* layer evaluates — the split-and-contract step at
`src/search.cpp:296-308` (`product_update_split` followed by `bisim_contract`)
— as the prime suspect: if a branch state loses the null-event copies that an
oblivious agent's relation needs, that agent falsely appears to know.

Not verified: the above is a hypothesis from reading, not a bisected root cause.
A quick discriminating test would be to evaluate the goal on the branch state
before and after `bisim_contract`.
