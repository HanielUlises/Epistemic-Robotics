# Belief revision in a changing ward: an EPDDL domain with ranked worlds

Written by `ros2_ws/src/belief_revision_demo/tools/model.py`; do not edit by hand.

Four fire doors close four lobbies through the band between a hospital's hall
and its corridor. At one in the morning a porter nobody sees shuts the two
doors of the loop he walks; at two he opens them again, unless he is held up.
The couriers know every loop is possible and rank them by the porter's habits.

Every world carries its Spohn rank as a grade atom `g0`..`g6`, fixed in the
problem and never changed by an action. With `m` the least grade among the
worlds agent `r` cannot rule out, belief is the formula

```
B_r phi       = OR_k ( [r](not g_0 and ... and not g_{k-1}) and <r>g_k and [r](g_k -> phi) )
not B_r not phi = OR_k ( [r](not g_0 and ... and not g_{k-1}) and <r>(g_k and phi) )
```

so a look or a report, which removes worlds, revises belief exactly as Spohn's
conditioning does. Every relation stays an equivalence. The second formula
guards `enter-<d>-<side>` (a courier enters a lobby only if it does not believe
its door shut) and `seek-<p>-<q>` (only towards a door it does not believe
shut); `cross-<d>-<side>` requires `[r] open-<d>`.

The porter's second round is an ontic action every courier sees happen,
`round`, whose effect depends on an atom `reopens` nobody observes: the
nondeterminism of what he does at two is fixed at dusk and carried in the
worlds, with its rank added to the loop's.

## Files

```
belief-revision.epddl           the domain
instances/act<k>-<f>.epddl      act 1 (one courier), 2 (two), 3 (two, there and back), ranked by
                                f = explanation, door-by-door, full-meet or knowledge
kd45/                           the control: act 1 with the courier's relation pointing at the
                                rostered night only, a KD45 belief without ranks
```

## Reproducing

```sh
L=~/plank/benchmarks/libraries/intermediate.epddl
plank export -d belief-revision.epddl -p instances/act1-explanation.epddl -l $L -o out
epistemic_planner --task out/act1-explanation.json --plan plan.json --conditional --consistent-beliefs
                                # Aletheia: a policy of depth 20
plank export -d kd45/belief-revision-kd45.epddl -p kd45/act1-kd45.epddl -l $L -o out
epistemic_planner --task out/act1-kd45.json --plan plan.json --conditional --consistent-beliefs
                                # Aletheia: search space exhausted, no policy
```

The policies the demonstration executes are planned by
`belief_revision_demo/tools/plan.py`, which minimises expected time with
worlds weighted by their ranks, and checked by `tools/policy_check.py`.
