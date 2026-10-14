# watcher-mobile

The watcher of `../watcher/`, which may now change posts. The carrier `r1`
must still take its load into the bay its order names, let the ally `r2` know
which, and not let the watcher `r3` know:

```
(and (or (in-x r1) (in-y r1)) ([Kw. r2] (dest-x)) (not ([Kw. r3] (dest-x))))
```

The watcher has K changes of post. Every drive of the carrier is an action
with one event per thing the watcher may do while it lasts, stay or go to the
other post; only the watcher tells them apart. The carrier can look down the
hall it stands in.

```
make_domain.py --moves K --out DIR [--no-gates]   # DIR/watcher-mobile.epddl, sightlines-mobile.epddl, instances/
gates/      K = 1, with the gates     (the shortest policy, executed as "shortest")
opaque/     K = 1, without the gates  (the opaque policy, executed as "opaque")
tools/      executable.py, opacity_sightlines.py, plan_opaque.py, study.py
```

Three things this domain turned up.

**Policies must branch on what their actor observes.** Aletheia's AO* gives
one branch per designated event. After a drive, that is one branch for "the
watcher stayed" and one for "it moved", which the carrier cannot tell apart: a
policy no robot can follow (`tools/executable.py` finds two such branch points
for K = 1). Aletheia's branch `feature/subjective-branching` adds
`--subjective auto`: events the action's actor cannot tell apart share one
branch. With it the planner finds the carrier looking down the hall before
it tells, and switching windows when the watcher has come. Without the
option, the planner is unchanged: same results on its 28 benchmarks.

**A policy that is correct for the product update can still give the order
away to a watcher that knows the policy.** The shortest policy, with the
gates, goes to the gate approach of its bay or to the far window depending on
the bay; a watcher that knows that reads the bay off where it sees the carrier
(`tools/opacity_sightlines.py`, for every K). Without the gates the carrier's
visible choices depend on where the watcher is and not on the order, and the
policy is opaque for K = 0..3.

**The boards' reading must not be hidden from the ally.** Declared oblivious,
the ally holds the lamp impossible and its beliefs collapse; Kw r2 holds only
vacuously. Every plan here is made with `--consistent-beliefs`.

`tools/study.py` writes the planning study, K = 0..3 with and without gates,
objective against subjective branching, into
`artifacts/watcher-moves/planning.json`; `ros2_ws/src/watcher_moves_demo`
executes the K = 1 instances in Gazebo against a watcher that stays and one
that chases.
