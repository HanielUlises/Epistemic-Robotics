# watcher

A carrier `r1`, an ally `r2` and a watcher `r3`. The goal is that the load be
in the bay the order names, that the ally know which bay, and that the watcher
not:

```
(and (or (in-x r1) (in-y r1)) ([Kw. r2] (dest-x)) (not ([Kw. r3] (dest-x))))
```

Four worlds, one per order (bay X or Y) and watcher's post (west or east). What
the watcher observes of the gate and of each window is an observability
condition on its post: fully if it is on that side, otherwise oblivious. No
sequence of actions meets the goal in more than two of the four worlds; the
policy reads the duty board first (`peek`) and tells the ally on the side the
watcher is not.

```sh
plank export -d watcher.epddl -p instances/problem_1.epddl \
  -l ~/plank/benchmarks/libraries/intermediate.epddl sightlines.epddl -o out
epistemic_planner --task out/problem_1.json --plan out/plan.json --conditional --consistent-beliefs   # depth 5, 68 expanded
epistemic_planner --task out/problem_1.json --plan out/seq.json --gbfs --no-helpful  # exhausts
```

`sightlines.epddl` adds the quasi-private ontic action type: two designated
events with effects and a null event. It is not called `watcher`: plank stops
with an unknown exception when a library and its domain share a name.

`watcher-sightlines.epddl`, with `instances/problem_sightlines.epddl`, is the
same domain with every observability condition set to what
`tools/check_views.py` finds: what the boards say seen only by the carrier, the
back route by nobody, the drive to a window only from that side. Aletheia
returns the same policy in the same 68 expansions with `--consistent-beliefs`,
and under these declarations it is opaque to a watcher that knows it and stays
at its post (`ros2_ws/src/watcher_demo/tools/opacity.py`, step 6 of
`tools/validate.sh`). The reading of the boards stays semi-private: declared
unseen by the ally, the ally's beliefs collapse at the lamp and its knowledge
holds only vacuously, which `--consistent-beliefs` refuses.

The executor in `ros2_ws/src/watcher_demo` runs it in Gazebo; the page is
<https://hanielulises.github.io/Epistemic-Robotics/watcher.html>.
