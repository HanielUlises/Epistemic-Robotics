# v2: contamination with a location

**This is now built.** The demo is `survey_sites_launch.py`, the domain is
`epddl/survey-sites.epddl`, and the package README describes what it does. This
file is kept as the design note it was: the measurements below are the ones
that decided the shape of it, and the four things it said a v2 would have to
build are the four things that were built.

What follows is the note as written, with the outcome of each item marked.

## The question

The demo as it ships evaluates the *execution* of an epistemic policy at
warehouse scale, and says plainly that it does not claim floor area has
anything to do with epistemic difficulty. That disclaimer is honest and it is
also structural: `(contaminated)` is a proposition with no argument, there is
one site, and the planner therefore cannot care where anything is. Floor and
epistemics sit side by side and never meet.

They meet if contamination has a location. Give the domain

```
(:types site)
(:predicates
  (contaminated ?s - site)
  (on-site ?i - agent ?s - site))
```

and say in the initial state that exactly one of *k* sites is contaminated,
which one being what nobody knows. The team must now work out *which*, and the
sites are metres apart on a roadmap derived from the world.

## What was measured

`survey-sites.epddl` is that domain, `gen.py` writes the *k*-site problem, and
`survey-sites.pddl` is the classical half the executor would drive. Grounded
with `plank`, solved by `plansys2/EpistemicPlanSolver` with `strategy=aostar`
and `heuristic=ks`, three agents throughout.

| variant | atoms | worlds | actions | AO\* depth | expanded | policy nodes |
| --- | --- | --- | --- | --- | --- | --- |
| shipping demo, propositional | 4 | 2 | 30 | 3 | 40 | 4 |
| *k* = 2 sites | 8 | 2 | 48 | 3 | 147 | 4 |
| *k* = 3 sites | 12 | 3 | 72 | 6 | 838 168 | 8 |
| *k* = 4 sites | 16 | 4 | 96 | not solved in 901 s | — | — |

*k* = 3 is solved inside the planner's default 15 s budget. It has since been
timed: the planner logs the start of the search and the solution 6.4 s apart,
so 838 168 expansions is about 131 000 a second. That figure is not a boast
about the search — it is the reason the bringup broke, since the planner and
the bringup's own lifecycle calls are served on one executor in one process.

*k* = 4 is not solved. Under the default budget it reports `[aostar] Timeout at
depth 7`; under a 900 s budget it reports `[aostar] Timeout at depth 8` after
901 s and returns no plan. Both are the budget running out, not the search
space being refuted, so no claim is made here that *k* = 4 is unsolvable. What
is measured is the price of the extra site: sixty times the budget bought one
further depth iteration. A v2 that wants four sites needs a better heuristic,
not a longer wait.

The interesting column is `expanded`. Going from one site to two costs a factor
of four. Going from two to three costs a factor of 5 700, and the plan gets
three actions longer. Whatever a v2 does, it does at *k* = 3.

## The result worth having

At *k* = 3 the policy is this, projected onto its classical actions:

```
0:  (goto relay a07)
1:  (goto scout a15)
2:  (scan scout a15)
3:  (relay scout relay a07)
3:  (relay scout relay a15)
5:  (scan relay a07)
6:  (relay relay scout a23)
6:  (relay relay scout a07)
```

Two sites are scanned, `a15` and `a07`. **No robot ever goes to `a23` and no
robot ever looks at it**, and the team still ends up knowing its state and
relaying it. With exactly one site contaminated, ruling out two settles the
third, and the derived knowledge then travels over the private channel like any
other finding.

That is a properly epistemic result and it is the argument for a v2: the team
comes to know something about a place none of its members visited, by
elimination, and the negative conjunct still holds over it. It cannot be
produced by the shipping demo at any floor size, because with one site there is
nothing to eliminate.

It is also the point at which floor geometry starts to matter. Which sites to
inspect, and in what order, becomes a decision, and the sites are transits
apart on a graph derived from the world. That is the coupling the current page
correctly says it does not have.

## What a v2 would have to build

1. **Per-site execution.** `config/warehouse_xl_survey.json` maps an action
   name to one waypoint. It would need a waypoint per site, and
   `scripts/scan_perception.py` would need to know which site it is standing
   at instead of holding one hard-coded `site_x`/`site_y`.

   *Built.* `config/warehouse_xl_sites.json` uses `waypoint_arg` and a `zones`
   table, so the bridge resolves the destination from the action's own
   argument. `scripts/site_perception.py` watches every (robot, site) pair.
   What this did not anticipate is that the bridge would need changing too:
   one performer per action name and one latched observation topic means the
   second scan is handed the first one's answer. Readings carry their site now.

2. **Three sites on the roadmap.** `tools/make_dlw_graph.py` already takes
   `--site X,Y,NAME` and can pin more than one.

   *Built, and it needed a chooser.* `tools/check_sites.py` decides what can be
   a site at all: open to a laser in an empty world, and with somewhere legal
   for the pallet to stand. Thirteen of thirty-nine aisle waypoints qualify.

3. **Three pallet positions.** `tools/with_camera.py --pallet` places one; a
   v2 needs the world built with the pallet at whichever site the run is
   testing, and the clean variant with none.

   *Built:* `tools/make_site_worlds.sh` writes all four.

4. **A mapping decision.** `draft_epistemic_mapping` resolves `goto` and `scan`
   automatically but leaves all 54 relay entries for a human, because
   `relay-dirty` and `relay-clean` both correspond to the single classical
   `relay` and that correspondence is a modelling choice.

   *Built, once.* `tools/make_sites_mapping.py` states the decision as a rule
   and applies it to all 54. The decision is one line; writing it out
   fifty-four times by hand is the same decision plus fifty-three chances to
   mistype an agent name.

5. **Not anticipated: the floor was wrong.** The world carries a saved
   `<state>` block that Gazebo applies over the declared model poses, and
   fourteen of them disagree. Correcting it moved every charger and changed the
   graph from 261 waypoints to 289. See the package README.

## Three traps found on the way

**Site names may not contain underscores.** Grounded action names are joined
with `_`, and `draft_epistemic_mapping` splits on it to recover arguments, so
`aisle_07` becomes the two arguments `aisle` and `07` and the mapping is
silently wrong. The sites here are `a07`, `a15`, `a23`; in the built demo, and
on the corrected floor, they are `a17`, `a31` and `a06`.

**And a third trap, found in the building.** Four files name the sites: the
EPDDL problem, the bridge's task map, the launch that places their pallets, and
the mission node that declares them as objects. Nothing checked they agreed.
Renaming three of the four left the executor checking an over-all condition
over a site that was not an object of the problem, which it reports fifty times
a second without ever saying that. The mission reads its sites from the EPDDL
now, and the launch refuses to start if its own list disagrees.

**`:forall` is not a goal formula.** plank rejects it in `(:goal ...)`, so the
goal is written out one conjunct per site. `gen.py` does that expansion.

## Reproducing

`survey-sites-problem-3.epddl` is the exact problem the *k* = 3 row was measured
on, with its sites named after the aisles they would stand for. `gen.py k`
writes the same problem for any *k*, with sites named `a01`, `a02`, and so on;
the two are the same problem up to renaming.

```
LIB=<plansys2_epddl_grounder>/libraries/intermediate.epddl
plank export -d survey-sites.epddl -p survey-sites-problem-3.epddl -l "$LIB" -o out
ros2 run plansys2_epistemic_planner draft_epistemic_mapping \
     -t out/survey-sites-problem-3.json -e survey-sites.epddl \
     -p survey-sites.pddl -o mapping.json
# finish the 54 relay entries by hand, substitute the three paths into
# eplansys_demo's params/survey.yaml, bring plansys2 up against
# survey-sites.pddl and request a plan through plansys2_terminal
```

Raise `plan_solver_timeout` on the *planner node* to search longer, in the
`planner: ros__parameters:` block of the params file. The same parameter on the
terminal's client governs only how long that client waits for the service to
answer; setting it there leaves the solver on its 15 s default and looks
exactly like the solver giving up early.
