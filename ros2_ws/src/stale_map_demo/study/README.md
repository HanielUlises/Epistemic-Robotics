# The communication study

When does an epistemic plan outperform a fleet that just shares its maps, and
why? This directory runs the plan and three map-sharing protocols on the same
instances, under four regimes, and measures the differences that the
propositions of the report predict.

```
python3 run_study.py --out /tmp/study --seeds 200 --workers 10    # 2400 instances, ~15 min
python3 run_study.py --out /tmp/study --seeds 20 --regimes budget  # a quick look
python3 analyze.py --csv /tmp/study/study.csv --out /tmp/study/analysis
```

The report is *When Planning Pays* (`when_planning_pays.pdf` on the site), with
its page `when_planning_pays.html`.

## The claim

With an unlimited budget on a connected radio graph, goals that mention only
the floor, and a merge rule that orders readings as they were taken, flooding
leaves every robot holding the fleet's freshest reading of every bay, and its
haulers do what they would do knowing every reading in the fleet. The plan then
saves only messages. Each condition, removed, separates the two:

| condition removed | what happens | measured |
| --- | --- | --- |
| unlimited budget | flooding sends 2\|E\|\|T\| messages before it knows who needs what; the plan one per misled hauler | 95% success at 3 to 6 messages for the plan, 96 to 512 for flooding |
| a goal over the floor alone (secrecy) | flooding meets a goal of the form ¬B_c φ only if no robot observed φ | predicted on 600 of 600 instances; the plan succeeded on all 600 |
| a goal over the floor alone (joint crossing) | no separation without a budget: a common map and a common choice rule stand in for B_h B_k | every protocol 100% unlimited; under a budget the plan 95% at 4, no protocol at 64 |
| observation (knowledge by elimination) | a reading protocol needs up to B−1 readings where one report of a belief suffices | median 1 to 2 messages against 224 to 336 |
| a common clock | recency inverts two readings when the clock offset exceeds the time between them | recency falls to 45% at σ = 40 s; merging by version stays at 100%, with no planner |

## Files

| file | what it does |
| --- | --- |
| `instances.py` | the instance generators: `budget`, `secrecy`, `joint`, `elimination`, `skew` |
| `domain.py` | writes an instance as EPDDL: the change actions with their observers, map reports with the `B_i B_j` guard, solo and joint crossings; S5 private announcements for elimination |
| `epistemic.py` | grounds with plank, solves with Aletheia (`--consistent-beliefs`), replays the policy through the product update, and shortens it by action elimination |
| `protocols.py` | flood, pull and gossip over readings with a stamp, a version and an observer; the four merge rules; what the haulers then do |
| `run_study.py` | the cells, every strategy on every instance, one CSV row per instance, strategy, rule, budget and clock deviation; stops if a plan that succeeds in the model fails on the readings |
| `analyze.py` | Wilson intervals, exact McNemar and Wilcoxon signed-rank tests on paired instances; `tables.md`, `summary.json` and the five figures |

An instance is kept only if the shift fails with no message. Every strategy runs
on the same instances, so every comparison is paired.

## On the floor

`tools/stale_maps.py` writes a fifth floor, `secret`: the radio floor with r4 a
contractor's robot that hauls and must end the shift not believing `t1`
blocked. `tools/fleets.py` computes, with `protocols.py`, the sends of three
protocols on that floor, and the mission runs them in order through the relay:

```
python3 tools/fleets.py --fleets secret flood flood3 pull
ros2 launch stale_map_demo stale_maps_launch.py fleet:=secret      # also flood, flood3, pull
```

| fleet | messages | delivered | contractor sent the secret |
| --- | --- | --- | --- |
| flood | 189 | r2, r4, r8 | yes, by r2 |
| flood3 | 3 | none | no |
| pull | 126 | r2, r4, r8 | yes, by r1 |
| **secret** (the plan) | **3** | **r2, r4, r8** | **no** |

The mission fails a run that does not deliver the predicted haulers, send the
predicted number of messages, or leak exactly when the prediction says.

## An earlier pull

The first run used a pull that stopped asking at the first bay a hauler then
believed open. A staging nobody saw leaves that bay open on every map, so it
failed where flooding did not. The pull here asks about every bay.
