# replanning-news

A courier must reach a resident through one of three fire doors, at least one
of them open, crossing only a door it knows to be open. A scout can see the
middle door from the kitchenette's hatch; a cleaning robot can only speak. The
domain is `replanning-news.epddl`; the problems are written with explicit
Kripke models by `ros2_ws/src/replanning_news_demo/tools/model.py`:

| problem | agents | worlds | news, announced into the model | worlds after |
| --- | --- | --- | --- | --- |
| `act1` | c | 7 | `(not open-e)` | 3 (`act1-news`) |
| `act2` | c, s | 7 | `open-m <-> open-e` | 3 (`act2-news`) |
| `act3` | c, s, k | 14 | `(Kw k open-e)` | 7 (`act3-news`) |

In `act3` each valuation appears twice, once where the cleaner saw the east
door on its round and once where it did not; no atom records the difference,
so the news removes worlds and no valuation.

Aletheia's AO* (`--conditional --consistent-beliefs`) solves the three at depth
7, and the restricted problems at depth 5, 5 and 6.

```sh
plank export -d replanning-news.epddl -p instances/act3.epddl -l intermediate.epddl -o OUT
epistemic_planner --task OUT/act3.json --plan OUT/plan.json --conditional --consistent-beliefs
```
