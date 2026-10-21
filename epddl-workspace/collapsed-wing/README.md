# collapsed-wing

Two scouts, `s1` and `s2`, and a medic `m`. The medic must reach a victim in
the north wing of a building, by one of three ways: the west doorway, the east
doorway, or the middle passage between them. Rubble may block any of the
three, at least one is clear, and nobody knows which: seven worlds, all
designated.

```
(:goal (with-victim m))
```

Each crossing requires that the medic know the way is clear, `([m] (clear-w))`,
and not that it be clear. `s1` can reach the east doorway and `s2` the west; a
scout at its doorway decides it (`inspect`, semi-private sensing: the others
see it look, not what it saw) and can say what it knows over the radio
(`tell`, public sensing, with a `[Kw.]` part shared by both events so that
ePlanSys's executor can dispatch it). Nobody can stand where the whole middle
passage is seen, so whether it is clear can only be known by elimination.

```sh
plank export -d collapsed-wing.epddl -p instances/problem_1.epddl \
  -l ~/plank/benchmarks/libraries/intermediate.epddl -o out
epistemic_planner --task out/problem_1.json --plan out/plan.json --conditional --consistent-beliefs
```

Aletheia's AO* returns, at depth 7 after 424 expansions, a policy with three
leaves. Both scouts drive to their doorways first; `s1` looks at the east
doorway and says what it found. Clear, the medic crosses there. Blocked, `s2`
looks at the west doorway: clear, it says so and the medic crosses there;
blocked, `s2` says the middle passage is clear, which it knows from what it
saw, what it heard `s1` say, and that at least one way is clear, and the medic
crosses the passage nobody has seen.

The executor in `ros2_ws/src/collapsed_wing_demo` runs it in Gazebo against two
classical formulations; the page is
<https://hanielulises.github.io/Epistemic-Robotics/collapsed_wing.html>.
