# gossip-coverage

Bramblett and Bezzo's team out of radio contact, "Epistemic planning for
multi-robot systems in communication-restricted environments" (Frontiers in
Robotics and AI 10:1149439, 2023), as an EPDDL task: the decision a robot
makes when it has found a task that needs several robots.

Every robot tracks its particle of the common belief, so it stays where the
others believe it to be (the paper's empathy). A robot leaves its particle
only for work it has been given, and goes back to it when the work is done.
Whether a teammate is away on other work, `(busy ?r)`, is known to that
teammate alone.

The finder knows of the task, `(task)`; nobody else does. The task needs
`(need k)` robots at its site together. A robot goes to the site, or tells
another, only when it knows of the task, `([?r] (task))`. Telling takes being
where the other is: `call ?i ?j` takes `i` to `j`'s particle, and `j` is
there and hears it (the met event, which requires `(task)`, so `j` learns
it) or is busy elsewhere (the absent event, by which `i` learns that). A
robot that knows a teammate busy may wait at its particle for it to come
back, `await`, or `leave`. A robot calls only teammates whose cells in the
Voronoi partition of the common belief border its own, `(near ?i ?j)`, the
paper's eq. (9): news for a robot further away goes from neighbour to
neighbour. The counter `(c0)` to `(c8)` is how many robots have gone to the
site (`go-site-n`); the arrival that makes as many as the task needs does it
(`do-task-k`).

```
(:goal (done))                                     ; Case II
(:goal (and (done) ([r1] (task)) ... ([rN] (task))))  ; Case I: every robot learns of it
```

The paper's statuses are facts: exploring is a robot at its particle, neither
`out` nor `sited`; gossiping is `(out ?r)`, `(by ?r ?j)`; completing a goal is
`(sited ?r)`; and its action library, perceive and announce, is the call.

```sh
plank export -d gossip-coverage.epddl -p instances/sim_case2.epddl \
  -l ~/plank/benchmarks/libraries/intermediate.epddl gossip.epddl -o out
epistemic_planner --task out/sim_case2.json --plan out/policy.json \
  --strategy replan --subjective auto --consistent-beliefs
```

`--subjective auto` makes every action branch on what its actor observes,
which is the paper's joint policy: "robots need to map indistinguishable
epistemic states to the same actions". AO* finds the same depths for teams of
up to four and does not reach the depths of larger teams; the replanning
strategy plans every instance of the demonstration in under 0.15 s.

## Instances

`instances/` holds the four situations of the paper, written from their
geometry by `ros2_ws/src/gossip_coverage_demo/tools/instances.py`:

| instance | the paper | robots | need | busy |
| --- | --- | --- | --- | --- |
| `hardware_case1` | Fig. 10, laboratory | 3 | 2, everyone told | none |
| `hardware_case2` | Fig. 11, its second task | 3 | 3 | at most 2 |
| `sim_case1` | Fig. 5, simulation | 5 | 3, everyone told | none |
| `sim_case2` | Case II, simulation | 8 | 3 | at most 2 |

The demonstration writes one instance per task found, from what the finder
knows (`gossip/epddl.py`, `gossip/epistemic.py`).

## Who observes what

`gossip.epddl` holds the two action types, a copy of belief-empathy's
`hailing.epddl` under its own name: a teammate out of contact is `Unsure` of
what another robot does. The paper's eq. (18) says that "disconnected robots'
knowledge is not impacted", which read literally is the oblivious observer of
`variants/gossip-oblivious.epddl`. Under it a robot out of contact drops the
actual world from what it considers possible at the first call it does not
see, "knows" that the robot called does not know of the task, and believes a
contradiction once the news reaches it from a robot it believed ignorant, or
once it is waited for after an absence it did not see. None of the four
canonical instances has a policy with consistent beliefs under that reading
(the arrival that does the task is a third cause: its robot believes the others
never arrived, since in this domain a robot at the site does not watch the
others arrive). `tools/oblivious.py` and `tools/validate.sh` show both.

## Files

- `gossip-coverage.epddl`: the domain, for teams of up to eight.
- `gossip.epddl`: the action types `unseen-ontic` and `hail`.
- `instances/`: the four canonical instances.
- `variants/gossip-oblivious.epddl`: the oblivious reading, for comparison.

The kinematic simulator, the Gazebo package and the checks are in
`ros2_ws/src/gossip_coverage_demo`; the page is
<https://hanielulises.github.io/Epistemic-Robotics/gossip_coverage.html>.
