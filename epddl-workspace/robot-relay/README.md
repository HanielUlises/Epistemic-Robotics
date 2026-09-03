# robot-relay

A fleet of couriers moving one package across a facility, as a KD45ₙ
(belief) epistemic planning domain — where robot-warehouse is S5ₙ.

## Why KD45, not S5

robot-warehouse asks one question — which of two bays holds the pallet — and
answers it once, by looking. Every robot's belief there is always *true*:
inspecting only narrows an open disjunction, nothing a robot comes to
believe is ever contradicted by what actually happened, and the domain is
finitary S5 throughout.

This domain starts the same way — the package is in one of two bays and a
robot has to look — but does not stop there. Once found, the package is
*carried*, hop by hop, across the floor to a depot at the far end, and
carrying is where the frame stops being S5. Nothing about the carry is
fleet-wide by default: the courier's pickup, every leg of the relay, and the
drop at the depot are each witnessed only by whoever happens to already be
standing where it happens. A robot stationed anywhere else keeps believing
the package is exactly where it last saw it — nowhere, at the start — and
once the courier moves, that belief is not merely stale, it is *false*: the
accessibility relation stops being reflexive at the world the courier is
actually in. The model is KD45ₙ, not S5ₙ, from that leg on, and stays that
way until something tells the idle robot otherwise.

Only one robot per instance is on courier duty (`:fact courier`, checked by
`go`, `pickup`, `relay` and `deliver`). Every other robot is present, mobile
in principle, able to sense and talk — just never assigned to carry, and in
these instances never moved at all. That is deliberate: an early draft left
idle robots free to walk, and the search promptly worked out that walking to
wherever the courier was headed and simply standing on the near end of one
relay hop was enough to learn everything for free, by the same mechanism
that makes relay private in the first place. Nothing wrong with that as a
strategy — it is a legitimate way to gather information in this domain — but
it defeats the point of an instance built to show that broadcast is
*necessary*. Pinning the idle robots down is what keeps the demonstration
honest.

## The toolbox

| action | type | role |
|---|---|---|
| `go` | `public-ontic` | the courier's own movement; fleet-wide, ordinary telemetry |
| `scan` | `private-sensing` | fully private origin check, nobody else even notified |
| `inspect` | `semi-private-sensing` | origin check the fleet sees happen, not the outcome |
| `pickup` | `private-ontic` | need-to-know: only robots already at the bay see it |
| `relay` | `private-ontic` | **creates** the false belief, one hop at a time |
| `deliver` | `private-ontic` | need-to-know drop-off; `(delivered)` is ontic, not a belief goal |
| `broadcast` | `public-announcement` | repairs everyone's belief in one action |
| `radio` | `private-announcement` | repairs one named robot's belief, nobody else hears it |

`broadcast` and `radio` are truthful: their positive event requires
`(and (package-at ?z) ([?i] (package-at ?z)))` — the speaker must believe
what it says, and it must be true.

## The floor, as a graph

```
   home            everyone starts here; the courier's first move leaves it
    |               for good, and no location-gated event ever touches it
   lane            the service lane, the one path between everything else
   /  \
 bayA  bayB        two candidate origins, one mouth each, no edge between them
   \
   lane -- relay1 -- relay2 -- depot     the carry, three private hops long
```

`home` is where every idle robot in these instances is left standing: it is
never the source zone of a pickup, a relay hop, or a delivery, in either
branch. That is what makes `home` a safe place to plant a bystander whose
belief you want to stay untouched by everything except an explicit message.

## Running it

```bash
bash validate.sh
```

Needs [plank](https://github.com/HanielUlises/plank) for parsing, type
checking, grounding and validation, and the epistemic planner for the
search. Both are found on `PATH`, or through `PLANK` and
`EPISTEMIC_PLANNER`.

It runs six things, in order: one robot, two, three, four, five — the
empirical ceiling for a demo-length run, not a round number picked in
advance — and then two plans that must be rejected. A scaling table closes
it out.

### 1. One robot

r1 finds the package and delivers it, alone. The goal is the same shape as
robot-warehouse's solo instance — `(delivered)` and `[Kw. r1] (package-at
bayA)` — and for the same reason: r1 is never oblivious to its own actions,
so the KD45ₙ frame has nothing to diverge *from* yet. This instance exists
to show the domain reduces to something ordinary with one agent, before the
next one shows what a second agent changes.

### 2. Two robots

r2 waits at home the whole shift. Unlike robot-warehouse, where a distant
robot learns the origin bay for free by watching a public pickup, nothing
here reaches r2 without somebody sending it: pickup is need-to-know, every
relay hop is need-to-know, delivery is need-to-know. The accepted plan
inspects, then **broadcasts the origin bay before the package ever leaves
it** — broadcasting later would mean announcing a location the package is no
longer at, which is exactly why the search finds this ordering and not some
other one, not because it was told to.

### 3 and 4. Three and four robots

Same shift, one or two more idle robots at home, wanting the same fact.
Reaching each one with its own `radio` call would cost one action per
listener; the fleet channel — `broadcast` — reaches all of them in the same
single action it took for r2 alone. The plan does not grow with the fleet;
the model the planner has to search to be sure of that does.

### 5. Five robots

As massive as this AO* build finishes in a demo-length run. Five robots
solved in the time this script gives it; six was tried in testing and left
running well past the point a validate.sh run is worth waiting on for a
demo. The scaling table at the end of the script is where that cost is
actually visible — see below.

### Two plans that must be rejected

**A sequence, where a policy is needed.** `inspect`, then `pickup_r1_bayA`
regardless of what it found. After `inspect` the model still designates both
bays open, and `pickup_r1_bayA` is applicable in only one of them — `plank`
reports it not applicable, the same rejection robot-warehouse demonstrates
for its own domain.

**Picking up without ever sensing.** `pickup` carries a modal precondition —
a robot may lift the package only where it *believes* the package is there —
and going straight there without inspecting or scanning fails it. Drop that
conjunct from the domain and a robot that happens to be standing in the
right bay picks up a package it has no reason to believe is there, which is
the classical domain this one is deliberately not.

## What scaling actually costs here

robot-warehouse's own scaling section makes a point of what an extra robot
*doesn't* cost: the plan stays the same shape, because a public pickup
informs the whole fleet whether the planner spends an action on it or not.
This domain cannot make that claim. Every idle robot really does hold a
false belief about the package's location until it is told, so `broadcast`
is load-bearing at every fleet size in the table above — never a robot-
warehouse-style action the search declines to use. What grows instead is the
cost of being *sure*: checking a modal goal for one more agent means
checking it against one more relation over the same set of worlds, and
`validate.sh` prints exactly how that compounds, run over run, agents up to
5 against the search nodes it took.

## The files

| file | what it is |
| --- | --- |
| `robot-relay-domain.epddl` | the domain: driving, sensing, carrying, delivering, telling |
| `instances/problem_1.epddl` | one robot |
| `instances/problem_2.epddl` | two -- the idle one needs telling |
| `instances/problem_3.epddl` | three -- one broadcast reaches both idle robots |
| `instances/problem_4.epddl` | four |
| `instances/problem_5.epddl` | five -- the ceiling for a demo-length run |
| `validate.sh` | grounds, solves and reads out all five, then the two rejections, then the scaling table |

## Reproducing by hand

```sh
L=~/plank/benchmarks/libraries/intermediate.epddl
plank parse    -d robot-relay-domain.epddl -p instances/problem_2.epddl -l $L
plank ground   -d robot-relay-domain.epddl -p instances/problem_2.epddl -l $L
plank export   -d robot-relay-domain.epddl -p instances/problem_2.epddl -l $L -o out
plank validate -d robot-relay-domain.epddl -p instances/problem_1.epddl -l $L -a \
  "go_r1_home_lane" "go_r1_lane_bayA" "pickup_r1_bayA"   # false: no belief basis yet
```

The exported tasks are consumed directly by the epistemic planner:

```sh
epistemic_planner --task out/problem_2.json --plan out/problem_2-plan.json --conditional
```
