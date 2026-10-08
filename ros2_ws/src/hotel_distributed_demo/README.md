# hotel_distributed_demo

A leak in the Open-RMF hotel that no robot can place alone, and an all-clear
that names no room and would still tell the guest which one.

Water is coming through from one of four guest rooms: room 1 or room 15, on
L2 or L3. On its night round the cleaner found it coming down one of the two
risers, so it knows the column and not the floor. The building's flow alarm
reports to the front desk, so the concierge knows the floor and not the
column. The porter carries the valve key and is the only one of the three that
can ride a lift; it knows neither. A guest is in the lobby. The room is
distributed knowledge of the cleaner and the concierge, and the robot that has
to act on it knows nothing.

The hotel is the stock `rmf_demos` hotel, its three floors, three fleets and
two lifts unchanged: the cleaner is `cleanerBotA_1`, whose fleet's graph has
no upper floor; the concierge `tinyBot_1`; the porter `deliveryBot_1`, whose
graph has no lobby vertex on the ground floor, so that the restaurant is the
only place all three fleets reach. Robots of different fleets share no
network. A robot can tell another only by being where it is, and then
everyone standing there hears it; the public address reaches everyone,
the guest included.

```
ros2 launch hotel_distributed_demo hotel_distributed_launch.py                      # the epistemic fleet
ros2 launch hotel_distributed_demo hotel_distributed_launch.py fleet:=filter
ros2 launch hotel_distributed_demo hotel_distributed_launch.py fleet:=broadcast
ros2 launch hotel_distributed_demo hotel_distributed_launch.py fleet:=siloed leak:=L2_room15
bash tools/validate.sh                                                              # eight checks, no simulator
python3 tools/hotel_domain.py --out /tmp/hd --executor                              # the domain and every fleet
python3 tools/trace.py --task /tmp/hotel_distributed_validate/filter/filter.json \
    --whole /tmp/hotel_distributed_validate/epistemic/epistemic.json \
    --plan /tmp/hotel_distributed_validate/filter/plan.json --leak L3_room1
python3 tools/reach.py --task /tmp/hotel_distributed_validate/siloed/siloed.json \
    --whole /tmp/hotel_distributed_validate/epistemic/epistemic.json
python3 tools/scaling.py --cells 2x2 2x3 3x2 --out /tmp/hotel_scaling
```

## The goal, and four fleets

The whole goal has three conjuncts:

| conjunct | formula | in words |
| --- | --- | --- |
| safe | `safe` | the leak is contained |
| stand-down | `E_R E_R safe`, R = cleaner, concierge, porter | every responder knows that every responder knows it |
| secret | `<guest> source(z)` for every room z | the guest cannot rule out any room |

`tools/hotel_domain.py` writes one domain and one problem per fleet. The
fleets run the same planner over the same building; they differ only in the
channels they use and the goal they plan for:

| fleet | channels | plans for | stands for |
| --- | --- | --- | --- |
| `epistemic` | in person, public address | the whole goal | the planner given the hotel as it is |
| `broadcast` | in person, public address | safe and stand-down | a fleet that does not model the guest as someone to keep in the dark |
| `filter` | in person, all-clear on the public address | safe and stand-down | a rule that keeps any message naming a floor or a column off the public address |
| `siloed` | none between fleets | safe | Open-RMF as deployed: fleets share the traffic schedule and nothing they observe |

Every run is judged by the whole goal, at the actual world, whatever its fleet
planned for.

## What the planner returns

AO* at one thread, the hotel's four rooms:

| fleet | depth | leaves | expanded | time | lift rides | rooms opened | messages, in person + public | safe | stand-down | secret |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- | --- | --- |
| epistemic | 7 | 4 | 387 003 | 157 s | 2 | 0 | 3 + 0 | 4/4 | 4/4 | 4/4 |
| filter | 6 | 4 | 105 519 | 28 s | 1 | 0 | 2 + 1 | 4/4 | 4/4 | 0/4 |
| broadcast | 5 | 4 | 26 532 | 7 s | 1 | 0 | 0 + 3 | 4/4 | 4/4 | 0/4 |
| siloed | 8 | 4 | 394 987 | 8 s | 2 | 3 | 0 + 0 | 4/4 | 0/4 | 4/4 |

Rides, rooms opened and messages are the most along any branch; the last
three columns count the leaves at which each conjunct holds, by
`tools/trace.py`.

The epistemic policy, for every room:

```
go(concierge, lobby, restaurant)
tell-column(cleaner, restaurant)           e-col-room1 | e-col-room15
  tell-floor(concierge, restaurant)        e-floor-L2  | e-floor-L3
    go(porter, restaurant, <room>)
    contain(porter, <room>)
    go(porter, <room>, restaurant)
    tell-safe(porter, restaurant)
```

The filter's is the same up to the containment, and ends with `page-safe`
from the room: one action shorter, and one lift ride fewer.

## Why

* **The concierge leaves the lobby.** What is said in the lobby the guest
  hears; told there, the floor halves the rooms the guest considers.
* **The porter rides back down.** An all-clear over the public address names
  no room, and still tells the guest which room it was: the guest saw the
  porter ride to it, a containment needs the porter in the leaking room, and
  in the model after the containment the worlds where it happened are
  exactly the worlds where that room is the source. Announcing that it
  happened leaves only those. This is why the content filter fails, at every
  leaf, and why no fleet with the public address alone has any policy for the
  whole goal.
* **Nobody is told in the siloed fleet.** The cleaner cannot reach a room,
  so it never sees a containment, and without a channel nothing else it
  observes depends on one. It never knows the leak is contained.

`tools/reach.py` settles the two fleets the planner cannot: it walks every
model reachable by any action and any outcome. With no channel, 1 500 models
and the stand-down holds in none; with the public address alone, 4 220 and
the whole goal holds in none. The same search on the epistemic fleet finds
the whole goal reachable.

## On the floor

* **Setup**, before planning: the concierge goes to the lobby, the porter to
  the kitchen and the cleaner to its dock in the restaurant, by Open-RMF tasks,
  one after another.
  The model starts there. The guest is a figure in the lobby, and the water a
  pool in the leaking room; neither collides and no robot sees either.
* **Moves** are Open-RMF tasks through `eplansys_rmf_bridge`, a ride between
  floors included. The concierge and the porter cannot share the restaurant's
  waypoint, so the porter waits at the kitchen's, through the same room.
* **Speech acts** are the bridge's local actions. What the cleaner says of the
  column and the concierge of the floor, and what an inspection finds, the
  bridge takes from the leak the launch names, and says so in the log.
* **Stand-down**, after the policy: the cleaner and the concierge each do
  what they know at the actual world. One that knows the leak is contained
  stands down, the cleaner to its charger and the concierge to the desk; one
  that does not stays where it is.
* **The verdict** is the knowledge view's, at the actual world, and the
  mission completes only if it is the verdict the analysis gives that fleet.

## The recorded runs

All four fleets, the leak in L3 room 1, recorded with `tools/record_demo.sh`.
Every figure is read from the run's log; times are wall clock while recording,
with Gazebo rendering in software at a real-time factor between 0.5 and 0.8.

| fleet | planned in | lift rides | rooms opened | told, paged | policy run | verdict |
| --- | ---: | ---: | ---: | --- | ---: | --- |
| siloed | 13 s | 2 | 3 | 0, 0 | 895 s | stand-down fails: the cleaner and the concierge do not know, and stay |
| broadcast | 8 s | 1 | 0 | 0, 3 | 327 s | secret fails: the guest knows the room from the second page |
| filter | 39 s | 1 | 0 | 2, 1 | 588 s | secret fails: the guest knows the room from the all-clear |
| epistemic | 229 s | 2 | 0 | 3, 0 | 1058 s | the whole goal |

Each mission completed with the verdict the analysis gives its fleet. In this
world the epistemic run is not the fastest; what it buys is the goal, and the
rooms the siloed fleet opened.

## Found on the way

* **A telling the executor would not dispatch.** The executor checks the modal
  part of every designated event's precondition as a requirement of the
  action. A telling whose events require knowing different values could never
  pass. Each event now requires its value and that the speaker knows which
  value holds, the same for every event, which is equivalent given that
  exactly one value holds.
* **A verdict over too few rooms.** After the all-clear the model has one
  world, and the knowledge view read the rooms off it, so the secret held of
  the one room left. The mission expected the secret to fail for the filter
  and refused the run; the view now keeps every room the run has had.
* **Two robots in one corridor.** Sent to the restaurant at once, the porter
  and the cleaner deadlocked in Open-RMF's negotiation for twenty minutes. The
  crew now sends the robots one after another.
* **plank.** Domain constants are numbered ahead of the agents, which breaks
  observability conditions that name an agent; a library's relation
  variables are substituted textually, so `?f` in a library captured an
  action parameter `?f`; and `[C. group]` over a declared group parses and
  does not ground. Values have their own atoms, the library uses `?x` and
  `?y`, and the goal asks for E_R E_R.

## Files

| file | contents |
| --- | --- |
| `tools/hotel_domain.py` | the domain, the earshot library and every fleet's problem for F floors and K columns, with the action mapping and the classical model |
| `tools/trace.py` | the product update, after `coordinated_attack_demo`'s, what every agent knows after every action, and each leaf judged by the whole goal |
| `tools/reach.py` | every model a fleet can reach, for the fleets the planner cannot settle |
| `tools/validate.sh` | grounds, solves, traces and searches; eight checks, no simulator |
| `tools/scaling.py` | the planning table as the building grows |
| `tools/record_demo.sh` | one fleet on an Xvfb display, Gazebo from above, with the run's log beside it; the floor follower and the film composer it is used with are kept out of the repository |
| `scripts/knowledge_view.py` | what every agent knows at the actual world after every update, logged as `[knows]` |
| `scripts/hotel_crew.py` | the setup, the guest and the water, and the stand-down |
| `src/mission_node.cpp` | sets the scene, plans, runs the policy, and judges the run |
| `launch/hotel_distributed_launch.py` | the hotel, ePlanSys, the bridge, the crew, the knowledge view and the mission |
| `config/task_map.json` | the bridge's map of agents to robots and actions to Open-RMF tasks |
