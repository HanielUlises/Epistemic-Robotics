# warehouse_xl_rmf_demo

A larger warehouse floor for Open-RMF: thirty by fifty metres, thirty-four
aisles, and one service lane that every aisle opens onto.

|  | small warehouse | this |
| --- | --- | --- |
| floor | 14 x 21 m, 296 m² | 30 x 50 m, 1 500 m² |
| aisles | 2 | 34 |
| waypoints | 12 | 132 |
| directed lanes | 11 | 262 |
| robots | 2 | 3 |

## Why it is built rather than downloaded

There is no large warehouse map for Open-RMF to download. Measured against
their own meshes rather than their descriptions, the candidates on Gazebo Fuel
each fail in a different way: `OpenRobotics/Depot` is detailed and collides
with nothing but the floor; `OpenRobotics/Warehouse` is genuinely 30 x 50 m and
is an empty box, 1.2% occupied at robot height and all of it pillars;
`industrial-warehouse` is the AWS small warehouse recomposed from the same
parts. The largest maps in `rmf_demos`, the airport terminal and the campus,
are not warehouses.

So the hall is the Fuel model, whose collision mesh is the building, and the
shelving inside it is AWS RoboMaker's, whose racks have collision meshes a
laser can see. The arrangement is ours.

## The shape of the floor is the shape of the argument

An aisle opens onto the service lane and onto nothing else. A robot standing on
the lane cannot see down an aisle; it has to go to the mouth. That is what
gives an epistemic domain something to be uncertain about, and it is why the
floor was not simply made bigger and emptier -- a large open hall is a longer
demonstration of a weaker claim.

## Everything is generated

`tools/layout.py` holds the floor, and the world, the navigation graph and the
building map are all written from it. None of the three is edited by hand, so
none of them can drift from the others.

```
python3 tools/make_world.py        --out worlds/warehouse_xl.world
python3 tools/make_nav_graph.py    --out maps/nav_graphs/0.yaml
python3 tools/make_building_map.py --nav-graph maps/nav_graphs/0.yaml \
                                   --out maps/warehouse_xl.building.yaml
python3 tools/plot_floor.py        --out floor.png
```

Every waypoint and every lane is checked against the floor before it is
written, and the check is not decoration. It refused three layouts that looked
right: aisle ends that met the pillar row, a north cross aisle that ran through
the last row of shelving once the row count changed, and -- from the fleet
config rather than the graph -- two robots nominating one charger, which is the
"Failed negotiation" that a waypoint holding exactly one robot produces.

## Running it

```
ros2 launch warehouse_xl_rmf_demo warehouse_xl_fleet.launch.py
ros2 launch warehouse_xl_rmf_demo warehouse_xl_fleet.launch.py headless:=true
```

`server_uri:=ws://localhost:7879` points the fleet adapter at the epistemic
bridge, the same as the small warehouse demo.

### The fastapi and pydantic clash

`fleet_manager` imports `fastapi`, and the `fastapi` that Jammy packages is
0.63, written against pydantic 1. A pydantic 2 installed in `~/.local` shadows
the apt pydantic 1.8 for every interpreter on the machine, and the import then
fails:

```
pydantic.errors.PydanticUserError: Field 'type_' defined on a base class was
overridden by a non-annotated attribute
```

Without the fleet manager no robot moves, and this stops every `rmf_demos`
fleet rather than only this one. The launch file sets `PYTHONNOUSERSITE=1`,
which restores the pair apt installed together and needs nothing installed or
uninstalled. Nothing RMF launches comes from `~/.local`, so ignoring the user
site costs nothing.

## The recorded run

```
  t=0s   (11.00, -23.64)  charger_east_south
  t=12s  ( 9.06, -22.05)  onto the south cross aisle
  t=24s  ( 3.29, -22.04)  west along it, ~0.58 m/s
  t=36s  ( 0.00, -22.04)  lane_00, the foot of the service lane
  t=48s  ( 0.01, -16.86)  north up the lane
  t=60s  ( 0.01, -11.35)
  t=72s  ( 0.00,  -5.59)
  t=84s  ( 2.70,   0.25)  turned east at the mouth of aisle 8
  t=96s  ( 4.19,   0.25)  east_08
```

r1 covers 35.9 m of path for 25 m of separation, because the racks leave no way
into an aisle but the lane. `docs/r1_run.csv` holds the samples and
`docs/warehouse_xl_run.png` draws them over the graph.

```
python3 tools/record_run.py --seconds 120 --out run.csv
python3 tools/plot_floor.py --run run.csv --out run.png
```

### What the run does not yet establish

The trace is r1's alone. Three tasks pinned to three robots were accepted and
executed concurrently without a process failure, but how the schedule mediates
the service lane under sustained three-robot traffic -- the lane being the one
resource all thirty-four aisles share -- has not been measured. That is the
next thing worth knowing about this floor.

## Third-party models

See `models/ATTRIBUTION.md`. The hall is CC BY 4.0 and the shelving is MIT.

## The floor is no longer generated

`warehouse_xl.world` and `tools/layout.py` built a hall from the Fuel model and
placed shelving in it parametrically. The floor now used instead is
`dynamic_logistics_warehouse` — the AWS small warehouse tiled and furnished by
hand, 42 x 63 m, 153 obstacles and nine walking actors — which is both larger
and better furnished than anything that could be justified generating.

It is GPL-2.0 while this repository is Apache-2.0, so it is fetched rather than
vendored:

```
git clone https://github.com/belal-ibrahim/dynamic_logistics_warehouse \
    ros2_ws/third_party/dynamic_logistics_warehouse
```

Note that the upstream `package.xml` declares Apache 2.0 while the repository
ships GPL-2.0 in `LICENSE`; the more restrictive of the two is assumed.

Nothing about the placement of 153 hand-set obstacles is parametric, so the
navigation graph is derived from the world rather than declared:

```
python3 tools/read_world.py                      # what is floor, what is obstacle
python3 tools/make_dlw_graph.py --world <path>   # rasterise, lattice, thin
python3 tools/plot_dlw.py --world <path> ...     # look at the result
```

`config/aws_footprints.json` holds the collision extents of the fourteen AWS
assets, measured from the meshes themselves. Three of the fourteen are
structure rather than obstacle, and `GroundB` in particular is a 14 x 21 m
floor tile: treated as an obstacle it fills the warehouse solid.

## The epistemic mission

```
ros2 launch warehouse_xl_rmf_demo survey_xl_launch.py
```

The survey domain of `eplansys`, unchanged, over this floor. The site is
`aisle_07`, which a robot must enter before it can tell anything about it.

```
at the site (0.35 m from it), nearest return 0.32 m (< 0.70) -> e-scan-dirty
scan: perception reported e-scan-dirty
applied scan_relay -> e-scan-dirty: 2 worlds, 1 designated
relay says e-scan-dirty on /eplansys/channel/private/scout
mission complete

ok   (Kw scout contaminated) holds
ok   (Kw relay contaminated) holds
ok   (Kw observer contaminated) does not hold
ok   scout was spoken to, 1 time(s)
ok   observer was spoken to by nobody
```

### The outcome is sensed

`scan_relay` is a physical action. `scripts/scan_perception.py` subscribes to
`/scan` and `/fleet_states` and withholds any verdict until the fleet has put
the robot within 0.40 m of the site on three consecutive reports. Only then
does it read the nearest finite return, compare it against a 0.70 m threshold
and publish `e-scan-dirty` or `e-scan-clean`, which the bridge prefers over the
task map's `default_outcome`. Only `r1` carries a laser: Gazebo names a
plugin's node after the plugin, so three robots from one model file give three
nodes called `/lds_driver` and the server segfaults.

The floor is built twice to show the action discriminates. The variants differ
by one pallet inside `aisle_07`, 0.80 m from the waypoint, and by nothing else:

| variant | nearest return | outcome | branch |
| --- | --- | --- | --- |
| `site:=clean` | 0.89 m, aisle clutter | `e-scan-clean` | `relay-clean_relay_scout` |
| `site:=dirty` | 0.32 m, the pallet | `e-scan-dirty` | `relay-dirty_relay_scout` |

The arrival radius is the delicate part. At its first value of 1.20 m the
reading was taken a metre short of the waypoint, where the pallet reads about
1.0 m and falls the wrong side of the threshold: the node then reported
`e-scan-clean` in both worlds and every check still passed.

### Recording

```
bash tools/record_demo.sh dirty /tmp/raw.mkv /tmp/run.log
python3 tools/make_captions.py --log /tmp/run.log --t0 $(cat /tmp/raw.t0) \
        --out /tmp/caps.tsv
python3 tools/make_video.py --raw /tmp/raw.mkv --captions /tmp/caps.tsv \
        --start 52 --trim 40 --out /tmp/run.mp4
```

`record_demo.sh` runs the mission on an Xvfb display carrying Gazebo and RViz
and nothing else, so a full-screen grab cannot pick up the real desktop.
`make_captions.py` reads the caption track off the mission's own log, so a
caption saying the robot sensed something appears at the frame in which it
did.

## The robots

Three TurtleBot3 Waffles: ROBOTIS's geometry on rmf_demos' TinyRobot drive
skeleton. Both halves are deliberate and `models/TurtleBot3Waffle/model.sdf`
explains why — briefly, a Waffle built to what `slotcar` documents loads,
initialises, publishes state and does not move, and turtlebot3's meshes are
millimetre STLs beside placeholder cubes.

## The multi-site mission

```
bash tools/make_site_worlds.sh /tmp
ros2 launch warehouse_xl_rmf_demo survey_sites_launch.py \
     contaminated:=a06 world:=/tmp/warehouse_xl_a06.world
```

The survey above has one site and asks whether it is contaminated. This one
has three and asks *which*. That is one line of EPDDL — `(contaminated ?s -
site)` instead of `(contaminated)` — and it is the line that couples the floor
to the epistemics, because the sites are transits apart on a roadmap derived
from the world and deciding which to inspect is now a decision.

|  | single site | three sites |
| --- | --- | --- |
| atoms | 4 | 12 |
| worlds, designated | 2, 2 | 3, 3 |
| ground actions | 30 | 72 |
| AO\* depth | 3 | 6 |
| expansions | 40 | 838 168 |
| policy nodes, leaves | 4, 2 | 8, 3 |
| robots that move | 1 | 2 |
| sites scanned | 1 | at most 2 of 3 |

The expansion count is the column worth reading. One site to two costs a factor
of four; two to three costs a factor of 5 700. Four sites is not solved: under
the default fifteen-second budget the search reports `Timeout at depth 7`, and
under a nine-hundred-second budget `Timeout at depth 8` after 901 s. Both are
the budget running out rather than the space being refuted, so nothing here
says four sites is unsolvable. What is measured is the price of one more site:
sixty times the budget bought one further depth iteration.

### The policy, and the site nobody visits

```
(goto relay a06)
  (goto scout a17)
    (scan scout a17)                       [sensing]
      e-scan-dirty -> (relay scout relay a06)
      e-scan-clean -> (relay scout relay a17)
        (scan relay a06)                   [sensing]
          e-scan-dirty -> (relay relay scout a31)
          e-scan-clean -> (relay relay scout a06)
```

Which robot is sent where is the planner's choice and not ours: the domain is
symmetric in its three sites, and the assignment above is what the search
returned. It happens to give the relay a 36.5 m errand and the scout a 26.8 m
one, and to leave `a31` alone.

Two things in it are worth stating plainly.

**Nobody ever goes to a31.** Exactly one site is contaminated, so ruling out
two settles the third. Down the leaf where both scans come back clean the team
finishes knowing the state of a site no robot approached and no laser was
pointed at — 26.5 m from a17 and 41.4 m from a06 — and the negative conjunct,
that the observer must not come to know, still holds over it. The single-site
demo cannot produce a result of this kind at any floor size, because with one
site there is nothing to eliminate.

**Every announcement is a `relay-clean`,** including the two that convey which
site is dirty. `(relay scout relay a06)` on the dirty branch says only that the
scout knows a06 is clean — and that is enough, because the scout could only
know that in the world where its own scan found a17. What the listener learns
is not the content of the message but what the speaker's knowing it rules out.
The team's conclusion is second-order, and it is the planner's, not ours.

### Three worlds, three leaves

The floor is built four times and the variants differ by where one pallet
stands and by nothing else — same hall, same 155 obstacles, same nine actors,
same graph.

| `contaminated:=` | scan a17 | scan a06 | leaf | actions | checks |
| --- | --- | --- | --- | --- | --- |
| `a17` | dirty, 0.33 m | not reached | `(relay scout relay a06)` | 4 | 11 of 11 |
| `a06` | clean, 1.08 m | dirty, 0.41 m | `(relay relay scout a31)` | 6 | 11 of 11 |
| `a31` | clean, 0.97 m | clean, 2.25 m | `(relay relay scout a06)` | 6 | 11 of 11 |
| `none` | — | — | — | control, see below | |

The readings are the ones the runs reported. The checks are nine formulas put
to the epistemic state after the fleet stopped — that the scout and the relay
each know whether each of the three sites is contaminated, and that the
observer does not — and two transcripts of who was actually spoken to.

In the `a31` row nothing is ever measured at `a31`, and
`(Kw scout contaminated_a31)` holds all the same.

The model, as the third run reported it:

```
applied goto_relay_a06:                    3 worlds, 3 designated
applied goto_scout_a17:                    3 worlds, 3 designated
applied scan_scout_a17 -> e-scan-clean:    3 worlds, 2 designated
applied relay-clean_scout_relay_a17:       5 worlds, 2 designated
applied scan_relay_a06 -> e-scan-clean:    5 worlds, 1 designated
applied relay-clean_relay_scout_a06:       8 worlds, 1 designated
```

Sensing contracts the designated set; private speech adds worlds, because the
agents outside the audience must go on considering possible a world in which
nothing was said. A clean scan settles nothing on its own — it takes the
designated set from three to two — which is why this branch costs two scans
and two announcements where the `a17` branch costs one of each.

`none` is a control and not a fourth case: the domain says exactly one site is
contaminated, so a world with no pallet contradicts the problem the planner was
given. Running it shows what the fleet does when the world and the model
disagree, which is worth knowing and is not a demonstration of anything
working.

### Sites are chosen by measurement

`tools/check_sites.py` decides what can be a site, from the world's own
collision extents — the same `config/aws_footprints.json` the navigation graph
is rasterised from, so the two cannot disagree.

```
python3 tools/check_sites.py                              # every usable aisle
python3 tools/check_sites.py --site a17 --site a31 --site a06
```

A waypoint qualifies when a laser standing on it reads open in an empty world,
and when the pallet has somewhere to stand that is on the floor, clear of the
racks, and clear of every lane the fleet may drive. Thirteen of the thirty-nine
aisle waypoints qualify.

| site | at | empty world reads | pallet | reads | off the nearest lane |
| --- | --- | --- | --- | --- | --- |
| `a17` | (1.49, 15.75) | 1.48 m | west | 0.35 m | 0.35 m |
| `a31` | (15.24, 37.00) | 1.24 m | west | 0.35 m | 0.35 m |
| `a06` | (30.24, −1.75) | 1.71 m | east | 0.35 m | 0.35 m |

`a17` is a dead end: one lane in, and it is the way out. The lane clearance is
the check the single-site demo shipped without and then needed — a pallet jack
placed for appearance overlapped a charger's lane by 20 cm, and a robot stood
pressed against it while RMF reported its task underway and nothing anywhere
reported a collision.

### Two lasers

The single-site demo fits a laser to one robot, and says why: Gazebo Classic
names a plugin's node after the plugin, so two robots from one model file give
two nodes called `/lds_driver` and the server segfaults. This mission scans two
sites with two robots, so `tools/with_lidar.py` writes a copy of the
laser-carrying model per sensing robot with the ray plugin in that robot's own
namespace. A scan is then on `/r1/scan` and `/r2/scan`, and
`scripts/site_perception.py` watches every (robot, site) pair rather than one
robot at one place.

## The floor was wrong, and the correction moves everything

Four defects were found bringing the multi-site mission up. Three are in the
composition; the fourth is in the floor, and it invalidates numbers this README
and the published page previously reported.

**The world carries a `<state>` block, and Gazebo applies it.** A saved
`<state>` overrides the poses declared on the models themselves. This world has
one, recording 178 poses, and fourteen disagree with the declaration: by 0.19 m
for one floor slab, 3.8 m and 4.1 m for two more, 49.8 m for a fourth, and
149 m for a cluttering prop the declaration puts inside the building and the
state puts well outside it. `read_world.py` read the declarations, so the
navigation graph was a floor plan of a warehouse the simulator does not load —
obstacles mapped where the floor is clear, clear floor where an obstacle
stands, and slabs in the wrong place.

**Two `<state>` poses have their numbers run together.** They read
`13.8354 41.7933-0.191335 ...`: the separator before a negative value is
missing. Splitting on whitespace yields a token that is not a number, and the
obvious `try: float(...) except: 0, 0` then puts a 14 × 21 m floor slab at the
origin without a word. The numbers are read out with a regular expression now,
which cannot fail that way.

**Two obstacles were nested inside a group.** A `<model name="Untitled">` holds
a bucket and a desk placed relative to it. A scan of the world's direct
children sees only the group, whose name is in no footprint table, so both were
dropped and the floor they stand on was rasterised as free.

Together these change the derived floor and everything downstream of it:

| | before | after |
| --- | --- | --- |
| obstacles | 153 | 155 |
| waypoints | 261 | 289 |
| directed lanes | 1 652 | 1 870 |
| aisle waypoints | 35 | 39 |
| chargers moved | — | all three, one by 11 m |

**The figures the published page reports are superseded by these.** The
charger move is why `warehouse_xl_fleet.launch.py` now reads spawn poses from
the graph instead of holding them as constants: a pose copied from an older
graph puts a robot somewhere RMF does not think it is, and the first command is
then issued from a place the robot is not.

The single-site demo's own site does not survive the correction — on the
corrected floor no side of it admits the pallet — so `survey_xl_launch.py` now
runs at `a17` as well.

## What else broke

**The bringup lost a race the small domain never let it lose.** The mission's
readiness test was "does the domain expert answer", and a *configured* node
answers — several seconds before the bringup activates anything. The
single-site search is forty expansions and returns before anyone notices. This
one holds the process for seven seconds; the lifecycle manager's activate calls
go unanswered, the bringup reports `Failed to start plansys2!`, and the planner
then hands a perfectly good policy to a system that has already shut down. The
mission fails at `start_plan_execution` with `send_goal failed`, forty lines
and several seconds away from the cause. `survey_sites_mission` now waits for
every managed node to be *active*.

**One observation topic cannot serve two scans.** The bridge has one performer
per action name and one observation topic, so both scans run through one
`observed_` — and the topic is latched, so the second scan is handed the first
one's answer the moment it subscribes. Timestamping the readings and ignoring
stale ones looks like the fix and is not: a robot commonly reaches its site
during the `goto` that precedes the scan, so the correct reading is already
seconds old when the scan begins, and the rule would discard the right answer
and fall through to the task map's stand-in. Readings are published as
`<site> <outcome>` now and the bridge looks up the site its action names.

**`--pallet -4.15,2.0,0` is not an argument.** The value begins with a minus,
so argparse reads the next token as an option unless it parses as a plain
negative number — which `-4.15,2.0,0` does not. It fails with `expected one
argument` and names the option rather than the value. `record_demo.sh` had the
same latent bug.

**Four files named the sites and nothing checked they agreed.** The EPDDL
problem, the bridge's task map, the launch that places the pallets, and the
mission node that declares them as objects. Three were renamed and the fourth
was not, so the mission declared objects the policy does not mention and the
executor sat checking `over all (on_site scout a17)` — a condition over a site
that is not an object of the problem — fifty times a second, for ever, without
ever saying so. The mission reads its sites from the EPDDL now, and the launch
refuses to start if its own list disagrees.

**The classical precondition encoded the assumption the domain exists to
deny.** `survey-sites.pddl`'s `relay` required `(at start (scanned ?from ?s))`
— the speaker must have scanned the site it reports on. That is exactly what
this domain denies: with one of three contaminated, ruling two out settles the
third, and the announcement carrying the finding names a site the speaker has
never been to. The condition is now `(has_scanned ?from)`, which says the
speaker has looked at something.

What makes it worth recording is that only one branch shows it. Where the
first scan comes back clean, the relay is about the site just scanned and the
condition holds; where it comes back dirty, the relay is about a different site
and the executor sits on `Error checking at start reqs: (and (scanned scout
a06))` for ever. Two of the three worlds ran green with the wrong condition in
place.

**And one thing that went right, which is worth as much.** `mission_check`
holds the single-site formulas as its defaults, and this domain has no atom
called `contaminated`: plank grounds `(contaminated ?s)` into `contaminated_a17`
and its fellows. Given the wrong formulas the check reported them `UNCHECKED`
and failed the run, rather than reporting that nothing holds. That distinction
is the one the single-site page argued for — a transcript that fails to arrive
is not an agent that heard nothing — and here it is the difference between
"your formulas are wrong" and "your mission failed".
