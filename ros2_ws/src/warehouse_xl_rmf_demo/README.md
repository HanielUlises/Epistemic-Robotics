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
