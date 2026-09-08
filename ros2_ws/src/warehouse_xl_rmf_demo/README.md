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

### Known environment defect

`fleet_manager` will not import on a machine whose `fastapi` comes from apt and
whose `pydantic` is 2.x:

```
pydantic.errors.PydanticUserError: Field 'type_' defined on a base class was
overridden by a non-annotated attribute
```

This is not particular to this package -- it stops every `rmf_demos` fleet,
including the small warehouse demo -- and the robots do not move until it is
resolved, by pinning `pydantic<2` or by installing a newer `fastapi`. Gazebo,
the traffic schedule, the building map server and the fleet adapter all come up
regardless.

## Third-party models

See `models/ATTRIBUTION.md`. The hall is CC BY 4.0 and the shelving is MIT.
