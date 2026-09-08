# Third-party models

## Warehouse (the hall)

`models/Warehouse` is vendored from Gazebo Fuel, unmodified.

- **Model:** `Warehouse`, by Filipe Almeida (MOV.AI)
- **Source:** https://app.gazebosim.org/OpenRobotics/fuel/models/Warehouse
- **Licence:** Creative Commons Attribution 4.0 International (CC BY 4.0)
- **Used for:** the 30 x 50 m building shell, its roof, walls and ten pillars

It is vendored rather than fetched at run time so that the world loads without
network access and so that the geometry the navigation graph was checked
against cannot change underneath it.

It was chosen over `OpenRobotics/Depot`, which is the model usually reached for
and is far more detailed, because Depot's only collision geometry is a ground
plane: its racks and walls are visuals, a laser passes straight through them,
and a floor plan rasterised from its collision meshes comes back empty. This
model's collision mesh is the building.

What it does not provide is shelving. Measured at robot height its interior is
1.2% occupied and all of that is pillars, so the racks are ours -- see
`tools/layout.py`.

## Shelving and clutter

The `aws_robomaker_warehouse_*` models are AWS RoboMaker's, from
`aws-robomaker-small-warehouse-world`, and are not vendored here: they come
from the `aws_robomaker_small_warehouse_world` package the launch file puts on
`GAZEBO_MODEL_PATH`. They are used because they carry real collision meshes.

- **Source:** https://github.com/aws-robotics/aws-robomaker-small-warehouse-world
- **Licence:** MIT
