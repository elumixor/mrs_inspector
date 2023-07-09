# MRS Inspector

Inspect different points with multiple UAVs.

## How to use:

To launch the node:

```bash
roslaunch mrs_inspector mrs_inspector.launch
```

To specify custom world bounds or obstacles files use:

```bash
roslaunch mrs_inspector mrs_inspector.launch world_bounds_file:=/path/to/world_bounds.yml obstacles_file:=/path/to/obstacles.[dae|asc]
```

`world_bounds_file` is a YAML file that specifies the world bounds. See [example](resources/world_bounds.yaml).

`obstacle_file` can either be:

- Collada `dae` file ([example](resources/models/Towers/meshes/towers.dae))
- `.asc` file that is just a list of points in the format `x y z` ([example](resources/obstacles.asc))

## Whole example

```bash
$(rospack find mrs_inspector)/tmux_scripts/start.sh
```

This will launch custom gazebo world, spawn 2 UAVs and command them to inspect several points.
