# skill23_graph_planner — nav2_secret_mode

Lightweight **Nav2 GlobalPlanner** on your GeoJSON corridor graph.

## Modes (ADEV dashboard)

| # | Mode | Description |
|---|------|-------------|
| 1 | `nav2` | Standard Nav2 |
| 2 | `nav2_guard` | Nav2 + wheel safety guard |
| 3 | `nav2_secret_mode` | GraphPlanner + stock BT + RPP |

## Params (planner only changed)

Copied from each robot’s stock Nav2 yaml; **only** `planner_server.GridBased` updated:

| Robot | File |
|-------|------|
| mecanum | `config/nav2_secret_mecanum.yaml` |
| allterrain | `config/nav2_secret_allterrain.yaml` |
| xdrive | `config/nav2_secret_xdrive.yaml` |

```yaml
GridBased:
  plugin: skill23_graph_planner/GraphPlanner
  graph_filepath: ""          # filled at launch
  path_density: 0.05
  max_snap_distance: 1.5
  allow_same_node: true
```

## Pipeline

```
AMCL → NavigateToPose (stock BT) → GraphPlanner → /plan → RPP (+ local costmap)
```

## Launch

```bash
source ~/skill23_ws/install/setup.bash

~/skill23_ws/src/adev_dashboard/scripts/launch_adev_dashboard.sh nav2_secret_mode mecanum \
  ~/skill23_ws/test_map11.yaml \
  ~/skill23_ws/test_map11_graph.geojson
```

Alias: `secret_mode` also works as the mode argument.
