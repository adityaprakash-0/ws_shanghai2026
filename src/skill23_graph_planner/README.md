# skill23_graph_planner — secret corridor navigation

Plans paths **only along GeoJSON corridor edges** (between walls).
Local following uses stock **RPP** (same as normal nav2).

## Two ADEV modes

| Mode | Planner | Controller | Use when |
|------|---------|------------|----------|
| `nav2` | Theta\* | RPP | Dynamic obstacles / free space |
| `nav2_secret_mode` | **GraphPlanner** | RPP | Fixed corridors (your graph) |

## How it works

```
goal → GraphPlanner (snap to nodes, Dijkstra on edges) → /plan
     → RPP FollowPath → /cmd_vel
```

## Params

| Robot | File |
|-------|------|
| mecanum | `config/nav2_secret_mecanum.yaml` |
| xdrive | `config/nav2_secret_xdrive.yaml` |
| allterrain | `config/nav2_secret_allterrain.yaml` |

```yaml
GridBased:
  plugin: skill23_graph_planner/GraphPlanner
  graph_filepath: ""   # set at launch
  path_density: 0.05
  max_snap_distance: 1.5
```

BT: `config/navigate_secret.xml` replans slowly (0.2 Hz).

## Launch

```bash
source ~/skill23_ws/install/setup.bash
~/skill23_ws/src/adev_dashboard/scripts/launch_adev_dashboard.sh nav2_secret_mode mecanum \
  ~/skill23_ws/test_map11.yaml \
  ~/skill23_ws/test_map11_graph.geojson
```

## Build

```bash
cd ~/skill23_ws
colcon build --packages-select skill23_graph_planner --symlink-install
source install/setup.bash
```
