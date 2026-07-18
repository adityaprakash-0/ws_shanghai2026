# ADEV Dashboard (Pi side)

Three navigation modes — pick at launch time.

## Modes

| Mode | Launch | Robots | Stack |
|------|--------|--------|-------|
| **nav2** | `mode_nav2/` | allterrain, mecanum, xdrive | `studica_control/navigation_*.py` |
| **nav2_guard** | `mode_nav2_guard/` | mecanum, xdrive | `skill23_nav_guard/navigation_guard_*.py` |
| **nav2_secret_mode** | `mode_nav2_secret_mode/` | all three | GraphPlanner (`skill23_graph_planner`) + stock BT + RPP |

## Quick start

```bash
~/skill23_ws/src/adev_dashboard/scripts/launch_adev_dashboard.sh
```

Or non-interactive:

```bash
# Standard Nav2
~/skill23_ws/src/adev_dashboard/scripts/launch_adev_dashboard.sh nav2 mecanum

# Nav2 + wheel sensor guard
~/skill23_ws/src/adev_dashboard/scripts/launch_adev_dashboard.sh nav2_guard mecanum

# Secret mode (corridor GraphPlanner)
~/skill23_ws/src/adev_dashboard/scripts/launch_adev_dashboard.sh nav2_secret_mode mecanum \
  ~/skill23_ws/test_map11.yaml \
  ~/skill23_ws/test_map11_graph.geojson
```

## Folder layout

```
adev_dashboard/
  launch/
    nav2_dashboard.launch.py              # mode 1
    nav2_guard_dashboard.launch.py        # mode 2
    nav2_secret_mode_dashboard.launch.py  # mode 3
  scripts/
    launch_adev_dashboard.sh
    mode_nav2/launch.sh
    mode_nav2_guard/launch.sh
    mode_nav2_secret_mode/launch.sh
```
