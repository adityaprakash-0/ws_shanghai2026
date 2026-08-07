# ADEV Dashboard (Pi side)

Two navigation modes — pick at launch time.

## Modes

| Mode | When to use | Stack |
|------|-------------|-------|
| **nav2** | Dynamic obstacles / free-space | Theta\* planner + RPP |
| **nav2_secret_mode** | Fixed corridors between walls | GraphPlanner + RPP |

Both modes work on allterrain, mecanum, xdrive, and grip.

## Quick start

```bash
~/skill23_ws/src/adev_dashboard/scripts/launch_adev_dashboard.sh
```

Or non-interactive:

```bash
# Dynamic Nav2 (Theta* + RPP)
~/skill23_ws/src/adev_dashboard/scripts/launch_adev_dashboard.sh nav2 mecanum

# Secret corridors (GraphPlanner + RPP)
~/skill23_ws/src/adev_dashboard/scripts/launch_adev_dashboard.sh nav2_secret_mode mecanum \
  ~/skill23_ws/test_map11.yaml \
  ~/skill23_ws/test_map11_graph.geojson
```

## Folder layout

```
adev_dashboard/
  launch/
    nav2_dashboard.launch.py
    nav2_secret_mode_dashboard.launch.py
  scripts/
    launch_adev_dashboard.sh
    mode_nav2/launch.sh
    mode_nav2_secret_mode/launch.sh
```

Pick-place and vision are separate packages and are not affected by these modes.
