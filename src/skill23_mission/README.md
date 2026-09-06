# skill23_mission

Standalone Nav2 bringup + terminal sequence runner. Dashboard stays available for marking maps/waypoints.

## Layout

```
skill23_mission/
  config/
    hw_params/          # per-robot Titan/HW params
    nav2_params/        # Nav2 + BT XML
    sequences.yaml      # edit sequences here
  maps/
    <name>/
      map.yaml
      map.pgm
      waypoints.json
      sequences.json    # optional (from Studio Save to Pi)
  launch/
    bringup.launch.py   # Terminal 1 — hardware + Nav2
    mission.launch.py   # Terminal 2 — pick sequence & run
```

## Usage

Edit the top of each launch file (`ROBOT`, `MAP_NAME`), then:

```bash
cd ~/skill23_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Terminal 1 — stack (edit ROBOT / MAP_NAME in bringup.launch.py)
ros2 launch skill23_mission bringup.launch.py

# Set initial pose in RViz (map frame)

# Terminal 2 — mission (edit MAP_NAME in mission.launch.py)
ros2 launch skill23_mission mission.launch.py
# prompts: Enter sequence number …
```

Map folder: `skill23_mission/maps/<MAP_NAME>/map.yaml`  
Or set `MAP_NAME` to a studio project name to load `~/skill23_ws/studio_maps/<name>`.

Skip prompt: set `AUTO_SEQUENCE = '1'` in `mission.launch.py`.

## From Studio dashboard

1. Mark waypoints (offline or arena)
2. Build sequences in deploy panel
3. Click **Save to Pi** → writes `studio_maps/<map>/waypoints.json` + `sequences.json`
4. Either:
   - run mission with `map:=<map>` (loads studio_maps), or
   - copy those files into `src/skill23_mission/maps/<map>/` and rebuild

## Per-waypoint action

Edit `skill23_mission/mission_runner.py` → `waypoint_action()`.  
Default: wait 3s (or `action.seconds` from waypoints.json).

## Rebuild after editing maps/configs

```bash
colcon build --packages-select skill23_mission --symlink-install
source install/setup.bash
```
