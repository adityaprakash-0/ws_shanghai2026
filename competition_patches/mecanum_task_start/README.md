# Competition patch — mecanum PID task + START button (DIO 8)

Offline kit: copy onto Pi competition workspace, rebuild, run.

## What this does

1. Waits for physical **start_switch** on **DIO pin 8**
2. Hardware polarity: unpressed=`True`, pressed=`False` on `/start_switch/state`
3. Task manager **inverts** → pressed becomes logical `True` → mission starts
4. Runs `mission_plan` (move / turn) via `/pid_move` and `/pid_turn`
5. While driving, **light_status** sets tower green=moving / red=stopped from `/cmd_vel`
6. When last step finishes → task manager sends **solid red** to `/light_tower/set` and prints `MISSION COMPLETE`

## Files to copy

| This patch file | Destination on Pi |
|---|---|
| `mypackage/mypackage/task_manager_mecanum.py` | `src/mypackage/mypackage/task_manager_mecanum.py` |
| `studica_control/launch/mecanum_task_m.py` | `src/studica_ws/src/studica_control/launch/mecanum_task_m.py` |
| `studica_control/config/params_mecanum_task.yaml` | `src/studica_ws/src/studica_control/config/params_mecanum_task.yaml` **NEW** |

(Adjust `studica_ws` path if your layout differs.)

## Build

```bash
cd ~/skill23_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mypackage studica_control mecanum
source install/setup.bash
```

## Run

```bash
export ROS_DOMAIN_ID=0
# optional if sharing topics over Wi‑Fi:
# export ROS_LOCALHOST_ONLY=0
# export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

ros2 launch studica_control mecanum_task_m.py
```

Then press the physical START button. Terminal should show:

```text
start_switch: hw_raw=False  pressed_logical=True
>>> START BUTTON PRESSED — beginning mission sequence
Task 1/N: MOVE …
Action DONE …
========== MISSION COMPLETE ==========
Sending solid RED to light tower (finish)
```

## Edit the sequence

In `task_manager_mecanum.py`:

```python
self.mission_plan = [
    ('move', 1.0, 0.0, 0.0),   # distance_m, axis 0=fwd/1=strafe, heading_lock_deg
    # ('turn', -90.0),
    # ('move', 0.5, 1.0, 0.0),
]
```

## Wiring checklist

| Item | Topic / config |
|------|----------------|
| Start button | `/start_switch/state` (`std_msgs/Bool`) |
| DIO | `dio.enabled: true`, sensor `start_switch`, **pin 8**, input |
| Light tower | `light_tower.enabled: true` |
| Gamepad | **disabled** in `params_mecanum_task.yaml` (PID owns `/cmd_vel`) |
| Move goals | `/pid_move/goal` → done `/pid_move/done` |
| Turn goals | `/pid_turn/goal` → done `/pid_turn/done` |
| Finish light | service `/light_tower/set` params `"red"` |

## Params note

Launch uses **`params_mecanum_task.yaml`** (not the old `params_mecanum.yaml`) so everyday teleop params stay unchanged.

## Debug

```bash
ros2 topic echo /start_switch/state
# unpressed → data: true
# pressed   → data: false

ros2 service call /light_tower/set studica_control/srv/SetData "{params: 'red'}"
```
