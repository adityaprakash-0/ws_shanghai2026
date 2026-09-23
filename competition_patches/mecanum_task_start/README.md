# Competition patch — mecanum PID task + START button (DIO 8)

Offline kit: copy onto Pi competition workspace, rebuild, run.

## What this does

1. Waits for physical **start_switch** on **DIO pin 8**
2. Hardware polarity: unpressed=`True`, pressed=`False` on `/start_switch/state`
3. Task manager **inverts** → pressed becomes logical `True` → mission starts
4. Runs `mission_plan` (move / turn) via `/pid_move` and `/pid_turn`
5. While driving, **light_tower** (`follow_cmd_vel`) sets colors from `/cmd_vel`
   (default: green=moving, red=stopped — tune in `params_mecanum_task.yaml`)
6. When last step finishes → task manager sends **solid red** to `/light_tower/set` and prints `MISSION COMPLETE`

**No `light_status` node** — that was a separate mecanum helper that also called
`/light_tower/set` and would fight the light_tower component. Not launched.

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

## Where solid red is sent at the end

File: `mypackage/mypackage/task_manager_mecanum.py`

- After the last step’s `/pid_*/done`, `_start_next_task()` sees
  `current_step >= len(mission_plan)` and calls **`_mission_complete()`**
- `_mission_complete()` → `_set_light('red')` → service **`/light_tower/set`**
  with `params: "red"` (solid, no blink)
- Also prints `MISSION COMPLETE` to the terminal

## Wait time between sequence steps

Same file, top constants:

```python
START_SETTLE_SEC = 0.5   # after START press, before first task
BETWEEN_STEPS_SEC = 1.0  # after each move/turn DONE, before next task
```

Change `BETWEEN_STEPS_SEC` if you want a longer/shorter pause between steps.

## Where to configure speeds

Not in task_manager — in the **PID C++ nodes** (rebuild `mecanum` after edit):

**Move** — `src/mecanum/src/pid_move.cpp` (constructor):

```cpp
min_move_speed_ = 0.13;
max_move_speed_ = 0.20;
// also Kp_dist_, Kd_dist_, dist_tolerance_, Kp_angle_, …
```

**Turn** — `src/mecanum/src/pid_turn.cpp` (constructor):

```cpp
min_speed_ = 0.13;
max_speed_ = 0.20;
// also Kp_, Kd_, tolerance_
```

Then:

```bash
colcon build --packages-select mecanum && source install/setup.bash
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
