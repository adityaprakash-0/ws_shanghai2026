# Competition patch — mecanum colore mission (detect mid-move)

Offline kit for a **mecanum-only** autonomous sequence that pauses on a named
color (red / green / blue / orange / purple), buzzes, flashes green, then
finishes the remaining distance.

## Behavior (what you asked for)

1. Launch HW in terminal 1 (does **not** start the task manager).
2. In terminal 2 run `task_manager_mecanum_colore` — it **asks a color name for
   each `colore_on`** in the plan (1st, then 2nd, …), then waits.
3. Press physical **START** (DIO 8) → sequence begins.
4. Example plan: strafe 50 cm → colore on → move forward 200 cm → colore off →
   strafe 70 cm → colore on → move 100 cm → colore off.
5. While colore is **on** and the robot is moving, `colore_detect` watches for
   the assigned color. On match:
   - publish `/pid_move/cancel` (immediate stop; **no** `/pid_move/done`)
   - buzzer **3 s** via `/light_tower/set`
   - green flash **2 s** (`green:2.0`)
   - turn watch **off**
   - publish a new move for the **remaining** distance (e.g. found at 1.30 m of
     2.00 m → continue 0.70 m), then continue the rest of the plan.

## Wiring (topics / services)

```text
                    ┌─────────────────────┐
  DIO8 start_switch │  /start_switch/state│──► task_manager_mecanum_colore
                    └─────────────────────┘         │
                                                    │ /pid_move/goal  (Point)
                                                    │ /pid_move/cancel (Bool)
                                                    │ /pid_turn/goal   (Float32)
                                                    ▼
                                             pid_move / pid_turn
                                                    │
                                                    │ /cmd_vel
                                                    ▼
                                              mecanum_node
                                                    │
                         /robot_distance_x|y ◄──────┘
                         /gyro_z

  Colore USB ──► /colore0/color_info ──► colore_detect
                       ▲                      │
                       │   /colore_detect/watch (String color name, ""=off)
                       │   /colore_detect/found (Bool) ──► task manager
                       │
  light_tower ◄── /light_tower/set  (buzzer | green:2.0 | red | off)
```

| Piece | Topic / service | Notes |
|-------|-----------------|--------|
| Start button | `/start_switch/state` | HW: unpressed=`True`, pressed=`False`; task mgr **inverts** |
| Move goal | `/pid_move/goal` | `Point.x=dist_m`, `y=0 fwd / 1 strafe`, `z=heading` |
| Move cancel | `/pid_move/cancel` | **New** in `pid_move.cpp` — stop without `/done` |
| Move done | `/pid_move/done` | Normal completion only |
| Turn | `/pid_turn/goal` / `/done` | Same as mecanum task |
| Distance | `/robot_distance_x`, `/robot_distance_y` | Used to compute remaining after color hit |
| Colore HW | `/colore0/color_info` | CIE XYZ + RGB (`ColoreColorMsg`) |
| Watch | `/colore_detect/watch` | `std_msgs/String` color name; empty = off |
| Found | `/colore_detect/found` | `std_msgs/Bool` latched once per watch |
| Lights | `/light_tower/set` | `SetData` params: `buzzer`, `green:2.0`, `red`, `off` |

**Gamepad is off** in params so PID owns `/cmd_vel`.  
**No `light_status` node** (would fight light_tower).

## Files to copy

| This patch file | Destination on Pi |
|---|---|
| `mypackage/mypackage/colore_detect_node.py` | `src/mypackage/mypackage/colore_detect_node.py` **NEW** |
| `mypackage/mypackage/task_manager_mecanum_colore.py` | `src/mypackage/mypackage/task_manager_mecanum_colore.py` **NEW** |
| `mypackage/setup.py` | `src/mypackage/setup.py` (adds console_scripts) |
| `mecanum/src/pid_move.cpp` | `src/mecanum/src/pid_move.cpp` (adds `/pid_move/cancel`) |
| `studica_control/launch/mecanum_colore_task_m.py` | `src/studica_ws/src/studica_control/launch/…` **NEW** |
| `studica_control/config/params_mecanum_colore_task.yaml` | `src/studica_ws/src/studica_control/config/…` **NEW** |

## Build

```bash
cd ~/skill23_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mypackage studica_control mecanum
source install/setup.bash
```

Confirm executables:

```bash
ros2 pkg executables mypackage | grep -E 'colore|mecanum_colore'
# colore_detect
# task_manager_mecanum_colore
```

## Run

**Terminal 1 — hardware + detect:**

```bash
export ROS_DOMAIN_ID=0
ros2 launch studica_control mecanum_colore_task_m.py
```

Wait until you see Colore / `colore_detect ready` and mecanum/PID up.

**Terminal 2 — mission (interactive):**

```bash
source ~/skill23_ws/install/setup.bash
ros2 run mypackage task_manager_mecanum_colore
```

You will see:

```text
=== Mecanum Colore Mission — assign colors ===
Allowed: red, green, blue, orange, purple
This mission has 2 colore_on segment(s).

Color name for colore_on #1: red
  → slot 1 = red
Color name for colore_on #2: blue
  → slot 2 = blue

Colors saved. Press the physical START button when ready.
```

Press **START**. Example log flow:

```text
>>> START PRESSED — beginning colore mission
MOVE 0.5 m (strafe)
COLORE ON — watching "red"
MOVE 2.0 m (fwd/back) [watching red]
!!! COLOR FOUND "red" at 1.300 m — stopping, then finish remaining 0.700 m
Buzzer ON for 3s → Green flash for 2s → Continuing remaining 0.700 m
COLORE OFF
MOVE 0.7 m (strafe)
COLORE ON — watching "blue"
…
========== MISSION COMPLETE ==========
```

## Edit the sequence

In `task_manager_mecanum_colore.py`, edit **`DEFAULT_MISSION_PLAN`** (one place —
prompts count `colore_on` from the same list):

```python
DEFAULT_MISSION_PLAN = [
    ('move', 0.50, 1.0, 0.0),   # dist_m, axis 0=fwd/1=strafe, heading_lock
    ('colore_on',),             # prompts for color #1 before START
    ('move', 2.00, 0.0, 0.0),   # watched while colore active
    ('colore_off',),
    ('move', 0.70, 1.0, 0.0),
    ('colore_on',),             # prompts for color #2
    ('move', 1.00, 0.0, 0.0),
    ('colore_off',),
    # ('turn', -90.0),
]
```

Timings at top of the same file:

```python
BUZZER_SEC = 3.0
GREEN_FLASH_SEC = 2.0
BETWEEN_STEPS_SEC = 1.0
```

## Add a new color

1. Open `colore_detect_node.py`.
2. Add an entry to **`REFERENCE_XY`** (CIE chromaticity xy under your arena light):

```python
REFERENCE_XY = {
    'red': (0.64, 0.33),
    'green': (0.30, 0.60),
    'blue': (0.15, 0.20),
    'orange': (0.55, 0.41),
    'purple': (0.28, 0.18),
    'yellow': (0.45, 0.48),   # NEW — calibrate on floor
}
```

`ALLOWED` is derived from those keys, so the terminal prompt accepts the new name
automatically. Rebuild / reinstall `mypackage`.

### How to calibrate xy

With HW + `colore_detect` running:

```bash
ros2 topic echo /colore0/color_info
```

Hold the target color in front of the sensor. Note XYZ →  
`x = X/(X+Y+Z)`, `y = Y/(X+Y+Z)`. Put that pair in `REFERENCE_XY`.

If matches are flaky, raise/lower `match_threshold` (launch param default `0.08`)
or `min_brightness`.

## Params notes

`params_mecanum_colore_task.yaml`:

- `colore.enabled: true`, `publish_outputs: ["info"]`
- `serial_port: "/dev/ttyACM0"` — change if needed (`ls /dev/ttyACM*`)
- `light_tower.override_hold_sec: 6.0` so follow_cmd_vel does not steal the
  tower during buzzer (3 s) + green flash (2 s)
- `gamepad.enabled: false`
- `dio.start_switch` pin **8**

Everyday teleop params (`params_mecanum.yaml`) are untouched.

## Debug checklist

```bash
# Colore publishing?
ros2 topic echo /colore0/color_info

# Detect node watching?
ros2 topic pub --once /colore_detect/watch std_msgs/msg/String "{data: 'red'}"
ros2 topic echo /colore_detect/found

# Cancel works?
ros2 topic pub --once /pid_move/cancel std_msgs/msg/Bool "{data: true}"

# Lights
ros2 service call /light_tower/set studica_control/srv/SetData "{params: 'buzzer'}"
ros2 service call /light_tower/set studica_control/srv/SetData "{params: 'green:2.0'}"
ros2 service call /light_tower/set studica_control/srv/SetData "{params: 'off'}"

# Start switch polarity
ros2 topic echo /start_switch/state
# unpressed → true ; pressed → false
```

## Relation to mecanum_task_start

Plain PID sequence without color: use `competition_patches/mecanum_task_start`.  
This patch adds Colore + mid-move interrupt + remaining distance. Do not run both
task managers at once.
