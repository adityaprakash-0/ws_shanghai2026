# Competition patch — mecanum colore mission (detect mid-move)

Offline kit for a **mecanum-only** autonomous sequence that pauses on a named
color (red / green / blue / orange / purple), buzzes, flashes green, then
finishes the remaining distance.

---

## How to launch

**Build once** (after copying these files onto the Pi):

```bash
cd ~/skill23_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mypackage studica_control mecanum
source install/setup.bash
```

**Terminal 1 — HW + PID + colore_detect** (no task manager):

```bash
export ROS_DOMAIN_ID=0
source ~/skill23_ws/install/setup.bash
ros2 launch studica_control mecanum_colore_task_m.py
```

Wait until you see Colore up and `colore_detect ready`.

**Terminal 2 — interactive mission node:**

```bash
source ~/skill23_ws/install/setup.bash
ros2 run mypackage task_manager_mecanum_colore
```

1. It asks **Color name for colore_on #1**, then **#2**, … (one per `colore_on` in the plan).
2. Type e.g. `red` then Enter. It saves the slots.
3. Press the physical **START** button (DIO pin 8).
4. Sequence runs; watch both terminals for logs.

Optional params file override:

```bash
ros2 launch studica_control mecanum_colore_task_m.py \
  params_file:=/full/path/to/params_mecanum_colore_task.yaml
```

---

## START button polarity (inverted — yes)

Hardware `/start_switch/state` (`std_msgs/Bool`):

| Physical button | Topic value |
|-----------------|-------------|
| **Not pressed** | `True` |
| **Pressed**     | `False` |

Task manager has **`invert_start: True`** (default). Code:

```python
# task_manager_mecanum_colore.py — _on_start_switch
raw = bool(msg.data)
pressed = (not raw) if self._invert else raw   # invert_start=True → pressed when raw=False
```

So a physical press (`raw=False`) becomes `pressed_logical=True` and starts the mission.
You will see:

```text
start_switch: hw_raw=False  pressed_logical=True
>>> START PRESSED — beginning colore mission
```

---

## Where it stops / buzzes / flashes green / finishes remaining

All in `mypackage/mypackage/task_manager_mecanum_colore.py`:

| Step | Function | What happens |
|------|----------|----------------|
| 1. Color found | `_on_color_found` | **Immediately** calls `_emergency_stop()` → publishes **`/cmd_vel` zero** + **`/pid_move/cancel`** (pid does **not** publish `/done`) |
| 2. Buzzer | same → `_set_light('buzzer')` | service `/light_tower/set` params `"buzzer"` for **`BUZZER_SEC` (3.0)** |
| 3. Green flash | `_after_buzzer` | `_set_light('green:2.0')` for **`GREEN_FLASH_SEC` (2.0)** |
| 4. Remaining | `_after_green` | publishes new `/pid_move/goal` with `remain = goal − traveled` (color watch already off) |

Terminal prints you should see on hit:

```text
>>> COLOR FOUND red  xy=(…) dist=…          # from colore_detect
>>> EMERGENCY STOP: /cmd_vel=0 + /pid_move/cancel
!!! COLOR FOUND "red" at 1.300 m — stopped; will finish remaining 0.700 m …
>>> BUZZER ON for 3.0s
>>> GREEN FLASH for 2.0s
>>> GREEN FLASH done — resuming remaining distance
>>> CONTINUING remaining 0.700 m (color watch OFF)
```

`pid_move` also logs `pid_move CANCELED — dist_x=…`.

### Immediate stop (low lag)

On match the task manager **stops first**, then does lights:

1. Zero `/cmd_vel` directly (mecanum stops without waiting only on cancel)
2. `/pid_move/cancel` so PID loop goes inactive and will not keep driving
3. Then buzzer / green / remaining

Colore sample rate is **`publish_rate_hz: 20`** (~50 ms max sensor lag) in
`params_mecanum_colore_task.yaml`. Detect + cancel is event-driven (no extra sleep).

---

## Calibrate colors — step by step

Matching uses **CIE chromaticity** from XYZ:

```text
x = X / (X + Y + Z)
y = Y / (X + Y + Z)
```

Distance to the reference must be ≤ `match_threshold` (default **0.08**).

### Step 1 — Launch HW only

```bash
ros2 launch studica_control mecanum_colore_task_m.py
```

### Step 2 — Watch raw XYZ / RGB

```bash
ros2 topic echo /colore0/color_info
```

Hold the **real arena target** (the exact card/tape the robot will see) ~same
distance/angle as on the course. Note several samples of `x`, `y`, `z` (and `r,g,b`).

### Step 3 — Compute xy

Example: X=0.42, Y=0.22, Z=0.08 → sum=0.72 →  
`x=0.42/0.72=0.583`, `y=0.22/0.72=0.306` → that is your red reference.

Or use live logs: with detect running, force a watch:

```bash
ros2 topic pub --once /colore_detect/watch std_msgs/msg/String "{data: 'red'}"
```

With `log_xy_hz: 2.0` (launch default) the detect terminal prints every ~0.5 s:

```text
  xy=(0.583,0.306)  dist_to_red=0.045  thresh=0.08  RGB=(…)
```

When `dist_to_<color>` stays **below** threshold on the target and **above** on
non-targets, the reference/threshold are good.

### Step 4 — Write reference into code

Edit `colore_detect_node.py`:

```python
REFERENCE_XY = {
    'red': (0.583, 0.306),      # your measured values
    'green': (0.30, 0.60),
    'blue': (0.15, 0.20),
    'orange': (0.55, 0.41),
    'purple': (0.28, 0.18),
}
```

Rebuild:

```bash
colcon build --packages-select mypackage && source install/setup.bash
```

### Step 5 — Tune threshold if needed

| Problem | Fix |
|---------|-----|
| Misses the color / only matches when very close | **Raise** `match_threshold` (e.g. `0.10`–`0.12`) in launch |
| False triggers on other colors / floor | **Lower** threshold (e.g. `0.05`) or re-calibrate xy |
| Matches black / empty | Raise `min_brightness` (default `0.02`) |
| Slow to notice color | Raise `colore0.publish_rate_hz` (already 20); check USB port |

In launch file:

```python
parameters=[{
    'match_threshold': 0.08,   # try 0.05 .. 0.12
    'min_brightness': 0.02,
    'log_xy_hz': 2.0,          # 0 = quiet; 2 = calibration prints
}],
```

---

## Add a new color — step by step

1. Calibrate that color (steps above) → get `(x, y)`.
2. Open `src/mypackage/mypackage/colore_detect_node.py`.
3. Add one line to `REFERENCE_XY`:

```python
REFERENCE_XY = {
    'red': (0.64, 0.33),
    'green': (0.30, 0.60),
    'blue': (0.15, 0.20),
    'orange': (0.55, 0.41),
    'purple': (0.28, 0.18),
    'yellow': (0.45, 0.48),   # NEW
}
```

4. `ALLOWED` is built from those keys — the prompt accepts `yellow` automatically.
5. Rebuild `mypackage`.
6. Put `('colore_on',)` in `DEFAULT_MISSION_PLAN` and type `yellow` when asked.

No YAML change required (matching is in the Python node, not HW `match_references`).

---

## Tune stop / timings

File: `task_manager_mecanum_colore.py` (top constants):

```python
START_SETTLE_SEC = 0.5    # after START before first step
BETWEEN_STEPS_SEC = 1.0   # pause between finished steps
BUZZER_SEC = 3.0          # light_tower buzzer after color hit
GREEN_FLASH_SEC = 2.0     # green:2.0 blink duration
REMAIN_TOL_M = 0.03       # skip remaining move if |remain| smaller
```

PID speed (how hard it brakes/rolls after cancel) is in `mecanum/src/pid_move.cpp`
(`max_move_speed_`, etc.). Cancel itself publishes zero twist immediately.

`light_tower.override_hold_sec: 6.0` in colore params so follow_cmd_vel does not
steal the tower during buzzer+green.

---

## Edit the sequence

Same file, **`DEFAULT_MISSION_PLAN`** (prompts count `colore_on` from this list):

```python
DEFAULT_MISSION_PLAN = [
    ('move', 0.50, 1.0, 0.0),   # dist_m, axis 0=fwd / 1=strafe, heading_lock
    ('colore_on',),
    ('move', 2.00, 0.0, 0.0),
    ('colore_off',),
    ('move', 0.70, 1.0, 0.0),
    ('colore_on',),
    ('move', 1.00, 0.0, 0.0),
    ('colore_off',),
]
```

---

## Wiring summary

| Piece | Topic / service | Notes |
|-------|-----------------|--------|
| Start | `/start_switch/state` | unpressed=`True`, pressed=`False`; **invert** in task mgr |
| Move | `/pid_move/goal` | `x=dist`, `y=0/1`, `z=heading` |
| Cancel | `/pid_move/cancel` + `/cmd_vel=0` | immediate stop, no `/done` |
| Distance | `/robot_distance_x\|y` | remaining = goal − traveled |
| Colore | `/colore0/color_info` | XYZ + RGB |
| Watch / found | `/colore_detect/watch`, `/found` | String / Bool |
| Lights | `/light_tower/set` | `buzzer`, `green:2.0`, `red`, `off` |

---

## Files to copy

| This patch file | Destination on Pi |
|---|---|
| `mypackage/mypackage/colore_detect_node.py` | `src/mypackage/mypackage/` |
| `mypackage/mypackage/task_manager_mecanum_colore.py` | `src/mypackage/mypackage/` |
| `mypackage/setup.py` | `src/mypackage/setup.py` |
| `mecanum/src/pid_move.cpp` | `src/mecanum/src/pid_move.cpp` |
| `studica_control/launch/mecanum_colore_task_m.py` | `src/studica_ws/src/studica_control/launch/` |
| `studica_control/config/params_mecanum_colore_task.yaml` | `src/studica_ws/src/studica_control/config/` |

## Debug

```bash
ros2 topic echo /colore0/color_info
ros2 topic echo /colore_detect/found
ros2 topic echo /start_switch/state   # True=up, False=pressed
ros2 topic pub --once /colore_detect/watch std_msgs/msg/String "{data: 'red'}"
ros2 service call /light_tower/set studica_control/srv/SetData "{params: 'buzzer'}"
ros2 service call /light_tower/set studica_control/srv/SetData "{params: 'green:2.0'}"
ros2 topic pub --once /pid_move/cancel std_msgs/msg/Bool "{data: true}"
```
