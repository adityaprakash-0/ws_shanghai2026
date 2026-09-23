# Competition patch — mecanum colore mission (detect mid-move)

Offline kit for a **mecanum-only** autonomous sequence that pauses on a named
color (red / green / blue / orange / purple), buzzes, flashes green, then
finishes the remaining distance.

---

## Complete installation and run order

Follow these steps in order. The examples assume:

```text
Competition patch: ~/Downloads/competition_patches/mecanum_colore_task
ROS workspace:     ~/skill23_ws
```

Change the first path if you saved `competition_patches` elsewhere.

### 1. Stop old ROS launches

Press Ctrl+C in terminals running old robot launches. Do not run the old
mecanum launch, calibration launch, and mission launch together.

### 2. Copy every patch file to the workspace

```bash
cd ~/skill23_ws

mkdir -p src/mypackage/mypackage
mkdir -p src/mypackage/launch
mkdir -p src/mecanum/src
mkdir -p src/studica_ws/src/studica_control/config
mkdir -p src/studica_ws/src/studica_control/launch

cp ~/Downloads/competition_patches/mecanum_colore_task/mypackage/mypackage/colore_calibrate_node.py \
   src/mypackage/mypackage/
cp ~/Downloads/competition_patches/mecanum_colore_task/mypackage/mypackage/colore_detect_node.py \
   src/mypackage/mypackage/
cp ~/Downloads/competition_patches/mecanum_colore_task/mypackage/mypackage/task_manager_mecanum_colore.py \
   src/mypackage/mypackage/
cp ~/Downloads/competition_patches/mecanum_colore_task/mypackage/launch/colore_calibrate.launch.py \
   src/mypackage/launch/
cp ~/Downloads/competition_patches/mecanum_colore_task/mypackage/setup.py \
   src/mypackage/setup.py

cp ~/Downloads/competition_patches/mecanum_colore_task/mecanum/src/pid_move.cpp \
   src/mecanum/src/pid_move.cpp

cp ~/Downloads/competition_patches/mecanum_colore_task/studica_control/config/params_mecanum_colore_task.yaml \
   src/studica_ws/src/studica_control/config/
cp ~/Downloads/competition_patches/mecanum_colore_task/studica_control/launch/mecanum_colore_task_m.py \
   src/studica_ws/src/studica_control/launch/
```

What was copied:

| File | Purpose |
|------|---------|
| `colore_calibrate_node.py` | Takes 40 readings and prints the XY reference |
| `colore_calibrate.launch.py` | Interactive calibration launch; starts Colore HW and calibration node |
| `colore_detect_node.py` | Detects the saved color during the mission |
| `task_manager_mecanum_colore.py` | Prompts colors, waits START, runs the mission and hit actions |
| `setup.py` | Registers all three Python executables and installs launch files |
| `pid_move.cpp` | Adds immediate `/pid_move/cancel` support |
| `params_mecanum_colore_task.yaml` | HW params: Titan, IMU, DIO 8, light tower, Colore USB |
| `mecanum_colore_task_m.py` | Launches mission hardware, PID nodes and `colore_detect` |

### 3. Rebuild all changed packages

The first build must include **all three** changed packages:

```bash
cd ~/skill23_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mypackage mecanum studica_control
source install/setup.bash
```

Confirm the new Python nodes exist:

```bash
ros2 pkg executables mypackage | grep colore
```

Expected entries include:

```text
mypackage colore_calibrate
mypackage colore_detect
mypackage task_manager_mecanum_colore
```

### 4. Calibrate each physical color

The calibration launch starts the control server itself. Do **not** start
`mecanum_colore_task_m.py` while calibrating.

```bash
source ~/skill23_ws/install/setup.bash
ros2 launch mypackage colore_calibrate.launch.py
```

The terminal asks:

```text
Enter the color name to calibrate: red
Place the "red" target under the sensor at competition height.
Press ENTER when the target is steady:
```

Keep the target still. The node takes 40 readings and prints:

```text
COPY THIS LINE into REFERENCE_XY:
    'red': (0.5830, 0.3070),
```

Save that printed line. Press Ctrl+C after the result. Run the same calibration
launch again for green, blue, orange, purple, or any new color. You can collect
all references before editing the main detector.

### 5. Put calibrated references into the detector

Open:

```text
~/skill23_ws/src/mypackage/mypackage/colore_detect_node.py
```

Replace the example values inside `REFERENCE_XY` with the printed values:

```python
REFERENCE_XY = {
    'red': (0.5830, 0.3070),
    'green': (0.3012, 0.5841),
    'blue': (0.1620, 0.1840),
    'orange': (0.5310, 0.4020),
    'purple': (0.2810, 0.1910),
    'yellow': (0.4521, 0.4718),  # optional new color
}
```

Do not create two entries with the same name; replace the old line.

### 6. Rebuild only `mypackage` after changing references

```bash
cd ~/skill23_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mypackage
source install/setup.bash
```

### 7. Start the actual mission hardware — Terminal 1

```bash
export ROS_DOMAIN_ID=0
source ~/skill23_ws/install/setup.bash
ros2 launch studica_control mecanum_colore_task_m.py
```

This launch uses:

```text
~/skill23_ws/src/studica_ws/src/studica_control/config/params_mecanum_colore_task.yaml
```

It starts:

1. `control_server` — Titan, IMU, DIO 8, light tower and Colore USB
2. `mecanum_node`
3. `pid_move`
4. `pid_turn`
5. `colore_detect`

It deliberately does **not** start the task manager because the task manager
must ask for colors in a separate interactive terminal.

Wait until Terminal 1 shows `colore_detect ready`.

### 8. Start the task manager — Terminal 2

```bash
export ROS_DOMAIN_ID=0
source ~/skill23_ws/install/setup.bash
ros2 run mypackage task_manager_mecanum_colore
```

It asks one color for every `('colore_on',)` in `DEFAULT_MISSION_PLAN`:

```text
Color name for colore_on #1: red
Color name for colore_on #2: blue
Colors saved. Press the physical START button when ready.
```

### 9. Press the physical START button

The button is DIO pin 8:

```text
Not pressed → /start_switch/state = True
Pressed     → /start_switch/state = False
```

The task manager inverts this value, so the mission starts when the physical
button produces `False`.

### 10. Observe the color-hit sequence

When the selected color is found during a watched move:

1. `/cmd_vel=0` and `/pid_move/cancel` stop the robot
2. Buzzer runs for 3 seconds
3. Green flashes for 2 seconds
4. Color watching turns off
5. Robot completes the remaining move distance
6. Mission continues

Optional mission params override:

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

## Straight driving, strafing and reset settling

Every move goal is:

```python
('move', distance_m, axis, heading_deg)
# axis: 0.0 = forward/back; 1.0 = strafe
```

`pid_move` uses two controls at the same time:

1. Encoder distance PID drives `linear.x` for forward/back or `linear.y` for
   strafing.
2. IMU heading PID continuously calculates `heading_deg - /gyro_z` and sends
   correction on `angular.z`.

Therefore this strafe holds the robot at zero relative yaw:

```python
('move', 0.50, 1.0, 0.0)
```

If one wheel slips and the robot **turns**, the IMU sees the heading error and
the mecanum mixer applies opposite wheel correction to return toward 0 degrees.
The correction is limited to `angular.z = ±0.2`.

Important limitation: an IMU corrects **rotation**, not sideways translational
slip. If the complete robot slides sideways without changing angle, wheel
encoders cannot measure that floor slip. So heading stays straight, but no
wheel-only system can guarantee perfect field position on a slippery surface.

Before every move, `pid_move` now:

1. Publishes zero `/cmd_vel`
2. Requests IMU reset
3. Requests reset of all four encoders
4. Holds the robot stopped for **1000 ms**
5. Clears PID error history
6. Activates movement using fresh feedback

The terminal prints:

```text
Resetting Hardware...
IMU reset requested
Encoder reset requested on m_0..m_3
Holding stopped for 1000 ms reset settling time
PID ACTIVE ...
```

`BETWEEN_STEPS_SEC = 1.0` in the task manager also gives one second between
completed sequence steps. The PID's reset settling second starts after the next
move goal, so movement never begins immediately on stale reset values.

### Final light after all steps

Yes. `_mission_complete()` sends:

```python
self._set_light('red')
```

That means **solid red**, not blinking red. Since the stopped color in
`params_mecanum_colore_task.yaml` is also red, it remains solid red after the
light-tower override expires.

---

## Automatic calibration — simplest method

The new calibration node does the XYZ → xy calculation and averaging for you.
Do **not** run the normal mecanum launch at the same time.

### Step 1 — Run one command

```bash
source ~/skill23_ws/install/setup.bash
ros2 launch mypackage colore_calibrate.launch.py
```

This calibration launch uses:

```text
studica_control/config/params_mecanum_colore_task.yaml
```

That params file enables Colore on `/dev/ttyACM0` and publishes
`/colore0/color_info` at 20 Hz.

### Step 2 — Answer the prompts

Example:

```text
Enter the color name to calibrate (example: red or yellow): yellow

Place the "yellow" target under the sensor at competition height.
Keep the lighting the same as the arena.
Press ENTER when the target is steady:
```

Put the real color card/tape under the sensor, at the same height and lighting
used during the competition. Keep it still, then press Enter.

### Step 3 — Wait for 40 readings

`colore_calibrate` reads 40 messages from `/colore0/color_info`, converts each
XYZ reading to xy, then uses the median to reject an occasional bad sample.

It prints:

```text
========== CALIBRATION COMPLETE ==========
Color: yellow
Reference xy (median): (0.4521, 0.4718)
Reading spread: x=0.0031, y=0.0028 (smaller is better)

COPY THIS LINE into REFERENCE_XY:
    'yellow': (0.4521, 0.4718),
```

Copy the exact printed line.

### Step 4 — Save the reference

Open:

```text
src/mypackage/mypackage/colore_detect_node.py
```

Paste the line inside `REFERENCE_XY`:

```python
REFERENCE_XY = {
    'red': (0.64, 0.33),
    'green': (0.30, 0.60),
    'blue': (0.15, 0.20),
    'orange': (0.55, 0.41),
    'purple': (0.28, 0.18),
    'yellow': (0.4521, 0.4718),  # printed by calibration
}
```

For an existing color such as red, replace its old line instead of adding a
second red line.

### Step 5 — Rebuild

```bash
cd ~/skill23_ws
colcon build --packages-select mypackage
source install/setup.bash
```

`ALLOWED` is generated automatically from `REFERENCE_XY`, so newly added colors
immediately become valid choices in the task-manager prompt.

### Step 6 — Repeat for every color

Stop the calibration launch with Ctrl+C, run it again, enter the next color
name, and copy the new line. Calibrate each color with the same sensor height
and arena lighting.

### Tune detection threshold if needed

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

## What the calibration code does

| File / node | Job |
|-------------|-----|
| `colore_calibrate.launch.py` | Asks color name and asks you to place it, starts the Colore HW using `params_mecanum_colore_task.yaml`, then starts the calibration node |
| `colore_calibrate_node.py` / `colore_calibrate` | Takes 40 readings, converts XYZ to xy, finds median reference, prints the line to copy |
| `colore_detect_node.py` / `colore_detect` | Used during the actual mission; compares live xy against saved `REFERENCE_XY` |
| `task_manager_mecanum_colore.py` | Asks which saved color each `colore_on` should detect and runs the mission |

No YAML color reference is required: saved references live in
`colore_detect_node.py`.

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
| `mypackage/mypackage/colore_calibrate_node.py` | `src/mypackage/mypackage/` |
| `mypackage/mypackage/colore_detect_node.py` | `src/mypackage/mypackage/` |
| `mypackage/mypackage/task_manager_mecanum_colore.py` | `src/mypackage/mypackage/` |
| `mypackage/launch/colore_calibrate.launch.py` | `src/mypackage/launch/` |
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
