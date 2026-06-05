# skill23_pick_place

Standalone **pick-and-place** stack for allterrain / mecanum / xdrive.

- **Does not** use Nav2 or the ADEV mission runner
- Bridge on **HTTP :8081** (ADEV stays on :8080)
- Elevator: `titan1` CAN **43**, motor **m_0**
- Linear servo PWM **14**, gripper PWM **15**
- Navigation: wheel odometry + ultrasonic/IR fusion (no LiDAR)

## Bring-up (Pi)

```bash
cd ~/skill23_ws
source /opt/ros/humble/setup.bash
# after colcon build:
source install/setup.bash

~/skill23_ws/src/studica_ws/scripts/ros2_pi.sh launch skill23_pick_place pick_place.launch.py \
  robot:=mecanum

# optional camera align / preview
~/skill23_ws/src/studica_ws/scripts/ros2_pi.sh launch skill23_pick_place pick_place.launch.py \
  robot:=mecanum enable_vision:=true
```

`robot:=` also accepts `allterrain` and `xdrive`.

**Do not** run Nav2 / ADEV dashboard launches at the same time on the same VMX — they share ultrasonics / Sharp / Titan buses.

## Laptop UI

```bash
cd laptop/pick_place_dashboard
chmod +x start.sh
./start.sh          # http://localhost:3001
```

Set Pi Bridge URL to `http://<pi-ip>:8081`.

Features: load occupancy map, place object/station stickers, build
`goto → align → pick → goto → place` missions, live pose/ranges/arm, E-stop.

## Calibration checklist

1. **CAN 43** — confirm Titan1 responds; watch `/titan1/m_0/encoder` while jogging elevator.
2. **Elevator targets** — edit `elevator_targets_json` in `config/pp_<robot>.yaml` (`up`, `up_mid`, `down_mid`, `down` counts).
3. **Servo angles** — tune `linear_angles_json` (`retract` / `extend`) and `gripper_angles_json` (`open` / `close`).
4. **Encoder sign** — if elevator runs away from target, flip `invert_motor` / `invert_encoder` under `titan1.m_0` in `config/params_<robot>_pp.yaml`.
5. **Wheel scale** — drivetrain `dist_per_tick` already feeds `/wheel/odom` → `/pp/odom`.
6. **Cobra tape** — place black tape; tune `tape_threshold` and `confirm_frames` under `pp_align_cobra`.
7. **Camera align** — only with `enable_vision:=true`; colors use `skill23_vision` HSV table.

## Mission schema

```json
{
  "version": 1,
  "name": "pick_red_box",
  "steps": [
    {"type": "goto", "x": 1.2, "y": 0.4, "yaw": 0.0, "xy_tol": 0.02, "yaw_tol": 0.05},
    {"type": "align", "mode": "cobra", "timeout_sec": 8, "confirm_frames": 20},
    {"type": "pick", "elevator": "down", "extend": "extend", "gripper": "close", "elevator_after": "up"},
    {"type": "goto", "x": 0.3, "y": 0.4, "yaw": 3.14},
    {"type": "align", "mode": "camera", "colors": ["red"], "timeout_sec": 8},
    {"type": "place", "elevator": "down", "extend": "extend", "gripper": "open", "elevator_after": "up"}
  ]
}
```

### Arrival gate

After every `goto`, the mission manager:

1. Waits for the go-to-goal controller result  
2. Re-samples `/pp/odom` for N settle frames  
3. Requires `|dx|,|dy| < xy_tol`, `|dyaw| < yaw_tol`, near-zero `/pp/cmd_vel`  
4. Only then allows `align` / `pick` / `place`  
5. Never opens the gripper if arrival failed  

## Package layout

| Path | Role |
|------|------|
| `config/params_*_pp.yaml` | Studica hardware (titan0+titan1, servos, US, IR, Cobra) |
| `config/pp_*.yaml` | Software node params |
| `launch/pick_place.launch.py` | Full PP bring-up |
| `laptop/pick_place_dashboard/` | UI (SCP to laptop) |

## Topics (namespaced)

| Topic | Meaning |
|-------|---------|
| `/pp/odom` | PP odometry frame |
| `/pp/cmd_vel` | PP velocity (relayed to `/cmd_vel`) |
| `/pp/dis_data` | `[front_us, back_us, left_ir, right_ir]` |
| `/pp/nav/goto` | Absolute pose command (JSON) |
| `/pp/align/cobra` / `/pp/align/camera` | Align commands |
| `/pp/manipulator/cmd` | Elevator + servos |
| `/pp/mission/cmd` | `set_task` / `start` / `stop` |
