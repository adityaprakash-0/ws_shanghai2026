# Studica Colore color sensor — how it works (studica_control)

Reference for the **Colore** ROS 2 component in `studica_control`.  
Source of truth in this repo:

- `src/studica_ws/src/studica_control/src/components/colore_component.cpp`
- `src/studica_ws/src/studica_control/include/studica_control/colore_component.hpp`
- Messages: `msg/ColoreColorMsg.msg`, `msg/ColoreMatch.msg`
- Drivers: `drivers/colore.hpp` (CAN), `drivers/colore_usb.hpp` (USB)
- Params example: `config/params_template.yaml` under `colore:`

This README is documentation only (no code to copy unless you enable Colore in your own `params_*.yaml`).

---

## What Colore is

Studica **Colore** is a color sensor board. The ROS component talks to it over:

| Transport | How | Typical port / ID |
|-----------|-----|-------------------|
| **USB** | CDC serial text protocol | `/dev/ttyACM0` |
| **CAN** | Classic CAN 2.0B telemetry | `can_id` (often `0`) |

In both modes the component treats **CIE XYZ** as the source of truth, then derives:

- normalized **sRGB** for `/color`
- 0–255 RGB + hex for `/color_info`
- chromaticity **xy** for optional color **matching**

It runs inside **`control_server`** (`manual_composition`) when `colore.enabled: true`.

---

## Enable it (params)

Under `control_server.ros__parameters` in your HW params (e.g. `params_mecanum.yaml`):

```yaml
colore:
  enabled: true
  sensors: ["colore0"]          # one node per name
  colore0:
    transport: "usb"            # "usb" | "can" (also accepts "serial" → usb)
    can_id: 0                   # used when transport: can
    serial_port: "/dev/ttyACM0" # used when transport: usb
    publish_rate_hz: 10         # publish + sample rate
    publish_outputs: ["info"]   # optional extras: "info", "match"
    brightness: 50              # LED brightness 0–255; <0 skips set
    measmode: "off"             # off | auto | fixed  (startup only)
    measmode_z_mm: -1.0         # only for measmode: fixed
    # match_references: ["red", "blue"]   # see matching section
    # match_red_xy: [0.45, 0.32]
    # match_blue_xy: [0.18, 0.15]
    match_threshold: 0.05
```

**Humble tip:** do not put an empty YAML list `match_references: []` — omit the key instead (comment in template).

After editing params, restart `control_server` / your launch that loads that params file.

---

## Parameters (detailed)

| Parameter | Meaning |
|-----------|---------|
| `colore.enabled` | Load Colore component(s) into `control_server`. |
| `colore.sensors` | List of sensor instance names (each becomes a ROS node with that name). |
| `colore.<name>.transport` | `"usb"` / `"serial"` or `"can"`. |
| `colore.<name>.can_id` | CAN device id for CAN transport. |
| `colore.<name>.serial_port` | USB serial device path. |
| `colore.<name>.publish_rate_hz` | How often to publish. Also sets sensor sample interval ≈ `1000/rate` ms (clamped 20–2000 ms). |
| `colore.<name>.publish_outputs` | Extra topics: `"info"` → `/color_info`, `"match"` → `/color_match`. `/color` is always published. |
| `colore.<name>.brightness` | Onboard illuminator brightness. Applied at startup if `>= 0`. |
| `colore.<name>.measmode` | Measurement / multi-flash mode at **startup only**: `off`, `auto`, `fixed`. Not a runtime `colore_cmd`. |
| `colore.<name>.measmode_z_mm` | Distance (mm) when `measmode: fixed`. |
| `colore.<name>.match_references` | List of label names for xy matching (e.g. `["red","green"]`). |
| `colore.<name>.match_<label>_xy` | `[x, y]` chromaticity reference for that label. |
| `colore.<name>.match_threshold` | Max xy distance to count as a match (default `0.05`). |

---

## Topics published

Assume sensor name `colore0` → node name `colore0`.

### Always: `/colore0/color` — `std_msgs/ColorRGBA`

- `r`, `g`, `b` — **normalized sRGB 0.0–1.0** derived from XYZ  
- `a` — always `1.0`

```bash
ros2 topic echo /colore0/color
```

### Optional: `/colore0/color_info` — `studica_control/msg/ColoreColorMsg`

Enabled when `publish_outputs` contains `"info"` (or `"color_info"`).

| Field | Meaning |
|-------|---------|
| `seq` | Sample sequence from sensor |
| `x`, `y`, `z` | CIE XYZ |
| `r`, `g`, `b` | 0–255 sRGB bytes |
| `hex` | e.g. `"#RRGGBB"` |

```bash
ros2 topic echo /colore0/color_info
```

### Optional: `/colore0/color_match` — `studica_control/msg/ColoreMatch`

Enabled when `publish_outputs` contains `"match"` (or `"color_match"`) **and** you have references.

| Field | Meaning |
|-------|---------|
| `x`, `y` | Current chromaticity (X/(X+Y+Z), Y/(X+Y+Z)) |
| `label` | Closest reference name, or `"unknown"` |
| `confidence` | `1 - dist/threshold` if within threshold, else `0` |

Matching: Euclidean distance in xy to each reference; pick nearest; accept if `dist <= match_threshold`.

```bash
ros2 topic echo /colore0/color_match
```

Example with matching:

```yaml
colore0:
  transport: "usb"
  serial_port: "/dev/ttyACM0"
  publish_rate_hz: 10
  publish_outputs: ["info", "match"]
  brightness: 50
  measmode: "off"
  match_references: ["red", "blue"]
  match_red_xy: [0.50, 0.32]
  match_blue_xy: [0.18, 0.15]
  match_threshold: 0.05
```

---

## Service: `/colore0/colore_cmd` — `studica_control/srv/SetData`

| `params` | Extra fields | Effect |
|----------|--------------|--------|
| `get_config` | — | Read sensor config string / CAN colorformat |
| `set_brightness` | `initparams.n_encoder` = 0–255 | Change brightness at runtime |
| `learn_color,<name>` | — | Snapshot **current** xy as reference `<name>` (runtime; updates/adds to in-memory list) |

Examples:

```bash
# Read config
ros2 service call /colore0/colore_cmd studica_control/srv/SetData "{params: 'get_config'}"

# Brightness 80 (uses initparams.n_encoder as the int value)
ros2 service call /colore0/colore_cmd studica_control/srv/SetData \
  "{params: 'set_brightness', initparams: {n_encoder: 80}}"

# Learn current color as reference "cone_red"
ros2 service call /colore0/colore_cmd studica_control/srv/SetData \
  "{params: 'learn_color,cone_red'}"
```

`learn_color` needs at least one successful XYZ sample first (`have_last_xy_`).  
References learned at runtime are **in memory only** (lost on restart) unless you also write them into YAML.

**Not a command:** `measmode` — set only in params at startup.

---

## Internal data flow (how it works)

```text
Sensor (USB serial or CAN XYZ telem)
        │
        ▼
  publish_color() timer @ publish_rate_hz
        │
        ├─ fill XYZ
        ├─ XYZ → sRGB  → publish /color
        ├─ RGB + hex   → publish /color_info  (if info)
        └─ XYZ → xy    → publish /color_match (if match + refs)
```

USB path: firmware is configured to stream **XYZ** (`ConfigureStreaming("xyz", sample_ms)`).  
CAN path: reads `COLORE_CAN_TELEM_XYZ` frames (values scaled ×10000 as int16).

---

## How to use it day-to-day

1. Plug Colore (USB or CAN).  
2. Set `colore.enabled: true` and transport/port in params.  
3. Launch HW (`studica_launch` / your teleop / task launch that uses that params file).  
4. Confirm topics:

```bash
ros2 topic list | grep colore
ros2 topic hz /colore0/color
ros2 topic echo /colore0/color_info
```

5. For classification: either put `match_*_xy` in YAML, or call `learn_color,<name>` while pointing at the object, then enable `"match"` in `publish_outputs`.  
6. In your own node, subscribe to `/colore0/color_match` and act when `label != "unknown"`.

---

## Quick troubleshooting

| Symptom | Likely cause |
|---------|----------------|
| No `/colore0/...` topics | `colore.enabled: false` or wrong params file |
| USB open error | Wrong `serial_port`, permissions, or another process holding ttyACM0 |
| Samples but no publish (USB) | No XYZ line yet — wait or check firmware streaming |
| Match always `unknown` | No refs / threshold too tight / bad lighting |
| `learn_color` fails | No color data yet — wait for `/color` first |
| Empty `match_references: []` crash | Omit the key on Humble |

---

## Related files in the tree

```text
studica_control/
  src/components/colore_component.cpp
  include/studica_control/colore_component.hpp
  msg/ColoreColorMsg.msg
  msg/ColoreMatch.msg
  config/params_template.yaml   # documented defaults
drivers/
  colore.hpp / colore.cpp       # CAN
  colore_usb.hpp / colore_usb.cpp  # USB
```
