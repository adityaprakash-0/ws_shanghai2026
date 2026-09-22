# Competition patch — vision-only commissioning (no OMS)

Offline copy kit for a competition workspace **without internet**.

Replace the two Python files below on the Pi, rebuild `skill23_studio`, restart Studio, then run Commissioning as usual (**Start Nav2+camera** → **Start mission**).

## What this changes

Mission flow after **Start mission** (commission module):

1. Drive to next **pick** sticker (`pick1`, `pick2`, …)
2. Optional base yaw align (no OMS)
3. **Classify only**: color stream ON → YOLO → color OFF → `class_name`
4. Drive to drop from dashboard **drop assignments** (Cylinder → dropN, …)
5. **Wait 5 seconds** at drop (no OMS)
6. Repeat until pick counts done → drive **home** sticker
7. **Solid red** on light tower (enable `light_tower` in `hw_params` if needed)

**OMS does not launch** during commission:

- Deploy sets `oms_enabled=false` for commission
- Pick/drop never call `ensure_oms`
- Home OMS / OMS start is blocked while deploy module is `commission`
- Launch was already `with_oms:=false` in `studio_commission.launch.py`

Nav module + full OMS pick code paths are left in the repo; commission simply bypasses them.

---

## What is **pick count**? (example)

**Count** = how many times that pick location must still be visited before the mission is done with it.

Set per pick waypoint in the Commissioning dashboard (**Count** / `pick_count` on `pick1`, `pick2`, …). Default is **1**.

### Example

Map stickers:

- `pick1` with **Count = 2**
- `pick2` with **Count = 1**
- `drop1` = Cylinder, `drop2` = Semisphere
- `home`

What happens:

1. Go `pick1` → see Cylinder → go `drop1` → wait 5s → `pick1` remaining **1**
2. Go `pick2` → see Semisphere → go `drop2` → wait 5s → `pick2` remaining **0**
3. Go `pick1` again (still 1 left) → classify → drop → wait → `pick1` remaining **0**
4. All counts 0 → go **home** → solid red

So Count=2 on `pick1` means “visit this pick twice” (two objects over the mission), not “detect two classes in one stop.”

---

## Where to change **object names**

Names must match **everywhere** (dashboard list ↔ Python ↔ YOLO `metadata.yaml`).

| What | File on Pi |
|---|---|
| Backend list + drop map keys | `src/skill23_studio/skill23_studio/commission_mission.py` → `COMMISSION_OBJECTS = ('Cylinder', 'Semisphere', 'Rectangle', 'Mystery')` |
| Laptop dashboard dropdown | `laptop/skill23_studio/js/app.js` → `COMMISSION_OBJECTS = [...]` |
| YOLO class names | `src/skill23_studio/models/yolov8n_ncnn/metadata.yaml` → `names:` (must match training labels) |

Example: rename `Semisphere` → `Sphere`:

1. Change the string in `commission_mission.py` and `app.js`
2. Retrain / export NCNN so `metadata.yaml` has `Sphere`
3. Rebuild `skill23_studio` and refresh the laptop page

If YOLO says `Cylinder` but the dashboard only lists other names, drop routing will fail (“no drop assigned”).

---

## Where to put your trained **NCNN** model (`best_ncnn` folder)

Default folder the node loads:

```text
~/skill23_ws/src/skill23_studio/models/yolov8n_ncnn/
```

Required files **inside that folder** (exact names):

```text
model.ncnn.param
model.ncnn.bin
metadata.yaml
```

### What to do with your `best_ncnn` / export folder

1. Copy those three files into  
   `src/skill23_studio/models/yolov8n_ncnn/`  
   (overwrite the old ones).
2. Rebuild so install share gets them:

```bash
cd ~/skill23_ws
colcon build --packages-select skill23_studio
source install/setup.bash
```

3. Restart Studio / Detect+Align (Start Nav2+camera again).

### Optional: keep the model somewhere else

In `src/skill23_studio/config/wp_detect_align.yaml` set:

```yaml
model_dir: /home/vmx/skill23_ws/src/skill23_studio/models/yolov8n_ncnn
```

(or any absolute path that contains `model.ncnn.param` + `model.ncnn.bin`). Empty `model_dir: ''` = use the default folder above.

Do **not** leave the folder named only `best_ncnn` unless you either rename it to `yolov8n_ncnn` under `models/` or point `model_dir` at it. The code looks for `model.ncnn.param` / `model.ncnn.bin` inside the directory you give it.

---

## Files to replace on the competition machine

Copy from this folder into your `skill23_ws` (same relative paths):

| File in this folder | Replace on competition Pi |
|---|---|
| `skill23_studio/studio_bridge_node.py` | `src/skill23_studio/skill23_studio/studio_bridge_node.py` |
| `skill23_studio/nav/detect_align_node.py` | `src/skill23_studio/skill23_studio/nav/detect_align_node.py` |

Example (on the Pi, from `~/skill23_ws`):

```bash
# USB / scp these two files into place, then:
cp /path/to/competition_patches/commission_vision_only/skill23_studio/studio_bridge_node.py \
   src/skill23_studio/skill23_studio/studio_bridge_node.py

cp /path/to/competition_patches/commission_vision_only/skill23_studio/nav/detect_align_node.py \
   src/skill23_studio/skill23_studio/nav/detect_align_node.py

source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select skill23_studio
source install/setup.bash

# Restart Studio as you usually do, e.g.:
# sudo -E env PATH=$PATH PYTHONPATH=$PYTHONPATH LD_LIBRARY_PATH=$LD_LIBRARY_PATH \
#   ros2 launch skill23_studio studio.launch.py
```

## Dashboard checklist

1. Connect to Pi → choose map with stickers: `pick1`…, `drop1`…, `home`
2. Set **drop assignments** (object class → drop sticker)
3. Set **Count** on each pick WP if you need multiple visits (default 1)
4. Deploy module: **Commissioning**
5. **Start Nav2+camera** (commission) — Detect+Align on, Align on
6. Do **not** press Home OMS / Run OMS
7. **Start mission** (or physical START) — vision-only loop runs
8. Optional: `light_tower.enabled: true` in `config/hw_params/hw_params_<robot>.yaml` for final red

## Verify OMS did not start

While mission runs:

```bash
ros2 node list | grep -i oms || echo "no oms nodes"
ps aux | grep -E 'oms\.launch|test_oms|oms_all' | grep -v grep || echo "no oms processes"
```

## Restore full OMS commissioning later

Keep backups of the two original files before replacing, then restore them and rebuild `skill23_studio`.
