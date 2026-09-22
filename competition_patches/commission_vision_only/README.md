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
3. Deploy module: **Commissioning**
4. **Start Nav2+camera** (commission) — Detect+Align on, Align on
5. Do **not** press Home OMS / Run OMS
6. **Start mission** (or physical START) — vision-only loop runs
7. Optional: `light_tower.enabled: true` in `config/hw_params/hw_params_<robot>.yaml` for final red

## Verify OMS did not start

While mission runs:

```bash
ros2 node list | grep -i oms || echo "no oms nodes"
ps aux | grep -E 'oms\.launch|test_oms|oms_all' | grep -v grep || echo "no oms processes"
```

## Restore full OMS commissioning later

Keep backups of the two original files before replacing, then restore them and rebuild `skill23_studio`.
