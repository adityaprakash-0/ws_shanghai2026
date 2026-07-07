# ADEV Dashboard (Pi side)

The **full step-by-step guide** for laptop + Pi setup is here:

**`~/skill23_ws/laptop/adev_dashboard/README.md`**

Copy that folder to your laptop (`~/Music/adev_dashboard`) and follow the README there.

Quick Pi launch (use **sudo prefix** so VMX topics like `/battery_state` are visible to all nodes):

```bash
source ~/skill23_ws/install/setup.bash
~/skill23_ws/src/studica_ws/scripts/ros2_pi.sh launch adev_dashboard adev_dashboard.launch.py robot:=allterrain
```

Or the shortcut script:

```bash
~/skill23_ws/src/adev_dashboard/scripts/launch_adev_dashboard.sh allterrain
```

For any `ros2` command on the Pi (topics, nodes, echo), use the same prefix:

```bash
~/skill23_ws/src/studica_ws/scripts/ros2_pi.sh topic list
~/skill23_ws/src/studica_ws/scripts/ros2_pi.sh topic echo /battery_state --once
```
