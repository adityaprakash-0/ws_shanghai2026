# Autonomous frontier mapping

```bash
cd ~/skill23_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select explore_lite_msgs explore_lite studica_control --symlink-install
source install/setup.bash

ros2 launch studica_control autonomous_mapping.launch.py robot:=xdrive
# robot:= grip | mecanum | xdrive | allterrain
```

Stack order: Titan → lidars → drive → EKF → slam_toolbox (sync) → Nav2 navigation → `explore_lite`.

Vendored explorer: `src/m_explore_ros2/` (`explore_lite` + `explore_lite_msgs`).
