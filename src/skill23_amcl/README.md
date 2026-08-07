# skill23_amcl

Vendored **Navigation2 Humble `nav2_amcl`** (C++), packaged as `skill23_amcl` for the
custom navigation stack.

- ROS package name: `skill23_amcl`
- C++ namespace / includes: still `nav2_amcl::*` (upstream source fidelity)
- Executable: `amcl` (lifecycle node — activate via `nav2_lifecycle_manager`)
- I/O: `/map`, `/scan`, `/odometry` (or remapped), `/initialpose` → `/amcl_pose`, TF `map→odom`

## Build (do once at end of edits)

```bash
cd ~/skill23_ws
colcon build --packages-select skill23_amcl --symlink-install
source install/setup.bash
```

## Run

Prefer `skill23_custom_nav` launch, or:

```bash
ros2 launch skill23_amcl amcl.launch.py params_file:=/path/to/custom_nav2_param.yaml
```

Upstream license: LGPL-2.1-or-later (Andrew Howard / Nav2).
