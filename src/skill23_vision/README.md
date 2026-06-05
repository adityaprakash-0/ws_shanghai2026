# Optional Orbbec color vision for competition tasks.
#
# Default: OFF — normal navigation only (no camera CPU).
#
# Enable with either dashboard mode:
#   ros2 launch adev_dashboard nav2_dashboard.launch.py robot:=mecanum enable_vision:=true
#   ros2 launch adev_dashboard nav2_secret_mode_dashboard.launch.py robot:=mecanum enable_vision:=true
#
# CPU design:
#   - The Orbbec driver node stays launched, but vision_gate calls the driver's
#     /camera/toggle_color service to STOP the hardware color stream while idle.
#   - During look_color (or Preview ON), it starts the stream, subscribes,
#     detects HSV colors and publishes JPEG preview.
#   - After the block it unsubscribes and stops the hardware stream again.
#   - enable_color must remain true in the launch configuration; otherwise the
#     Orbbec driver does not create /camera/toggle_color.
#
# Laptop Task Builder (adev_dashboard):
#   Toggle "Use task blocks" → stack Navigate / Look color / Wait → Save JSON → Mission Start.
#
# Example task JSON:
# {
#   "version": 1,
#   "name": "find_red",
#   "steps": [
#     {"type": "navigate", "x": 1.0, "y": 2.0, "yaw": 0.0, "ignore_yaw": true},
#     {"type": "look_color", "colors": ["red", "blue"], "timeout_sec": 5.0,
#      "on_found": "continue", "on_miss": "continue"},
#     {"type": "wait", "seconds": 1.0}
#   ]
# }
#
# Colors: red orange yellow green blue purple white black
