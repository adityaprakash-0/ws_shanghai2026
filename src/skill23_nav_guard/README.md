# skill23_nav_guard

Nav2 + per-wheel IR/ultrasonic safety guard (pulse lateral nudge).

## Sensor layout

| Wheel | Sensor | Topic |
|-------|--------|-------|
| Front-left | Ultrasonic | `/ultrasonic_fl/range` |
| Front-right | IR | `/ir_fr/range` |
| Rear-left | Ultrasonic | `/ultrasonic_rl/range` |
| Rear-right | IR | `/ir_rr/range` |

## cmd_vel flow

```
Nav2 RPP -> velocity_smoother -> /cmd_vel_nav_smoothed -> wheel_safety_guard -> /cmd_vel -> drive node
```

Launch via dashboard mode **nav2_guard** (`launch_adev_dashboard.sh`).
