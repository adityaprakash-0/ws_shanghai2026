# Odometry calibration — grip
Generated: 2026-08-24T21:03:55 (2 m tape corrected to 1.75 m)

## Test settings
- linear_duty (cmd_vel.linear.x): 0.25
- strafe_duty (cmd_vel.linear.y): 0.25
- move_duration_s_per_m (timer only): 5.0
- **/cmd_vel is Titan duty (−1…1), NOT m/s**

## Summary
- **Forward odom scale** (real/odom): **0.9848**
  → If odom reads high, scale encoders down (~1.0154× ticks).

## Trials

| motion | cmd(m) | duty | time(s) | odom_fwd | odom_lat | real(m) | scale | yaw(deg) |
|--------|--------|------|---------|----------|----------|---------|-------|----------|
| forward | 0.50 | 0.25 | 2.50 | +0.4067 | +0.0002 | 0.3950 | 0.9712 | +0.06 |
| forward | 1.00 | 0.25 | 5.00 | +0.8825 | +0.0002 | 0.8800 | 0.9972 | +0.09 |
| forward | 2.00 | 0.25 | 10.00 | +1.7746 | +0.0048 | 1.7500 | 0.9861 | +0.34 |
