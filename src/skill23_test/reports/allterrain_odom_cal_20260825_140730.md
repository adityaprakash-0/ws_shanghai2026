# Odometry calibration — allterrain
Generated: 2026-08-25T14:07:30

## Test settings
- linear_duty (cmd_vel.linear.x): 0.25
- strafe_duty (cmd_vel.linear.y): 0.25
- move_duration_s_per_m (timer only): 5.0
- **/cmd_vel is Titan duty (−1…1), NOT m/s**

## Summary
- **Forward odom scale** (real/odom, multiply odom to get meters): **0.9966**
  → If odom reads high, scale encoders down (~1.0034× ticks).

## EKF notes
- Tune `allterrain/config/ekf_allterrain.yaml` from this test (per-robot EKF, not skill23_custom_nav).
- Compare wheel odom scale to 1.0 before trusting x,y,vx fusion.
- Large IMU yaw drift during straight runs → check IMU mount / calibration.
- If stuck wheels still advance odom, reduce EKF trust on wheel x,y,vx.

## Trials

| motion | cmd(m) | duty(−1…1) | time(s) | odom_fwd | odom_lat | real(m) | scale | yaw(deg) |
|--------|--------|------------|---------|----------|----------|---------|-------|----------|
| forward | 0.50 | 0.25 | 2.50 | +0.5357 | -0.0011 | 0.5300 | 0.9893 | -0.02 |
| forward | 1.00 | 0.25 | 5.00 | +1.0764 | +0.0018 | 1.0750 | 0.9987 | +0.24 |
| forward | 2.00 | 0.25 | 10.00 | +2.1708 | +0.0114 | 2.1750 | 1.0019 | +0.60 |
