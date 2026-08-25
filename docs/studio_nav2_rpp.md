# Studio Nav2 — Regulated Pure Pursuit (RPP)

How Humble RPP follows a path in `skill23_studio`, using the **grip** FollowPath block in `src/skill23_studio/config/nav2_params_grip.yaml`.

Source of truth: Humble `nav2_regulated_pure_pursuit_controller` (`computeVelocityCommands`, `applyConstraints`, `rotateToHeading`). Docs: [nav2_regulated_pure_pursuit_controller (Humble)](https://docs.ros.org/en/humble/p/nav2_regulated_pure_pursuit_controller/).

RPP does **not** track the orange line like a train. Each controller tick (~7 Hz) it picks a point ahead on the path (the **carrot**), then publishes only:

- `cmd_vel.linear.x` (`vx`)
- `cmd_vel.angular.z` (`wz`)

---

## Current grip numbers

| Item | Value |
|---|---|
| Controller rate | 7 Hz |
| Goal XY / yaw | 0.10 m / 0.15 rad |
| Progress checker | 0.5 m in 15 s |

```yaml
desired_linear_vel: 0.30
lookahead_dist: 0.15
min_lookahead_dist: 0.15
max_lookahead_dist: 0.40
lookahead_time: 0.8
use_velocity_scaled_lookahead_dist: false
transform_tolerance: 1.0
min_approach_linear_velocity: 0.1
approach_velocity_scaling_dist: 0.40
use_collision_detection: false
max_allowed_time_to_collision_up_to_carrot: 0.6
use_regulated_linear_velocity_scaling: true
regulated_linear_scaling_min_radius: 0.30
regulated_linear_scaling_min_speed: 0.20
use_cost_regulated_linear_velocity_scaling: false
cost_scaling_dist: 0.30
cost_scaling_gain: 2.0
use_rotate_to_heading: true
rotate_to_heading_angular_vel: 0.25
rotate_to_heading_min_angle: 0.30
allow_reversing: false
```

`max_angular_vel` is **not** a Humble RPP parameter. If present in YAML it is ignored. Tracking `wz` is `vx * curvature`; in-place spin uses `rotate_to_heading_angular_vel`.

---

## Two circles (do not mix them)

### Circle 1 — lookahead (pick the carrot)

A circle of radius **`lookahead_dist` (0.15 m) centred on the robot**.

RPP walks the path and takes the first pose that is **at least 0.15 m** away. That point is the carrot. The **whole path does not lie on this circle** — only the carrot sits on the rim.

```
              path to the waypoint
                    ────────●──── carrot (on the 0.15 m rim)
                   /
        0.15 m    /
      ○········●········○     lookahead circle
      ·        ▲ robot   ·
      ·        | heading ·
      ○··················○
```

- Inside the circle: too close to be the carrot  
- On the rim: carrot (aim here this tick)  
- Outside: further along the path (ignored this tick)  
- If nothing is 0.15 m away (almost at the goal): use the **last** pose  

With `use_velocity_scaled_lookahead_dist: false`, lookahead stays **0.15 m**. `min_lookahead_dist`, `max_lookahead_dist`, and `lookahead_time` are unused.

### Circle 2 — tracking radius `R` (how tight the turn is)

After the carrot is chosen, RPP asks: *if I keep my current heading and must pass through that carrot, what circular arc is that?*

That arc:

- starts at the robot  
- matches heading (straight out the nose)  
- goes through the carrot  

Its radius is **`R`**. This is **not** 0.15 m.

If the carrot is on the 0.15 m rim at heading error `α`:

`R = 0.15 / (2 × sin(α))`

| Carrot angle `α` | Lookahead circle | Tracking `R` |
|---|---|---|
| 0° (straight) | carrot in front | huge (almost a line) |
| 5° | on the 0.15 m rim | ≈ 0.86 m |
| 15° | on the 0.15 m rim | ≈ 0.29 m |
| 17° (`rotate_to_heading_min_angle`) | on the rim | ≈ 0.26 m |

**Example:** carrot 15° left on the 0.15 m rim → lookahead radius is still 0.15 m, but driving `R ≈ 0.29 m`. Two numbers, two jobs.

`regulated_linear_scaling_min_radius: 0.30` means: *if this driving circle is tighter than 0.30 m, start slowing* (only while still **driving**).

---

## Three modes (one per tick)

Order in code: **A, then B, then C**.

```
Pick carrot (0.15 m)
        │
        ▼
  carrot < 0.10 m XY? ──yes──► A  vx=0, spin to goal yaw
        │ no
        ▼
  angle to carrot > 17°? ──yes──► B  vx=0, spin to face carrot
        │ no
        ▼
        C  drive: vx then wz = vx × k
```

### A — At the goal

**When:** distance to carrot **< `xy_goal_tolerance` (0.10 m)**.

| Param | Role |
|---|---|
| `use_rotate_to_heading: true` | Enables this mode |
| `rotate_to_heading_angular_vel: 0.25` | `wz = ±0.25` |
| `lookahead_dist` | Still used to define the carrot |
| `transform_tolerance` | TF |

**Command:** `vx = 0`, `wz = ±0.25` until yaw is within **0.15 rad**.

Turn slowdown, approach slowdown, cruise 0.30, cost scaling, collision time: **off**.

### B — Pointing the wrong way

**When:** not A, and `|atan2(carrot_y, carrot_x)| > 0.30 rad (~17°)**.

| Param | Role |
|---|---|
| `use_rotate_to_heading: true` | Enables this mode |
| `rotate_to_heading_min_angle: 0.30` | Spin if more than ~17° off |
| `rotate_to_heading_angular_vel: 0.25` | Spin at ~14°/s |
| `lookahead_dist` | Carrot used to measure that angle |

**Command:** `vx = 0`, `wz = ±0.25` until the nose is within 17° of the carrot.

A **90° planner corner** almost always lands here (carrot jumps to the side, angle ~90° > 17°). The robot **stops and spins**, then drives the next straight. It does **not** creep at 0.20 around the square.

If `use_rotate_to_heading` were `false`, RPP would keep `vx > 0` and turn while driving (more corner-cutting). Cannot combine rotate-to-heading with `allow_reversing: true` (Humble disables reversing).

### C — Normal driving

**When:** farther than 0.10 m **and** heading error ≤ 17°.

Start `vx = desired_linear_vel` (0.30), then constraints, then `wz = vx * k` with `k = 2y / (x²+y²)`.

---

## Mode C speed pipeline (order matters)

1. `vx = 0.30`
2. **Curvature scaling** (if enabled and `R < 0.30 m`): `vx *= R / 0.30`
3. **Turn floor:** `vx` not below **0.20**
4. **Approach scaling** (if remaining path length **< 0.40 m**): `vx *= dist_to_last / 0.40`
5. **Approach floor:** not below **0.10**
6. Clamp to `[0, 0.30]`; sign always forward (`allow_reversing: false`)

**Near the goal, approach wins.** The 0.20 floor is only for the turn step. Approach runs **after** and can take speed to **0.10**.

Example: last **0.20 m** **and** a tight turn so step 3 gives 0.20:

| Left | After turn floor | Approach | Result |
|---|---|---|---|
| 0.50 m | 0.20 | off | **0.20** |
| 0.20 m | 0.20 | × 0.5 → 0.10 | **0.10** |
| 0.10 m | 0.20 | would be 0.05 → floor 0.10 | **0.10** |

If heading error > 17°: Mode B, `vx = 0` — neither 0.20 nor 0.10 applies.

---

## Parameter notes

### Cruise and lookahead

| Param | Effect |
|---|---|
| `desired_linear_vel: 0.30` | Straight-line cruise (the real speed cap) |
| `lookahead_dist: 0.15` | Carrot 15 cm ahead. Short = tracks 90° rails tightly, twitchier. Long = smoother, cuts corners |
| `use_velocity_scaled_lookahead_dist: false` | Lookahead not `vx × lookahead_time` |
| `min/max_lookahead_dist`, `lookahead_time` | Unused while the flag is false |
| `transform_tolerance: 1.0` | Allow TF up to 1 s stale |
| `allow_reversing: false` | Never negative `vx` |

### Approach (last stretch only)

| Param | Effect |
|---|---|
| `approach_velocity_scaling_dist: 0.40` | Start slowing when remaining path **< 0.40 m** |
| `min_approach_linear_velocity: 0.10` | Creep floor on that stretch |

If remaining path **> 0.40 m**: no change.  
If **< 0.40 m**: `vx = 0.30 × (distance_to_last / 0.40)`, not below 0.10, until Mode A (`vx = 0`).

Larger 0.40 → brakes earlier. Without the floor, speed would go toward 0 and stall before the goal checker.

### Collision (local costmap only)

| Param | Effect |
|---|---|
| `use_collision_detection: false` | RPP does **not** abort on lethal cells |
| `max_allowed_time_to_collision_up_to_carrot: 0.6` | Unused while detection is false |

If detection is **true**: project the **footprint** on the **local** costmap (`odom`, 3×3 m), not global, for up to 0.6 s or until the carrot. Lethal (254) → throw, FollowPath fails. Soft inflation does not abort. It does not slow down; it **aborts**.

Stuck detection then is the **progress checker** (0.5 m in 15 s), then BT recovery.

### Curvature (tight turns while driving)

| Param | Effect |
|---|---|
| `use_regulated_linear_velocity_scaling: true` | Slow when tracking `R < 0.30 m` |
| `regulated_linear_scaling_min_radius: 0.30` | Threshold |
| `regulated_linear_scaling_min_speed: 0.20` | Floor after that slowdown only |

If the flag is **false**: while driving, stay at **0.30** even on a tight tracking circle. 90° corners still spin (Mode B). Last 0.40 m still uses approach scaling.

`R` is **not** “the map corner is 90°.” A square path looks like a small `R` to RPP because the carrot jumps sideways — but then Mode B usually runs instead of this slowdown.

### Cost scaling (off)

| Param | Effect |
|---|---|
| `use_cost_regulated_linear_velocity_scaling: false` | Inflation under the robot does not reduce `vx` |
| `cost_scaling_dist` / `cost_scaling_gain` | Unused |

### Rotate in place

| Param | Effect |
|---|---|
| `use_rotate_to_heading: true` | Modes A and B |
| `rotate_to_heading_min_angle: 0.30` | ~17°: above this, stop and spin |
| `rotate_to_heading_angular_vel: 0.25` | In-place `wz` |

Smaller min angle → spins more often, squarer 90° tracking. Larger (e.g. 45°) → drives through bigger heading errors, cuts corners.

| Turn needed | Spin time at 0.25 rad/s |
|---|---|
| 17° | ~1 s |
| 45° | ~3 s |
| 90° | ~6 s |

Humble also ramps `wz` with `max_angular_accel` (default 3.2 if unset) so spin does not jump instantly, and slows to avoid overshooting heading.

---

## What applies in each mode

| | A goal | B wrong way | C drive |
|---|---|---|---|
| `vx` | **0** | **0** | **0.30 → 0.20 → 0.10** |
| `wz` | **±0.25** | **±0.25** | **vx × k** |
| Lookahead 0.15 | yes (pick carrot) | yes | yes |
| Rotate vel 0.25 | **yes** | **yes** | no |
| Min angle 0.30 | no (already at goal) | **yes** | already decided not B |
| Curvature 0.30 / 0.20 | no | no | **yes** |
| Approach 0.40 / 0.10 | no | no | **yes** if path < 0.40 m |
| Collision 0.6 | unused (flag false) | unused | unused |
| Cost scaling | unused | unused | unused |

---

## Worked examples

### Straight 2 m then 90° then waypoint

1. Straight: carrot 15 cm ahead, `α ≈ 0°` → C, `R` huge, `vx ≈ 0.30`, `wz ≈ 0`
2. 90° corner: carrot jumps left, `α ≈ 90° > 17°` → **B**, `vx = 0`, `wz = +0.25` for ~6 s
3. Next straight: C again, `vx ≈ 0.30`
4. Last 0.40 m: approach, `vx` down toward **0.10**
5. Inside 0.10 m XY: **A**, `vx = 0`, spin to waypoint yaw at 0.25

### Mild bend while still facing the carrot (≤ 17°)

Carrot on 0.15 m rim, 15° left → `R ≈ 0.29 m` < 0.30 → slow toward `0.30 × 0.29/0.30`, floor **0.20**, still driving.

### Last 0.20 m with a tight curve, still Mode C

Turn floor 0.20, then approach `0.20 × (0.20/0.40) = 0.10` → **0.10 m/s** (approach wins).

---

## Humble vs extra YAML keys

Humble RPP **does** use: `desired_linear_vel`, lookahead set, approach pair, collision pair, curvature trio, cost trio, rotate-to-heading trio, `allow_reversing`, `transform_tolerance`, `max_angular_accel`.

Humble RPP **does not** use (ignored if present): `max_linear_vel`, `min_linear_vel`, `max_angular_vel`, `min_angular_vel`, `max_linear_accel`, `max_linear_decel`, `max_angular_decel`, `use_fixed_curvature_lookahead`, `curvature_lookahead_dist`, `use_dynamic_window`, `allow_obstacle_checking_beyond_goal`, YAML `min_distance_to_obstacle`.

---

## Related (not RPP, but next to it)

- **Goal checker:** XY 0.10 m / yaw 0.15 rad ends FollowPath after Mode A.
- **Progress checker:** if the robot barely moves 0.5 m in 15 s, FollowPath fails → BT recovery (`navigate_nav2.xml`).
- **Collision abort** (if enabled) uses **local** costmap only, not global.
- Recovery behaviours (backup, drive-on-heading) are the BT / `behavior_server`, not these RPP keys.
