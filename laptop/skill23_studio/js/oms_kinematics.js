/** OMS forward kinematics — hierarchical joint model (digital twin).
 *  +X front, +Y left, +Z up. Demo mode uses this without hardware feedback.
 */
(() => {
  const DEG = Math.PI / 180;

  const DEFAULT_CONFIG = {
    oms_rotation_min_deg: -90,
    oms_rotation_max_deg: 90,
    lift_lower_z_mm: 155,
    lift_upper_z_mm: 455,
    lift_travel_mm: 300,
    linear_home_x_mm: 220,
    linear_max_extension_mm: 110,
    linear_travel_mm: 110,
    linear_ref_z_lower_lift_mm: 110,
    linear_ref_z_upper_lift_mm: 375,
    rotation_joint_offset_mm: 110,
    gripper_length_mm: 200,
    total_oms_height_mm: 575,
    turntable_diameter_mm: 180,
    rail_height_mm: 575,
    camera_tilt_deg: 35,
    camera_tx_mm: 0,
    camera_ty_mm: 0,
    camera_tz_mm: 520,
    vertical_servo_down_deg: -90,
    vertical_servo_up_deg: 90,
    rotation_servo_in_deg: -30,
    rotation_servo_out_deg: -150,
    // aliases kept for older saved viz configs
    rotation_servo_horizontal_deg: -30,
    rotation_servo_vertical_deg: -150,
    gripper_open_norm: 0,
    gripper_closed_norm: 1,
  };

  function clamp(v, lo, hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  function mergeConfig(cfg) {
    return { ...DEFAULT_CONFIG, ...(cfg || {}) };
  }

  function linearMaxMm(cfg) {
    const c = mergeConfig(cfg);
    if (c.linear_max_extension_mm > 0) return c.linear_max_extension_mm;
    return Math.max(1, c.linear_travel_mm || 145);
  }

  function clampJoints(j, cfg) {
    const c = mergeConfig(cfg);
    const liftMax = c.lift_travel_mm > 0
      ? c.lift_travel_mm
      : (c.lift_upper_z_mm - c.lift_lower_z_mm);
    return {
      rotation_deg: clamp(Number(j.rotation_deg) || 0, c.oms_rotation_min_deg, c.oms_rotation_max_deg),
      lift_mm: clamp(Number(j.lift_mm) || 0, 0, liftMax),
      linear_mm: clamp(Number(j.linear_mm) || 0, 0, linearMaxMm(c)),
      vertical: j.vertical === 'up' ? 'up' : 'down',
      rotation_servo: (
        j.rotation_servo === 'out'
        || j.rotation_servo === 'vertical'
        || j.rotation_servo === 'v'
      ) ? 'out' : 'in',
      gripper: clamp(Number(j.gripper) || 0, 0, 1),
    };
  }

  function homeJoints() {
    return {
      rotation_deg: 0,
      lift_mm: 0,
      linear_mm: 0,
      vertical: 'down',
      rotation_servo: 'out',
      gripper: 0,
    };
  }

  function liftPhysicalZmm(joints, cfg) {
    const c = mergeConfig(cfg);
    return c.lift_upper_z_mm - (Number(joints.lift_mm) || 0);
  }

  function linearRefZmm(joints, cfg) {
    const c = mergeConfig(cfg);
    const liftMax = c.lift_travel_mm > 0
      ? c.lift_travel_mm
      : (c.lift_upper_z_mm - c.lift_lower_z_mm);
    const t = liftMax > 0 ? clamp((Number(joints.lift_mm) || 0) / liftMax, 0, 1) : 0;
    return c.linear_ref_z_upper_lift_mm
      + t * (c.linear_ref_z_lower_lift_mm - c.linear_ref_z_upper_lift_mm);
  }

  function limitSwitches(joints, cfg) {
    const c = mergeConfig(cfg);
    const j = clampJoints(joints, c);
    const liftMax = c.lift_travel_mm > 0
      ? c.lift_travel_mm
      : (c.lift_upper_z_mm - c.lift_lower_z_mm);
    const linMax = linearMaxMm(c);
    const rotTol = 0.5;
    const liftTol = 1.0;
    const linTol = 1.0;
    return {
      rotation_home: Math.abs(j.rotation_deg) <= rotTol,
      lift_upper: j.lift_mm <= liftTol,
      lift_lower: j.lift_mm >= liftMax - liftTol,
      linear_home: j.linear_mm <= linTol,
    };
  }

  // --- 4x4 column-major matrices (for FK chain) ---
  function m4() {
    return new Float64Array([
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1,
    ]);
  }

  function m4Mul(a, b) {
    const o = m4();
    for (let c = 0; c < 4; c += 1) {
      for (let r = 0; r < 4; r += 1) {
        o[c * 4 + r] = (
          a[r] * b[c * 4 + 0]
          + a[4 + r] * b[c * 4 + 1]
          + a[8 + r] * b[c * 4 + 2]
          + a[12 + r] * b[c * 4 + 3]
        );
      }
    }
    return o;
  }

  function m4T(x, y, z) {
    const o = m4();
    o[12] = x; o[13] = y; o[14] = z;
    return o;
  }

  function m4Rz(rad) {
    const c = Math.cos(rad);
    const s = Math.sin(rad);
    const o = m4();
    o[0] = c; o[1] = s;
    o[4] = -s; o[5] = c;
    return o;
  }

  function m4Ry(rad) {
    const c = Math.cos(rad);
    const s = Math.sin(rad);
    const o = m4();
    o[0] = c; o[8] = -s;
    o[2] = s; o[10] = c;
    return o;
  }

  function m4Rx(rad) {
    const c = Math.cos(rad);
    const s = Math.sin(rad);
    const o = m4();
    o[5] = c; o[6] = s;
    o[9] = -s; o[10] = c;
    return o;
  }

  function m4Point(m, x, y, z) {
    const w = m[3] * x + m[7] * y + m[11] * z + m[15];
    const iw = Math.abs(w) > 1e-9 ? 1 / w : 1;
    return [
      (m[0] * x + m[4] * y + m[8] * z + m[12]) * iw,
      (m[1] * x + m[5] * y + m[9] * z + m[13]) * iw,
      (m[2] * x + m[6] * y + m[10] * z + m[14]) * iw,
    ];
  }

  function verticalAngleRad(joints, cfg) {
    const c = mergeConfig(cfg);
    // Tool extends along local +X after this joint.
    // DOWN: Ry(-90°) maps +X → world −Z.  UP: 0° keeps +X forward (+X world at yaw 0).
    const down = (c.vertical_servo_down_deg != null
      ? c.vertical_servo_down_deg : -90) * DEG;
    const up = (c.vertical_servo_up_deg != null
      ? c.vertical_servo_up_deg : 0) * DEG;
    return joints.vertical === 'up' ? up : down;
  }

  function rotationServoTransform(joints, cfg) {
    const c = mergeConfig(cfg);
    // out (was vertical): apply out angle. in (was horizontal): identity.
    if (joints.rotation_servo !== 'out') {
      return m4();
    }
    const ang = (
      c.rotation_servo_out_deg != null
        ? c.rotation_servo_out_deg
        : (c.rotation_servo_vertical_deg != null ? c.rotation_servo_vertical_deg : 0)
    ) * DEG;
    if (joints.vertical === 'down') {
      // In: gripper hangs −Z.  Out: ACW → along arm (+X).
      return m4Ry(ang);
    }
    // Arm up (+X): roll jaws 90° without changing reach direction.
    return m4Rx(ang);
  }

  function rotationServoAngleRad(joints, cfg) {
    void joints;
    void cfg;
    return 0;
  }

  /** Forward kinematics — returns link transforms (m) and tool pose. */
  function forwardKinematics(jointsIn, cfg) {
    const c = mergeConfig(cfg);
    const j = clampJoints(jointsIn, c);
    const yaw = j.rotation_deg * DEG;
    const liftZ = liftPhysicalZmm(j, c) / 1000;
    const linX = (c.linear_home_x_mm + j.linear_mm) / 1000;
    const stem = c.rotation_joint_offset_mm / 1000;
    const jaws = c.gripper_length_mm / 1000;

    let T = m4();
    const links = {};

    const T_rot = m4Rz(yaw);
    T = m4Mul(T, T_rot);
    links.OMS_ROTATION = T.slice();

    const T_lift = m4T(0, 0, liftZ);
    T = m4Mul(T, T_lift);
    links.LIFT_ASSEMBLY = T.slice();

    const T_lin = m4T(linX, 0, 0);
    T = m4Mul(T, T_lin);
    links.LINEAR_AXIS = T.slice();

    const T_vert = m4Ry(verticalAngleRad(j, c));
    T = m4Mul(T, T_vert);
    links.VERTICAL_SERVO = T.slice();

    const T_rservo = m4T(stem, 0, 0);
    T = m4Mul(T, T_rservo);
    const T_rrot = rotationServoTransform(j, c);
    T = m4Mul(T, T_rrot);
    links.ROTATION_SERVO = T.slice();

    const T_gripBase = m4T(jaws / 2, 0, 0);
    T = m4Mul(T, T_gripBase);
    links.GRIPPER = T.slice();

    const tip = m4Point(T, jaws / 2, 0, 0);
    const origin = m4Point(T, 0, 0, 0);

    const camT = m4Mul(
      links.OMS_ROTATION,
      m4T(c.camera_tx_mm / 1000, c.camera_ty_mm / 1000, c.camera_tz_mm / 1000),
    );
    const camR = m4Mul(camT, m4Ry(-(c.camera_tilt_deg || 35) * DEG));
    links.CAMERA = camR.slice();

    return {
      joints: j,
      links,
      tool: {
        x_m: tip[0], y_m: tip[1], z_m: tip[2],
        x_mm: tip[0] * 1000, y_mm: tip[1] * 1000, z_mm: tip[2] * 1000,
        mount_x_m: origin[0], mount_y_m: origin[1], mount_z_m: origin[2],
      },
      lift_physical_z_mm: liftPhysicalZmm(j, c),
      linear_ref_z_mm: linearRefZmm(j, c),
      limits: limitSwitches(j, c),
      config: c,
    };
  }

  function analysis(joints, cfg) {
    const fk = forwardKinematics(joints, cfg);
    const home = forwardKinematics(homeJoints(), cfg);
    const p = fk.tool;
    const h = home.tool;
    const dx = p.x_m - h.x_m;
    const dy = p.y_m - h.y_m;
    const dz = p.z_m - h.z_m;
    return {
      x_m: p.x_m, y_m: p.y_m, z_m: p.z_m,
      x_mm: p.x_mm, y_mm: p.y_mm, z_mm: p.z_mm,
      yaw_deg: fk.joints.rotation_deg,
      lift_mm: fk.joints.lift_mm,
      linear_mm: fk.joints.linear_mm,
      lift_physical_z_mm: fk.lift_physical_z_mm,
      linear_ref_z_mm: fk.linear_ref_z_mm,
      vertical: fk.joints.vertical,
      rotation_servo: fk.joints.rotation_servo,
      gripper: fk.joints.gripper,
      from_home_mm: Math.sqrt(dx * dx + dy * dy + dz * dz) * 1000,
      limits: fk.limits,
      front: '+X',
    };
  }

  function lerp(a, b, t) {
    return a + (b - a) * t;
  }

  function lerpJoints(a, b, t) {
    const vert = t < 0.5 ? a.vertical : b.vertical;
    const rs = t < 0.5 ? a.rotation_servo : b.rotation_servo;
    return {
      rotation_deg: lerp(a.rotation_deg, b.rotation_deg, t),
      lift_mm: lerp(a.lift_mm, b.lift_mm, t),
      linear_mm: lerp(a.linear_mm, b.linear_mm, t),
      vertical: vert,
      rotation_servo: rs,
      gripper: lerp(a.gripper, b.gripper, t),
    };
  }

  function easeInOut(t) {
    return t < 0.5 ? 2 * t * t : 1 - Math.pow(-2 * t + 2, 2) / 2;
  }

  /** Smooth motion toward target joints (returns interpolated state). */
  function stepMotion(current, target, dtSec, cfg, speed = 1) {
    const c = clampJoints(current, cfg);
    const g = clampJoints(target, cfg);
    const rate = Math.max(0.05, speed);
    const step = rate * dtSec;
    const next = {
      rotation_deg: c.rotation_deg + clamp(g.rotation_deg - c.rotation_deg, -step * 90, step * 90),
      lift_mm: c.lift_mm + clamp(g.lift_mm - c.lift_mm, -step * 120, step * 120),
      linear_mm: c.linear_mm + clamp(g.linear_mm - c.linear_mm, -step * 120, step * 120),
      vertical: g.vertical,
      rotation_servo: g.rotation_servo,
      gripper: c.gripper + clamp(g.gripper - c.gripper, -step * 2, step * 2),
    };
    return clampJoints(next, cfg);
  }

  function jointsFromLegacy(yawDeg, liftMm, linearMm, wrist, roll, gripperNorm) {
    const rollIn = roll === 'in' || roll === 'horizontal' || roll === 'h';
    return {
      rotation_deg: yawDeg,
      lift_mm: liftMm,
      linear_mm: linearMm,
      vertical: wrist === 'up' ? 'up' : 'down',
      rotation_servo: rollIn ? 'in' : 'out',
      gripper: gripperNorm == null ? 0 : gripperNorm,
    };
  }

  function configFromDims(dims) {
    const d = dims || {};
    const upper = Number(d.lift_upper_z_mm) || Number(d.z_top_mm) || 455;
    const lower = Number(d.lift_lower_z_mm) || Number(d.z_floor_mm) || 155;
    const travel = Number(d.lift_travel_mm) || Math.max(1, upper - lower);
    return mergeConfig({
      lift_upper_z_mm: upper,
      lift_lower_z_mm: lower,
      lift_travel_mm: travel,
      linear_home_x_mm: Number(d.linear_home_x_mm) || 220,
      linear_travel_mm: Number(d.linear_travel_mm) || 110,
      linear_max_extension_mm: Number(d.linear_max_extension_mm) || 110,
      linear_ref_z_lower_lift_mm: Number(d.linear_ref_z_lower_lift_mm) || 110,
      linear_ref_z_upper_lift_mm: Number(d.linear_ref_z_upper_lift_mm) || 375,
      rotation_joint_offset_mm: Number(d.gripper_servo_mm) || 110,
      gripper_length_mm: Number(d.gripper_length_mm) || 200,
      turntable_diameter_mm: Number(d.turntable_diameter_mm) || 180,
      rail_height_mm: Number(d.rail_height_mm) || Number(d.total_oms_height_mm) || 575,
      total_oms_height_mm: Number(d.total_oms_height_mm) || 575,
      camera_tilt_deg: Number(d.camera_tilt_deg) || 35,
      camera_tz_mm: Number(d.camera_tz_mm) || (upper - 20),
    });
  }

  function solveIK(targetPose, cfg) {
    void targetPose;
    void cfg;
    return {
      ok: false,
      message: 'IK not implemented — joint-space demo validates FK chain',
      joints: null,
    };
  }

  function isReachable(x_m, y_m, z_m, cfg) {
    const c = mergeConfig(cfg);
    const yaw = Math.atan2(y_m, x_m) * (180 / Math.PI);
    if (yaw < c.oms_rotation_min_deg || yaw > c.oms_rotation_max_deg) {
      return { ok: false, reason: 'yaw out of range' };
    }
    const z_mm = z_m * 1000;
    if (z_mm < c.lift_lower_z_mm || z_mm > c.lift_upper_z_mm) {
      return { ok: false, reason: 'lift Z out of range' };
    }
    const r_mm = Math.hypot(x_m, y_m) * 1000;
    const ext = r_mm - c.linear_home_x_mm;
    if (ext < 0 || ext > linearMaxMm(c)) {
      return { ok: false, reason: 'linear extension out of range' };
    }
    return { ok: true, reason: 'reachable (coarse)' };
  }

  window.OmsKinematics = {
    DEFAULT_CONFIG,
    mergeConfig,
    homeJoints,
    clampJoints,
    linearMaxMm,
    liftPhysicalZmm,
    linearRefZmm,
    limitSwitches,
    forwardKinematics,
    analysis,
    lerpJoints,
    easeInOut,
    stepMotion,
    jointsFromLegacy,
    configFromDims,
    solveIK,
    isReachable,
    m4Point,
    m4Mul,
    m4T,
    m4Rz,
    m4Ry,
    m4Rx,
  };
})();
