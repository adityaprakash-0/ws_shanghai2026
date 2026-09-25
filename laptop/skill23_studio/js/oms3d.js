/** OMS 3D digital twin — hierarchical FK rendering (Canvas 2D).
 *  Uses OmsKinematics for joint transforms; +X front, +Y left, +Z up.
 */
(() => {
  const DEG = Math.PI / 180;
  const K = () => window.OmsKinematics;

  function pose(yawDeg, liftMm, linearMm, dims) {
    const joints = K().jointsFromLegacy(yawDeg, liftMm, linearMm, 'down', 'vertical', 0);
    const a = K().analysis(joints, K().configFromDims(dims));
    return {
      x: a.x_m, y: a.y_m, z: a.z_m,
      r: Math.hypot(a.x_m, a.y_m),
      yaw: (a.yaw_deg || 0) * DEG,
      lift_mm: a.lift_mm,
      linear_mm: a.linear_mm,
      yaw_deg: a.yaw_deg,
      lift_physical_z_mm: a.lift_physical_z_mm,
    };
  }

  function analysis(yawDeg, liftMm, linearMm, dims) {
    const joints = K().jointsFromLegacy(yawDeg, liftMm, linearMm, 'down', 'vertical', 0);
    return K().analysis(joints, K().configFromDims(dims));
  }

  function create(canvas) {
    const ctx = canvas.getContext('2d');
    const state = {
      joints: K().homeJoints(),
      simLimits: K().limitSwitches(K().homeJoints(), K().configFromDims({})),
      hwLimits: { rotation: false, lift_up: false, lift_down: false, linear: false },
      useSimLimits: false,
      dims: {},
      config: K().configFromDims({}),
      az: 38 * DEG,
      el: 28 * DEG,
      zoom: 0.82,
      dragging: false,
      lastX: 0,
      lastY: 0,
    };

    const views = {
      iso: { az: 38 * DEG, el: 28 * DEG },
      front: { az: 0, el: 8 * DEG },
      top: { az: 0, el: 88 * DEG },
      side: { az: -90 * DEG, el: 8 * DEG },
    };

    function fk(joints) {
      return K().forwardKinematics(joints, state.config);
    }

    function resize() {
      const r = canvas.getBoundingClientRect();
      const dpr = Math.min(2, window.devicePixelRatio || 1);
      canvas.width = Math.max(320, r.width) * dpr;
      canvas.height = Math.max(240, r.height) * dpr;
      draw();
    }

    function project(x, y, z) {
      const c = Math.cos(-state.az);
      const s = Math.sin(-state.az);
      const x1 = x * c - y * s;
      const y1 = x * s + y * c;
      const ce = Math.cos(-state.el);
      const se = Math.sin(-state.el);
      const x2 = x1 * ce + z * se;
      const z2 = -x1 * se + z * ce;
      const w = canvas.width;
      const h = canvas.height;
      const span = 1.05;
      const scale = Math.min(w, h) * state.zoom / span;
      return { x: w * 0.52 - y1 * scale, y: h * 0.78 - z2 * scale, d: x2 };
    }

    function ptFromM4(m, lx, ly, lz) {
      return K().m4Point(m, lx, ly, lz);
    }

    function addBox(faces, cx, cy, cz, sx, sy, sz, yawRad, color, alpha, pitch) {
      const hx = sx / 2; const hy = sy / 2; const hz = sz / 2;
      const pit = Number(pitch) || 0;
      const cp = Math.cos(pit); const sp = Math.sin(pit);
      const rotZ = (x, y) => {
        const c = Math.cos(yawRad); const s = Math.sin(yawRad);
        return [x * c - y * s, x * s + y * c];
      };
      const local = [
        [hx, hy, hz], [hx, -hy, hz], [-hx, -hy, hz], [-hx, hy, hz],
        [hx, hy, -hz], [hx, -hy, -hz], [-hx, -hy, -hz], [-hx, hy, -hz],
      ].map(([lx, ly, lz]) => {
        const x1 = lx * cp + lz * sp;
        const z1 = -lx * sp + lz * cp;
        const [rx, ry] = rotZ(x1, ly);
        return [cx + rx, cy + ry, cz + z1];
      });
      [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [2, 3, 7, 6], [0, 3, 7, 4], [1, 2, 6, 5]]
        .forEach((idx) => {
          const pts = idx.map((i) => local[i]);
          const depth = pts.reduce((s, p) => s + p[0], 0) / 4;
          faces.push({ pts, color, alpha: alpha == null ? 1 : alpha, depth });
        });
    }

    function linkYawPitch(m) {
      const o = ptFromM4(m, 0, 0, 0);
      const ex = ptFromM4(m, 1, 0, 0);
      const dx = ex[0] - o[0];
      const dy = ex[1] - o[1];
      const dz = ex[2] - o[2];
      return {
        o,
        yaw: Math.atan2(dy, dx),
        pitch: Math.atan2(dz, Math.hypot(dx, dy)),
      };
    }

    function addBoxAtLink(faces, m, sx, sy, sz, color, alpha) {
      const { o, yaw, pitch } = linkYawPitch(m);
      addBox(faces, o[0], o[1], o[2], sx, sy, sz, yaw, color, alpha, pitch);
    }

    function addBoxAlongLocalX(faces, m, length, sy, sz, color, alpha) {
      const o = ptFromM4(m, 0, 0, 0);
      const tip = ptFromM4(m, length, 0, 0);
      const cx = (o[0] + tip[0]) / 2;
      const cy = (o[1] + tip[1]) / 2;
      const cz = (o[2] + tip[2]) / 2;
      const dx = tip[0] - o[0];
      const dy = tip[1] - o[1];
      const dz = tip[2] - o[2];
      const yaw = Math.atan2(dy, dx);
      const pitch = Math.atan2(dz, Math.hypot(dx, dy));
      addBox(faces, cx, cy, cz, length, sy, sz, yaw, color, alpha, pitch);
    }

    function addPoly(faces, pts, color, alpha) {
      const depth = pts.reduce((s, p) => s + p[0], 0) / pts.length;
      faces.push({ pts, color, alpha: alpha == null ? 1 : alpha, depth });
    }

    function paint(faces) {
      faces.sort((a, b) => a.depth - b.depth);
      faces.forEach((f) => {
        const qs = f.pts.map((p) => project(p[0], p[1], p[2]));
        ctx.beginPath();
        qs.forEach((q, i) => (i ? ctx.lineTo(q.x, q.y) : ctx.moveTo(q.x, q.y)));
        ctx.closePath();
        ctx.globalAlpha = f.alpha;
        ctx.fillStyle = f.color;
        ctx.fill();
        ctx.strokeStyle = 'rgba(0,0,0,0.18)';
        ctx.lineWidth = 1;
        ctx.stroke();
        ctx.globalAlpha = 1;
      });
    }

    function limitColor(on) { return on ? '#ff3b30' : '#8a94a6'; }

    function buildModel(joints, ghost) {
      const faces = [];
      const fkRes = fk(joints);
      const c = fkRes.config;
      const lim = state.useSimLimits ? fkRes.limits : {
        rotation_home: state.hwLimits.rotation,
        lift_upper: state.hwLimits.lift_up,
        lift_lower: state.hwLimits.lift_down,
        linear_home: state.hwLimits.linear,
      };
      const alpha = ghost ? 0.2 : 1;
      const blue = ghost ? '#FF9933' : '#1e6bb8';
      const blueHi = ghost ? '#FF9933' : '#2b7de9';
      const black = ghost ? '#FF9933' : '#1a1d24';
      const yellow = ghost ? '#FF9933' : '#f5c400';
      const grey = ghost ? '#FF9933' : '#3d4450';
      const white = ghost ? '#FF9933' : '#eef1f6';
      const tableR = (c.turntable_diameter_mm || 180) / 2000;
      const railH = (c.rail_height_mm || 575) / 1000;
      const gap = 0.042;
      const stem = (c.rotation_joint_offset_mm || 110) / 1000;
      const jaws = (c.gripper_length_mm || 200) / 1000;
      const liftZ = fkRes.lift_physical_z_mm / 1000;
      const yaw = joints.rotation_deg * DEG;
      const homeX = (c.linear_home_x_mm || 220) / 1000;
      const linExt = joints.linear_mm / 1000;
      const tipX = homeX + linExt;

      if (!ghost) {
        addBox(faces, 0, 0, 0.008, tableR * 2.4, tableR * 2.4, 0.016, 0, '#cfd6e2', 1);
        for (let i = 0; i < 16; i += 1) {
          const a0 = (i / 16) * Math.PI * 2;
          const a1 = ((i + 1) / 16) * Math.PI * 2;
          addPoly(faces, [
            [Math.cos(a0) * tableR, Math.sin(a0) * tableR, 0.018],
            [Math.cos(a1) * tableR, Math.sin(a1) * tableR, 0.018],
            [Math.cos(a1) * tableR * 0.7, Math.sin(a1) * tableR * 0.7, 0.018],
            [Math.cos(a0) * tableR * 0.7, Math.sin(a0) * tableR * 0.7, 0.018],
          ], white, 1);
        }
      }

      // Mast rails (rotate with OMS).
      const rot = fkRes.links.OMS_ROTATION;
      const railC = ptFromM4(rot, 0, 0, railH / 2);
      addBox(faces, railC[0], railC[1] + gap, railC[2], 0.024, 0.02, railH, yaw, blue, alpha);
      addBox(faces, railC[0], railC[1] - gap, railC[2], 0.024, 0.02, railH, yaw, blue, alpha);
      addBox(faces, railC[0], railC[1], 0.04, 0.055, 0.08, 0.05, yaw, black, alpha);

      // Lift carriage.
      const liftM = fkRes.links.LIFT_ASSEMBLY;
      addBoxAtLink(faces, liftM, 0.08, 0.1, 0.06, blueHi, alpha);

      // Linear rail: lift carriage → tip (vertical mount). Home limit at homeX.
      const liftOrigin = ptFromM4(liftM, 0, 0, 0);
      const armLen = Math.max(0.06, tipX);
      addBox(
        faces,
        liftOrigin[0] + (armLen / 2) * Math.cos(yaw),
        liftOrigin[1] + (armLen / 2) * Math.sin(yaw),
        liftZ, armLen, 0.03, 0.024, yaw, black, alpha,
      );

      // Vertical servo → stem → rotation servo → gripper (FK link frames).
      const vertM = fkRes.links.VERTICAL_SERVO;
      const rsM = fkRes.links.ROTATION_SERVO;
      const gripM = fkRes.links.GRIPPER;
      const tip = fkRes.tool;
      const gOpen = 1 - (joints.gripper || 0);
      const fingerW = 0.012 + gOpen * 0.018;

      addBoxAtLink(faces, vertM, 0.04, 0.04, 0.032, grey, alpha);
      addBoxAlongLocalX(faces, vertM, stem, 0.03, 0.03, grey, alpha);
      addBoxAtLink(faces, rsM, 0.038, 0.038, 0.032, '#505868', alpha);
      const jawM = K().m4Mul(gripM, K().m4T(-jaws / 2, 0, 0));
      addBoxAlongLocalX(faces, jawM, jaws, 0.048, 0.026, yellow, alpha);

      const gOrigin = ptFromM4(gripM, 0, 0, 0);
      const gX = ptFromM4(gripM, 1, 0, 0);
      const gx = gX[0] - gOrigin[0];
      const gy = gX[1] - gOrigin[1];
      const gz = gX[2] - gOrigin[2];
      const gYaw = Math.atan2(gy, gx);
      const gPitch = Math.atan2(gz, Math.hypot(gx, gy));
      const fp = ptFromM4(gripM, jaws, fingerW, 0);
      const fn = ptFromM4(gripM, jaws, -fingerW, 0);
      const fingerSx = joints.rotation_servo === 'in' ? 0.012 : 0.012;
      const fingerSy = joints.rotation_servo === 'in' ? 0.02 : 0.04;
      const fingerSz = joints.rotation_servo === 'in' ? 0.04 : 0.02;
      addBox(faces, fp[0], fp[1], fp[2], fingerSx, fingerSy, fingerSz, gYaw, yellow, alpha, gPitch);
      addBox(faces, fn[0], fn[1], fn[2], fingerSx, fingerSy, fingerSz, gYaw, yellow, alpha, gPitch);

      if (!ghost) {
        addBox(faces, tip.x_m, tip.y_m, tip.z_m, 0.014, 0.014, 0.014, 0, '#e53935', 0.95);
      }

      // Camera + frustum.
      if (!ghost && fkRes.links.CAMERA) {
        const cam = ptFromM4(fkRes.links.CAMERA, 0, 0, 0);
        addBox(faces, cam[0], cam[1], cam[2], 0.035, 0.05, 0.03, yaw, '#455a64', 0.85);
        const fr = ptFromM4(fkRes.links.CAMERA, 0.12, 0, -0.08);
        addPoly(faces, [cam, ptFromM4(fkRes.links.CAMERA, 0.15, 0.06, -0.04),
          ptFromM4(fkRes.links.CAMERA, 0.15, -0.06, -0.04), fr], 'rgba(33,150,243,0.12)', 0.5);
      }

      if (!ghost) {
        const rotSw = ptFromM4(rot, -tableR * 0.85, 0, 0.04);
        addBox(faces, rotSw[0], rotSw[1], rotSw[2], 0.022, 0.022, 0.022, 0,
          limitColor(lim.rotation_home), lim.rotation_home ? 1 : 0.55);
        const upSw = ptFromM4(liftM, 0.02, gap, (c.lift_upper_z_mm / 1000) - liftZ - 0.01);
        addBox(faces, upSw[0], upSw[1], upSw[2], 0.02, 0.02, 0.02, yaw,
          limitColor(lim.lift_upper), lim.lift_upper ? 1 : 0.55);
        const dnSw = ptFromM4(liftM, 0.02, gap, (c.lift_lower_z_mm / 1000) - liftZ + 0.01);
        addBox(faces, dnSw[0], dnSw[1], dnSw[2], 0.02, 0.02, 0.02, yaw,
          limitColor(lim.lift_lower), lim.lift_lower ? 1 : 0.55);
        const linSw = ptFromM4(liftM, homeX, 0, 0);
        addBox(faces, linSw[0], linSw[1], linSw[2], 0.02, 0.02, 0.02, yaw,
          limitColor(lim.linear_home), lim.linear_home ? 1 : 0.55);
      }

      return { faces, fk: fkRes, limits: lim };
    }

    function drawAxes() {
      const L = 0.18;
      [
        { a: [0, 0, 0], b: [L, 0, 0], c: '#c62828', t: '+X' },
        { a: [0, 0, 0], b: [0, L, 0], c: '#138808', t: '+Y' },
        { a: [0, 0, 0], b: [0, 0, L], c: '#000080', t: '+Z' },
      ].forEach((a) => {
        const p = project(...a.a); const q = project(...a.b);
        ctx.strokeStyle = a.c; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.moveTo(p.x, p.y); ctx.lineTo(q.x, q.y); ctx.stroke();
        ctx.fillStyle = a.c;
        ctx.font = `${Math.round(11 * (canvas.width / 700))}px system-ui, sans-serif`;
        ctx.fillText(a.t, q.x + 6, q.y - 4);
      });
    }

    function drawHud(model) {
      const fkRes = model.fk;
      const j = fkRes.joints;
      const lim = model.limits;
      const fs = Math.round(11 * (canvas.width / 700));
      ctx.font = `bold ${fs}px system-ui, sans-serif`;
      ctx.fillStyle = 'rgba(0,0,128,0.82)';
      const yawTxt = `yaw ${j.rotation_deg >= 0 ? '+' : ''}${j.rotation_deg.toFixed(1)}°`;
      ctx.fillText(yawTxt, 14, 20);
      ctx.font = `${fs}px system-ui, sans-serif`;
      ctx.fillText(
        `lift ${j.lift_mm.toFixed(0)} mm  Z=${fkRes.lift_physical_z_mm.toFixed(0)}  lin ${j.linear_mm.toFixed(0)} mm`,
        14, 20 + fs + 4);
      const items = [
        ['Rot', lim.rotation_home], ['Up', lim.lift_upper],
        ['Dn', lim.lift_lower], ['Lin', lim.linear_home],
      ];
      let x = 14; const y = canvas.height - 14;
      items.forEach(([label, on]) => {
        ctx.fillStyle = on ? '#ff3b30' : '#8a94a6';
        ctx.beginPath(); ctx.arc(x, y, 5, 0, Math.PI * 2); ctx.fill();
        ctx.fillStyle = '#000080';
        ctx.fillText(label, x + 8, y + 3);
        x += ctx.measureText(label).width + 22;
      });
      const tip = fkRes.tool;
      ctx.fillStyle = 'rgba(0,0,128,0.75)';
      ctx.fillText(
        `tool (${tip.x_mm.toFixed(0)}, ${tip.y_mm.toFixed(0)}, ${tip.z_mm.toFixed(0)}) mm`,
        14, canvas.height - 32);
    }

    function draw() {
      const w = canvas.width; const h = canvas.height;
      ctx.clearRect(0, 0, w, h);
      const g = ctx.createLinearGradient(0, 0, w, h);
      g.addColorStop(0, '#fff8ee'); g.addColorStop(1, '#e8eef8');
      ctx.fillStyle = g; ctx.fillRect(0, 0, w, h);
      const ghost = buildModel(K().homeJoints(), true);
      const live = buildModel(state.joints, false);
      paint(ghost.faces.concat(live.faces));
      drawAxes();
      drawHud(live);
    }

    canvas.addEventListener('pointerdown', (ev) => {
      state.dragging = true;
      state.lastX = ev.clientX; state.lastY = ev.clientY;
      canvas.setPointerCapture(ev.pointerId);
    });
    canvas.addEventListener('pointerup', () => { state.dragging = false; });
    canvas.addEventListener('pointermove', (ev) => {
      if (!state.dragging) return;
      state.az -= (ev.clientX - state.lastX) * 0.01;
      state.el = Math.max(4 * DEG, Math.min(89 * DEG, state.el + (ev.clientY - state.lastY) * 0.01));
      state.lastX = ev.clientX; state.lastY = ev.clientY;
      draw();
    });
    canvas.addEventListener('wheel', (ev) => {
      ev.preventDefault();
      state.zoom = Math.max(0.45, Math.min(2.2, state.zoom * (ev.deltaY > 0 ? 0.92 : 1.08)));
      draw();
    }, { passive: false });
    window.addEventListener('resize', resize);

    return {
      pose, analysis, resize, draw,
      setPose(yawDeg, liftMm, linearMm, wrist, roll, gripper) {
        state.joints = K().jointsFromLegacy(yawDeg, liftMm, linearMm, wrist, roll, gripper);
        draw();
      },
      setJoints(joints) {
        state.joints = K().clampJoints(joints, state.config);
        state.simLimits = K().limitSwitches(state.joints, state.config);
        draw();
      },
      getJoints() { return { ...state.joints }; },
      getFk() { return fk(state.joints); },
      setWrist(w) {
        state.joints.vertical = w === 'up' ? 'up' : 'down';
        draw();
      },
      setRoll(r) {
        state.joints.rotation_servo = (r === 'out' || r === 'vertical' || r === 'v')
          ? 'out'
          : 'in';
        draw();
      },
      setGripper(norm) {
        state.joints.gripper = Math.max(0, Math.min(1, Number(norm) || 0));
        draw();
      },
      setLimits(lim) {
        Object.assign(state.hwLimits, lim || {});
        draw();
      },
      setSimLimits(on) {
        state.useSimLimits = !!on;
        draw();
      },
      setDims(dims) {
        state.dims = { ...dims };
        state.config = K().configFromDims(dims);
        draw();
      },
      setView(name) {
        const v = views[name] || views.iso;
        state.az = v.az; state.el = v.el;
        draw();
      },
      dims() { return { ...state.dims }; },
      getPose() {
        const j = state.joints;
        return {
          yaw: j.rotation_deg,
          liftMm: j.lift_mm,
          linearMm: j.linear_mm,
          wrist: j.vertical,
          roll: j.rotation_servo,
          gripper: j.gripper,
        };
      },
    };
  }

  window.OmsViz = { pose, analysis, create };
})();
