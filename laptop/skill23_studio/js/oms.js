/** OMS Software dashboard */
(() => {
  const $ = (id) => document.getElementById(id);
  const params = new URLSearchParams(window.location.search);
  const state = {
    robot: params.get('robot') || 'allterrain',
    connected: false,
    ready: false,
    source: 'none',
    manual: false,
    demoMode: false,
    demo: false,
    demoT: 0,
    simTarget: null,
    simCurrent: null,
    lastAnimMs: 0,
    lastOms: null,
    config: {},
    wrist: 'down',
    roll: 'out',
    notes: [],
    homeSeq: ['rotation', 'lift', 'linear', 'vertical', 'rotation_servo', 'gripper'],
    homeTicks: {},
    lastVizKey: '',
    moving: false,
  };

  const HOME_STEPS = [
    { id: 'rotation', label: 'Rotation' },
    { id: 'lift', label: 'Lift' },
    { id: 'linear', label: 'Linear' },
    { id: 'vertical', label: 'Vertical' },
    { id: 'rotation_servo', label: 'Rot. servo' },
    { id: 'gripper', label: 'Gripper' },
  ];

  const canvas = $('omsCanvas');
  const viz = window.OmsViz.create(canvas);
  viz.resize();

  function setConnected(ok) {
    state.connected = ok;
    $('connBadge').classList.toggle('on', ok);
    $('connText').textContent = ok ? 'Online' : 'Offline';
    const badge = $('linkBadge');
    const connectRow = $('connectRow');
    if (badge) {
      badge.textContent = ok
        ? `Linked · ${StudioApi.getBaseUrl()}`
        : 'Not linked — open from Studio after Connect';
      badge.classList.toggle('ok', ok);
      badge.classList.toggle('fail', !ok);
    }
    if (connectRow) {
      // Only show Connect if opened alone / link failed.
      connectRow.hidden = ok || params.get('from') === 'studio';
    }
  }

  function setReady(ready, source, message) {
    state.ready = !!ready;
    state.source = source || 'none';
    document.body.classList.toggle('ready-on', state.ready);
    const pill = $('readyPill');
    if (!state.connected) {
      pill.textContent = 'Offline';
    } else if (state.ready) {
      pill.textContent = source === 'commission' ? 'Ready · commissioning' : 'Ready';
    } else {
      pill.textContent = 'Not ready';
    }
    if (message) $('runHint').textContent = message;
    else if (!state.connected) {
      $('runHint').textContent = 'Connect to the Pi bridge (same URL as Skill23 Studio).';
    } else if (state.ready) {
      $('runHint').textContent = (
        source === 'commission'
          ? 'OMS nodes came up with commissioning. Do not press Run OMS. Stop nav in Studio to shut everything down.'
          : source === 'standalone'
            ? 'Standalone OMS (test_oms) is running. Stop OMS here when finished.'
            : 'OMS topics are live. Run is not needed.'
      );
    } else {
      $('runHint').textContent = 'Not ready. Select robot → Run OMS. Press Home when you want to home.';
    }
  }

  function selectRobot(name) {
    state.robot = name;
    document.querySelectorAll('#robotChips .chip').forEach((c) => {
      c.classList.toggle('active', c.dataset.robot === name);
    });
  }

  function num(id, fallback) {
    const v = parseFloat($(id).value);
    return Number.isFinite(v) ? v : fallback;
  }

  function fillIfIdle(id, value, digits) {
    const el = $(id);
    if (!el || document.activeElement === el) return;
    if (value == null || !Number.isFinite(Number(value))) return;
    el.value = Number(value).toFixed(digits == null ? 1 : digits);
  }

  function dimsFromForm() {
    const upper = num('vizZTop', 455);
    const lower = num('vizZFloor', 155);
    let travel = num('vizLiftTravel', 300);
    if (Math.abs((upper - lower) - travel) > 1) {
      travel = Math.max(1, upper - lower);
    }
    return {
      lift_upper_z_mm: upper,
      lift_lower_z_mm: lower,
      lift_travel_mm: travel,
      z_top_mm: upper,
      z_floor_mm: lower,
      linear_travel_mm: num('vizLinTravel', 110),
      linear_max_extension_mm: num('vizLinMaxExt', 110),
      linear_home_x_mm: num('vizHomeX', 220),
      linear_max_reach_mm: num('vizLinMax', 330),
      linear_ref_z_lower_lift_mm: num('vizLinZLow', 110),
      linear_ref_z_upper_lift_mm: num('vizLinZUp', 375),
      total_oms_height_mm: num('vizOmsHeight', 575),
      rail_height_mm: num('vizRail', 575),
      turntable_diameter_mm: num('vizTable', 180),
      camera_tilt_deg: 35,
      gripper_servo_mm: num('vizGripServo', 110),
      gripper_length_mm: num('vizGripLen', 200),
    };
  }

  function kinematicsConfig() {
    return window.OmsKinematics.configFromDims(dimsFromForm());
  }

  function currentJoints() {
    const p = viz.getPose();
    return window.OmsKinematics.jointsFromLegacy(
      p.yaw, p.liftMm, p.linearMm, p.wrist, p.roll, p.gripper);
  }

  function renderJointStatePanel(joints, limits) {
    const el = $('jointStateText');
    if (!el) return;
    const cfg = kinematicsConfig();
    const fk = window.OmsKinematics.forwardKinematics(joints, cfg);
    const lim = limits || fk.limits;
    const j = fk.joints;
    const tip = fk.tool;
  const sw = (on) => (on ? 'PRESSED' : 'RELEASED');
    el.textContent = [
      `OMS Rotation     : ${j.rotation_deg >= 0 ? '+' : ''}${j.rotation_deg.toFixed(1)}°`,
      `Lift             : ${j.lift_mm.toFixed(1)} mm from home`,
      `Physical Lift Z  : ${fk.lift_physical_z_mm.toFixed(1)} mm`,
      `Linear Extension : ${j.linear_mm.toFixed(1)} mm`,
      `Vertical Servo   : ${j.vertical.toUpperCase()}`,
      `Rotation Servo   : ${j.rotation_servo.toUpperCase()}`,
      `Gripper          : ${j.gripper < 0.5 ? 'OPEN' : 'CLOSED'}`,
      '',
      'Tool tip (world):',
      `  X=${tip.x_mm.toFixed(1)}  Y=${tip.y_mm.toFixed(1)}  Z=${tip.z_mm.toFixed(1)} mm`,
      '',
      'Limit switches:',
      `  Rotation Home  : ${sw(lim.rotation_home)}`,
      `  Lift Upper     : ${sw(lim.lift_upper)}`,
      `  Lift Lower     : ${sw(lim.lift_lower)}`,
      `  Linear Home    : ${sw(lim.linear_home)}`,
    ].join('\n');
  }

  function note(msg) {
    const line = `${new Date().toLocaleTimeString()}  ${msg}`;
    state.notes.unshift(line);
    state.notes = state.notes.slice(0, 40);
    const el = $('activityLog');
    if (el) el.textContent = state.notes.join('\n');
  }

  function currentYawLiftLin() {
    const o = state.lastOms || {};
    return [
      Number((o.pose || {}).yaw_deg) || Number((o.rotation || {}).angle_deg) || 0,
      Number((o.pose || {}).lift_mm) || Number((o.lift || {}).pos_mm) || 0,
      Number((o.pose || {}).linear_mm) || Number((o.linear || {}).pos_mm) || 0,
    ];
  }

  function renderAnalysisFromJoints(joints) {
    const a = window.OmsKinematics.analysis(joints, kinematicsConfig());
    const w = joints.vertical || state.wrist || 'down';
    const yawTxt = `${a.yaw_deg >= 0 ? '+' : ''}${a.yaw_deg.toFixed(1)}°`;
    const rows = [
      ['World X', `${a.x_mm.toFixed(1)} mm`, 'Forward from turntable centre.'],
      ['World Y', `${a.y_mm.toFixed(1)} mm`, 'Left of centre (ACW+).'],
      ['World Z', `${a.z_mm.toFixed(1)} mm`, `Lift Z=${a.lift_physical_z_mm.toFixed(0)} mm from ground.`],
      ['Yaw', yawTxt, 'OMS rotation around +Z.'],
      ['Lift ↓', `${a.lift_mm.toFixed(1)} mm`, 'Down from upper home. 0 = top.'],
      ['Linear', `${a.linear_mm.toFixed(1)} mm`, 'Extension from rear home.'],
      ['Vertical', w === 'up' ? 'UP → +X' : 'DOWN → −Z', 'Vertical servo joint.'],
      ['Rot. servo', a.rotation_servo, 'Gripper roll orientation.'],
      ['From home', `${a.from_home_mm.toFixed(1)} mm`, '3D distance from home pose.'],
      ['Front', a.front, 'Gripper-forward at yaw 0.'],
    ];
    $('analysisBox').innerHTML = rows.map(([k, v, tip]) => (
      `<div class="stat" title="${tip}"><div class="k">${k}</div><div class="v">${v}</div></div>`
    )).join('');
    renderJointStatePanel(joints, a.limits);
  }

  function renderAnalysis(yaw, lift, linear, wrist) {
    const joints = window.OmsKinematics.jointsFromLegacy(
      yaw, lift, linear, wrist || state.wrist, state.roll, viz.getPose().gripper || 0);
    renderAnalysisFromJoints(joints);
  }

  function applyJoints(joints) {
    const j = window.OmsKinematics.clampJoints(joints, kinematicsConfig());
    state.wrist = j.vertical;
    state.roll = j.rotation_servo;
    viz.setDims(dimsFromForm());
    viz.setJoints(j);
    renderAnalysisFromJoints(j);
    updateDemoUi(j);
  }

  function applyPose(yaw, lift, linear, wrist, roll) {
    const joints = window.OmsKinematics.jointsFromLegacy(
      yaw, lift, linear, wrist || state.wrist, roll || state.roll, viz.getPose().gripper || 0);
    applyJoints(joints);
  }

  function axesBusy(oms) {
    const moving = new Set(['goto', 'homing', 'running', 'moving']);
    return ['rotation', 'lift', 'linear', 'vertical', 'rotation_servo', 'gripper', 'home']
      .some((ax) => moving.has(String(((oms || {})[ax] || {}).state || '').toLowerCase()));
  }

  function snapVizFromOms(oms, force) {
    const o = oms || state.lastOms || {};
    const busy = axesBusy(o);
    state.moving = busy;
    if (!force && busy) return;
    const yaw = Number((o.pose || {}).yaw_deg);
    const yawV = Number.isFinite(yaw) ? yaw : (Number((o.rotation || {}).angle_deg) || 0);
    const lift = Number((o.pose || {}).lift_mm);
    const liftV = Number.isFinite(lift) ? lift : (Number((o.lift || {}).pos_mm) || 0);
    const lin = Number((o.pose || {}).linear_mm);
    const linV = Number.isFinite(lin) ? lin : (Number((o.linear || {}).pos_mm) || 0);
    const vPose = String((o.vertical || {}).pose || '').toLowerCase();
    if (vPose === 'up' || vPose === 'down') state.wrist = vPose;
    const rPose = String((o.rotation_servo || {}).pose || '').toLowerCase();
    if (rPose === 'out' || rPose === 'vertical' || rPose === 'home' || rPose === 'v') {
      state.roll = 'out';
    } else if (rPose === 'in' || rPose === 'horizontal' || rPose === 'h') {
      state.roll = 'in';
    }
    const key = [yawV.toFixed(2), liftV.toFixed(1), linV.toFixed(1), state.wrist, state.roll].join('|');
    if (!force && key === state.lastVizKey) return;
    state.lastVizKey = key;
    applyPose(yawV, liftV, linV, state.wrist, state.roll);
  }

  function resetDashboard(message) {
    state.homeTicks = {};
    state.wrist = 'down';
    state.roll = 'out';
    state.lastVizKey = '';
    state.moving = false;
    state.lastOms = null;
    renderHomeSeq();
    const st = $('homeSeqState');
    if (st) {
      st.textContent = 'idle';
      st.className = 'live';
    }
    applyPose(0, 0, 0, 'down', 'out');
    viz.setLimits({
      rotation: false, lift_up: false, lift_down: false, linear: false,
    });
    ['liveRotation', 'liveLift', 'liveLinear', 'liveVertical', 'liveRotServo', 'liveGripper']
      .forEach((id) => { if ($(id)) $(id).textContent = '—'; });
    if (message) note(message);
  }

  function homeLabel(id) {
    const hit = HOME_STEPS.find((s) => s.id === id);
    return hit ? hit.label : id;
  }

  function tickMark(status) {
    if (status === 'ok') return { ch: '✓', cls: 'ok' };
    if (status === 'fail') return { ch: '✗', cls: 'fail' };
    if (status === 'running') return { ch: '●', cls: 'run' };
    if (status === 'skip') return { ch: '–', cls: 'skip' };
    return { ch: '○', cls: 'pending' };
  }

  function renderHomeSeq() {
    const box = $('homeSeq');
    const add = $('homeSeqAdd');
    if (!box) return;
    box.innerHTML = state.homeSeq.map((id, i) => {
      const info = (state.homeTicks[id] || {});
      const st = info.status || 'pending';
      const mark = tickMark(st);
      return `<div class="home-step ${mark.cls}" data-id="${id}" title="${st}">`
        + `<span class="tick ${mark.cls}">${mark.ch}</span>`
        + `<label><input type="checkbox" checked data-home-on="${id}" />`
        + `<b>${i + 1} ${homeLabel(id)}</b></label>`
        + `<span class="msg">${info.message || ''}</span>`
        + `<button type="button" class="btn btn-ghost tiny" data-home-up="${id}">↑</button>`
        + `<button type="button" class="btn btn-ghost tiny" data-home-dn="${id}">↓</button>`
        + `</div>`;
    }).join('');
    const used = new Set(state.homeSeq);
    const extras = HOME_STEPS.filter((s) => !used.has(s.id));
    if (add) {
      add.innerHTML = extras.length
        ? extras.map((s) => `<button type="button" class="chip" data-home-add="${s.id}">+ ${s.label}</button>`).join('')
        : '';
    }
    box.querySelectorAll('[data-home-on]').forEach((el) => {
      el.onchange = () => {
        const id = el.getAttribute('data-home-on');
        if (!el.checked) {
          state.homeSeq = state.homeSeq.filter((x) => x !== id);
          renderHomeSeq();
        }
      };
    });
    box.querySelectorAll('[data-home-up]').forEach((el) => {
      el.onclick = () => moveHomeStep(el.getAttribute('data-home-up'), -1);
    });
    box.querySelectorAll('[data-home-dn]').forEach((el) => {
      el.onclick = () => moveHomeStep(el.getAttribute('data-home-dn'), 1);
    });
    if (add) {
      add.querySelectorAll('[data-home-add]').forEach((el) => {
        el.onclick = () => {
          const id = el.getAttribute('data-home-add');
          if (!state.homeSeq.includes(id)) state.homeSeq.push(id);
          renderHomeSeq();
        };
      });
    }
  }

  function moveHomeStep(id, delta) {
    const i = state.homeSeq.indexOf(id);
    const j = i + delta;
    if (i < 0 || j < 0 || j >= state.homeSeq.length) return;
    const copy = state.homeSeq.slice();
    const tmp = copy[i];
    copy[i] = copy[j];
    copy[j] = tmp;
    state.homeSeq = copy;
    renderHomeSeq();
  }

  function paintHomeTicks() {
    const box = $('homeSeq');
    if (!box) return;
    box.querySelectorAll('.home-step').forEach((el) => {
      const id = el.getAttribute('data-id');
      const info = state.homeTicks[id] || {};
      const st = info.status || 'pending';
      const mark = tickMark(st);
      const tick = el.querySelector('.tick');
      if (tick) {
        tick.className = `tick ${mark.cls}`;
        tick.textContent = mark.ch;
      }
      el.classList.remove('ok', 'fail', 'run', 'skip', 'pending');
      el.classList.add(mark.cls);
      const msg = el.querySelector('.msg');
      if (msg) msg.textContent = info.message || '';
      el.title = [st, info.message].filter(Boolean).join(' — ');
    });
  }

  function applyHomeTicks(home) {
    const steps = (home && home.steps) || {};
    state.homeTicks = steps;
    const st = $('homeSeqState');
    if (st) {
      const bits = [home && home.state ? home.state : 'idle'];
      if (home && home.current) bits.push(home.current);
      if (home && home.failed_step) bits.push(`fail ${home.failed_step}`);
      if (home && home.message) bits.push(home.message);
      st.textContent = bits.join(' · ');
      st.className = 'live'
        + ((home && home.state === 'home_failed') ? ' fail' : '')
        + ((home && home.state === 'homing') ? ' run' : '')
        + ((home && home.state === 'idle' && home.message === 'all homed') ? ' ok' : '');
    }
    paintHomeTicks();
    if (home && home.state === 'idle' && home.message === 'all homed') {
      state.wrist = 'down';
      state.roll = 'out';
      state.lastVizKey = '';
      applyPose(0, 0, 0, 'down', 'out');
    }
  }

  function liveText(axis) {
    const o = state.lastOms || {};
    const s = o[axis] || {};
    const bits = [];
    if (s.angle_deg != null) bits.push(`${Number(s.angle_deg).toFixed(1)}°`);
    if (s.pos_mm != null) bits.push(`${Number(s.pos_mm).toFixed(1)} mm`);
    if (s.pose) bits.push(String(s.pose));
    if (s.state) bits.push(String(s.state));
    if (s.homed) bits.push('homed');
    if (s.home_switch) bits.push('SW');
    if (s.up_sw) bits.push('UP');
    if (s.down_sw) bits.push('DN');
    return bits.join(' · ') || '—';
  }

  function renderHardware(list) {
    const rows = (list || []).map((h) => (
      `<div class="hw-row"><b>${h.name}</b><span>${h.hw}<br>${h.home}<br><code>${h.topics}</code><br>${h.range}</span></div>`
    ));
    $('hwTable').innerHTML = rows.join('') || '<p class="hint">Connect to load Titan / VMX map.</p>';
  }

  function renderDebug(oms, stacks) {
    const rot = (oms && oms.rotation) || {};
    const lift = (oms && oms.lift) || {};
    const lin = (oms && oms.linear) || {};
    const home = (oms && oms.home) || {};
    const last = (oms && oms.last_result) || {};
    const lines = [
      `source=${oms && oms.source}  ready=${oms && oms.ready}  standalone=${oms && oms.standalone}`,
      `stacks ${JSON.stringify(stacks || {})}`,
      `rotation ${rot.angle_deg}° state=${rot.state} homed=${rot.homed} sw=${rot.home_switch}`,
      `lift ${lift.pos_mm} mm state=${lift.state} up=${lift.up_sw} down=${lift.down_sw}`,
      `linear ${lin.pos_mm} mm state=${lin.state} sw=${lin.home_switch}`,
      `home seq ${home.state || '—'} ${home.failed_step || ''} ${home.message || ''}`,
      `last ${JSON.stringify(last)}`,
    ];
    $('debugBox').textContent = lines.join('\n');
  }

  function applyTelemetry(payload) {
    const oms = (payload && payload.oms) || payload || {};
    const stacks = (payload && payload.stacks) || {};
    const wasReady = state.ready;
    state.lastOms = oms;
    const ready = !!oms.ready;
    setReady(ready, oms.source, '');
    if (wasReady && !ready) {
      resetDashboard('OMS went offline — dashboard reset.');
      return;
    }
    $('liveRotation').textContent = liveText('rotation');
    $('liveLift').textContent = liveText('lift');
    $('liveLinear').textContent = liveText('linear');
    $('liveVertical').textContent = liveText('vertical');
    $('liveRotServo').textContent = liveText('rotation_servo');
    $('liveGripper').textContent = liveText('gripper');
    applyHomeTicks(oms.home || {});
    renderHardware(oms.hardware);
    renderDebug(oms, stacks);
    viz.setLimits({
      rotation: !!(oms.rotation || {}).home_switch,
      lift_up: !!(oms.lift || {}).up_sw,
      lift_down: !!(oms.lift || {}).down_sw,
      linear: !!(oms.linear || {}).home_switch,
    });
    if (!state.demoMode && !state.demo) snapVizFromOms(oms, false);
    // Target fields are user-owned — never auto-overwrite (use Sync buttons).
  }

  function applyConfig(cfg) {
    state.config = cfg || {};
    const r = cfg.oms_rotation || {};
    const l = cfg.oms_lift || {};
    const n = cfg.oms_linear || {};
    const v = cfg.oms_vertical || {};
    const rs = cfg.oms_rotation_servo || {};
    const g = cfg.oms_gripper || {};
    const vz = cfg.oms_viz || {};
    if (r.duty != null) $('dutyRotation').value = r.duty;
    if (r.min_deg != null) $('minRotation').value = r.min_deg;
    if (r.max_deg != null) $('maxRotation').value = r.max_deg;
    if (l.duty != null) $('dutyLift').value = l.duty;
    if (l.min_mm != null) $('minLift').value = l.min_mm;
    if (l.max_mm != null) $('maxLift').value = l.max_mm;
    if (n.speed != null) $('speedLinear').value = n.speed;
    if (n.min_mm != null) $('minLinear').value = n.min_mm;
    if (n.max_mm != null) $('maxLinear').value = n.max_mm;
    if (v.down_deg != null) $('cfgVertDown').value = v.down_deg;
    if (v.up_deg != null) $('cfgVertUp').value = v.up_deg;
    if (rs.home_deg != null) $('cfgRotServoHome').value = rs.home_deg;
    if (rs.alt_deg != null) $('cfgRotServoAlt').value = rs.alt_deg;
    if (g.home_deg != null) $('cfgGripClose').value = g.home_deg;
    if (g.alt_deg != null) $('cfgGripOpen').value = g.alt_deg;
    if (vz.lift_travel_mm != null) $('vizLiftTravel').value = vz.lift_travel_mm;
    if (vz.lift_upper_z_mm != null && $('vizZTop')) $('vizZTop').value = vz.lift_upper_z_mm;
    else if (vz.z_top_mm != null && $('vizZTop')) $('vizZTop').value = vz.z_top_mm;
    if (vz.lift_lower_z_mm != null && $('vizZFloor')) $('vizZFloor').value = vz.lift_lower_z_mm;
    else if (vz.z_floor_mm != null && $('vizZFloor')) $('vizZFloor').value = vz.z_floor_mm;
    if (vz.linear_travel_mm != null) $('vizLinTravel').value = vz.linear_travel_mm;
    if (vz.linear_max_extension_mm != null && $('vizLinMaxExt')) {
      $('vizLinMaxExt').value = vz.linear_max_extension_mm;
    }
    if (vz.linear_home_x_mm != null) $('vizHomeX').value = vz.linear_home_x_mm;
    if (vz.linear_max_reach_mm != null && $('vizLinMax')) $('vizLinMax').value = vz.linear_max_reach_mm;
    if (vz.linear_ref_z_lower_lift_mm != null && $('vizLinZLow')) {
      $('vizLinZLow').value = vz.linear_ref_z_lower_lift_mm;
    }
    if (vz.linear_ref_z_upper_lift_mm != null && $('vizLinZUp')) {
      $('vizLinZUp').value = vz.linear_ref_z_upper_lift_mm;
    }
    if (vz.total_oms_height_mm != null && $('vizOmsHeight')) {
      $('vizOmsHeight').value = vz.total_oms_height_mm;
    }
    if (vz.rail_height_mm != null) $('vizRail').value = vz.rail_height_mm;
    if (vz.turntable_diameter_mm != null) $('vizTable').value = vz.turntable_diameter_mm;
    if (vz.gripper_servo_mm != null && $('vizGripServo')) $('vizGripServo').value = vz.gripper_servo_mm;
    if (vz.gripper_length_mm != null && $('vizGripLen')) $('vizGripLen').value = vz.gripper_length_mm;
    const h = cfg.oms_home || {};
    if (Array.isArray(h.sequence) && h.sequence.length) {
      const next = h.sequence.map((x) => String(x)).filter(
        (id) => HOME_STEPS.some((s) => s.id === id));
      if (next.length) state.homeSeq = next;
      renderHomeSeq();
    }
    viz.setDims(dimsFromForm());
    refreshDemoSliders();
  }

  function syncTarget(id, value, digits) {
    const el = $(id);
    if (!el || value == null || !Number.isFinite(Number(value))) return;
    el.value = Number(value).toFixed(digits == null ? 1 : digits);
  }

  function refreshDemoSliders() {
    const cfg = kinematicsConfig();
    const liftMax = cfg.lift_travel_mm || 300;
    const linMax = window.OmsKinematics.linearMaxMm(cfg);
    const liftEl = $('demoLift');
    const linEl = $('demoLin');
    if (liftEl) {
      liftEl.max = String(liftMax);
      $('maxLift').value = liftMax;
    }
    if (linEl) {
      linEl.max = String(linMax);
      $('maxLinear').value = linMax;
    }
  }

  function updateDemoUi(joints) {
    const j = joints || currentJoints();
    const cfg = kinematicsConfig();
    const fk = window.OmsKinematics.forwardKinematics(j, cfg);
    const lim = fk.limits;
    const setLim = (id, on, label) => {
      const el = $(id);
      if (!el) return;
      el.textContent = `${label}: ${on ? 'PRESSED' : 'RELEASED'}`;
      el.classList.toggle('on', !!on);
    };
    if ($('demoRotVal')) $('demoRotVal').textContent = `${j.rotation_deg.toFixed(1)}°`;
    if ($('demoRot')) $('demoRot').value = j.rotation_deg;
    if ($('demoLiftVal')) $('demoLiftVal').textContent = `${j.lift_mm.toFixed(0)} mm`;
    if ($('demoLiftZ')) $('demoLiftZ').textContent = fk.lift_physical_z_mm.toFixed(0);
    if ($('demoLift')) $('demoLift').value = j.lift_mm;
    if ($('demoLinVal')) $('demoLinVal').textContent = `${j.linear_mm.toFixed(0)} mm`;
    if ($('demoLin')) $('demoLin').value = j.linear_mm;
    if ($('demoGripVal')) $('demoGripVal').textContent = j.gripper < 0.5 ? 'OPEN' : 'CLOSED';
    if ($('demoGrip')) $('demoGrip').value = Math.round(j.gripper * 100);
    setLim('limRot', lim.rotation_home, 'Rot SW');
    setLim('limLiftUp', lim.lift_upper, 'Upper');
    setLim('limLiftDn', lim.lift_lower, 'Lower');
    setLim('limLin', lim.linear_home, 'Linear home');
  }

  function setDemoMode(on) {
    state.demoMode = !!on;
    document.body.classList.toggle('demo-on', state.demoMode);
    const card = $('demoCard');
    const panel = $('jointStatePanel');
    if (card) card.hidden = !state.demoMode;
    if (panel) panel.hidden = !state.demoMode;
    viz.setSimLimits(state.demoMode);
    const btn = $('btnDemoMode');
    if (btn) btn.textContent = state.demoMode ? 'Exit demo' : 'Demo mode';
    if (state.demoMode) {
      state.simTarget = window.OmsKinematics.homeJoints();
      state.simCurrent = { ...state.simTarget };
      applyJoints(state.simCurrent);
      $('vizHint').hidden = false;
      $('vizHint').textContent = 'Demo mode — simulated joints, FK digital twin. No hardware commands.';
    } else {
      $('vizHint').hidden = true;
      snapVizFromOms(state.lastOms || {}, true);
    }
  }

  function demoTargetFromUi() {
    const cfg = kinematicsConfig();
    return window.OmsKinematics.clampJoints({
      rotation_deg: num('demoRot', 0),
      lift_mm: num('demoLift', 0),
      linear_mm: num('demoLin', 0),
      vertical: state.wrist,
      rotation_servo: state.roll,
      gripper: (num('demoGrip', 0) / 100),
    }, cfg);
  }

  function homeAllDemo() {
    state.simTarget = window.OmsKinematics.homeJoints();
    state.wrist = 'down';
    state.roll = 'out';
  }

  async function connect() {
    const fromStudio = params.get('from') === 'studio';
    const pi = (params.get('pi') || '').trim();
    const url = pi || $('piUrl').value.trim() || StudioApi.getDefaultUrl();
    if ($('piUrl')) $('piUrl').value = url;
    StudioApi.setBaseUrl(url);
    try {
      await StudioApi.status();
      setConnected(true);
      const cfg = await StudioApi.omsConfig();
      if (cfg.ok) applyConfig(cfg.config || {});
      StudioApi.connectStream((t) => applyTelemetry(t));
      const st = await StudioApi.omsStatus();
      applyTelemetry(st);
      if (fromStudio) {
        $('runHint').textContent = (
          'Linked from Studio. Select robot → Run OMS. '
          + 'Logs show below (studio_maps/_logs/oms.log). Press Home to home.');
      }
    } catch (e) {
      setConnected(false);
      setReady(false, 'none', `Link failed: ${e.message}`);
      if (fromStudio) {
        const row = $('connectRow');
        if (row) row.hidden = false;
        $('runHint').textContent = (
          'Could not reach the Pi bridge. Start studio.launch.py, '
          + 'Connect in Studio, then reopen OMS Software.');
      }
    }
  }

  async function refreshLogs() {
    if (!state.connected) return;
    try {
      const logs = await StudioApi.logs();
      $('omsLog').textContent = logs.oms || logs.running || '—';
    } catch (_) {}
  }

  function needManual() {
    if (!state.manual) {
      alert('Press Start teleop first, then use Go.');
      return false;
    }
    if (!state.ready) {
      alert('OMS is not Ready. Press Run OMS (or wait for commissioning) first.');
      return false;
    }
    return true;
  }

  async function sendCmd(body) {
    if (!needManual()) return;
    try {
      await StudioApi.omsCmd(body);
      note(`cmd ${body.axis} ${body.mode || 'goto'}`);
    } catch (e) {
      note(`cmd failed: ${e.message || e}`);
      alert(e.message || String(e));
    }
  }

  $('btnConnect').onclick = () => connect();
  if ($('piUrl')) $('piUrl').value = StudioApi.getBaseUrl();
  selectRobot(state.robot);
  document.querySelectorAll('#robotChips .chip').forEach((chip) => {
    chip.onclick = () => selectRobot(chip.dataset.robot);
  });

  document.querySelectorAll('[data-view]').forEach((btn) => {
    btn.onclick = () => viz.setView(btn.dataset.view);
  });

  $('btnRunOms').onclick = async () => {
    if (!state.connected) {
      return alert('Not linked to Studio. Connect in Skill23 Studio first, then reopen OMS Software.');
    }
    try {
      const res = await StudioApi.omsStart({ robot: state.robot });
      applyTelemetry(res);
      const msg = res.message || (res.already ? 'Already running.' : 'Launching OMS.');
      note(msg);
      $('runHint').textContent = msg
        + (res.log ? ` Log: ${res.log}` : '');
      if (!res.already) {
        note('OMS idle until Home. Full log on this page + studio_maps/_logs/oms.log');
      }
    } catch (e) {
      note(`Run failed: ${e.message || e}`);
      alert(e.message || String(e));
    }
  };
  $('btnStopOms').onclick = async () => {
    try {
      const res = await StudioApi.omsStop();
      resetDashboard(res.message || 'OMS stopped.');
      setConnected(state.connected);
      setReady(false, 'none', res.message || 'OMS stopped.');
      $('runHint').textContent = res.message || 'OMS stopped.';
      try { await refreshLogs(); } catch (_) {}
      if (res.ok === false) {
        alert(res.error || res.message || 'OMS stop incomplete.');
      } else {
        alert(res.message || 'OMS stopped. Dashboard reset.');
      }
    } catch (e) {
      resetDashboard(`Stop failed: ${e.message || e}`);
      setReady(false, 'none', `Stop failed: ${e.message || e}`);
      note(`Stop failed: ${e.message || e}`);
      alert(e.message || String(e));
    }
  };
  $('btnHome').onclick = async () => {
    if (!state.ready) return alert('OMS is not Ready.');
    if (!state.homeSeq.length) return alert('Select at least one home step.');
    try {
      state.homeTicks = Object.fromEntries(
        state.homeSeq.map((id) => [id, { status: 'pending', message: '' }]));
      renderHomeSeq();
      await StudioApi.omsCmd({
        axis: 'home',
        mode: 'home',
        sequence: state.homeSeq.slice(),
      });
      note('Home sequence: ' + state.homeSeq.join(' → '));
      // 3D snaps to home when /oms/home/status reports all homed.
    } catch (e) {
      alert(e.message || String(e));
    }
  };
  function setTeleop(on) {
    state.manual = !!on;
    document.body.classList.toggle('manual-on', state.manual);
    $('btnManual').textContent = state.manual ? 'Stop teleop' : 'Start teleop';
    if ($('teleopHint')) {
      $('teleopHint').textContent = state.manual
        ? 'Teleop ON — Go / Up / Rear / servo buttons send commands.'
        : 'Press Start teleop, then Go / Up / Rear.';
    }
  }
  $('btnManual').onclick = () => setTeleop(!state.manual);
  $('btnStopAxes').onclick = () => {
    ['rotation', 'lift', 'linear'].forEach((axis) => {
      StudioApi.omsCmd({ axis, mode: 'stop' }).catch(() => {});
    });
  };

  document.querySelectorAll('.go').forEach((btn) => {
    btn.onclick = () => {
      const axis = btn.dataset.go;
      const mode = btn.dataset.mode;
      if (axis === 'rotation' && !mode) {
        return sendCmd({
          axis: 'rotation',
          mode: 'goto',
          angle_deg: num('tgtRotation', 0),
          duty: num('dutyRotation', 0.1),
        });
      }
      if (axis === 'lift' && !mode) {
        return sendCmd({
          axis: 'lift',
          mode: 'goto',
          pos_mm: Math.min(num('maxLift', 240), Math.max(0, num('tgtLift', 0))),
          duty: num('dutyLift', 0.5),
        });
      }
      if (axis === 'linear' && !mode) {
        const maxLin = num('maxLinear', 110);
        return sendCmd({
          axis: 'linear',
          mode: 'goto',
          pos_mm: Math.min(maxLin, Math.max(0, num('tgtLinear', 0))),
          speed: num('speedLinear', 30),
        });
      }
      if (axis === 'vertical') {
        const wrist = mode === 'up' ? 'up' : 'down';
        state.wrist = wrist;
        state.lastVizKey = '';
        applyPose(...currentYawLiftLin(), wrist, state.roll);
        return sendCmd({
          axis: 'vertical',
          mode: mode || 'down',
          up_deg: num('cfgVertUp', 60),
          down_deg: num('cfgVertDown', -40),
        });
      }
      if (axis === 'rotation_servo' && !mode) {
        return sendCmd({
          axis: 'rotation_servo',
          mode: 'goto',
          angle_deg: num('tgtRotServo', 0),
        });
      }
      if (axis === 'rotation_servo') {
        state.roll = (mode === 'out' || mode === 'vertical' || mode === 'v')
          ? 'out'
          : 'in';
        state.lastVizKey = '';
        applyPose(...currentYawLiftLin(), state.wrist, state.roll);
        return sendCmd({
          axis: 'rotation_servo',
          mode,
          home_deg: num('cfgRotServoHome', 0),
          alt_deg: num('cfgRotServoAlt', 110),
        });
      }
      if (axis === 'gripper' && !mode) {
        return sendCmd({
          axis: 'gripper',
          mode: 'goto',
          angle_deg: num('tgtGripper', 150),
        });
      }
      if (axis === 'gripper') {
        return sendCmd({
          axis: 'gripper',
          mode,
          home_deg: num('cfgGripClose', 150),
          alt_deg: num('cfgGripOpen', -150),
        });
      }
      return sendCmd({ axis, mode: mode || 'goto' });
    };
  });

  $('btnSaveYaml').onclick = async () => {
    const updates = {
      oms_rotation: {
        duty: num('dutyRotation', 0.1),
        min_deg: num('minRotation', -90),
        max_deg: num('maxRotation', 90),
      },
      oms_lift: {
        duty: num('dutyLift', 0.5),
        min_mm: num('minLift', 0),
        max_mm: num('maxLift', 240),
      },
      oms_linear: {
        speed: num('speedLinear', 30),
        min_mm: num('minLinear', 0),
        max_mm: num('maxLinear', 110),
      },
      oms_vertical: {
        down_deg: num('cfgVertDown', -40),
        up_deg: num('cfgVertUp', 60),
      },
      oms_rotation_servo: {
        home_deg: num('cfgRotServoHome', 0),
        alt_deg: num('cfgRotServoAlt', 110),
      },
      oms_gripper: {
        home_deg: num('cfgGripClose', 150),
        alt_deg: num('cfgGripOpen', -150),
      },
      oms_home: {
        sequence: state.homeSeq.slice(),
      },
      oms_viz: dimsFromForm(),
    };
    try {
      const res = await StudioApi.omsSaveConfig({ updates });
      if (!res.ok) throw new Error(res.error || 'save failed');
      applyConfig(res.config || updates);
      const msg = `Saved oms.yaml: ${(res.written || []).join(', ')}`;
      $('saveHint').textContent = msg;
      note(msg);
    } catch (e) {
      note(`Save failed: ${e.message || e}`);
      alert(e.message || String(e));
    }
  };

  ['vizLiftTravel', 'vizLinTravel', 'vizHomeX', 'vizRail', 'vizTable',
   'vizZTop', 'vizZFloor', 'vizLinMax', 'vizLinMaxExt', 'vizLinZLow', 'vizLinZUp',
   'vizOmsHeight', 'vizGripServo', 'vizGripLen'].forEach((id) => {
    const el = $(id);
    if (!el) return;
    el.addEventListener('change', () => {
      state.lastVizKey = '';
      refreshDemoSliders();
      if (state.demoMode) {
        applyJoints(currentJoints());
      } else {
        snapVizFromOms(state.lastOms || {}, true);
      }
    });
  });

  $('btnDemoMode').onclick = () => setDemoMode(!state.demoMode);
  if ($('btnHomeAll')) {
    $('btnHomeAll').onclick = () => {
      homeAllDemo();
      if (!state.demoMode) setDemoMode(true);
    };
  }

  ['demoRot', 'demoLift', 'demoLin', 'demoGrip'].forEach((id) => {
    const el = $(id);
    if (!el) return;
    el.addEventListener('input', () => {
      if (!state.demoMode) setDemoMode(true);
      state.simTarget = demoTargetFromUi();
    });
  });

  document.querySelectorAll('[data-demo-rot]').forEach((btn) => {
    btn.onclick = () => {
      if (!state.demoMode) setDemoMode(true);
      const v = parseFloat(btn.getAttribute('data-demo-rot'));
      if ($('demoRot')) $('demoRot').value = v;
      state.simTarget = demoTargetFromUi();
    };
  });

  const bindDemo = (id, fn) => { if ($(id)) $(id).onclick = fn; };
  bindDemo('demoVertDown', () => {
    if (!state.demoMode) setDemoMode(true);
    state.wrist = 'down';
    state.simTarget = demoTargetFromUi();
    state.simTarget.vertical = 'down';
  });
  bindDemo('demoVertUp', () => {
    if (!state.demoMode) setDemoMode(true);
    state.wrist = 'up';
    state.simTarget = demoTargetFromUi();
    state.simTarget.vertical = 'up';
  });
  bindDemo('demoRollH', () => {
    if (!state.demoMode) setDemoMode(true);
    state.roll = 'in';
    state.simTarget = demoTargetFromUi();
    state.simTarget.rotation_servo = 'in';
  });
  bindDemo('demoRollV', () => {
    if (!state.demoMode) setDemoMode(true);
    state.roll = 'out';
    state.simTarget = demoTargetFromUi();
    state.simTarget.rotation_servo = 'out';
  });
  bindDemo('demoGripOpen', () => {
    if (!state.demoMode) setDemoMode(true);
    if ($('demoGrip')) $('demoGrip').value = 0;
    state.simTarget = demoTargetFromUi();
  });
  bindDemo('demoGripClose', () => {
    if (!state.demoMode) setDemoMode(true);
    if ($('demoGrip')) $('demoGrip').value = 100;
    state.simTarget = demoTargetFromUi();
  });

  if ($('btnSyncRot')) {
    $('btnSyncRot').onclick = () => {
      syncTarget('tgtRotation', (state.lastOms || {}).rotation?.angle_deg, 1);
    };
  }
  document.querySelectorAll('.sync-target').forEach((btn) => {
    btn.onclick = () => {
      const id = btn.getAttribute('data-sync');
      const oms = state.lastOms || {};
      if (id === 'tgtLift') syncTarget(id, oms.lift?.pos_mm, 1);
      if (id === 'tgtLinear') syncTarget(id, oms.linear?.pos_mm, 1);
    };
  });

  function tickSim(now) {
    if (state.demoMode && state.simTarget) {
      const dt = Math.min(0.05, (now - (state.lastAnimMs || now)) / 1000);
      state.lastAnimMs = now;
      if (!state.simCurrent) state.simCurrent = { ...state.simTarget };
      state.simCurrent = window.OmsKinematics.stepMotion(
        state.simCurrent, state.simTarget, dt, kinematicsConfig(), 1.2);
      applyJoints(state.simCurrent);
    }
    requestAnimationFrame(tickSim);
  }
  requestAnimationFrame(tickSim);

  applyPose(0, 0, 0);
  refreshDemoSliders();
  renderJointStatePanel(window.OmsKinematics.homeJoints());
  renderHomeSeq();
  renderHardware([]);
  setReady(false, 'none');
  if (params.get('from') !== 'studio') {
    const row = $('connectRow');
    if (row) row.hidden = false;
  }
  connect().catch(() => {});
  setInterval(refreshLogs, 2000);
})();
