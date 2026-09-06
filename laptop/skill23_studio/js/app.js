/** Skill23 Studio app — wizard + stage flows */
(() => {
  const SEQ_LS = 'skill23_studio_sequences';
  const DEFAULT_SEQ = 'default';
  const COMMISSION_OBJECTS = ['Cylinder', 'Semisphere', 'Rectangle', 'Mystery'];
  const state = {
    connected: false,
    robot: 'grip',
    mapName: '',
    mode: 'offline',
    phase: 'home',
    waypoints: [],
    restrictions: [],
    pendingEdits: [],
    lastPose: null,
    telemetry: null,
    sequenceBuilder: [],
    sequences: JSON.parse(localStorage.getItem(SEQ_LS) || '{}'),
    deployInFlight: false,
    deployModule: 'nav',
    commissionConfig: null,
    waypointInputMode: 'live',
    simActive: false,
    simPoints: [],
    simStyle: 'planned',   // planned | smoothed — which sim path to draw
    simPlannerId: 'GridBased',  // ClearancePlanner plugin id for GC / Nav2 sim
    autoDetectAlign: true,      // commissioning: launch Detect+Align with Nav2+camera
    lastHwAlert: '',
    localizeRunning: false,
    healthSessionId: '',
    healthStartedAt: 0,
    healthLastWrite: 0,
    healthLastSig: '',
    wpTaskIndex: 0,
  };

  const $ = (id) => document.getElementById(id);

  /** Button ids that start nav — disable together while one is working. */
  const DEPLOY_BTN_IDS = [
    'btnDeployNav2',
    'btnDeployNav2Cam',
  ];

  function setButtonLoading(btn, loading, busyText) {
    if (!btn) return;
    if (loading) {
      if (btn.dataset.origText == null) btn.dataset.origText = btn.textContent;
      btn.classList.add('loading');
      btn.disabled = true;
      if (busyText) btn.textContent = busyText;
    } else {
      btn.classList.remove('loading');
      btn.disabled = false;
      if (btn.dataset.origText != null) {
        btn.textContent = btn.dataset.origText;
        delete btn.dataset.origText;
      }
    }
  }

  function setDeploySiblingsDisabled(activeBtn, disabled) {
    DEPLOY_BTN_IDS.forEach((id) => {
      const b = $(id);
      if (b && b !== activeBtn) b.disabled = disabled;
    });
  }

  /** Wrap async work: spinner on button, block double-click. */
  async function runWithLoading(btn, fn, busyText = 'Working…') {
    if (!btn || btn.classList.contains('loading')) return;
    const lockDeploy = DEPLOY_BTN_IDS.includes(btn.id);
    setButtonLoading(btn, true, busyText);
    if (lockDeploy) setDeploySiblingsDisabled(btn, true);
    try {
      return await fn();
    } catch (e) {
      alert(e.message || String(e));
      throw e;
    } finally {
      setButtonLoading(btn, false);
      if (lockDeploy) setDeploySiblingsDisabled(btn, false);
    }
  }

  function bindLoading(id, fn, busyText) {
    const el = $(id);
    if (!el) return;
    el.onclick = (ev) => runWithLoading(ev.currentTarget, fn, busyText);
  }

  function batteryCardHtml(pctId, voltId, fillId) {
    return (
      '<div class="battery-title">Battery</div>'
      + `<div class="value" id="${pctId}">—</div>`
      + `<div class="label" id="${voltId}">—</div>`
      + `<div class="battery-bar"><div class="fill" id="${fillId}" style="width:0%"></div></div>`
    );
  }

  function ensureBatteryUi() {
    if (!$('batteryPct')) {
      const host = document.querySelector('.topbar-right') || document.querySelector('.topbar');
      if (host) {
        const el = document.createElement('div');
        el.id = 'batteryWidget';
        el.className = 'battery-card';
        el.innerHTML = batteryCardHtml('batteryPct', 'batteryVolt', 'batteryFill');
        host.insertBefore(el, host.firstChild);
      }
    }
  }

  function updateBattery(bat) {
    ensureBatteryUi();
    bat = bat || {};
    const slots = [
      ['batteryPct', 'batteryVolt', 'batteryFill'],
      ['batteryPctDeploy', 'batteryVoltDeploy', 'batteryFillDeploy'],
    ];
    slots.forEach(([pctId, voltId, fillId]) => {
      const pctEl = $(pctId);
      const voltEl = $(voltId);
      const fillEl = $(fillId);
      if (!pctEl) return;
      if (bat.valid) {
        pctEl.textContent = `${bat.percentage ?? 0}%`;
        if (voltEl) voltEl.textContent = `${bat.voltage ?? 0} V`;
        if (fillEl) fillEl.style.width = `${Math.min(100, bat.percentage ?? 0)}%`;
      } else {
        pctEl.textContent = 'N/A';
        if (voltEl) {
          voltEl.textContent =
            bat.source === 'waiting_for_control_server' ? 'no VMX' : '—';
        }
        if (fillEl) fillEl.style.width = '0%';
      }
    });
  }

  const views = {
    home: $('view-home'),
    create: $('view-create'),
    import: $('view-import'),
    mode: $('view-mode'),
    deployPick: $('view-deploy-pick'),
    stage: $('view-stage'),
  };

  function showView(name) {
    Object.values(views).forEach((v) => v && v.classList.remove('active'));
    if (views[name]) views[name].classList.add('active');
  }

  function setPhase(phase) {
    state.phase = phase;
    if (state.simActive && phase !== 'deploy') exitSim();
    $('phasePill').textContent = phase === 'deploy-pick'
      ? 'Choose deploy'
      : (phase === 'deploy'
        ? (state.deployModule === 'commission' ? 'Commissioning' : 'Navigation')
        : phase);
    const mapping = phase === 'mapping';
    const improve = phase === 'improve';
    const waypoints = phase === 'waypoints';
    const deploy = phase === 'deploy';
    const deployPick = phase === 'deploy-pick';
    const stage = mapping || improve || waypoints || deploy;
    const stageView = document.getElementById('view-stage');
    if (stageView) stageView.classList.toggle('deploy-layout', deploy);

    $('blockWarnings').classList.toggle('on', !deploy);
    $('blockMapping').classList.toggle('on', mapping);
    $('blockImprove').classList.toggle('on', improve);
    $('blockWaypoints').classList.toggle('on', waypoints);
    $('blockDeploy').classList.toggle('on', deploy);
    $('blockTeleop').classList.toggle('on',
      mapping
      || (improve && state.mode === 'arena')
      || (waypoints && state.mode === 'arena')
      || deploy);
    const poseRow = $('improvePoseRow');
    if (poseRow) poseRow.style.display = (improve && state.mode === 'arena') ? '' : 'none';
    const wpPoseRow = $('wpPoseRow');
    if (wpPoseRow) wpPoseRow.style.display = (waypoints && state.mode === 'arena') ? '' : 'none';
    const locBox = $('localizeControls');
    if (locBox) locBox.hidden = !(waypoints && state.mode === 'arena');
    const nsp = $('nodeStatusPanel');
    if (nsp) nsp.hidden = !deploy;
    const dap = $('detectAlignPanel');
    if (dap) dap.hidden = !(deploy && state.deployModule === 'commission');
    const cmp = $('commissionPanel');
    if (cmp) cmp.hidden = !(deploy && state.deployModule === 'commission');
    const hud = $('deployHud');
    if (hud) {
      hud.hidden = !deploy;
      hud.classList.toggle('module-nav', deploy && state.deployModule !== 'commission');
      hud.classList.toggle('module-commission', deploy && state.deployModule === 'commission');
    }
    applyDeployModuleUi();

    if (stage) {
      showView('stage');
      StudioMap.resize();
      if (mapping) StudioMap.setTool('pan');
      if (improve) StudioMap.setTool('pan');
      if (waypoints) {
        StudioMap.setTool(
          state.mode === 'offline' || state.waypointInputMode === 'map'
            ? 'waypoint'
            : 'pan');
        updateWaypointInputUi();
      }
      if (deploy) {
        StudioMap.setTool('pose');
        renderDeployWpStrip();
        renderWpTaskEditors();
        refreshSeqSelect();
        bindToolButtons();
        if (state.deployModule === 'commission') {
          loadCommissionConfig().catch((e) => console.warn(e));
        }
      }
    } else if (deployPick) {
      showView('deployPick');
    }
    const topBat = $('batteryWidget');
    if (topBat) topBat.style.display = deploy ? 'none' : '';
  }

  function busyStacks() {
    const stacks = (state.telemetry && state.telemetry.stacks) || {};
    return ['nav', 'localize', 'slam', 'oms'].filter((k) => stacks[k]);
  }

  function alertIfBusy(action, ignore) {
    const skip = new Set(ignore || []);
    const which = busyStacks().filter((k) => !skip.has(k));
    if (!which.length) return false;
    alert(
      `Cannot ${action}: hardware already running (${which.join(', ')}).\n`
      + 'Stop that stack first so Titan is not launched twice.');
    return true;
  }

  function syncOmsOpenButtons() {
    ['btnOpenOms', 'btnOpenOmsDeploy'].forEach((id) => {
      const el = $(id);
      if (!el) return;
      el.disabled = !state.connected;
      el.title = state.connected
        ? 'Open OMS Software (already linked to this Pi)'
        : 'Connect to the Pi first, then open OMS Software';
    });
  }

  function openOmsSoftware() {
    if (!state.connected) {
      alert('Connect to the Pi first (Connect button on this page).\n'
        + 'OMS Software opens only after Studio is Online — no second Connect.');
      return;
    }
    const url = new URL('oms.html', window.location.href);
    if (state.robot) url.searchParams.set('robot', state.robot);
    url.searchParams.set('from', 'studio');
    url.searchParams.set('pi', StudioApi.getBaseUrl());
    window.open(url.href, 'skill23-oms');
  }

  function setConnected(ok) {
    state.connected = ok;
    $('connBadge').classList.toggle('on', ok);
    $('connText').textContent = ok ? 'Online' : 'Offline';
    if (!ok) updateBattery({ valid: false, source: 'waiting_for_control_server' });
    syncOmsOpenButtons();
  }

  async function connect() {
    const url = $('piUrl').value.trim() || StudioApi.getDefaultUrl();
    StudioApi.setBaseUrl(url);
    try {
      await StudioApi.status();
      setConnected(true);
      StudioApi.connectStream(onTelemetry);
      await refreshMaps();
    } catch (e) {
      setConnected(false);
      alert(`Connect failed: ${e.message}`);
    }
  }

  async function refreshMaps() {
    try {
      const data = await StudioApi.maps();
      const list = $('recentList');
      list.innerHTML = '';
      (data.maps || []).forEach((m) => {
        const el = document.createElement('div');
        el.className = 'recent-item';
        el.innerHTML =
          `<div class="recent-main"><b>${m.name}</b>`
          + `<span>${m.robot || ''} · ${m.has_map ? 'map' : 'empty'}</span></div>`
          + `<button class="btn-del" title="Delete project" type="button">✕</button>`;
        el.querySelector('.recent-main').onclick = () => openProject(m);
        el.querySelector('.btn-del').onclick = async (ev) => {
          ev.stopPropagation();
          if (!confirm(`Delete project "${m.name}"? This cannot be undone.`)) return;
          try {
            await StudioApi.mapDelete(m.name);
            if (state.mapName === m.name) {
              state.mapName = '';
              showView('home');
              setPhase('home');
            }
            await refreshMaps();
          } catch (e) {
            alert(e.message);
          }
        };
        list.appendChild(el);
      });
    } catch (_) {}
  }

  async function openProject(m) {
    state.mapName = m.name;
    state.robot = m.robot || 'grip';
    await StudioApi.session({ map_name: m.name, robot: state.robot, mode: 'offline' });
    $('modeMapLabel').textContent = m.name;
    showView('mode');
    setPhase('mode');
  }

  function renderNodeStatus(t) {
    const ul = $('nodeStatusList');
    if (!ul) return;
    const nodes = t.nodes || {};
    const rows = [];
    const isNav2 = String(t.nav_stack || '') === 'nav2';
    const title = $('nodeStatusTitle');
    if (title) title.textContent = isNav2 ? 'Nav2 status' : 'Stack status';

    const amclOk = !!(t.amcl && t.amcl.ok && (t.amcl.age == null || t.amcl.age < 1.5));
    rows.push({
      name: 'AMCL',
      cls: amclOk ? 'ok' : (t.amcl && t.amcl.ok ? 'warn' : 'bad'),
      detail: t.amcl
        ? `cov=${t.amcl.cov} age=${t.amcl.age ?? '—'}s`
        : 'no pose',
    });
    rows.push({
      name: 'World / map',
      cls: t.world_ready ? 'ok' : 'warn',
      detail: t.world_ready ? 'ready' : 'not ready',
    });
    rows.push({
      name: 'Laser scan',
      cls: (t.scan_count > 8 && (t.scan_age == null || t.scan_age < 1.0)) ? 'ok' : 'bad',
      detail: `pts=${t.scan_count || 0}`
        + (t.scan_raw_count != null ? ` / beams=${t.scan_raw_count}` : '')
        + ` age=${t.scan_age ?? '—'}s`,
    });
    const ca = t.collision_avoidance || {};
    rows.push({
      name: 'Collision avoid',
      cls: isNav2 ? 'ok' : 'warn',
      detail: ca.label
        || (isNav2
          ? 'Path-ahead footprint (local cost ≥254)'
          : '—'),
    });
    const cam = t.camera || {};
    let camCls = 'bad';
    if (cam.process && cam.toggle_ready && (!cam.on || cam.stream)) {
      camCls = 'ok';
    } else if (cam.process) {
      camCls = 'warn';
    }
    rows.push({
      name: 'Camera',
      cls: camCls,
      detail: !cam.process
        ? 'orbbec node off'
        : (cam.on
          ? (cam.stream
            ? 'stream ON · publishing /camera/*'
            : 'stream requested — waiting for toggle_color')
          : (cam.toggle_ready ? 'node up · stream OFF' : 'node up · toggle_color…')),
    });

    const wanted = isNav2
      ? [
        ['planner', 'Planner'],
        ['controller', 'Controller'],
        ['bt_navigator', 'BT navigator'],
        ['behavior', 'Behavior server'],
        ['smoother', 'Smoother'],
        ['map_server', 'Map server'],
        ['ekf', 'EKF'],
        ['lidar', 'Lidar'],
      ]
      : [
        ['planner', 'Planner'],
        ['path_smoother', 'Path smoother'],
        ['controller', 'Controller'],
        ['mission', 'Mission'],
        ['collision', 'Obstacle / collision'],
        ['recovery', 'Recovery'],
        ['goal_checker', 'Goal checker'],
      ];
    wanted.forEach(([key, label]) => {
      const n = nodes[key];
      if (!n) {
        rows.push({
          name: label,
          cls: 'bad',
          detail: isNav2 ? 'not running' : 'no status (is nav running?)',
        });
        return;
      }
      const st = String(n.state || '').toLowerCase();
      let cls = 'warn';
      if (['idle', 'ready', 'ok', 'running', 'tracking', 'wait_goal', 'done'].includes(st)) cls = 'ok';
      if (['error', 'fail', 'failed', 'fault', 'stopped', 'stale'].includes(st)) cls = 'bad';
      if (['recovering', 'waiting', 'planning'].includes(st)) cls = 'warn';
      rows.push({
        name: label,
        cls,
        detail: `${n.state || '—'}${n.detail ? ` — ${n.detail}` : ''}`,
      });
    });

    if (isNav2) {
      const m = t.mission || {};
      rows.push({
        name: 'Mission',
        cls: (m.state === 'running' || m.state === 'done') ? 'ok'
          : (m.state ? 'warn' : 'bad'),
        detail: `${m.state || 'idle'}${m.detail ? ` — ${m.detail}` : ''}`,
      });
    }

    ul.innerHTML = rows.map((r) =>
      `<li><span class="dot ${r.cls}"></span>`
      + `<div><div class="ns-name">${r.name}</div>`
      + `<div class="ns-detail">${r.detail}</div></div></li>`).join('');

    maybeWriteHealthLog(t, rows);
  }

  function ensureHealthSession() {
    if (state.healthSessionId && state.healthStartedAt) return;
    const stamp = new Date().toISOString().replace(/[:.]/g, '-');
    state.healthSessionId = `${state.mapName || 'map'}_${stamp}`;
    state.healthStartedAt = Date.now();
  }

  function maybeWriteHealthLog(t, rows) {
    if (state.phase !== 'deploy') return;
    ensureHealthSession();
    const sig = rows.map((r) => `${r.name}:${r.cls}:${r.detail}`).join('|');
    const now = Date.now();
    // Write on change, or at least every 5s while Deploy is open.
    if (sig === state.healthLastSig && (now - state.healthLastWrite) < 5000) return;
    state.healthLastSig = sig;
    state.healthLastWrite = now;
    const payload = {
      session: state.healthSessionId,
      t: (t && t.ts) || (now / 1000),
      runtime_s: Math.round((now - state.healthStartedAt) / 1000),
      map: state.mapName || '',
      robot: state.robot || '',
      rows: rows.map((r) => ({ name: r.name, status: r.cls, detail: r.detail })),
    };
    fetch('/api/health-log', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    }).catch(() => {});
  }

  function renderDetectAlign(t) {
    const d = t.detect_align || {};
    const oms = t.oms || {};
    const stateEl = $('daState');
    if (!stateEl) return;
    const st = String(d.state || 'idle');
    stateEl.textContent = st;
    const aligned = $('daAligned');
    if (aligned) {
      aligned.textContent = d.aligned
        ? `yes${d.stopped ? ' · stopped' : ''}`
        : (st === 'idle' ? '—' : 'no');
    }
    const omsEl = $('daOms');
    if (omsEl) {
      const ang = Number(d.oms_angle_deg != null ? d.oms_angle_deg : oms.angle_deg);
      const homed = oms.homed === true;
      const omsState = String(oms.state || '');
      if (Number.isFinite(ang)) {
        omsEl.textContent = `${ang.toFixed(1)}°${homed ? '' : ' · homing'}${omsState && omsState !== 'idle' ? ` · ${omsState}` : ''}`;
      } else {
        omsEl.textContent = omsState || '—';
      }
    }
    const depth = $('daDepth');
    if (depth) {
      const optical = Number(d.depth_m);
      const horiz = Number(d.depth_horizontal_m);
      if (Number.isFinite(optical)) {
        depth.textContent = Number.isFinite(horiz)
          ? `${optical.toFixed(3)} m  (horiz ${horiz.toFixed(3)} m)`
          : `${optical.toFixed(3)} m`;
      } else {
        depth.textContent = '—';
      }
    }
    const obj = $('daObject');
    if (obj) {
      const name = d.class_name || '';
      const conf = Number(d.conf);
      const bits = [];
      if (name) bits.push(name);
      if (Number.isFinite(conf)) bits.push(`${Math.round(conf * 100)}%`);
      obj.textContent = bits.join(' · ') || '—';
    }
    const grip = $('daGrip');
    if (grip) {
      const gu = Number(d.gripper_u);
      grip.textContent = Number.isFinite(gu) ? `u ${gu.toFixed(3)}` : 'center';
    }
    const center = $('daCenter');
    if (center) {
      const ox = Number(d.obj_cx);
      const fx = Number(d.frame_cx);
      center.textContent = (Number.isFinite(ox) && Number.isFinite(fx))
        ? `obj ${ox.toFixed(0)}  frame ${fx.toFixed(0)}`
        : '—';
    }
    const uEl = $('daU');
    if (uEl) {
      const u = Number(d.u_norm);
      uEl.textContent = Number.isFinite(u) ? u.toFixed(3) : '—';
    }
    const cf = $('daConfirm');
    if (cf) {
      cf.textContent = d.confirm_need
        ? `${d.confirm || 0}/${d.confirm_need}`
        : '—';
    }
    const res = $('daResult');
    if (res) {
      if (d.last_ok === true) res.textContent = `PASS  ${d.last_result || ''}`.trim();
      else if (d.last_ok === false) res.textContent = `FAIL  ${d.last_result || ''}`.trim();
      else res.textContent = '—';
    }
    const hint = $('daHint');
    if (hint) hint.textContent = d.message || 'Snapshot YOLO + OMS rotate until a Detect+Align waypoint';
    const log = $('daLog');
    if (log) {
      const lines = [
        d.message || '',
        Number.isFinite(Number(d.u_norm)) ? `u=${Number(d.u_norm).toFixed(3)}` : '',
        Number.isFinite(Number(d.oms_angle_deg != null ? d.oms_angle_deg : oms.angle_deg))
          ? `oms=${Number(d.oms_angle_deg != null ? d.oms_angle_deg : oms.angle_deg).toFixed(1)}°`
          : '',
        d.last_result ? `result ${d.last_ok ? 'ok' : 'fail'}: ${d.last_result}` : '',
      ].filter(Boolean);
      log.textContent = lines.join('\n') || '—';
    }
  }

  function onTelemetry(t) {
    state.telemetry = t;
    setConnected(true);
    if (t.robot_dims) StudioMap.setDims(t.robot_dims);
    if (t.pose) {
      state.lastPose = t.pose;
      StudioMap.setPose(t.pose);
    }
    if (t.map) StudioMap.setMap(t.map);
    updateSimStyleUi();
    // Deploy / localize UI: map + waypoints + planned path only (no scan/trail/pose paint).
    const cleanMap = state.phase === 'deploy'
      || state.phase === 'waypoints'
      || state.phase === 'improve';
    if (typeof StudioMap.setShowScan === 'function') {
      StudioMap.setShowScan(!cleanMap);
    }
    if (typeof StudioMap.setShowPose === 'function') {
      StudioMap.setShowPose(state.phase === 'mapping' || state.phase === 'improve');
    }
    StudioMap.setScan(cleanMap ? [] : (t.scan || []));
    StudioMap.setTrail(cleanMap ? [] : (t.trail || []));
    StudioMap.setPaths(t.path || [], t.planner_path || [], t.smoothed_path || []);
    if (state.simActive) {
      const sim = t.sim || {};
      StudioMap.setSim(state.simPoints, simPathForStyle(sim), state.simStyle);
      updateSimHint(sim.result);
    }
    updateGcPlannerUi(t.stacks);

    updateBattery(t.battery);

    const warns = t.warnings || [];
    $('warnList').innerHTML = warns.map((w) => `<li>${w}</li>`).join('');

    const sess = t.session || {};
    if (sess.map_name) state.mapName = sess.map_name;

    const ds = $('deployStat');
    if (ds) {
      const m = t.mission || {};
      const r = t.reloc || {};
      const from = m.started_from === 'physical'
        ? 'physical button'
        : (m.started_from === 'dashboard' ? 'dashboard' : '—');
      const armed = Number.isFinite(+m.armed_count)
        ? `armed:${m.armed_count}${m.sequence ? ` (${m.sequence})` : ''}`
        : '';
      ds.textContent = [
        `AMCL: ${t.amcl && t.amcl.ok ? 'ok' : '—'} cov=${t.amcl ? t.amcl.cov : '—'}`,
        `goal: ${t.goal_status || '—'}`,
        `mission: ${m.state || m.status || '—'}`,
        `start from: ${from}`,
        armed,
        `procs: ${(t.logs && t.logs.running) || (t.processes || []).join(',') || '—'}`,
        t.estop ? 'E-STOP ACTIVE' : '',
        r.active ? `reloc:${r.motion} ${r.elapsed || 0}s` : (r.result ? `reloc:${r.result}` : ''),
      ].filter(Boolean).join(' · ');
      if (m.hw_alert && m.hw_alert !== state.lastHwAlert) {
        state.lastHwAlert = m.hw_alert;
        alert(m.hw_alert);
      }
    }

    const rh = $('relocHint');
    if (rh) {
      const r = t.reloc || {};
      if (r.active) {
        rh.textContent =
          `Reloc RUNNING (${r.motion}) ${r.elapsed || 0}s — spinning until cov stays low…`
          + ` cov=${t.amcl ? t.amcl.cov : '—'}`;
      } else if (r.result === 'ok') {
        rh.textContent = 'Reloc OK — pose locked';
      } else if (r.result === 'failed') {
        rh.textContent = 'Reloc FAILED — try Set pose, then spin again';
      } else if (r.result === 'stopped') {
        rh.textContent = 'Reloc stopped';
      } else {
        rh.textContent = 'Reloc: idle';
      }
    }

    renderNodeStatus(t);
    renderDetectAlign(t);
    renderCommissionTelemetry(t);

    const logRunning = $('logRunning');
    if (logRunning) logRunning.textContent = `running: ${(t.logs && t.logs.running) || '—'}`;
    const setLog = (id, text) => {
      const el = $(id);
      if (el) el.textContent = text || '(empty)';
    };
    if (t.logs) {
      setLog('logLocalize', t.logs.localize);
      setLog('logSlam', t.logs.slam);
      setLog('logNav', t.logs.nav);
      setLog('logDetectAlign', t.logs.detect_align);
      setLog('logOrbbec', t.logs.camera);
    }
    updateLocalizeButtons(t.stacks || {});
    const cam = t.camera || {};
    const camHint = $('camHint');
    if (camHint) {
      if (!cam.process) {
        camHint.textContent = cam.killed
          ? 'Orbbec node: killed (Camera ON or re-Deploy to relaunch)'
          : 'Orbbec node: not running (enter Deploy to launch)';
      } else if (!cam.toggle_ready) {
        camHint.textContent = 'Orbbec node up — waiting for /camera/toggle_color…';
      } else if (!cam.on) {
        camHint.textContent = 'Orbbec node running · stream OFF (toggle_color)';
      } else if (!cam.stream) {
        camHint.textContent = 'Stream requested — waiting for toggle_color…';
      } else {
        const topic = (cam.topics && cam.topics.color) || '/camera/color/image_raw';
        camHint.textContent = `Stream ON · publishing on ${topic}`
          + (cam.depth_stream ? ' (+ depth)' : '');
      }
    }
    const camLog = $('logCamera') || $('logOrbbec');
    if (camLog) {
      camLog.textContent = (cam.logs || []).join('\n')
        || (t.logs && t.logs.camera)
        || '(empty)';
    }

    $('stageHud').textContent = [
      state.mapName || '—',
      state.robot,
      state.mode,
      t.pose ? `(${t.pose.x.toFixed(2)}, ${t.pose.y.toFixed(2)}) yaw=${t.pose.yaw.toFixed(2)}` : 'no pose',
      t.map ? `res=${Number(t.map.resolution).toFixed(3)}m` : '',
    ].filter(Boolean).join(' · ');
  }

  function updateLocalizeButtons(stacks) {
    const running = !!(stacks && stacks.localize);
    state.localizeRunning = running;
    const start = $('btnStartLocalize');
    const stop = $('btnStopLocalize');
    if (start && !start.classList.contains('loading')) {
      start.disabled = running;
      start.title = running ? 'Localize already running' : 'Start map_server + AMCL';
    }
    if (stop && !stop.classList.contains('loading')) {
      stop.disabled = !running;
      stop.title = running ? 'Stop localization' : 'Localize already stopped';
    }
    const hint = $('locHint');
    if (hint) hint.textContent = running ? 'Localize: running' : 'Localize: stopped';
  }

  document.querySelectorAll('[data-go]').forEach((btn) => {
    btn.addEventListener('click', () => {
      const go = btn.getAttribute('data-go');
      if (go === 'home') { showView('home'); setPhase('home'); refreshMaps(); }
      else if (go === 'create') { showView('create'); setPhase('create'); }
      else if (go === 'import') { showView('import'); setPhase('import'); }
    });
  });

  $('btnConnect').onclick = (ev) => runWithLoading(
    ev.currentTarget, connect, 'Connecting…');

  function bindOmsOpen(id) {
    const el = $(id);
    if (el) el.onclick = () => openOmsSoftware();
  }
  bindOmsOpen('btnOpenOms');
  bindOmsOpen('btnOpenOmsDeploy');
  syncOmsOpenButtons();
  $('piUrl').value = StudioApi.getBaseUrl();

  document.querySelectorAll('#robotChips .chip').forEach((chip) => {
    chip.addEventListener('click', () => {
      document.querySelectorAll('#robotChips .chip').forEach((c) => c.classList.remove('active'));
      chip.classList.add('active');
      state.robot = chip.dataset.robot;
    });
  });

  bindLoading('btnStartMapping', async () => {
    const name = $('mapName').value.trim();
    if (!name) return alert('Enter a map name');
    if (alertIfBusy('start mapping', ['slam'])) return;
    const resolution = parseFloat($('mapResolution').value);
    const max_laser_range = parseFloat($('mapMaxRange').value);
    const min_laser_range = parseFloat($('mapMinRange').value);
    if (!(resolution >= 0.01 && resolution <= 0.2)) {
      return alert('Resolution must be between 0.01 and 0.20 m/cell');
    }
    state.mapName = name;
    await StudioApi.session({ map_name: name, robot: state.robot, mode: 'arena' });
    await StudioApi.mappingStart({
      map_name: name,
      robot: state.robot,
      resolution,
      max_laser_range,
      min_laser_range,
    });
    state.mode = 'arena';
    StudioMap.clearUndo();
    setPhase('mapping');
  }, 'Starting SLAM…');

  bindLoading('btnDoImport', async () => {
    const name = $('importName').value.trim();
    const yaml_path = $('importYaml').value.trim();
    const pgm_path = $('importPgm').value.trim() || null;
    const robot = $('importRobot').value;
    if (!name || !yaml_path) return alert('Name and YAML path required');
    await StudioApi.mapImport({ map_name: name, yaml_path, pgm_path, robot });
    state.mapName = name;
    state.robot = robot;
    $('modeMapLabel').textContent = name;
    showView('mode');
    setPhase('mode');
    refreshMaps();
  }, 'Importing…');

  async function pickMode(mode) {
    state.mode = mode;
    await StudioApi.session({
      map_name: state.mapName,
      robot: state.robot,
      mode,
    });
    StudioMap.clearUndo();
    try {
      const r = await StudioApi.restrictionsGet();
      state.restrictions = r.rectangles || [];
      StudioMap.setRestrictions(state.restrictions);
      ensureMapEditUi();
      updateZoneHint();
    } catch (_) {}
    try {
      const w = await StudioApi.waypointsGet();
      state.waypoints = w.waypoints || [];
      StudioMap.setWaypoints(state.waypoints);
      renderWpList();
    } catch (_) {}

    if (mode === 'arena') {
      await StudioApi.phase('waypoints');
      setPhase('waypoints');
      StudioMap.setShowTrail(false);
    } else {
      await StudioApi.phase('improve');
      setPhase('improve');
    }
  }
  $('btnModeArena').onclick = (ev) => runWithLoading(
    ev.currentTarget, () => pickMode('arena'), 'Opening…');
  $('btnModeOffline').onclick = (ev) => runWithLoading(
    ev.currentTarget, () => pickMode('offline'), 'Loading…');

  const btnStartLoc = $('btnStartLocalize');
  if (btnStartLoc) {
    btnStartLoc.onclick = async (ev) => {
      if (state.localizeRunning) {
        alert('Localize is already running.');
        updateLocalizeButtons({ localize: true });
        return;
      }
      await runWithLoading(ev.currentTarget, async () => {
        if (state.localizeRunning) {
          alert('Localize is already running.');
          return;
        }
        if (!state.mapName) return alert('No map selected');
        if (alertIfBusy('start localize', ['localize'])) return;
        const res = await StudioApi.localizeStart({
          map_name: state.mapName,
          robot: state.robot,
        });
        if (res.already) {
          alert('Localize is already running.');
          state.localizeRunning = true;
          return;
        }
        state.localizeRunning = true;
        StudioMap.setShowTrail(true);
        alert(
          'Started map_server + AMCL.\n'
          + 'Wait ~8s for lidars, then Set pose if the robot is off the map.\n'
          + 'Drive with teleop. At each stop, click Save waypoint (x, y, z, yaw).');
      }, 'Starting localize…');
      updateLocalizeButtons({ localize: state.localizeRunning });
    };
  }
  const btnStopLoc = $('btnStopLocalize');
  if (btnStopLoc) {
    btnStopLoc.onclick = async (ev) => {
      if (!state.localizeRunning) {
        alert('Localize is already stopped.');
        updateLocalizeButtons({ localize: false });
        return;
      }
      await runWithLoading(ev.currentTarget, async () => {
        if (!state.localizeRunning) {
          alert('Localize is already stopped.');
          return;
        }
        const res = await StudioApi.localizeStop();
        if (res.already) {
          alert('Localize is already stopped.');
          state.localizeRunning = false;
          return;
        }
        state.localizeRunning = false;
        StudioMap.setShowTrail(false);
      }, 'Stopping localize…');
      updateLocalizeButtons({ localize: state.localizeRunning });
    };
  }

  bindLoading('btnRemap', async () => {
    if (!confirm('Remap clears the live SLAM map. Continue?')) return;
    await StudioApi.mappingRemap();
  }, 'Remapping…');
  bindLoading('btnStopMapping', async () => {
    await StudioApi.mappingStop();
  }, 'Stopping…');
  bindLoading('btnClearTrail', async () => {
    await StudioApi.trailClear();
    StudioMap.clearTrailLocal();
  }, 'Clearing…');
  bindLoading('btnSaveMap', async () => {
    await StudioApi.mapSave({ map_name: state.mapName });
    try { await StudioApi.mappingStop(); } catch (_) {}
    StudioMap.clearTrailLocal();
    StudioMap.setShowTrail(false);
    $('modeMapLabel').textContent = state.mapName;
    showView('mode');
    setPhase('mode');
    alert('Map saved — mapping stopped.\nChoose Arena (live) to localize, or Offline to edit.');
  }, 'Saving map…');

  bindToolButtons();

  function clearMeasuresUi() { StudioMap.clearMeasures(); }
  ['btnClearMeasures', 'btnClearMeasuresMap', 'btnClearMeasuresDeploy'].forEach((id) => {
    const el = $(id);
    if (el) el.onclick = clearMeasuresUi;
  });

  StudioMap.init($('mapCanvas'), (x, y, kind, yaw) => {
    if (kind === 'simpt') {
      if (state.simPoints.length >= 2) state.simPoints = [];
      state.simPoints.push({
        x: +x.toFixed(3), y: +y.toFixed(3), yaw: +yaw.toFixed(3),
      });
      StudioMap.setSim(state.simPoints, []);
      updateSimHint();
      return;
    }
    if (kind === 'waypoint') {
      state.waypoints.push({
        x: +x.toFixed(3),
        y: +y.toFixed(3),
        z: 0,
        yaw: +yaw.toFixed(3),
        name: `wp${state.waypoints.length + 1}`,
        action: { type: 'wait', seconds: 0 },
      });
      const newIdx = state.waypoints.length - 1;
      StudioMap.setWaypoints(state.waypoints);
      // Auto-append into the mission sequence being built + refresh default 1→2→…
      state.sequenceBuilder.push(newIdx);
      ensureDefaultSequence();
      refreshSeqSelect();
      renderWpList();
      if (state.phase === 'deploy') {
        renderDeployWpStrip();
        renderWpTaskEditors();
        updateSeqPreview();
      }
      StudioApi.waypointsSave(cleanWaypoints(state.waypoints)).catch(() => {});
      return;
    }
    if (kind === 'pose') {
      StudioApi.setPose(x, y, yaw).catch((e) => alert(e.message));
      return;
    }
    if (kind === 'goal') {
      // In deploy, adding a goal appends to mission list preview only if desired —
      // keep separate from saved project waypoints; push as one-shot goal list
      const goals = state.waypoints.length
        ? state.waypoints
        : [{ x: +x.toFixed(3), y: +y.toFixed(3), yaw: +yaw.toFixed(3) }];
      if (!state.waypoints.length) {
        state.waypoints.push(goals[0]);
        StudioMap.setWaypoints(state.waypoints);
        renderWpList();
        renderDeployWpStrip();
      }
      StudioApi.setGoals(buildMissionWaypoints()).catch((e) => alert(e.message));
    }
  });

  StudioMap.setOnEditCells((action, cells) => {
    state.pendingEdits.push({ action, cells });
    updateZoneHint();
  });

  StudioMap.setOnRestrict((rects, info) => {
    state.restrictions = rects;
    updateZoneHint();
    if (info && (info.removed || info.cleared || info.restored)) {
      // A deleted zone must be unburned on the Pi too, otherwise its black
      // cells stay in map.pgm and the next burn paints the red box back.
      syncZoneEdits().catch((e) => alert(e.message || String(e)));
    }
  });

  /** Push painted cells + the current zone list, then re-burn from the base map. */
  async function runZoneSync() {
    if (state.pendingEdits.length) {
      const ops = state.pendingEdits;
      state.pendingEdits = [];
      try {
        await StudioApi.mapEdit(ops);
      } catch (e) {
        state.pendingEdits = ops.concat(state.pendingEdits);
        throw e;
      }
    }
    await StudioApi.mapRestrictions(state.restrictions, true);
    StudioMap.clearUndo();
    updateZoneHint();
  }

  /** Serialised: clicking Remove zone several times must not race on map.pgm. */
  let zoneSyncChain = Promise.resolve();
  function syncZoneEdits() {
    zoneSyncChain = zoneSyncChain.then(runZoneSync, runZoneSync);
    return zoneSyncChain;
  }

  /** Bind once per button — markup may be static or injected below. */
  function bindToolButtons(root) {
    (root || document).querySelectorAll('.tool[data-tool]').forEach((btn) => {
      if (btn.dataset.toolBound === '1') return;
      btn.dataset.toolBound = '1';
      btn.addEventListener('click', () => activateTool(btn));
    });
  }

  function ensureMapEditUi() {
    const existing = $('mapEditTools');
    if (existing) {
      wireMapEditUi(existing);
      return;
    }
    const anchor = $('btnUndoEdit');
    if (!anchor || !anchor.parentElement) return;
    const box = document.createElement('div');
    box.id = 'mapEditTools';
    box.className = 'edit-tools';
    box.innerHTML = [
      '<div class="edit-tools-title">Fast wall editing</div>',
      '<div class="tool-row">',
      '<button class="tool" type="button" data-tool="wallbox">Wall box</button>',
      '<button class="tool" type="button" data-tool="erasebox">Erase box</button>',
      '</div>',
      '<label class="edit-brush">',
      '<span>Brush <b id="brushLabel">1</b> cells</span>',
      '<input id="brushSize" type="range" min="1" max="60" step="1" value="1">',
      '</label>',
      '<div class="edit-hint">',
      'Wall/Erase drag paints a continuous stroke · Alt+wheel resizes the brush',
      ' · box tools fill a whole rectangle in one drag.',
      '</div>',
      '<div class="edit-tools-title">Restriction zones</div>',
      '<div class="tool-row">',
      '<button class="tool" type="button" data-tool="restrict">Add zone</button>',
      '<button class="tool" type="button" data-tool="restrictdel">Remove zone</button>',
      '</div>',
      '<button id="btnClearZones" class="btn btn-ghost" type="button">',
      'Clear all zones</button>',
      '<div class="edit-hint" id="zoneHint">No zones.</div>',
    ].join('');
    anchor.parentElement.insertBefore(box, anchor);
    wireMapEditUi(box);
  }

  function wireMapEditUi(box) {
    bindToolButtons(box);
    if (box.dataset.wired === '1') return;
    box.dataset.wired = '1';
    const slider = $('brushSize');
    const label = $('brushLabel');
    if (slider) {
      slider.addEventListener('input', () => {
        const cells = StudioMap.setBrush(slider.value);
        if (label) label.textContent = cells;
      });
    }
    const clear = $('btnClearZones');
    if (clear) {
      clear.onclick = () => {
        if (!state.restrictions.length) return alert('No zones to clear');
        if (!confirm(
          `Remove all ${state.restrictions.length} restriction zone(s) and clear `
          + 'the cells they blackened on the map?')) return;
        StudioMap.clearRestrictions();
      };
    }
    updateZoneHint();
  }

  /** Single active highlight across the built-in and injected tool buttons. */
  function activateTool(btn) {
    if (state.simActive) exitSim();
    document.querySelectorAll('.tool[data-tool]').forEach(
      (b) => b.classList.remove('active'));
    btn.classList.add('active');
    StudioMap.setTool(btn.dataset.tool);
  }

  function updateZoneHint() {
    const hint = $('zoneHint');
    if (!hint) return;
    const n = state.restrictions.length;
    const pending = state.pendingEdits.length;
    hint.textContent = [
      n ? `${n} zone(s).` : 'No zones.',
      'Remove zone: click inside a red box — it also clears its black cells.',
      pending ? `${pending} unsaved paint op(s).` : '',
    ].filter(Boolean).join(' ');
  }

  StudioMap.setOnBrushChange((cells) => {
    const slider = $('brushSize');
    const label = $('brushLabel');
    if (slider) slider.value = cells;
    if (label) label.textContent = cells;
  });

  ensureMapEditUi();

  $('btnUndoEdit').onclick = () => {
    if (!StudioMap.undo()) alert('Nothing to undo');
    else if (state.pendingEdits.length) state.pendingEdits.pop();
    updateZoneHint();
  };

  bindLoading('btnSaveEdits', async () => {
    if (!state.pendingEdits.length) return alert('No edits yet — paint walls/erase first');
    await StudioApi.mapEdit(state.pendingEdits);
    state.pendingEdits = [];
    StudioMap.clearUndo();
    updateZoneHint();
    alert('Edits saved to map.pgm');
  }, 'Saving edits…');

  bindLoading('btnSaveRestrictions', async () => {
    // Burn now so the map matches the zone list: added zones turn black,
    // removed zones go back to how the map looked before they were burned.
    await syncZoneEdits();
    alert(
      state.restrictions.length
        ? `${state.restrictions.length} zone(s) saved and applied to the map.`
        : 'All zones removed — map restored where they were burned.');
  }, 'Saving…');

  function ensureWaypointInputUi() {
    let box = $('wpInputModes');
    if (!box) {
      const save = $('btnSaveWp');
      if (!save || !save.parentElement) return;
      box = document.createElement('div');
      box.id = 'wpInputModes';
      box.className = 'wp-input-modes';
      box.innerHTML = [
        '<div class="wp-input-title">Waypoint input</div>',
        '<div class="wp-input-buttons">',
        '<button id="btnWpInputLive" class="chip" type="button">Teleop / robot pose</button>',
        '<button id="btnWpInputMap" class="chip" type="button">Mark directly on map</button>',
        '</div>',
      ].join('');
      save.parentElement.insertBefore(box, save);
    }
    if (box.dataset.wired === '1') return;
    box.dataset.wired = '1';
    const live = $('btnWpInputLive');
    const map = $('btnWpInputMap');
    if (live) live.onclick = () => setWaypointInputMode('live');
    if (map) map.onclick = () => setWaypointInputMode('map');
  }

  function setWaypointInputMode(mode) {
    state.waypointInputMode = mode === 'map' ? 'map' : 'live';
    StudioMap.setTool(state.waypointInputMode === 'map' ? 'waypoint' : 'pan');
    updateWaypointInputUi();
  }

  function updateWaypointInputUi() {
    ensureWaypointInputUi();
    const live = $('btnWpInputLive');
    const map = $('btnWpInputMap');
    const save = $('btnSaveWp');
    const hint = $('wpHint');
    const offline = state.mode === 'offline';
    const mapMode = offline || state.waypointInputMode === 'map';
    if (live) {
      live.classList.toggle('active', !mapMode);
      live.disabled = offline;
    }
    if (map) map.classList.toggle('active', mapMode);
    if (save) {
      save.disabled = mapMode;
      save.textContent = mapMode
        ? 'Click-drag map to add waypoint'
        : 'Save waypoint at robot pose';
    }
    if (hint) {
      hint.textContent = mapMode
        ? 'Click-drag on the map: press sets position, drag sets waypoint yaw.'
        : 'Drive with teleop, stop at the location, then Save waypoint at robot pose.';
    }
  }

  ensureWaypointInputUi();

  bindLoading('btnToWaypoints', async () => {
    await loadSavedWaypoints();
    await StudioApi.phase('waypoints');
    state.waypointInputMode = state.mode === 'offline' ? 'map' : 'live';
    setPhase('waypoints');
    StudioMap.setShowTrail(true);
    updateWaypointInputUi();
  }, 'Loading…');

  async function finishImproveToDeploy() {
    const res = await openDeployPicker({ saveCurrent: true });
    if (res && res.busy) {
      alert('Deploy already opening — please wait.');
      return;
    }
  }
  bindLoading('btnFinishImprove', () => finishImproveToDeploy(), 'Opening deploy…');

  bindLoading('btnSkipWaypoints', async () => {
    const wps = await loadSavedWaypoints();
    const res = await openDeployPicker({ saveCurrent: false });
    if (res && res.busy) {
      alert('Deploy already opening — please wait.');
      return;
    }
    alert(`Skipped waypoint editing — loaded ${wps.length} saved waypoint(s).`);
  }, 'Opening deploy…');

  bindLoading('btnReloadWp', async () => {
    const wps = await loadSavedWaypoints();
    alert(`Loaded ${wps.length} saved waypoint(s)`);
  }, 'Loading…');

  function renderWpList() {
    const list = $('wpList');
    if (list) {
      list.innerHTML = state.waypoints.map((w, i) => {
        const tag = w.sticker ? ` [${stickerDisplay(w.sticker)}]` : '';
        let task = '';
        if (w.action && w.action.type === 'wait') {
          task = ` · wait ${Math.max(0, Number(w.action.seconds) || 0)}s`;
        } else if (w.action && w.action.type === 'detect_align') {
          task = ' · detect+align';
        } else if (w.action && w.action.type === 'align') {
          task = ' · align yaw';
        } else if (w.action && w.action.type === 'align_move') {
          task = ` · align+${Number(w.action.distance_m) || 0}m`;
        }
        return `<li>${i + 1}.${tag} (${w.x}, ${w.y}, ${w.z ?? 0}) ${Math.round(((w.yaw || 0) * 180) / Math.PI)}°`
          + ` (${(w.yaw || 0).toFixed(2)} rad)${task}</li>`;
      }).join('');
    }
    renderWpTaskEditors();
    if (isCommission()) renderCommissionStickerSummary();
  }

  bindLoading('btnSaveWp', async () => {
    if (state.mode === 'arena') {
      const p = state.lastPose;
      if (!p) return alert('No live pose yet');
      state.waypoints.push({
        x: p.x, y: p.y, z: p.z ?? 0, yaw: p.yaw,
        name: `wp${state.waypoints.length + 1}`,
      });
    }
    StudioMap.setWaypoints(state.waypoints);
    renderWpList();
    await StudioApi.waypointsSave(state.waypoints);
  }, 'Saving…');

  bindLoading('btnSaveToPi', () => saveWaypointsAndSequencesToPi(), 'Saving to Pi…');
  bindLoading('btnSaveToPiDeploy', () => saveWaypointsAndSequencesToPi(), 'Saving to Pi…');

  bindLoading('btnClearWp', async () => {
    state.waypoints = [];
    StudioMap.setWaypoints([]);
    renderWpList();
    await StudioApi.waypointsSave([]);
  }, 'Clearing…');

  bindLoading('btnToDeploy', async () => {
    const res = await openDeployPicker({ saveCurrent: true });
    if (res && res.busy) {
      alert('Deploy already opening — please wait.');
      return;
    }
  }, 'Opening deploy…');

  bindLoading('btnSkipWpToDeploy', async () => {
    const wps = await loadSavedWaypoints();
    const res = await openDeployPicker({ saveCurrent: false });
    if (res && res.busy) {
      alert('Deploy already opening — please wait.');
      return;
    }
    alert(`Using ${wps.length} saved waypoint(s) (edits discarded).`);
  }, 'Opening deploy…');

  function renderDeployWpStrip() {
    const box = $('deployWpStrip');
    if (!box) return;
    box.innerHTML = '';
    state.waypoints.forEach((wp, i) => {
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.textContent = String(i + 1);
      btn.title = `(${wp.x}, ${wp.y})`;
      btn.onclick = () => {
        state.sequenceBuilder.push(i);
        updateSeqPreview();
      };
      box.appendChild(btn);
    });
    updateSeqPreview();
  }

  function updateSeqPreview() {
    const el = $('seqChainPreview');
    if (!el) return;
    el.textContent = state.sequenceBuilder.length
      ? state.sequenceBuilder.map((i) => i + 1).join(' → ')
      : '— (click waypoint numbers, or use default 1→2→3…)';
    renderWpTaskEditors();
    syncArmedMission();
  }

  function refreshSeqSelect() {
    const sel = $('seqSelect');
    if (!sel) return;
    const cur = sel.value;
    sel.innerHTML = '<option value="">— sequences —</option>';
    Object.keys(state.sequences).sort((a, b) => {
      if (a === DEFAULT_SEQ) return -1;
      if (b === DEFAULT_SEQ) return 1;
      return a.localeCompare(b);
    }).forEach((name) => {
      const opt = document.createElement('option');
      opt.value = name;
      const n = (state.sequences[name] || []).length;
      opt.textContent = name === DEFAULT_SEQ
        ? `default (1→${n || '…'})`
        : `${name} (${n})`;
      sel.appendChild(opt);
    });
    if (cur && state.sequences[cur]) sel.value = cur;
    else if (state.sequences[DEFAULT_SEQ]) sel.value = DEFAULT_SEQ;
  }

  function persistSequences() {
    localStorage.setItem(SEQ_LS, JSON.stringify(state.sequences));
    refreshSeqSelect();
  }

  async function saveWaypointsAndSequencesToPi() {
    ensureDefaultSequence();
    const res = await StudioApi.piSave(
      cleanWaypoints(state.waypoints),
      state.sequences || {});
    const nSeq = Object.keys(state.sequences || {}).length;
    alert(
      `Saved on Pi:\n`
      + `${res.waypoints_path || 'waypoints.json'} (${res.waypoints || 0} WPs)\n`
      + `${res.sequences_path || 'sequences.json'} (${res.sequences || nSeq} sequences)\n\n`
      + (res.hint || ''));
    return res;
  }

  function isCommission() {
    return state.deployModule === 'commission';
  }

  function nextSticker(kind) {
    const prefix = kind === 'drop' ? 'drop' : 'pick';
    const used = new Set(
      (state.waypoints || [])
        .map((w) => String(w.sticker || '').trim().toLowerCase())
        .filter(Boolean),
    );
    let n = 1;
    while (used.has(prefix + n) || used.has((kind === 'drop' ? 'd' : 'p') + n)) n += 1;
    return prefix + n;
  }

  function applyDeployModuleUi() {
    const hud = $('deployHud');
    if (hud) {
      hud.classList.toggle('module-nav', !isCommission());
      hud.classList.toggle('module-commission', isCommission());
    }
    const badge = $('deployModuleBadge');
    if (badge) {
      badge.textContent = isCommission()
        ? 'Commissioning'
        : 'Navigation and localisation';
    }
    const hint = $('missionSourceHint');
    if (hint && state.phase === 'deploy') {
      hint.innerHTML = isCommission()
        ? 'Commissioning Module G: mark pick1…, drop1…, home on waypoints. Assign objects→drops, Start Nav2+camera, then Start mission.'
        : 'Navigation: Start Nav2. Tick “Start Nav2 with Align node” to launch Align for Wait / Align / Align+distance WP tasks.';
    }
    const wpLabel = $('deployWpTasksLabel');
    if (wpLabel) {
      wpLabel.textContent = isCommission() ? 'Waypoint stickers' : 'WP tasks';
    }
    const inDeployCommission = state.phase === 'deploy' && isCommission();
    const cmp = $('commissionPanel');
    if (cmp) cmp.hidden = !inDeployCommission;
    const dap = $('detectAlignPanel');
    if (dap) dap.hidden = !(state.phase === 'deploy' && isCommission());
    const navAlignRow = $('navAlignRow');
    if (navAlignRow) {
      navAlignRow.style.display = (state.phase === 'deploy' && !isCommission()) ? '' : 'none';
    }
    // Always refresh WP task cards in both Navigation and Commissioning.
    renderWpTaskEditors();
    if (isCommission()) {
      renderCommissionPanel();
      renderCommissionStickerSummary();
    }
  }

  function renderCommissionStickerSummary() {
    const el = $('commissionStickerSummary');
    if (!el || !isCommission()) return;
    const wps = state.waypoints || [];
    const picks = wps
      .map((w, i) => ({ w, i }))
      .filter(({ w }) => stickerKind(w.sticker) === 'pick');
    const drops = wps
      .map((w, i) => ({ w, i }))
      .filter(({ w }) => stickerKind(w.sticker) === 'drop');
    const home = wps.find((w) => stickerKind(w.sticker) === 'home');
    const pickText = picks.length
      ? picks.map(({ w, i }) => {
        const n = Math.max(1, Number(w.pick_count) || 1);
        const lift = Math.max(0, Number(w.pick_lift_mm) || 0);
        return `${stickerDisplay(w.sticker)}×${n} @${lift}mm (WP ${i + 1})`;
      }).join(', ')
      : 'none yet';
    const dropText = drops.length
      ? drops.map(({ w, i }) => `${stickerDisplay(w.sticker)} (WP ${i + 1})`).join(', ')
      : 'none yet';
    const homeText = home
      ? `WP ${wps.indexOf(home) + 1}`
      : 'not set';
    el.innerHTML =
      `<div><strong>Picks</strong>: ${pickText}</div>`
      + `<div><strong>Drops</strong>: ${dropText}</div>`
      + `<div><strong>Home</strong>: ${homeText}</div>`;
    if (!wps.length) {
      el.insertAdjacentHTML(
        'beforeend',
        '<p class="hint">No waypoints yet — use <strong>Mark WP</strong> on the map.</p>');
    }
  }

  function stickerKind(sticker) {
    const s = String(sticker || '').trim().toLowerCase();
    if (!s) return '';
    if (s === 'home' || s === 'start') return 'home';
    if (/^(pick|p)\d+$/.test(s)) return 'pick';
    if (/^(drop|d)\d+$/.test(s)) return 'drop';
    return '';
  }

  function stickerDisplay(sticker) {
    const s = String(sticker || '').trim().toLowerCase();
    if (!s) return '';
    if (s === 'home' || s === 'start') return 'HOME';
    const pm = s.match(/^(pick|p)(\d+)$/);
    if (pm) return `P${pm[2]}`;
    const dm = s.match(/^(drop|d)(\d+)$/);
    if (dm) return `D${dm[2]}`;
    return s.toUpperCase();
  }

  function listDropStickers() {
    const drops = (state.waypoints || [])
      .map((w) => String(w.sticker || '').trim().toLowerCase())
      .filter((s) => stickerKind(s) === 'drop')
      .map((s) => {
        const m = s.match(/^(?:drop|d)(\d+)$/);
        return m ? `drop${m[1]}` : s;
      });
    return [...new Set(drops)].sort((a, b) => {
      const na = parseInt(a.replace(/\D/g, ''), 10) || 0;
      const nb = parseInt(b.replace(/\D/g, ''), 10) || 0;
      return na - nb;
    });
  }

  async function loadCommissionConfig() {
    try {
      const res = await StudioApi.commissionGet();
      state.commissionConfig = (res && res.config) || null;
    } catch (e) {
      console.warn('commission config load failed', e);
      state.commissionConfig = {
        drop_assignments: Object.fromEntries(COMMISSION_OBJECTS.map((o) => [o, ''])),
        detect_retries: 3,
        yaw_tolerance_deg: 1.0,
      };
    }
    renderCommissionPanel();
    renderCommissionStickerSummary();
    return state.commissionConfig;
  }

  function renderCommissionPanel() {
    const box = $('commissionDropAssign');
    if (!box || !isCommission()) return;
    const cfg = state.commissionConfig || {
      drop_assignments: Object.fromEntries(COMMISSION_OBJECTS.map((o) => [o, ''])),
    };
    const drops = listDropStickers();
    const assigns = cfg.drop_assignments || {};
    box.innerHTML = COMMISSION_OBJECTS.map((obj) => {
      const cur = String(assigns[obj] || '');
      const opts = ['<option value="">— drop —</option>']
        .concat(drops.map((d) =>
          `<option value="${d}" ${cur === d ? 'selected' : ''}>${d}</option>`));
      return `<label><span>${obj}</span><select data-cm-obj="${obj}">${opts.join('')}</select></label>`;
    }).join('');
    box.querySelectorAll('[data-cm-obj]').forEach((sel) => {
      sel.onchange = () => {
        if (!state.commissionConfig) {
          state.commissionConfig = { drop_assignments: {} };
        }
        if (!state.commissionConfig.drop_assignments) {
          state.commissionConfig.drop_assignments = {};
        }
        state.commissionConfig.drop_assignments[sel.getAttribute('data-cm-obj')] = sel.value;
      };
    });
    if (!drops.length) {
      box.insertAdjacentHTML(
        'beforeend',
        '<p class="hint">No drop locations yet — mark waypoints as drop1, drop2…</p>');
    }
    renderCommissionStickerSummary();
  }

  function renderCommissionTelemetry(t) {
    if (!isCommission()) return;
    const c = (t && t.commission) || {};
    const phaseEl = $('cmPhase');
    if (phaseEl) {
      const detail = c.detail ? ` — ${c.detail}` : '';
      phaseEl.textContent = `${c.phase || 'idle'}${detail}`;
    }
    const pickEl = $('cmPick');
    if (pickEl) {
      const total = Number(c.pick_total) || 0;
      const idx = Number(c.pick_index) || 0;
      const cur = c.current_pick || '';
      pickEl.textContent = total
        ? `${cur || '—'} (${Math.min(idx + 1, total)}/${total})`
        : (cur || '—');
    }
    const remEl = $('cmRemaining');
    if (remEl) {
      const rem = c.remaining || {};
      const parts = Object.keys(rem)
        .sort()
        .map((k) => `${k}:${rem[k]}`);
      remEl.textContent = parts.length ? parts.join(' ') : '—';
    }
    const liftEl = $('cmLift');
    if (liftEl) {
      const lift = Number(c.lift_mm);
      liftEl.textContent = Number.isFinite(lift) ? `${lift.toFixed(0)} mm` : '—';
    }
    $('cmDetected') && ($('cmDetected').textContent = c.detected_object || '—');
    $('cmDrop') && ($('cmDrop').textContent = c.target_drop || '—');
  }

  function wpActionSummary(wp) {
    const act = wp && wp.action;
    if (!act || !act.type) return 'none (pass-through)';
    if (act.type === 'wait') return `wait ${Math.max(0, Number(act.seconds) || 0)}s`;
    if (act.type === 'align') return 'align (saved yaw)';
    if (act.type === 'align_move') {
      return `align + ${Math.max(0.03, Number(act.distance_m) || 0.15)} m`;
    }
    return String(act.type);
  }

  function renderWpTaskEditors() {
    const box = $('wpTaskList');
    if (!box) return;
    box.innerHTML = '';
    const commission = isCommission();

    if (!state.waypoints.length) {
      box.innerHTML = '<p class="hint">No waypoints yet — use <strong>Mark WP</strong> on the map. Each mark is added to the mission sequence.</p>';
      return;
    }

    // Navigation: one compact editor — pick WP, pick action, set params, tick to save.
    if (!commission) {
      if (state.wpTaskIndex >= state.waypoints.length) state.wpTaskIndex = 0;
      const idx = Math.max(0, Math.min(state.waypoints.length - 1, state.wpTaskIndex | 0));
      state.wpTaskIndex = idx;
      const wp = state.waypoints[idx];
      const navTypes = ['wait', 'align', 'align_move'];
      const known = (wp.action && navTypes.includes(wp.action.type))
        ? wp.action
        : { type: 'wait', seconds: 0 };
      const wrap = document.createElement('div');
      wrap.className = 'wp-task-compact';
      const navActionHelp = (type) => {
        if (type === 'wait') {
          return '<p class="hint" style="margin:0.25rem 0 0">Stop at this WP for N seconds, then continue. No Align node needed.</p>';
        }
        if (type === 'align_move') {
          return '<p class="hint" style="margin:0.25rem 0 0">'
            + '<strong>Align + distance:</strong> rotate base to the saved WP yaw, then drive so '
            + 'front lidar reads <em>Distance</em> (m). Tuned in <code>wp_align_*.yaml</code> '
            + '(yaw_tolerance_deg, dist_tolerance_m, duties). '
            + 'Not Detect+Align (that is camera/OMS in Commissioning).</p>';
        }
        return '<p class="hint" style="margin:0.25rem 0 0">'
          + '<strong>Align (saved yaw):</strong> rotate the robot base to this WP’s map yaw '
          + '(AMCL + /cmd_vel). No camera/YOLO. Tuned in <code>wp_align_*.yaml</code>. '
          + 'For camera object Detect + OMS head Align, use Commissioning → Detect+Align '
          + '(<code>wp_detect_align.yaml</code>).</p>';
      };
      let paramHtml = '';
      if (known.type === 'wait') {
        paramHtml = '<label>Seconds'
          + `<input type="number" min="0" step="0.5" id="wpTaskParam" value="${Math.max(0, Number(known.seconds) || 0)}">`
          + '</label>'
          + navActionHelp('wait');
      } else if (known.type === 'align_move') {
        paramHtml = '<label>Distance (m)'
          + `<input type="number" min="0.03" max="5" step="0.01" id="wpTaskParam" value="${Math.max(0.03, Number(known.distance_m) || 0.15)}">`
          + '</label>'
          + navActionHelp('align_move');
      } else {
        paramHtml = navActionHelp('align');
      }
      const options = state.waypoints.map((w, i) => {
        const mark = w.sticker ? ` · ${stickerDisplay(w.sticker)}` : '';
        return `<option value="${i}" ${i === idx ? 'selected' : ''}>WP ${i + 1}${mark} — ${wpActionSummary(w)}</option>`;
      }).join('');
      wrap.innerHTML =
        `<p class="wp-task-summary">Select a waypoint, choose what happens there, then tick to save.</p>`
        + '<label>Waypoint'
        + `<select id="wpTaskSelect">${options}</select>`
        + '</label>'
        + '<div class="wp-task-row">'
        + '<label>Action'
        + '<select id="wpTaskAction">'
        + `<option value="wait" ${known.type === 'wait' ? 'selected' : ''}>Wait</option>`
        + `<option value="align" ${known.type === 'align' ? 'selected' : ''}>Align (saved yaw)</option>`
        + `<option value="align_move" ${known.type === 'align_move' ? 'selected' : ''}>Align + distance</option>`
        + '</select></label>'
        + '<button type="button" class="btn btn-saffron btn-tick" id="btnWpTaskSave" title="Save task for this waypoint">✓</button>'
        + '</div>'
        + `<div id="wpTaskParams">${paramHtml}</div>`
        + `<p class="wp-task-summary" id="wpTaskCurrent">Saved: <strong>${wpActionSummary(wp)}</strong></p>`;
      box.appendChild(wrap);

      const sel = $('wpTaskSelect');
      const act = $('wpTaskAction');
      const refreshParams = () => {
        const type = act.value;
        const params = $('wpTaskParams');
        if (!params) return;
        if (type === 'wait') {
          const sec = (state.waypoints[state.wpTaskIndex].action
            && state.waypoints[state.wpTaskIndex].action.type === 'wait')
            ? Number(state.waypoints[state.wpTaskIndex].action.seconds) || 0
            : 3;
          params.innerHTML = '<label>Seconds'
            + `<input type="number" min="0" step="0.5" id="wpTaskParam" value="${Math.max(0, sec)}">`
            + '</label>'
            + navActionHelp('wait');
        } else if (type === 'align_move') {
          const dist = (state.waypoints[state.wpTaskIndex].action
            && state.waypoints[state.wpTaskIndex].action.type === 'align_move')
            ? Number(state.waypoints[state.wpTaskIndex].action.distance_m) || 0.15
            : 0.15;
          params.innerHTML = '<label>Distance (m)'
            + `<input type="number" min="0.03" max="5" step="0.01" id="wpTaskParam" value="${Math.max(0.03, dist)}">`
            + '</label>'
            + navActionHelp('align_move');
        } else {
          params.innerHTML = navActionHelp('align');
        }
      };
      if (sel) {
        sel.onchange = () => {
          state.wpTaskIndex = Math.max(0, Number(sel.value) || 0);
          renderWpTaskEditors();
        };
      }
      if (act) act.onchange = refreshParams;
      const saveBtn = $('btnWpTaskSave');
      if (saveBtn) {
        saveBtn.onclick = () => {
          const i = state.wpTaskIndex;
          const target = state.waypoints[i];
          if (!target) return;
          const type = (act && act.value) || 'wait';
          const param = $('wpTaskParam');
          if (type === 'wait') {
            target.action = {
              type: 'wait',
              seconds: Math.max(0, Number(param && param.value) || 0),
            };
          } else if (type === 'align') {
            target.action = { type: 'align' };
          } else if (type === 'align_move') {
            target.action = {
              type: 'align_move',
              distance_m: Math.max(0.03, Math.min(5, Number(param && param.value) || 0.15)),
            };
          }
          renderWpList();
          StudioMap.setWaypoints(state.waypoints);
          StudioApi.waypointsSave(cleanWaypoints(state.waypoints)).catch((e) => console.warn(e));
          renderWpTaskEditors();
        };
      }
      return;
    }

    state.waypoints.forEach((wp, i) => {
      const card = document.createElement('div');
      card.className = 'wp-task-card';
      const sticker = String(wp.sticker || '');
      const sk = stickerKind(sticker);
      const stickerLabel = stickerDisplay(sticker);
      const stickerRow = '<div class="wp-sticker-row">'
        + `<button type="button" class="chip ${sk === 'pick' ? 'sticker-p' : ''}" data-sticker="pick">Pick ${sk === 'pick' ? stickerLabel : ''}</button>`
        + `<button type="button" class="chip ${sk === 'drop' ? 'sticker-d' : ''}" data-sticker="drop">Drop ${sk === 'drop' ? stickerLabel : ''}</button>`
        + `<button type="button" class="chip ${sk === 'home' ? 'sticker-home' : ''}" data-sticker="home">Home</button>`
        + '<button type="button" class="chip" data-sticker="clear">Clear</button>'
        + '</div>';
      let pickParams = '';
      if (sk === 'pick') {
        const count = Math.max(1, Math.min(20, Number(wp.pick_count) || 1));
        const lift = Math.max(0, Math.min(300, Number(wp.pick_lift_mm) || 0));
        pickParams = '<div class="wp-pick-params">'
          + `<label>Visits<input type="number" min="1" max="20" step="1" data-pick-count value="${count}"></label>`
          + `<label>Lift mm<input type="number" min="0" max="300" step="1" data-pick-lift value="${lift}"></label>`
          + '</div>';
      }
      card.innerHTML =
        `<header>WP ${i + 1}${stickerLabel ? ` · ${stickerLabel}` : ''}</header>`
        + stickerRow
        + pickParams;

      const countInp = card.querySelector('[data-pick-count]');
      if (countInp) {
        countInp.onchange = () => {
          wp.pick_count = Math.max(1, Math.min(20, Math.round(Number(countInp.value) || 1)));
          renderCommissionStickerSummary();
          StudioMap.setWaypoints(state.waypoints);
          StudioApi.waypointsSave(cleanWaypoints(state.waypoints)).catch((e) => console.warn(e));
        };
      }
      const liftInp = card.querySelector('[data-pick-lift]');
      if (liftInp) {
        liftInp.onchange = () => {
          wp.pick_lift_mm = Math.max(0, Math.min(300, Number(liftInp.value) || 0));
          StudioMap.setWaypoints(state.waypoints);
          StudioApi.waypointsSave(cleanWaypoints(state.waypoints)).catch((e) => console.warn(e));
        };
      }
      card.querySelectorAll('[data-sticker]').forEach((btn) => {
        btn.onclick = () => {
          const kind = btn.getAttribute('data-sticker');
          if (kind === 'clear') wp.sticker = '';
          else if (kind === 'home') {
            wp.sticker = stickerKind(wp.sticker) === 'home' ? '' : 'home';
          } else if (kind === 'pick' && stickerKind(wp.sticker) === 'pick') {
            wp.sticker = '';
          } else if (kind === 'drop' && stickerKind(wp.sticker) === 'drop') {
            wp.sticker = '';
          } else if (kind === 'pick' || kind === 'drop') {
            wp.sticker = nextSticker(kind);
            if (kind === 'pick') {
              if (!Number.isFinite(+wp.pick_count)) wp.pick_count = 1;
              if (!Number.isFinite(+wp.pick_lift_mm)) wp.pick_lift_mm = 0;
            }
          }
          renderWpList();
          renderCommissionPanel();
          renderWpTaskEditors();
          StudioMap.setWaypoints(state.waypoints);
          StudioApi.waypointsSave(cleanWaypoints(state.waypoints)).catch((error) => console.warn(error));
        };
      });
      box.appendChild(card);
    });
  }

  function cleanWaypoints(arr) {
    return (arr || [])
      .map((w) => {
        if (!w || !Number.isFinite(+w.x) || !Number.isFinite(+w.y)) return null;
        const o = { x: +w.x, y: +w.y, yaw: +(w.yaw || 0) };
        if (w.name) o.name = w.name;
        if (w.sticker) o.sticker = String(w.sticker).trim().toLowerCase();
        if (Number.isFinite(+w.pick_count)) {
          o.pick_count = Math.max(1, Math.min(20, Math.round(+w.pick_count)));
        }
        if (Number.isFinite(+w.pick_lift_mm)) {
          o.pick_lift_mm = Math.max(0, Math.min(300, +w.pick_lift_mm));
        }
        const act = w.action;
        if (act && act.type === 'wait') {
          o.action = {
            type: 'wait',
            seconds: Math.max(0, Number(act.seconds) || 0),
          };
        } else if (act && act.type === 'detect_align' && isCommission()) {
          o.action = { type: 'detect_align' };
        } else if (act && act.type === 'align' && !isCommission()) {
          o.action = { type: 'align' };
        } else if (act && act.type === 'align_move' && !isCommission()) {
          o.action = {
            type: 'align_move',
            distance_m: Math.max(0.03, Math.min(5, Number(act.distance_m) || 0.15)),
          };
        }
        return o;
      })
      .filter(Boolean);
  }

  /** Keep a sequence named "default" = waypoints in order 1→2→3… */
  function ensureDefaultSequence() {
    const n = state.waypoints.length;
    const indices = Array.from({ length: n }, (_, i) => i);
    const prev = state.sequences[DEFAULT_SEQ];
    const same = Array.isArray(prev)
      && prev.length === indices.length
      && prev.every((v, i) => v === indices[i]);
    if (!same) {
      state.sequences[DEFAULT_SEQ] = indices;
      persistSequences();
    } else {
      refreshSeqSelect();
    }
    const sel = $('seqSelect');
    if (sel && !sel.value && n > 0) {
      sel.value = DEFAULT_SEQ;
    }
    // If builder empty (or only tracking default), keep Seq preview = 1→2→…n
    const onDefault = !sel || !sel.value || sel.value === DEFAULT_SEQ;
    if (onDefault && n > 0) {
      const builderMatchesDefault = state.sequenceBuilder.length === indices.length
        && state.sequenceBuilder.every((v, i) => v === indices[i]);
      if (!state.sequenceBuilder.length || builderMatchesDefault
        || state.sequenceBuilder.every((v) => v >= 0 && v < n)) {
        // Prefer showing full default when builder empty; otherwise keep custom order
        // but drop out-of-range indices after WP clear.
        if (!state.sequenceBuilder.length) {
          state.sequenceBuilder = indices.slice();
        } else {
          state.sequenceBuilder = state.sequenceBuilder.filter((v) => v >= 0 && v < n);
          if (!state.sequenceBuilder.length) state.sequenceBuilder = indices.slice();
        }
      }
    }
  }

  function resolveMissionSelection() {
    ensureDefaultSequence();
    if (state.sequenceBuilder.length) {
      return {
        name: '(builder)',
        indices: state.sequenceBuilder.slice(),
      };
    }
    const sel = $('seqSelect');
    const name = (sel && sel.value) || DEFAULT_SEQ;
    if (name && state.sequences[name] && state.sequences[name].length) {
      return { name, indices: state.sequences[name].slice() };
    }
    if (state.waypoints.length) {
      return {
        name: DEFAULT_SEQ,
        indices: Array.from({ length: state.waypoints.length }, (_, i) => i),
      };
    }
    return { name: '', indices: [] };
  }

  function buildMissionWaypoints() {
    const sel = resolveMissionSelection();
    if (!sel.indices.length) return [];
    return cleanWaypoints(sel.indices.map((i) => state.waypoints[i]));
  }

  function syncArmedMission() {
    const sel = resolveMissionSelection();
    const mission = cleanWaypoints(sel.indices.map((i) => state.waypoints[i]));
    if (!mission.length) return Promise.resolve({ ok: false, count: 0 });
    return StudioApi.armMission(mission, sel.name).catch((e) => {
      console.warn('arm mission failed', e);
      return { ok: false, error: e.message };
    });
  }

  async function loadSavedWaypoints() {
    try {
      const w = await StudioApi.waypointsGet();
      state.waypoints = w.waypoints || [];
      StudioMap.setWaypoints(state.waypoints);
      renderWpList();
      renderDeployWpStrip();
      try {
        const s = await StudioApi.sequencesGet();
        const fromPi = (s && s.sequences) || {};
        if (fromPi && typeof fromPi === 'object' && Object.keys(fromPi).length) {
          state.sequences = { ...state.sequences, ...fromPi };
          persistSequences();
        }
      } catch (e) {
        console.warn('sequences load', e);
      }
      ensureDefaultSequence();
      syncArmedMission();
      return state.waypoints;
    } catch (e) {
      console.warn(e);
      return state.waypoints || [];
    }
  }

  async function openDeployPicker({ saveCurrent = false } = {}) {
    if (state.deployInFlight) {
      return { ok: false, busy: true };
    }
    state.deployInFlight = true;
    try {
      if (saveCurrent) {
        await StudioApi.waypointsSave(state.waypoints);
      } else {
        await loadSavedWaypoints();
      }
      if (state.pendingEdits.length) {
        await StudioApi.mapEdit(state.pendingEdits);
        state.pendingEdits = [];
        StudioMap.clearUndo();
      }
      try { await StudioApi.mapRestrictions(state.restrictions, false); } catch (_) {}
      try { await StudioApi.localizeStop(); } catch (_) {}
      try { await StudioApi.mappingStop(); } catch (_) {}
      const hint = $('deployPickHint');
      if (hint) {
        hint.textContent = `${state.waypoints.length} waypoint(s) on ${state.mapName || 'map'} · ${state.mode}`;
      }
      setPhase('deploy-pick');
      return { ok: true };
    } finally {
      state.deployInFlight = false;
    }
  }

  /** Open the Deploy view only. Nav is started from the Deploy buttons. */
  async function goDeploy({ saveCurrent = false, module = '' } = {}) {
    if (module === 'nav' || module === 'commission') {
      state.deployModule = module;
    }
    if (state.deployInFlight) {
      return { ok: false, busy: true };
    }
    state.deployInFlight = true;
    try {
      if (saveCurrent) {
        await StudioApi.waypointsSave(state.waypoints);
      } else {
        await loadSavedWaypoints();
      }
      if (state.pendingEdits.length) {
        await StudioApi.mapEdit(state.pendingEdits);
        state.pendingEdits = [];
        StudioMap.clearUndo();
      }
      try { await StudioApi.mapRestrictions(state.restrictions, false); } catch (_) {}
      try { await StudioApi.localizeStop(); } catch (_) {}
      try { await StudioApi.mappingStop(); } catch (_) {}
      await StudioApi.phase('deploy');
      state.healthSessionId = '';
      state.healthStartedAt = 0;
      state.healthLastWrite = 0;
      state.healthLastSig = '';
      setPhase('deploy');
      StudioMap.setTool('pose');
      renderDeployWpStrip();
      renderWpList();
      renderWpTaskEditors();
      ensureDefaultSequence();
      refreshSeqSelect();
      applyDeployModuleUi();
      if (isCommission()) await loadCommissionConfig();
      await syncArmedMission();
      return { ok: true };
    } finally {
      state.deployInFlight = false;
    }
  }

  $('btnSeqUndo').onclick = () => {
    state.sequenceBuilder.pop();
    updateSeqPreview();
  };
  $('btnSeqClear').onclick = () => {
    state.sequenceBuilder = [];
    updateSeqPreview();
  };
  $('btnSeqSave').onclick = () => {
    const name = ($('seqName').value || '').trim();
    if (!name) return alert('Enter a sequence name');
    if (name === DEFAULT_SEQ) {
      return alert('"default" is reserved for the auto 1→2→3… sequence');
    }
    if (!state.sequenceBuilder.length) return alert('Build a sequence first');
    state.sequences[name] = state.sequenceBuilder.slice();
    persistSequences();
    $('seqSelect').value = name;
    syncArmedMission();
    alert(`Saved sequence "${name}"`);
  };
  $('seqSelect').onchange = () => {
    const name = $('seqSelect').value;
    if (!name) {
      ensureDefaultSequence();
      $('seqSelect').value = DEFAULT_SEQ;
    }
    const picked = $('seqSelect').value;
    if (picked && state.sequences[picked]) {
      state.sequenceBuilder = state.sequences[picked].slice();
      updateSeqPreview();
    } else {
      syncArmedMission();
    }
  };
  $('btnSeqDelete').onclick = () => {
    const name = $('seqSelect').value;
    if (!name) return;
    if (name === DEFAULT_SEQ) {
      return alert('Cannot delete the default sequence');
    }
    delete state.sequences[name];
    persistSequences();
    state.sequenceBuilder = [];
    ensureDefaultSequence();
    updateSeqPreview();
  };

  async function startDeploy(startCamera = false) {
    if (state.deployInFlight) {
      alert('Deploy already in progress — please wait.');
      return;
    }
    const stacks = (state.telemetry && state.telemetry.stacks) || {};
    if (stacks.nav || stacks.slam || stacks.localize || stacks.oms) {
      const which = ['nav', 'localize', 'slam', 'oms'].filter((k) => stacks[k]).join(', ');
      alert(
        `Stack already running (${which}).\n`
        + 'Stop mapping, Stop localize, or Stop nav first.\n'
        + 'If OMS Software is in standalone Run, press Stop OMS there before Nav2.');
      return;
    }
    const camera = isCommission() || !!startCamera;
    const daChk = $('chkAutoDetectAlign');
    if (daChk) state.autoDetectAlign = !!daChk.checked;
    const startDetectAlign = camera ? !!state.autoDetectAlign : false;
    const alignChk = $('chkStartAlignNav');
    const withAlign = isCommission()
      ? false
      : (alignChk ? !!alignChk.checked : true);
    const res = await StudioApi.deployStart({
      burn_restrictions: true,
      stack: 'nav2',
      module: isCommission() ? 'commission' : 'nav',
      start_camera: camera,
      start_detect_align: startDetectAlign,
      with_align: withAlign,
    });
    if (camera) {
      alert(
        `Commissioning: Nav2 + camera + OMS`
        + `${startDetectAlign ? ' + Detect+Align' : ' (Detect+Align skipped)'}.\n`
        + 'Open OMS Software — it shows Ready when OMS nodes are up. Do not press Run OMS.\n'
        + `${res.map || ''}`);
    } else {
      alert(
        `Navigation: Nav2 started`
        + `${withAlign ? ' with Align node' : ' (Align node off)'}.\n`
        + `${res.map || ''}`);
    }
  }

  bindLoading('btnDeployNav2', () => startDeploy(false), 'Starting Nav2…');
  bindLoading('btnDeployNav2Cam', () => startDeploy(true), 'Starting Nav2+camera…');
  bindLoading('btnPickNav', () => goDeploy({ saveCurrent: false, module: 'nav' }), 'Opening…');
  bindLoading(
    'btnPickCommission',
    () => goDeploy({ saveCurrent: false, module: 'commission' }),
    'Opening…');
  const btnPickBack = $('btnDeployPickBack');
  if (btnPickBack) {
    btnPickBack.onclick = () => {
      setPhase('waypoints');
      renderWpList();
    };
  }
  const btnChangeModule = $('btnChangeModule');
  if (btnChangeModule) {
    btnChangeModule.onclick = () => setPhase('deploy-pick');
  }

  bindLoading('btnClearCostmaps', async () => {
    const hint = $('costmapHint');
    if (hint) hint.textContent = 'Clearing costmaps…';
    try {
      const res = await StudioApi.clearCostmaps();
      if (hint) {
        hint.textContent = res.ok ? 'Costmaps cleared.' : 'Clear failed — is Nav2 running?';
      }
    } catch (e) {
      if (hint) hint.textContent = e.message || 'Clear failed';
      throw e;
    }
  }, 'Clearing…');

  bindLoading('btnDeployStop', async () => {
    await StudioApi.deployStop();
  }, 'Stopping nav…');

  // ── Simulation: pick A + B, plan on the map, never move the robot ──────
  const SIM_HINT_IDLE =
    'Start GC+Planner or Enter sim — pick two map points; robot never moves.';

  function plannerStackLabel() {
    const t = state.telemetry || {};
    const sess = t.session || {};
    const stack = String(sess.nav_stack || '');
    if (stack === 'gc_planner') return 'GC+Planner';
    if (stack === 'nav2' || (t.stacks && t.stacks.nav)) return 'Nav2';
    return 'Custom';
  }

  function navStackLabel() {
    return plannerStackLabel();
  }

  function plannerStackReady(stacks) {
    const s = stacks || (state.telemetry && state.telemetry.stacks) || {};
    return !!(s.nav || s.gc_planner);
  }

  function updateGcPlannerUi(stacks) {
    const s = stacks || (state.telemetry && state.telemetry.stacks) || {};
    const on = !!s.gc_planner;
    const btnStart = $('btnGcPlannerStart');
    const btnStop = $('btnGcPlannerStop');
    if (btnStart) btnStart.hidden = on;
    if (btnStop) btnStop.hidden = !on;
  }

  function setGcPlannerHint(text) {
    const el = $('gcPlannerHint');
    if (el && text) el.textContent = text;
  }

  function navStackUsesNav2Planner() {
    const label = navStackLabel();
    return label === 'Nav2' || label === 'GC+Planner';
  }

  function updateSimStyleUi() {
    const sel = $('simStyle');
    if (!sel) return;
    const nav2 = navStackUsesNav2Planner();
    const smoothOpt = sel.querySelector('option[value="smoothed"]');
    if (smoothOpt) smoothOpt.hidden = nav2;
    if (nav2 && state.simStyle === 'smoothed') {
      state.simStyle = 'planned';
      sel.value = 'planned';
    }
  }

  /** Pick planned or smoothed sim path from telemetry per the dropdown. */
  function simPathForStyle(sim) {
    const nav2 = navStackUsesNav2Planner()
      || String(sim.stack || '').startsWith('nav2')
      || String(sim.stack || '') === 'gc_planner';
    if (!nav2 && state.simStyle === 'smoothed') {
      const sm = sim.path_smooth || [];
      if (sm.length) return sm;
    }
    return sim.path || [];
  }

  function updateSimHint(result) {
    const hint = $('simHint');
    if (!hint || !state.simActive) return;
    const stack = navStackLabel();
    const n = state.simPoints.length;
    if (n < 2) {
      hint.textContent = n === 0
        ? `Simulation ON (${stack}) — click the map to set point A (drag to aim B yaw).`
        : `Point A set (${stack}) — click the map to set point B.`;
      return;
    }
    if (result && result.ok === true) {
      const nav2 = navStackUsesNav2Planner()
        || String(result.stack || stack).startsWith('nav2')
        || String(result.stack || '') === 'gc_planner';
      const planner = result.planner_id || state.simPlannerId || 'GridBased';
      const shown = (!nav2 && state.simStyle === 'smoothed')
        ? `smoothed (${result.points_smooth || result.points} pts)`
        : `planned (${result.points} pts)`;
      hint.textContent = `${stack} [${planner}]: path found — showing ${shown}. `
        + 'Robot does not move in simulation.';
      return;
    }
    if (result && result.ok === false) {
      hint.textContent = `${stack}: ${result.detail || 'plan failed'}`;
      return;
    }
    if (result && result.ok == null) {
      hint.textContent = `${stack}: planning…`;
    } else {
      hint.textContent = `${stack}: A and B set — click Generate path.`;
    }
  }

  async function enterSim() {
    if (!plannerStackReady()) {
      try {
        const res = await StudioApi.gcPlannerStart({});
        setGcPlannerHint(res.message || 'GC+Planner started.');
      } catch (e) {
        alert(`Could not start GC+Planner: ${e.message}`);
        return;
      }
    }
    enterSimAfterPlanner();
  }

  function enterSimAfterPlanner() {
    state.simActive = true;
    state.simPoints = [];
    updateSimStyleUi();
    const controls = $('simControls');
    if (controls) controls.hidden = false;
    const enter = $('btnSimEnter');
    if (enter) enter.hidden = true;
    document.querySelectorAll('.tool[data-tool]').forEach(
      (b) => b.classList.remove('active'));
    StudioMap.setSim([], []);
    StudioMap.setTool('simpt');
    updateSimHint();
  }

  function exitSim() {
    state.simActive = false;
    state.simPoints = [];
    const controls = $('simControls');
    if (controls) controls.hidden = true;
    const enter = $('btnSimEnter');
    if (enter) enter.hidden = false;
    StudioMap.setSim([], []);
    StudioMap.setTool('pose');
    const hint = $('simHint');
    if (hint) hint.textContent = SIM_HINT_IDLE;
    StudioApi.simClear().catch(() => {});
  }

  const simStyleSel = $('simStyle');
  if (simStyleSel) {
    simStyleSel.onchange = () => {
      state.simStyle = simStyleSel.value === 'smoothed' ? 'smoothed' : 'planned';
      if (!state.simActive) return;
      const sim = (state.telemetry && state.telemetry.sim) || {};
      StudioMap.setSim(state.simPoints, simPathForStyle(sim), state.simStyle);
      updateSimHint(sim.result);
    };
  }

  function syncSimPlannerLabel() {
    const el = $('simPlannerActive');
    if (el) el.textContent = `Active planner: ${state.simPlannerId}`;
    const sel = $('simPlannerSelect');
    if (sel && sel.value !== state.simPlannerId) sel.value = state.simPlannerId;
  }

  const btnSimChoosePlanner = $('btnSimChoosePlanner');
  if (btnSimChoosePlanner) {
    btnSimChoosePlanner.onclick = () => {
      const sel = $('simPlannerSelect');
      const id = 'GridBased';
      if (sel) sel.value = id;
      state.simPlannerId = id;
      syncSimPlannerLabel();
      if (state.simActive) {
        const hint = $('simHint');
        if (hint) {
          hint.textContent = `Planner set to ${id} — click Generate to plan.`;
        }
      }
    };
  }
  syncSimPlannerLabel();

  bindLoading('btnGcPlannerStart', async () => {
    const res = await StudioApi.gcPlannerStart({});
    setGcPlannerHint(res.message || 'GC+Planner started.');
  }, 'Starting GC+Planner…');
  bindLoading('btnGcPlannerStop', async () => {
    const res = await StudioApi.gcPlannerStop();
    setGcPlannerHint(res.message || 'GC+Planner stopped.');
    if (state.simActive) exitSim();
  }, 'Stopping…');

  const btnSimEnter = $('btnSimEnter');
  if (btnSimEnter) {
    btnSimEnter.onclick = () => runWithLoading(btnSimEnter, () => enterSim(), 'Starting…');
  }
  const btnSimExit = $('btnSimExit');
  if (btnSimExit) btnSimExit.onclick = () => exitSim();
  const btnSimClearPts = $('btnSimClearPts');
  if (btnSimClearPts) {
    btnSimClearPts.onclick = () => {
      state.simPoints = [];
      StudioMap.setSim([], []);
      StudioApi.simClear().catch(() => {});
      updateSimHint();
    };
  }
  bindLoading('btnSimGenerate', async () => {
    if (state.simPoints.length < 2) {
      alert('Pick two points on the map first (A = start, B = goal).');
      return;
    }
    const [a, b] = state.simPoints;
    const gyaw = Math.abs(b.yaw || 0) > 1e-6
      ? b.yaw
      : Math.atan2(b.y - a.y, b.x - a.x);
    await StudioApi.simPlan({
      sx: a.x, sy: a.y, syaw: a.yaw || 0,
      gx: b.x, gy: b.y, gyaw,
      planner_id: state.simPlannerId || 'GridBased',
    });
    updateSimHint({ ok: null });
  }, 'Planning…');

  bindLoading('btnMissionStart', async () => {
    if (!state.waypoints.length) await loadSavedWaypoints();
    const m = (state.telemetry && state.telemetry.mission) || {};
    if (['running', 'wait_action', 'wait_goal', 'recovering'].includes(
      String(m.state || '').toLowerCase())) {
      alert('Mission already running — press Stop first.');
      return;
    }
    if (isCommission()) {
      const picks = (state.waypoints || []).filter((w) => stickerKind(w.sticker) === 'pick');
      if (!picks.length) {
        alert('Mark at least one pick location (pick1, pick2, …) on the map.');
        return;
      }
      const drops = listDropStickers();
      if (!drops.length) {
        alert('Mark at least one drop location (drop1, drop2, …) on the map.');
        return;
      }
      if (!state.commissionConfig) await loadCommissionConfig();
      const assigns = (state.commissionConfig && state.commissionConfig.drop_assignments) || {};
      const missing = COMMISSION_OBJECTS.filter((obj) => !assigns[obj]);
      if (missing.length) {
        const go = confirm(
          `Some objects have no drop assigned (${missing.join(', ')}).\n`
          + 'Start anyway? Unassigned detections will skip that pick.');
        if (!go) return;
      }
      await StudioApi.waypointsSave(cleanWaypoints(state.waypoints));
      await StudioApi.commissionSave(state.commissionConfig || {});
      const res = await StudioApi.start({ source: 'dashboard' });
      if (res && res.started === false && res.reason === 'already_running') {
        alert('Mission already running — press Stop first.');
        return;
      }
      if (res && res.ok === false) {
        alert(res.error || res.reason || 'Commission mission could not start.');
        return;
      }
      const rem = (res && res.remaining) || {};
      const remText = Object.keys(rem).map((k) => `${k}=${rem[k]}`).join(', ') || '—';
      alert(
        `Commissioning mission started\n`
        + `${picks.length} pick(s)\nCounts: ${remText}`);
      return;
    }
    ensureDefaultSequence();
    const sel = resolveMissionSelection();
    if (!sel.indices.length || !state.waypoints.length) {
      alert(
        'No sequence / waypoints for mission.\n'
        + 'Add waypoints first. A "default" sequence (1→2→3…) is created automatically.');
      return;
    }
    const mission = cleanWaypoints(sel.indices.map((i) => state.waypoints[i]));
    if (!mission.length) {
      alert('Selected sequence has no valid waypoints.');
      return;
    }
    await StudioApi.setGoals(mission, sel.name);
    await new Promise((r) => setTimeout(r, 150));
    const res = await StudioApi.start({ source: 'dashboard', sequence: sel.name });
    if (res && res.started === false && res.reason === 'already_running') {
      alert('Mission already running — press Stop first.');
      return;
    }
    if (res && res.ok === false && res.reason === 'no_sequence') {
      alert(res.error || 'No sequence armed.');
      return;
    }
    alert(
      `Mission started from dashboard\n`
      + `Sequence: ${sel.name}\n`
      + `${mission.length} waypoint(s)`);
  }, 'Starting mission…');

  const btnCommissionSave = $('btnCommissionSave');
  if (btnCommissionSave) {
    bindLoading('btnCommissionSave', async () => {
      if (!state.commissionConfig) await loadCommissionConfig();
      await StudioApi.waypointsSave(cleanWaypoints(state.waypoints));
      await StudioApi.commissionSave(state.commissionConfig || {});
      alert('Pick/drop assignments, counts, and lifts saved.');
    }, 'Saving…');
  }

  const btnCommissionHomeOms = $('btnCommissionHomeOms');
  if (btnCommissionHomeOms) {
    bindLoading('btnCommissionHomeOms', async () => {
      const res = await StudioApi.omsHome();
      if (res && res.ok === false) {
        alert(res.error || res.message || 'OMS home failed');
        return;
      }
      alert('OMS home command sent.');
    }, 'Homing OMS…');
  }

  bindLoading('btnMissionStop', async () => {
    await StudioApi.stop({ source: 'dashboard' });
  }, 'Stopping…');

  $('btnEstop').onclick = () => StudioApi.estop(true).catch((e) => alert(e.message));

  bindLoading('btnRelocThen', async () => {
    if (!state.waypoints.length) await loadSavedWaypoints();
    ensureDefaultSequence();
    const sel = resolveMissionSelection();
    const mission = cleanWaypoints(sel.indices.map((i) => state.waypoints[i]));
    if (!mission.length) {
      alert('No waypoints/sequence to start after reloc');
      return;
    }
    await StudioApi.armMission(mission, sel.name);
    await StudioApi.relocThenTask('spin', mission);
    alert(
      'Reloc+start: spinning until localization is stable,\n'
      + `then sequence "${sel.name}" starts automatically.`);
  }, 'Starting reloc…');

  $('btnRelocSpin').onclick = () => {
    StudioApi.relocStart('spin').catch((e) => alert(e.message));
  };
  $('btnRelocFront').onclick = () => {
    StudioApi.relocStart('front').catch((e) => alert(e.message));
  };
  $('btnRelocBack').onclick = () => {
    StudioApi.relocStart('back').catch((e) => alert(e.message));
  };
  $('btnRelocStop').onclick = () => {
    StudioApi.relocStop().catch((e) => alert(e.message));
  };
  const clearTrailBtn = $('btnClearTrailDeploy');
  if (clearTrailBtn) {
    bindLoading('btnClearTrailDeploy', async () => {
      await StudioApi.trailClear();
      StudioMap.clearTrailLocal();
      StudioMap.setPaths([], [], []);
    }, 'Clearing…');
  }

  const CMD = {
    forward: { vx: 0.18, vy: 0, wz: 0 },
    back: { vx: -0.18, vy: 0, wz: 0 },
    left: { vx: 0, vy: 0, wz: 0.4 },
    right: { vx: 0, vy: 0, wz: -0.4 },
    strafe_l: { vx: 0, vy: 0.15, wz: 0 },
    strafe_r: { vx: 0, vy: -0.15, wz: 0 },
    stop: { vx: 0, vy: 0, wz: 0 },
  };
  let holdTimer = null;
  let holdCmd = null;

  function startHold(name) {
    holdCmd = CMD[name] || CMD.stop;
    const send = () => {
      if (!holdCmd) return;
      StudioApi.teleop(holdCmd.vx, holdCmd.vy, holdCmd.wz).catch(() => {});
    };
    send();
    clearInterval(holdTimer);
    holdTimer = setInterval(send, 100);
  }
  function stopHold() {
    clearInterval(holdTimer);
    holdTimer = null;
    holdCmd = null;
    StudioApi.teleop(0, 0, 0).catch(() => {});
  }

  document.querySelectorAll('.hold').forEach((btn) => {
    const name = btn.dataset.cmd;
    btn.addEventListener('mousedown', (e) => { e.preventDefault(); btn.classList.add('pressed'); startHold(name); });
    btn.addEventListener('mouseup', () => { btn.classList.remove('pressed'); stopHold(); });
    btn.addEventListener('mouseleave', () => { btn.classList.remove('pressed'); stopHold(); });
    btn.addEventListener('touchstart', (e) => { e.preventDefault(); btn.classList.add('pressed'); startHold(name); }, { passive: false });
    btn.addEventListener('touchend', () => { btn.classList.remove('pressed'); stopHold(); });
  });

  function ensureCameraKillBtn() {
    if ($('btnCamKill')) return;
    const off = $('btnCamOff');
    const host = off ? off.parentElement : $('blockDeploy');
    if (!host) return;
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.id = 'btnCamKill';
    btn.className = off ? off.className : 'btn btn-ghost';
    btn.textContent = 'Kill camera node';
    btn.title = 'Fully stop Orbbec process (not just stream OFF)';
    if (off && off.nextSibling) host.insertBefore(btn, off.nextSibling);
    else host.appendChild(btn);
  }

  const btnCamOn = $('btnCamOn');
  const btnCamOff = $('btnCamOff');
  ensureCameraKillBtn();
  if (btnCamOn) {
    btnCamOn.onclick = (ev) => runWithLoading(
      ev.currentTarget, () => StudioApi.cameraOn(), 'Camera ON…');
  }
  if (btnCamOff) {
    btnCamOff.onclick = (ev) => runWithLoading(
      ev.currentTarget, () => StudioApi.cameraOff(), 'Camera OFF…');
  }
  const btnCamKill = $('btnCamKill');
  if (btnCamKill) {
    bindLoading('btnCamKill', async () => {
      await StudioApi.cameraKill();
      alert('Orbbec camera node stopped.');
    }, 'Killing camera…');
  }

  setPhase('home');
  showView('home');
  refreshSeqSelect();
  ensureBatteryUi();
})();
