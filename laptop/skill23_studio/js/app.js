/** Skill23 Studio app — wizard + stage flows */
(() => {
  const SEQ_LS = 'skill23_studio_sequences';
  const DEFAULT_SEQ = 'default';
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
    waypointInputMode: 'live',
    simActive: false,
    simPoints: [],
    simStyle: 'planned',   // planned | smoothed — which sim path to draw
    lastHwAlert: '',
    localizeRunning: false,
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
        refreshSeqSelect();
        bindToolButtons();
      }
    } else if (deployPick) {
      showView('deployPick');
    }
    const topBat = $('batteryWidget');
    if (topBat) topBat.style.display = deploy ? 'none' : '';
  }

  function setConnected(ok) {
    state.connected = ok;
    $('connBadge').classList.toggle('on', ok);
    $('connText').textContent = ok ? 'Online' : 'Offline';
    if (!ok) updateBattery({ valid: false, source: 'waiting_for_control_server' });
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
      detail: `pts=${t.scan_count || 0} age=${t.scan_age ?? '—'}s`,
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
      if (['error', 'fail', 'failed', 'fault', 'stopped'].includes(st)) cls = 'bad';
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
    StudioMap.setScan(t.scan || []);
    StudioMap.setTrail(t.trail || []);
    StudioMap.setPaths(t.path || [], t.planner_path || [], t.smoothed_path || []);
    if (state.simActive) {
      const sim = t.sim || {};
      StudioMap.setSim(state.simPoints, simPathForStyle(sim), state.simStyle);
      updateSimHint(sim.result);
    }

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
    const camLog = $('logCamera');
    if (camLog) {
      camLog.textContent = (cam.logs || []).join('\n') || '(empty)';
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
    const resolution = parseFloat($('mapResolution').value);
    const max_laser_range = parseFloat($('mapMaxRange').value);
    const min_laser_range = parseFloat($('mapMinRange').value);
    if (!(resolution > 0)) return alert('Choose a valid resolution');
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
      });
      StudioMap.setWaypoints(state.waypoints);
      renderWpList();
      if (state.phase === 'deploy') renderDeployWpStrip();
      StudioApi.waypointsSave(state.waypoints).catch(() => {});
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
    // Keep pendingEdits in sync loosely — full re-save uses current map from last paint ops
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
    $('wpList').innerHTML = state.waypoints.map((w, i) => {
      const tag = w.sticker ? ` [${String(w.sticker).toUpperCase()}]` : '';
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
    renderWpTaskEditors();
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

  function isCommission() {
    return state.deployModule === 'commission';
  }

  function nextSticker(kind) {
    const prefix = kind === 'drop' ? 'd' : 'p';
    const used = new Set(
      (state.waypoints || [])
        .map((w) => String(w.sticker || '').trim().toLowerCase())
        .filter(Boolean),
    );
    let n = 1;
    while (used.has(prefix + n)) n += 1;
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
        ? 'Commissioning: Start Nav2+camera, then Start mission. Per WP use Wait or Detect+Align. Stickers P1/D1 mark pick/drop.'
        : 'Navigation: Start Nav2, then Start mission. Per WP: Wait, Align (saved yaw), or Align+distance. Physical START = Titan1 m3 limit_rev.';
    }
    const dap = $('detectAlignPanel');
    if (dap) dap.hidden = !(state.phase === 'deploy' && isCommission());
  }

  function renderWpTaskEditors() {
    const box = $('wpTaskList');
    if (!box) return;
    box.innerHTML = '';
    const commission = isCommission();

    state.waypoints.forEach((wp, i) => {
      const type = (wp.action && wp.action.type) || 'wait';
      const known = commission
        ? (['wait', 'detect_align'].includes(type) ? wp.action : { type: 'wait', seconds: 0 })
        : (['wait', 'align', 'align_move'].includes(type)
          ? wp.action
          : { type: 'wait', seconds: 0 });
      const card = document.createElement('div');
      card.className = 'wp-task-card';
      let extra = '';
      if (known.type === 'wait') {
        extra = '<label>Seconds</label>'
          + `<input type="number" min="0" step="0.5" data-seconds value="${Math.max(0, Number(known.seconds) || 0)}">`;
      } else if (known.type === 'align_move') {
        extra = '<label>Distance (m)</label>'
          + `<input type="number" min="0.03" max="5" step="0.01" data-dist value="${Math.max(0.03, Number(known.distance_m) || 0.15)}">`;
      }
      const sticker = String(wp.sticker || '').toUpperCase();
      const stickerRow = commission
        ? '<div class="wp-sticker-row">'
          + `<button type="button" class="chip ${sticker.startsWith('P') ? 'sticker-p' : ''}" data-sticker="pick">Pick ${sticker.startsWith('P') ? sticker : ''}</button>`
          + `<button type="button" class="chip ${sticker.startsWith('D') ? 'sticker-d' : ''}" data-sticker="drop">Drop ${sticker.startsWith('D') ? sticker : ''}</button>`
          + '<button type="button" class="chip" data-sticker="clear">Clear</button>'
          + '</div>'
        : '';
      const taskOpts = commission
        ? (`<option value="wait" ${known.type === 'wait' ? 'selected' : ''}>Wait</option>`
          + `<option value="detect_align" ${known.type === 'detect_align' ? 'selected' : ''}>Detect+Align</option>`)
        : (`<option value="wait" ${known.type === 'wait' ? 'selected' : ''}>Wait</option>`
          + `<option value="align" ${known.type === 'align' ? 'selected' : ''}>Align (saved yaw)</option>`
          + `<option value="align_move" ${known.type === 'align_move' ? 'selected' : ''}>Align + distance</option>`);
      card.innerHTML =
        `<header>Waypoint ${i + 1}${sticker ? ` · ${sticker}` : ''}</header>`
        + '<label>Task</label>'
        + `<select data-task>${taskOpts}</select>`
        + extra
        + stickerRow;

      card.querySelector('[data-task]').onchange = (event) => {
        const val = event.target.value;
        if (val === 'wait') {
          wp.action = { type: 'wait', seconds: 3 };
        } else if (val === 'detect_align') {
          wp.action = { type: 'detect_align' };
        } else if (val === 'align') {
          wp.action = { type: 'align' };
        } else if (val === 'align_move') {
          wp.action = { type: 'align_move', distance_m: 0.15 };
        } else {
          wp.action = { type: 'wait', seconds: 0 };
        }
        renderWpList();
        StudioMap.setWaypoints(state.waypoints);
        StudioApi.waypointsSave(state.waypoints).catch((error) => console.warn(error));
      };
      const seconds = card.querySelector('[data-seconds]');
      if (seconds) {
        seconds.onchange = () => {
          wp.action = {
            type: 'wait',
            seconds: Math.max(0, Number(seconds.value) || 0),
          };
          renderWpList();
          StudioApi.waypointsSave(state.waypoints).catch((error) => console.warn(error));
        };
      }
      const dist = card.querySelector('[data-dist]');
      if (dist) {
        dist.onchange = () => {
          wp.action = {
            type: 'align_move',
            distance_m: Math.max(0.03, Math.min(5, Number(dist.value) || 0.15)),
          };
          renderWpList();
          StudioApi.waypointsSave(state.waypoints).catch((error) => console.warn(error));
        };
      }
      card.querySelectorAll('[data-sticker]').forEach((btn) => {
        btn.onclick = () => {
          const kind = btn.getAttribute('data-sticker');
          if (kind === 'clear') wp.sticker = '';
          else if (kind === 'pick' && String(wp.sticker || '').toLowerCase().startsWith('p')) {
            wp.sticker = '';
          } else if (kind === 'drop' && String(wp.sticker || '').toLowerCase().startsWith('d')) {
            wp.sticker = '';
          } else {
            wp.sticker = nextSticker(kind);
          }
          renderWpList();
          StudioMap.setWaypoints(state.waypoints);
          StudioApi.waypointsSave(state.waypoints).catch((error) => console.warn(error));
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
      setPhase('deploy');
      StudioMap.setTool('pose');
      renderDeployWpStrip();
      ensureDefaultSequence();
      refreshSeqSelect();
      applyDeployModuleUi();
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
    if (stacks.nav || stacks.slam || stacks.localize) {
      const which = ['nav', 'localize', 'slam'].filter((k) => stacks[k]).join(', ');
      alert(
        `Stack already running (${which}).\n`
        + 'Click Stop nav / stop mapping / stop localize first, then Start Nav2.');
      return;
    }
    const camera = isCommission() || !!startCamera;
    const res = await StudioApi.deployStart({
      burn_restrictions: true,
      stack: 'nav2',
      module: isCommission() ? 'commission' : 'nav',
      start_camera: camera,
    });
    alert(camera
      ? `Commissioning: Nav2+camera (streams off) + Detect+Align.\n${res.map || ''}`
      : `Navigation: Nav2 + Align YAML (saved yaw).\n${res.map || ''}`);
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
    'Pick two points on the map and generate a path — the robot never moves.';

  function navStackLabel() {
    return 'Nav2';
  }

  function updateSimStyleUi() {
    const sel = $('simStyle');
    if (!sel) return;
    const nav2 = navStackLabel().startsWith('Nav2');
    const smoothOpt = sel.querySelector('option[value="smoothed"]');
    if (smoothOpt) smoothOpt.hidden = nav2;
    if (nav2 && state.simStyle === 'smoothed') {
      state.simStyle = 'planned';
      sel.value = 'planned';
    }
  }

  /** Pick planned or smoothed sim path from telemetry per the dropdown. */
  function simPathForStyle(sim) {
    const nav2 = (sim.stack || navStackLabel()).startsWith('Nav2');
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
      const nav2 = (result.stack || stack).startsWith('Nav2');
      const shown = (!nav2 && state.simStyle === 'smoothed')
        ? `smoothed (${result.points_smooth || result.points} pts)`
        : `planned (${result.points} pts)`;
      hint.textContent = `${stack}: path found — showing ${shown}. `
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

  function enterSim() {
    const stacks = (state.telemetry && state.telemetry.stacks) || {};
    if (!stacks.nav) {
      alert('Start the nav stack first — the planner must be running for simulation.');
      return;
    }
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

  const btnSimEnter = $('btnSimEnter');
  if (btnSimEnter) btnSimEnter.onclick = () => enterSim();
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
    });
    updateSimHint({ ok: null });
  }, 'Planning…');

  bindLoading('btnMissionStart', async () => {
    if (!state.waypoints.length) await loadSavedWaypoints();
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
    const m = (state.telemetry && state.telemetry.mission) || {};
    if (['running', 'wait_action', 'wait_goal', 'recovering'].includes(
      String(m.state || '').toLowerCase())) {
      alert('Mission already running — press Stop first.');
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
