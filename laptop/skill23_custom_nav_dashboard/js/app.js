(() => {
  const $ = (id) => document.getElementById(id);

  const state = {
    sequences: {},
    sequenceBuilder: [],
    useSequences: false,
    simulation: false,
    simStart: null,
    simEnd: null,
    pathStyle: 'smoothed',
    lastSimPlanner: [],
    lastSimSmoothed: [],
  };

  let lastTelem = null;
  let waypoints = [];
  let lastCollisionKey = '';
  let lastRecoveryKey = '';

  function log(msg) {
    const el = $('log');
    if (!el) return;
    const line = document.createElement('div');
    line.textContent = `[${new Date().toLocaleTimeString()}] ${msg}`;
    el.prepend(line);
    while (el.children.length > 80) el.removeChild(el.lastChild);
  }

  function setStatus(ok, text) {
    $('connText').textContent = text;
    $('connDot').classList.toggle('online', !!ok);
  }

  function formatStep(step, i, status) {
    if (!step || typeof step !== 'object') {
      return { icon: '☐', text: `#${i + 1} —` };
    }
    let icon = '☐';
    if (status === 'done') icon = '✓';
    else if (status === 'active') icon = '→';
    else if (status === 'pending') icon = '○';

    const x = Number(step.x);
    const y = Number(step.y);
    const yawDeg = Number.isFinite(step.yaw)
      ? ((step.yaw * 180) / Math.PI).toFixed(0)
      : '?';
    const xy = (Number.isFinite(x) && Number.isFinite(y))
      ? `(${x.toFixed(2)}, ${y.toFixed(2)})`
      : '—';
    return { icon, text: `#${i + 1}  ${xy}  yaw=${yawDeg}°`, status };
  }

  function renderSteps(t) {
    const ul = $('stepList');
    if (!ul) return;
    ul.innerHTML = '';
    const mission = (t && t.mission) || {};
    const idx = mission.wp_idx != null ? mission.wp_idx
      : (mission.index != null ? mission.index : 0);
    const total = mission.wp_total != null ? mission.wp_total
      : (mission.total != null ? mission.total : 0);

    // Prefer mission waypoint list; fall back to local goals
    let wps = [];
    if (Array.isArray(mission.waypoints) && mission.waypoints.length) {
      wps = mission.waypoints;
    } else if (waypoints.length) {
      wps = waypoints;
    }

    if (!wps.length && !total) {
      ul.innerHTML = '<li class="hint">No waypoints — mark goals &amp; Start</li>';
      return;
    }

    const n = wps.length || total;
    for (let i = 0; i < n; i++) {
      let status = 'pending';
      const state = String(mission.state || '').toLowerCase();
      if (state === 'done') status = 'done';
      else if (i < idx) status = 'done';
      else if (i === idx && state !== 'idle' && state !== 'stopped') status = 'active';

      const step = wps[i] || { x: '?', y: '?' };
      const { icon, text } = formatStep(step, i, status);
      const li = document.createElement('li');
      li.className = `step-${status}`;
      li.innerHTML = `<span class="step-icon">${icon}</span><span class="step-text">${text}</span>`;
      ul.appendChild(li);
    }
  }

  function loadSequences() {
    try {
      state.sequences = JSON.parse(localStorage.getItem('cn_sequences') || '{}');
    } catch (_) {
      state.sequences = {};
    }
    state.useSequences = localStorage.getItem('cn_use_sequences') === '1';
    $('toggleSequences')?.classList.toggle('on', state.useSequences);
    $('sequencePanel')?.classList.toggle('hidden', !state.useSequences);
    refreshSequenceSelect();
  }

  function saveSequences() {
    localStorage.setItem('cn_sequences', JSON.stringify(state.sequences));
    refreshSequenceSelect();
  }

  function refreshSequenceSelect() {
    const sel = $('sequenceSelect');
    if (!sel) return;
    const cur = sel.value;
    sel.innerHTML = '<option value="">— choose sequence —</option>';
    Object.keys(state.sequences).sort().forEach((name) => {
      const opt = document.createElement('option');
      opt.value = name;
      opt.textContent = `${name} (${state.sequences[name].length} steps)`;
      sel.appendChild(opt);
    });
    if (cur && state.sequences[cur]) sel.value = cur;
  }

  function updateSeqChainPreview() {
    const el = $('seqChainPreview');
    if (!el) return;
    if (!state.sequenceBuilder.length) {
      el.textContent = '—';
      return;
    }
    el.textContent = state.sequenceBuilder.map((idx) => idx + 1).join(' → ');
  }

  function renderSequenceBuilder() {
    const box = $('sequenceBuilder');
    if (!box) return;
    box.innerHTML = '';
    if (!waypoints.length) {
      box.innerHTML = '<span class="seq-hint">Mark waypoints on the map first</span>';
      updateSeqChainPreview();
      return;
    }
    waypoints.forEach((wp, i) => {
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.textContent = String(i + 1);
      btn.title = `Add #${i + 1} (${wp.x.toFixed(2)}, ${wp.y.toFixed(2)})`;
      btn.onclick = () => {
        state.sequenceBuilder.push(i);
        updateSeqChainPreview();
      };
      box.appendChild(btn);
    });
    updateSeqChainPreview();
  }

  function buildMissionWaypoints() {
    if (!waypoints.length) return null;

    if (state.useSequences) {
      const name = $('sequenceSelect')?.value;
      if (!name) {
        alert('Turn on sequences — select one from the dropdown first');
        return null;
      }
      const seq = state.sequences[name];
      if (!seq?.length) {
        alert('Selected sequence is empty');
        return null;
      }
      const mission = [];
      for (const idx of seq) {
        if (idx < 0 || idx >= waypoints.length) {
          alert(`Sequence references missing waypoint #${idx + 1}`);
          return null;
        }
        mission.push({ ...waypoints[idx] });
      }
      return mission;
    }
    return waypoints.map((wp) => ({ ...wp }));
  }

  function renderWaypointList() {
    const ul = $('waypointList');
    if (!ul) return;
    ul.innerHTML = '';
    if (!waypoints.length) {
      ul.innerHTML = '<li class="hint">No goals — click+drag on map</li>';
      renderSequenceBuilder();
      return;
    }
    waypoints.forEach((wp, i) => {
      const li = document.createElement('li');
      const deg = ((wp.yaw || 0) * 180 / Math.PI).toFixed(0);
      li.innerHTML =
        `<span>#${i + 1}  x=${wp.x.toFixed(2)} y=${wp.y.toFixed(2)} yaw=${deg}°</span>` +
        `<span class="wp-actions"><button type="button" data-i="${i}" title="Remove">×</button></span>`;
      li.querySelector('button').onclick = () => {
        waypoints.splice(i, 1);
        state.sequenceBuilder = state.sequenceBuilder
          .filter((idx) => idx !== i)
          .map((idx) => (idx > i ? idx - 1 : idx));
        CnMap.setWaypoints(waypoints);
        renderWaypointList();
        pushGoalsPreview().catch((e) => log(e.message));
      };
      ul.appendChild(li);
    });
    renderSequenceBuilder();
  }

  function setModeBadge(mode) {
    const el = $('modeBadge');
    if (!el) return;
    const m = (mode || 'IDLE').toUpperCase();
    el.textContent = m;
    el.className = 'mode-badge mode-' + m.toLowerCase();
  }

  function updateBattery(bat) {
    bat = bat || {};
    if (bat.valid) {
      $('batteryPct').textContent = `${bat.percentage ?? 0}%`;
      $('batteryVolt').textContent = `${bat.voltage ?? 0} V`;
      $('batteryFill').style.width = `${Math.min(100, bat.percentage ?? 0)}%`;
    } else {
      $('batteryPct').textContent = 'N/A';
      $('batteryVolt').textContent =
        bat.source === 'waiting_for_control_server' ? 'no VMX' : '—';
      $('batteryFill').style.width = '0%';
    }
  }

  async function pushGoalsPreview() {
    if (state.useSequences) return;
    if (!waypoints.length) return;
    await CnApi.setGoals(waypoints);
    log(`Sent ${waypoints.length} goal(s) to mission`);
  }

  async function connect() {
    const url = $('piUrl').value.trim();
    CnApi.setBaseUrl(url);
    try {
      await CnApi.status();
      setStatus(true, 'ONLINE');
      log('Connected to ' + url);
      const maps = await CnApi.fetchMaps();
      const sel = $('mapSelect');
      sel.innerHTML = '';
      (maps.maps || []).forEach((p) => {
        const opt = document.createElement('option');
        opt.value = p;
        opt.textContent = p.split('/').pop();
        sel.appendChild(opt);
      });
      CnApi.connectStream(onTelem);
      // Fetch whatever map the robot already has — do NOT auto-load dropdown
      // map (that was swapping test_map1 under AMCL and exploding pose).
      try {
        const m = await CnApi.fetchMap();
        if (m && m.width) {
          CnMap.setMap(m);
          log(`Robot map: ${m.width}×${m.height}`);
        }
      } catch (_) {
        log('No map on robot yet — use Load Map from Pi');
      }
    } catch (e) {
      setStatus(false, 'OFFLINE');
      log('Connect failed: ' + e.message);
    }
  }

  async function loadSelectedMap() {
    const path = $('mapSelect').value;
    if (!path) return;
    try {
      await CnApi.loadMap(path);
      const m = await CnApi.fetchMap();
      CnMap.setMap(m);
      log('Map loaded on robot: ' + path.split('/').pop());
    } catch (e) {
      log('Map load failed: ' + e.message);
    }
  }

  function parseMapYaml(text) {
    const out = {};
    text.split('\n').forEach((line) => {
      const trimmed = line.split('#')[0].trim();
      if (!trimmed) return;
      const m = trimmed.match(/^([\w_]+):\s*(.+)$/);
      if (!m) return;
      const key = m[1];
      let val = m[2].trim();
      if (key === 'mode' || key === 'image') {
        out[key] = val.replace(/^["']|["']$/g, '');
      } else if (val.startsWith('[') && val.endsWith(']')) {
        out[key] = val.slice(1, -1).split(',').map((s) => parseFloat(s.trim()));
      } else if (/^-?\d+(\.\d+)?$/.test(val)) {
        out[key] = parseFloat(val);
      } else {
        out[key] = val;
      }
    });
    if (!out.origin || out.origin.length < 2) out.origin = [0, 0, 0];
    return out;
  }

  async function loadLocalMap() {
    const yamlFile = $('mapFile')?.files?.[0];
    let pgmFile = $('mapPgmFile')?.files?.[0];
    const statusEl = $('mapStatus');
    if (!yamlFile) {
      if (statusEl) statusEl.textContent = 'Select a .yaml map file first';
      alert('Select the .yaml file first');
      return;
    }
    try {
      const text = await yamlFile.text();
      const yamlMeta = parseMapYaml(text);
      if (!pgmFile) {
        const stem = yamlFile.name.replace(/\.(yaml|yml)$/i, '');
        const imageName = (yamlMeta.image || `${stem}.pgm`).toLowerCase();
        const files = [...($('mapPgmFile')?.files || [])];
        pgmFile = files.find((f) => f.name.toLowerCase() === imageName) || files[0];
      }
      if (!pgmFile) {
        if (statusEl) statusEl.textContent = 'Also select the matching .pgm';
        alert('Select the matching .pgm file');
        return;
      }
      const buf = new Uint8Array(await pgmFile.arrayBuffer());
      CnMap.setMapFromLocal(yamlMeta, buf);
      if (statusEl) statusEl.textContent = `Local: ${yamlFile.name} + ${pgmFile.name}`;
      log(`Local map: ${yamlFile.name} (display only — use Pi load for robot)`);
    } catch (e) {
      if (statusEl) statusEl.textContent = 'Map error: ' + e.message;
      log('Local map failed: ' + e.message);
    }
  }

  function formatNode(n) {
    if (!n || typeof n !== 'object') return '—';
    const st = n.state || '—';
    const d = n.detail ? ` · ${n.detail}` : '';
    return `${st}${d}`;
  }

  function onTelem(t) {
    lastTelem = t;
    if (t.robot_dims) CnMap.setDims(t.robot_dims);
    if (t.pose && Number.isFinite(t.pose.x) && Number.isFinite(t.pose.y)) {
      CnMap.setPose(t.pose);
      $('poseReadout').textContent =
        `x=${t.pose.x.toFixed(2)}  y=${t.pose.y.toFixed(2)}  yaw=${(t.pose.yaw * 180 / Math.PI).toFixed(1)}°`;
    } else {
      $('poseReadout').textContent = '— (set pose)';
    }

    if (!state.simulation) {
      CnMap.setPath(t.path || []);
      CnMap.setPathLayers(
        t.planner_path || [],
        t.smoothed_path || [],
        t.path_style || state.pathStyle);
      if (t.path_style && t.path_style !== state.pathStyle) {
        state.pathStyle = t.path_style;
        const sel = $('pathStyleSelect');
        if (sel) sel.value = state.pathStyle;
      }
    } else if (state.lastSimPlanner.length || state.lastSimSmoothed.length) {
      const active = state.pathStyle === 'smoothed'
        ? (state.lastSimSmoothed.length ? state.lastSimSmoothed : state.lastSimPlanner)
        : (state.lastSimPlanner.length ? state.lastSimPlanner : state.lastSimSmoothed);
      CnMap.setSimPaths(state.lastSimPlanner, state.lastSimSmoothed, active);
    }
    CnMap.setScan(t.scan || []);

    const mission = t.mission || {};
    const goalStatus = typeof t.goal_status === 'string'
      ? t.goal_status
      : (t.goal && (t.goal.state || t.goal.status)) || '—';
    const recovery = t.recovery || {};
    const collision = t.collision || {};
    const amcl = t.amcl || {};
    const nodes = t.nodes || {};

    // Prefer recovery phase while active; else mission mode
    let mode = t.mode || mission.state || 'IDLE';
    if (t.estop) mode = 'E_STOP';
    else if (recovery.state && recovery.state !== 'idle' && recovery.state !== 'fail') {
      mode = recovery.state; // wait | replan
    } else if (String(mission.state).toLowerCase() === 'recovering') {
      mode = 'recovering';
    }
    setModeBadge(mode);
    updateBattery(t.battery);

    $('missionReadout').textContent =
      `${mission.state || '—'}` +
      (mission.wp_idx != null || mission.index != null
        ? `  ·  wp ${(mission.wp_idx ?? mission.index) + 1}/${mission.wp_total ?? mission.total ?? '?'}`
        : '');

    $('amclReadout').textContent = amcl.ok
      ? `OK · age ${amcl.age_s ?? '?'}s`
      : (amcl.has_pose || amcl.held
        ? `pose held · no new msg ${amcl.age_s ?? '?'}s`
        : 'no pose — set initial pose');

    $('goalReadout').textContent = String(goalStatus);

    const plannerSt = t.planner_status || {};
    if ($('plannerReadout')) {
      $('plannerReadout').textContent =
        `${plannerSt.state || '—'}` +
        (plannerSt.message && plannerSt.message !== plannerSt.state
          ? ` · ${plannerSt.message}` : '');
    }

    if ($('worldReadout')) {
      $('worldReadout').textContent = t.world_ready
        ? 'ready · map + static EDT'
        : 'not ready — wait for world model';
      $('worldReadout').style.color = t.world_ready ? 'var(--success)' : 'var(--danger)';
    }

    const recDetail = recovery.detail || recovery.action || '';
    $('recoveryReadout').textContent =
      `${recovery.state || 'idle'}` +
      (recovery.active ? '  ·  ACTIVE' : '') +
      (recDetail ? `  ·  ${recDetail}` : '') +
      (recovery.attempts != null ? `  ·  #${recovery.attempts}` : '');

    // Collision: front buffer + clearances
    const cState = collision.state
      || (collision.blocked ? 'blocked' : (collision.near ? 'near' : 'clear'));
    let cExtra = '';
    if (collision.reason || collision.clearance) {
      const parts = [];
      if (collision.reason) parts.push(String(collision.reason));
      const cl = collision.clearance || {};
      if (cl.front != null) {
        parts.push(
          `F${Number(cl.front).toFixed(2)}`
          + ` R${Number(cl.rear != null ? cl.rear : 0).toFixed(2)}`
          + ` L${Number(cl.left != null ? cl.left : 0).toFixed(2)}`
          + ` Ri${Number(cl.right != null ? cl.right : 0).toFixed(2)}`);
      }
      cExtra = parts.length ? `  ·  ${parts.join(' · ')}` : '';
    } else if (collision.min_range_m != null) {
      cExtra = `  ·  ${Number(collision.min_range_m).toFixed(2)} m`;
    }
    $('collisionReadout').textContent = `${cState}${cExtra}`;
    if (collision.blocked) {
      $('collisionReadout').style.color = 'var(--danger)';
    } else if (collision.near) {
      $('collisionReadout').style.color = 'var(--warn, #c47a00)';
    } else {
      $('collisionReadout').style.color = '';
    }

    // Log collision / recovery transitions (not every telem tick).
    const cKey = [
      collision.blocked ? '1' : '0',
      collision.state || '',
      collision.reason || '',
    ].join('|');
    if (cKey !== lastCollisionKey && (collision.state || collision.reason != null)) {
      const prevBlocked = lastCollisionKey.startsWith('1|');
      lastCollisionKey = cKey;
      if (collision.blocked) {
        const cl = collision.clearance || {};
        log(
          `COLLISION: ${collision.reason || cState}`
          + ` · F${Number(cl.front || 0).toFixed(2)}`
          + ` R${Number(cl.rear || 0).toFixed(2)}`
          + ` L${Number(cl.left || 0).toFixed(2)}`
          + ` Ri${Number(cl.right || 0).toFixed(2)}`);
      } else if (prevBlocked) {
        log(`COLLISION clear: ${collision.reason || 'front clear'}`);
      } else if (collision.state === 'near') {
        log(`COLLISION near: ${collision.reason || 'front approaching'}`);
      }
    }

    const rKey = [
      recovery.state || 'idle',
      recovery.detail || recovery.action || '',
      recovery.attempts != null ? String(recovery.attempts) : '',
      recovery.active ? '1' : '0',
    ].join('|');
    if (rKey !== lastRecoveryKey && (recovery.state || recovery.detail)) {
      lastRecoveryKey = rKey;
      const d = recovery.detail || recovery.action || '';
      if (recovery.active || recovery.state !== 'idle' || d) {
        log(`RECOVERY: ${d || recovery.state}`);
      }
    }

    if ($('nodePlanner')) $('nodePlanner').textContent = formatNode(nodes.planner);
    if ($('nodeController')) $('nodeController').textContent = formatNode(nodes.controller);
    if ($('nodeMission')) $('nodeMission').textContent = formatNode(nodes.mission);
    if ($('nodeCollision')) $('nodeCollision').textContent = formatNode(nodes.collision);
    if ($('nodeRecovery')) $('nodeRecovery').textContent = formatNode(nodes.recovery);
    if ($('nodeGoal')) $('nodeGoal').textContent = formatNode(nodes.goal_checker);

    renderSteps(t);

    if (t.estop) $('estopBtn').classList.add('armed');
    else $('estopBtn').classList.remove('armed');
  }

  function updateSimPointsReadout() {
    const el = $('simPointsReadout');
    if (!el) return;
    const s = state.simStart;
    const e = state.simEnd;
    el.textContent =
      `Start: ${s ? `(${s.x.toFixed(2)}, ${s.y.toFixed(2)})` : '—'}` +
      ` · End: ${e ? `(${e.x.toFixed(2)}, ${e.y.toFixed(2)})` : '—'}`;
  }

  function onMapCommit(x, y, mode, yaw) {
    if (mode === 'sim_start' || mode === 'sim_end') return;
    const yawRad = typeof yaw === 'number' ? yaw : 0;
    const deg = (yawRad * 180 / Math.PI).toFixed(0);
    if (mode === 'pose') {
      CnApi.setPose(x, y, yawRad)
        .then(() => log(`Set pose x=${x.toFixed(2)} y=${y.toFixed(2)} yaw=${deg}°`))
        .catch((e) => log(e.message));
      return;
    }
    waypoints.push({ x, y, yaw: yawRad });
    CnMap.setWaypoints(waypoints);
    renderWaypointList();
    log(`Goal #${waypoints.length} x=${x.toFixed(2)} y=${y.toFixed(2)} yaw=${deg}°`);
    pushGoalsPreview().catch((e) => log(e.message));
  }

  function clearAllWaypoints() {
    waypoints = [];
    state.sequenceBuilder = [];
    CnMap.clearWaypoints();
    renderWaypointList();
    log('Cleared goals');
  }

  document.addEventListener('DOMContentLoaded', () => {
    $('piUrl').value = CnApi.getBaseUrl();
    CnMap.init($('mapCanvas'), onMapCommit);
    loadSequences();
    renderWaypointList();
    renderSteps([]);

    CnMap.onHoverCoords = (x, y) => {
      const el = $('mapHoverCoords');
      if (!el) return;
      if (x == null || y == null) {
        el.textContent = 'x=—  y=—';
        return;
      }
      el.textContent = `x=${x.toFixed(3)}  y=${y.toFixed(3)}`;
    };

    CnMap.onSimPoint = (mode, pt, start, end) => {
      state.simStart = start;
      state.simEnd = end;
      updateSimPointsReadout();
      if (mode === 'sim_start') {
        CnMap.setMode('sim_end');
        log('Start set — click map for END');
      } else {
        log('End set — press OK — Plan Path');
      }
    };

    $('toggleSim')?.addEventListener('click', () => {
      state.simulation = !$('toggleSim').classList.contains('on');
      $('toggleSim').classList.toggle('on', state.simulation);
      $('simPlanPanel')?.classList.toggle('hidden', !state.simulation);
      if (state.simulation) {
        state.simStart = null;
        state.simEnd = null;
        CnMap.clearSimPlan();
        updateSimPointsReadout();
        CnMap.setMode('sim_start');
        $('modeGoal')?.classList.remove('active');
        $('modePose')?.classList.remove('active');
        log('Simulation ON — pick Start & End, then OK (custom planner)');
      } else {
        CnMap.clearSimPlan();
        CnMap.setMode('goal');
        $('modeGoal')?.classList.add('active');
        log('Simulation OFF');
      }
    });

    $('btnSimStart')?.addEventListener('click', () => {
      CnMap.setMode('sim_start');
      log('Click map to set START');
    });
    $('btnSimEnd')?.addEventListener('click', () => {
      CnMap.setMode('sim_end');
      log('Click map to set END');
    });
    $('btnSimPlanClear')?.addEventListener('click', () => {
      state.simStart = null;
      state.simEnd = null;
      state.lastSimPlanner = [];
      state.lastSimSmoothed = [];
      CnMap.clearSimPlan();
      updateSimPointsReadout();
    });
    $('btnSimPlanOk')?.addEventListener('click', async () => {
      if (!state.simStart || !state.simEnd) {
        alert('Set both Start and End on the map first');
        return;
      }
      try {
        log('Planning with custom_nav A* + smoother…');
        const res = await CnApi.planPath(state.simStart, state.simEnd);
        const planner = res.planner_path || res.path || [];
        const smoothed = res.smoothed_path || [];
        if (!planner.length && !smoothed.length) {
          throw new Error(res.error || 'empty path');
        }
        state.lastSimPlanner = planner;
        state.lastSimSmoothed = smoothed;
        const active = (res.path_style || state.pathStyle) === 'smoothed'
          ? (smoothed.length ? smoothed : planner)
          : (planner.length ? planner : smoothed);
        CnMap.setSimPaths(planner, smoothed, active);
        CnMap.setSimMarkers(state.simStart, state.simEnd);
        log(`Plan OK — planner ${planner.length} · smoothed ${smoothed.length} · showing ${state.pathStyle}`);
      } catch (e) {
        log('Plan failed: ' + e.message);
        alert('Plan failed: ' + e.message);
      }
    });

    $('pathStyleSelect')?.addEventListener('change', async () => {
      const style = $('pathStyleSelect').value === 'normal' ? 'normal' : 'smoothed';
      state.pathStyle = style;
      CnMap.setPathStyle(style);
      try {
        await CnApi.setPathStyle(style);
        log(`Path style → ${style} (robot + dashboard)`);
      } catch (e) {
        log('path_style failed: ' + e.message);
      }
      if (state.simulation && (state.lastSimPlanner.length || state.lastSimSmoothed.length)) {
        const active = style === 'smoothed'
          ? (state.lastSimSmoothed.length ? state.lastSimSmoothed : state.lastSimPlanner)
          : (state.lastSimPlanner.length ? state.lastSimPlanner : state.lastSimSmoothed);
        CnMap.setSimPaths(state.lastSimPlanner, state.lastSimSmoothed, active);
      }
    });

    $('btnConnect').onclick = connect;
    $('btnLoadMap').onclick = loadSelectedMap;
    $('btnLoadLocalMap')?.addEventListener('click', () => loadLocalMap());
    $('modeGoal').onclick = () => {
      CnMap.setMode('goal');
      $('modeGoal').classList.add('active');
      $('modePose').classList.remove('active');
    };
    $('modePose').onclick = () => {
      CnMap.setMode('pose');
      $('modePose').classList.add('active');
      $('modeGoal').classList.remove('active');
    };
    $('btnClearGoals').onclick = clearAllWaypoints;
    $('btnClearWp').onclick = clearAllWaypoints;

    $('btnSetYaw').onclick = () => {
      const yawDeg = parseFloat($('wpYaw').value) || 0;
      if (!waypoints.length) {
        log('Add a waypoint first');
        return;
      }
      waypoints[waypoints.length - 1].yaw = yawDeg * Math.PI / 180;
      CnMap.setWaypoints(waypoints);
      renderWaypointList();
      log(`Set last waypoint yaw to ${yawDeg}°`);
      pushGoalsPreview().catch((e) => log(e.message));
    };

    $('toggleSequences')?.addEventListener('click', () => {
      state.useSequences = !$('toggleSequences').classList.contains('on');
      $('toggleSequences').classList.toggle('on', state.useSequences);
      $('sequencePanel')?.classList.toggle('hidden', !state.useSequences);
      localStorage.setItem('cn_use_sequences', state.useSequences ? '1' : '0');
      if (!state.useSequences) {
        pushGoalsPreview().catch((e) => log(e.message));
      }
    });

    $('btnSeqSave')?.addEventListener('click', () => {
      const name = ($('sequenceName')?.value || '').trim();
      if (!name) { alert('Enter a sequence name (e.g. Loop A)'); return; }
      if (!state.sequenceBuilder.length) {
        alert('Click waypoint numbers to build a chain first');
        return;
      }
      state.sequences[name] = [...state.sequenceBuilder];
      saveSequences();
      $('sequenceSelect').value = name;
      alert(`Saved "${name}": ${state.sequenceBuilder.map((i) => i + 1).join(' → ')}`);
    });

    $('btnSeqClear')?.addEventListener('click', () => {
      state.sequenceBuilder = [];
      updateSeqChainPreview();
    });

    $('btnSeqUndo')?.addEventListener('click', () => {
      if (state.sequenceBuilder.length) {
        state.sequenceBuilder.pop();
        updateSeqChainPreview();
      }
    });

    $('btnSeqDelete')?.addEventListener('click', () => {
      const name = $('sequenceSelect')?.value;
      if (!name) { alert('Select a sequence to delete'); return; }
      if (!confirm(`Delete sequence "${name}"?`)) return;
      delete state.sequences[name];
      saveSequences();
    });

    $('btnStart').onclick = async () => {
      try {
        if (!lastTelem || !lastTelem.pose) {
          log('Set pose before Start');
          return;
        }
        if (lastTelem && lastTelem.world_ready === false) {
          log('World not ready — wait for map/static EDT');
          return;
        }
        const mission = buildMissionWaypoints();
        if (!mission) return;
        await CnApi.setGoals(mission);
        log(
          state.useSequences
            ? `Sequence → ${mission.length} goal(s) sent`
            : `Sent ${mission.length} goal(s)`,
        );
        await CnApi.start();
        log('Start — mission plans path, controller follows lookahead');
      } catch (e) {
        log(e.message);
      }
    };
    $('btnStop').onclick = () => CnApi.stop().then(() => log('Stop')).catch((e) => log(e.message));
    $('estopBtn').onclick = async () => {
      const arm = !$('estopBtn').classList.contains('armed');
      await CnApi.estop(arm);
      log(arm ? 'E-STOP ON' : 'E-STOP OFF');
    };
    $('btnSetPoseOrigin').onclick = () => {
      CnApi.setPose(0, 0, 0).then(() => log('Set pose 0,0,0')).catch((e) => log(e.message));
    };
  });
})();
