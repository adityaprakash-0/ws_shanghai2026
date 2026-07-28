(() => {
  const $ = (id) => document.getElementById(id);

  const state = {
    sequences: {},
    sequenceBuilder: [],
    useSequences: false,
  };

  let lastTelem = null;
  let waypoints = [];

  function log(msg) {
    const el = $('log');
    const line = document.createElement('div');
    line.textContent = `[${new Date().toLocaleTimeString()}] ${msg}`;
    el.prepend(line);
  }

  function setStatus(ok, text) {
    $('connText').textContent = text;
    $('connDot').classList.toggle('online', !!ok);
  }

  function formatStep(step, i, status, progress, activeIdx) {
    if (!step || typeof step !== 'object') return { icon: '☐', text: `#${i + 1} —` };
    const t = step.type || '?';
    let icon = '☐';
    if (status === 'done') icon = '✗';
    else if (status === 'active') icon = '✓';

    let text = `#${i + 1} ${t}`;
    if (t === 'DRIVE') {
      const target = step.distance_m != null ? Number(step.distance_m) : null;
      const moved = (status === 'active' && progress && progress.kind === 'DRIVE')
        ? Number(progress.travelled_m || 0)
        : (status === 'done' ? target : 0);
      const tgt = target != null ? target.toFixed(2) : '?';
      const mv = moved != null ? Number(moved).toFixed(2) : '0.00';
      text = `#${i + 1} DRIVE  moved ${mv} / ${tgt} m`;
    } else if (t === 'TURN_LEFT_90' || t === 'TURN_RIGHT_90') {
      const dir = t === 'TURN_LEFT_90' ? 'LEFT' : 'RIGHT';
      const tgt = 90;
      const turned = (status === 'active' && progress && progress.kind === 'TURN')
        ? Number(progress.turn_deg || 0)
        : (status === 'done' ? tgt : 0);
      text = `#${i + 1} TURN ${dir}  ${turned.toFixed(1)} / ${tgt}°`;
    }
    return { icon, text, status };
  }

  function renderSteps(t) {
    const ul = $('stepList');
    if (!ul) return;
    ul.innerHTML = '';
    const steps = (t && (t.all_steps || t.remaining_steps)) || [];
    const statuses = (t && t.step_statuses) || [];
    const progress = (t && t.step_progress) || {};
    const idx = (t && t.fsm && t.fsm.primitive_idx != null)
      ? t.fsm.primitive_idx
      : (t && t.primitive_idx);
    // Prefer full plan list from fsm
    const all = (t && t.fsm && t.fsm.all_steps) || t.all_steps
      || (t && t.plan && t.plan.primitives) || steps;
    const st = (t && t.fsm && t.fsm.step_statuses) || statuses;
    const prog = (t && t.fsm && t.fsm.step_progress) || progress;

    if (!all || !all.length) {
      ul.innerHTML = '<li class="hint">No steps — set goal &amp; plan</li>';
      return;
    }
    all.forEach((step, i) => {
      let status = st[i];
      if (!status) {
        if (typeof idx === 'number') {
          status = i < idx ? 'done' : (i === idx ? 'active' : 'pending');
        } else {
          status = 'pending';
        }
      }
      const { icon, text } = formatStep(step, i, status, prog, idx);
      const li = document.createElement('li');
      li.className = `step-${status}`;
      li.innerHTML = `<span class="step-icon">${icon}</span><span class="step-text">${text}</span>`;
      ul.appendChild(li);
    });
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

  /** Build ordered goal list: all waypoints, or a saved sequence of indices. */
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
        // Drop sequence indices that pointed at removed / shifted waypoints
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

  /** Preview plan on Pi when sequences are off (order = map click order). */
  async function pushGoalsPreview() {
    if (state.useSequences) return;
    if (!waypoints.length) return;
    await CnApi.setGoals(waypoints);
    log(`Sent ${waypoints.length} goal(s) to planner`);
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
      if (sel.value) await loadSelectedMap();
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
      const m = await CnApi.fetchMap(path);
      CnMap.setMap(m);
      log('Map loaded: ' + path.split('/').pop());
    } catch (e) {
      log('Map load failed: ' + e.message);
    }
  }

  function onTelem(t) {
    lastTelem = t;
    if (t.robot_dims) CnMap.setDims(t.robot_dims);
    if (t.pose) {
      CnMap.setPose(t.pose);
      $('poseReadout').textContent =
        `x=${t.pose.x.toFixed(2)}  y=${t.pose.y.toFixed(2)}  yaw=${(t.pose.yaw * 180 / Math.PI).toFixed(1)}°`;
    } else {
      $('poseReadout').textContent = '— (set pose)';
    }

    CnMap.setPath(t.path || []);
    CnMap.setScan(t.scan || []);
    setModeBadge(t.mode || (t.fsm && t.fsm.state) || 'IDLE');
    updateBattery(t.battery);

    const w = t.watchdog || {};
    const f = t.fsm || {};
    const frontR = w.front_range_m ?? f.front_range_m;
    $('sensorsReadout').textContent =
      `Front: ${frontR ?? '—'} m  blocked=${w.front_blocked ? 'YES' : 'no'}` +
      ` (thresh ${w.front_block_thresh_m ?? '—'} m)\n` +
      `US L/R: ${w.us_left_m ?? '—'} / ${w.us_right_m ?? '—'} m\n` +
      `Wall L/R: ${w.wall_left ? 'Y' : 'n'}/${w.wall_right ? 'Y' : 'n'}  ` +
      `Open L/R: ${w.open_left ? 'Y' : 'n'}/${w.open_right ? 'Y' : 'n'}\n` +
      `Lidar gate: ${(t.lidar && t.lidar.awake) ? 'WAKE' : 'sleep'}`;

    $('decisionReadout').textContent = f.decision || '—';
    $('fsmReadout').textContent =
      `${f.state || '—'}  ·  step ${f.primitive_idx ?? 0}  ` +
      `(${f.primitives_left ?? 0} left)\n` +
      `recovery #${f.recovery_attempts ?? 0}`;

    const ps = t.planner_status || {};
    $('planReadout').textContent = ps.message || (t.plan && t.plan.primitives
      ? `${t.plan.primitives.length} primitives` : '—');

    renderSteps(t);

    if (t.estop) $('estopBtn').classList.add('armed');
    else $('estopBtn').classList.remove('armed');
  }

  function onMapCommit(x, y, mode, yaw) {
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

    $('btnConnect').onclick = connect;
    $('btnLoadMap').onclick = loadSelectedMap;
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
        const mission = buildMissionWaypoints();
        if (!mission) return;
        await CnApi.setGoals(mission);
        log(
          state.useSequences
            ? `Sequence → ${mission.length} goal(s) sent`
            : `Sent ${mission.length} goal(s)`,
        );
        await CnApi.start();
        log('Start');
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
