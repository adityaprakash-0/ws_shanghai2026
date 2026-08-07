/** ADEV Dashboard main application */
(() => {
  const state = {
    simulation: false,
    estopActive: false,
    connected: false,
    robot: 'allterrain',
    telemetry: {},
    sequences: {},
    sequenceBuilder: [],
    useSequences: false,
  };

  const ROBOT_MAPS = {
    allterrain: 'test_map10.yaml',
    mecanum: 'test_map6.yaml',
    xdrive: 'test_map10.yaml',
    grip: 'test_map10.yaml',
  };

  const $ = (id) => document.getElementById(id);

  const NAV_MODE_LABELS = {
    nav2: 'Nav2',
    nav2_secret_mode: 'Secret Mode',
  };

  function formatNavMode(mode) {
    return NAV_MODE_LABELS[mode] || mode || '—';
  }

  function formatNavStackState(t) {
    const nav2 = t.nav2 || {};
    if (t.mission?.active && (t.mission?.status === 'navigating' || t.mission?.status === 'retrying')) {
      return nav2.state === 'navigating' ? 'navigating' : (t.mission.status || 'navigating');
    }
    if (!nav2.available) {
      return nav2.state === 'unknown' ? 'starting…' : 'unavailable';
    }
    return nav2.state || 'ready';
  }

  function setConnStatus(msg, ok) {
    const el = $('connStatus');
    if (!el) return;
    el.textContent = msg || '';
    el.className = 'conn-status' + (ok === true ? ' ok' : ok === false ? ' err' : '');
  }

  function updateConnBadge(online, sim) {
    const dot = $('connDot');
    const text = $('connText');
    dot.className = 'conn-dot' + (sim ? ' sim' : online ? ' online' : '');
    text.textContent = sim ? 'SIMULATION' : online ? 'CONNECTED' : 'OFFLINE';
  }

  function renderTelemetry(t) {
    if (t._offline) {
      if (!state.simulation) {
        state.connected = false;
        updateConnBadge(false, false);
      }
      return;
    }
    state.connected = true;
    state.telemetry = t;
    updateConnBadge(true, state.simulation);

    $('poseX').textContent = t.pose?.x?.toFixed(2) ?? '—';
    $('poseY').textContent = t.pose?.y?.toFixed(2) ?? '—';
    $('poseYaw').textContent = t.pose?.yaw != null
      ? `${(t.pose.yaw * 180 / Math.PI).toFixed(1)}°` : '—';

    $('navMode').textContent = formatNavMode(t.nav_mode);
    $('navMode').title = t.nav2?.action ? `action: ${t.nav2.action}` : '';
    $('nav2State').textContent = formatNavStackState(t);
    $('nav2State').title = t.nav2?.action ? `action: ${t.nav2.action}` : '';
    const dwellRem = t.mission?.dwell_remaining ?? 0;
    if (t.mission?.status === 'dwelling' && dwellRem > 0) {
      $('missionState').textContent = 'waiting';
      $('missionDwell').textContent = `${Math.ceil(dwellRem)}s`;
    } else {
      $('missionState').textContent = t.mission?.status ?? 'idle';
      $('missionDwell').textContent = '—';
    }
    $('missionState').title = t.mission?.last_error || '';
    $('missionProgress').textContent = t.mission?.total
      ? `${t.mission.index + 1} / ${t.mission.total}` : '—';
    $('retryCount').textContent = t.mission?.retries ?? 0;

    const pi = t.path_info || {};
    const pathEl = $('pathInfo');
    if (pathEl) {
      const parts = [];
      if (pi.route_poses) parts.push(`route:${pi.route_poses}`);
      if (pi.plan_poses) parts.push(`plan:${pi.plan_poses}`);
      if (pi.local_poses) parts.push(`local:${pi.local_poses}`);
      pathEl.textContent = parts.length
        ? `${parts.join(' · ')} (${pi.last_topic || '—'})`
        : 'no path yet';
    }
    const dbgEl = $('navDebug');
    if (dbgEl && Array.isArray(t.debug_log)) {
      dbgEl.textContent = t.debug_log.slice(-12).join('\n');
    }

    const bat = t.battery || {};
    if (bat.valid) {
      $('batteryPct').textContent = `${bat.percentage ?? 0}%`;
      $('batteryVolt').textContent = `${bat.voltage ?? 0} V`;
      $('batteryFill').style.width = `${Math.min(100, bat.percentage ?? 0)}%`;
    } else {
      $('batteryPct').textContent = 'N/A';
      $('batteryVolt').textContent = bat.source === 'waiting_for_control_server' ? 'no VMX' : '—';
      $('batteryFill').style.width = '0%';
    }

    $('estopHw').textContent = t.estop_hw ? 'TRIGGERED' : 'OK';
    $('estopHw').style.color = t.estop_hw ? '#e63946' : '#2ecc71';

    $('robotState').textContent = t.mission?.active
      ? (t.mission?.status === 'dwelling'
        ? 'DWELL'
        : (t.mission?.mode === 'sim_path'
          ? 'SIM MISSION'
          : `${formatNavMode(t.nav_mode || t.mission?.mode)} MISSION`))
      : formatNavStackState(t) === 'navigating' ? 'NAVIGATING' : 'IDLE';

    if (!state.simulation) {
      MapView.setPose(t.pose || { x: 0, y: 0, yaw: 0 });
      MapView.setScan(t.scan);
      MapView.setPaths(t.global_path, t.local_path, t.route_path);
      MapView.setRobotDims(t.robot_dims);
      MapView.setWaypoints(MapView.getWaypoints(), t.mission?.index ?? 0);
    } else {
      MapView.setPose(t.pose || { x: 0, y: 0, yaw: 0 });
      MapView.setPaths(t.global_path, t.local_path, t.route_path);
      MapView.setRobotDims(t.robot_dims);
      MapView.setWaypoints(MapView.getWaypoints(), t.mission?.index ?? 0);
    }
  }

  function renderWaypointList(wps) {
    const ul = $('waypointList');
    ul.innerHTML = '';
    wps.forEach((wp, i) => {
      const li = document.createElement('li');
      li.className = i === (state.telemetry.mission?.index ?? 0) ? 'current' : '';
      const yawMark = wp.ignore_yaw ? ' ✓XY' : '';
      li.innerHTML = `<span>#${i + 1}  (${wp.x}, ${wp.y}, ${(wp.yaw * 180 / Math.PI).toFixed(0)}°${yawMark})</span>
        <span class="wp-actions">
          <label title="Ignore yaw at this goal"><input type="checkbox" data-i="${i}" class="wp-ignore-yaw" ${wp.ignore_yaw ? 'checked' : ''}></label>
          <button data-i="${i}" title="Remove">×</button>
        </span>`;
      li.querySelector('.wp-ignore-yaw').onchange = (e) => {
        wps[i].ignore_yaw = e.target.checked;
        MapView.setWaypoints(wps);
      };
      li.querySelector('button').onclick = () => {
        wps.splice(i, 1);
        MapView.setWaypoints(wps);
        renderWaypointList(wps);
        renderSequenceBuilder(wps);
      };
      ul.appendChild(li);
    });
    renderSequenceBuilder(wps);
  }

  function loadSequences() {
    try {
      state.sequences = JSON.parse(localStorage.getItem('adev_sequences') || '{}');
    } catch (_) {
      state.sequences = {};
    }
    state.useSequences = localStorage.getItem('adev_use_sequences') === '1';
    $('toggleSequences')?.classList.toggle('on', state.useSequences);
    $('sequencePanel')?.classList.toggle('hidden', !state.useSequences);
    refreshSequenceSelect();
  }

  function saveSequences() {
    localStorage.setItem('adev_sequences', JSON.stringify(state.sequences));
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

  function renderSequenceBuilder(wps) {
    const box = $('sequenceBuilder');
    if (!box) return;
    box.innerHTML = '';
    if (!wps.length) {
      box.innerHTML = '<span class="seq-hint">Mark waypoints on the map first</span>';
      updateSeqChainPreview(wps);
      return;
    }
    wps.forEach((wp, i) => {
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.textContent = String(i + 1);
      btn.title = `Add #${i + 1} (${wp.x}, ${wp.y})`;
      btn.onclick = () => {
        state.sequenceBuilder.push(i);
        updateSeqChainPreview(wps);
      };
      box.appendChild(btn);
    });
    updateSeqChainPreview(wps);
  }

  function updateSeqChainPreview(wps) {
    const el = $('seqChainPreview');
    if (!el) return;
    if (!state.sequenceBuilder.length) {
      el.textContent = '—';
      return;
    }
    el.textContent = state.sequenceBuilder.map((idx) => idx + 1).join(' → ');
  }

  function buildMissionWaypoints() {
    const wps = MapView.getWaypoints();
    if (!wps.length) return null;

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
        if (idx < 0 || idx >= wps.length) {
          alert(`Sequence references missing waypoint #${idx + 1}`);
          return null;
        }
        mission.push({ ...wps[idx] });
      }
      return mission;
    }
    return wps.map((wp) => ({ ...wp }));
  }

  async function connect() {
    const url = $('piUrl').value.trim();
    if (!url) {
      setConnStatus('Enter Pi Bridge URL (e.g. http://10.42.0.1:8080)', false);
      return;
    }
    AdevApi.setBaseUrl(url);
    localStorage.setItem('adev_pi_url', url);
    const btn = $('btnConnect');
    btn.disabled = true;
    setConnStatus('Connecting to Pi bridge…', null);
    try {
      const st = await AdevApi.status();
      state.connected = true;
      $('robotSelect').value = st.robot || state.robot;
      updateConnBadge(true, state.simulation);
      setConnStatus(`Connected to Pi (${st.robot})`, true);
      AdevApi.connectStream(renderTelemetry);
      await populateMaps();
      const mapOk = $('mapStatus')?.classList.contains('ok');
      if (!mapOk) await loadDefaultMap();
    } catch (e) {
      state.connected = false;
      updateConnBadge(false, state.simulation);
      const hint = e.message === 'Failed to fetch'
        ? `Cannot reach ${url}\n\nOn Pi terminal run:\nsource ~/skill23_ws/install/setup.bash\n~/skill23_ws/src/studica_ws/scripts/ros2_pi.sh launch adev_dashboard adev_dashboard.launch.py robot:=allterrain`
        : e.message;
      setConnStatus(hint, false);
      console.warn('Connect failed:', e.message);
    } finally {
      btn.disabled = false;
    }
  }

  async function loadDefaultMap() {
    const robot = $('robotSelect').value;
    const mapName = ROBOT_MAPS[robot] || 'test_map10.yaml';
    $('mapSelect').value = mapName;
    await loadMapByName(mapName);
  }

  async function loadMapByName(name) {
    if (state.simulation) {
      await loadMapSim(name);
      return;
    }
    try {
      const path = `/home/vmx/skill23_ws/${name}`;
      const info = await AdevApi.fetchMap(path);
      MapView.loadMap(info);
      setMapStatus('Loaded from Pi: ' + name, true);
    } catch (e) {
      console.warn('Map load failed:', e.message);
      setMapStatus('Pi map failed: ' + e.message, false);
    }
  }

  async function loadMapSim(name) {
    try {
      const path = `/home/vmx/skill23_ws/${name}`;
      const info = await AdevApi.fetchMap(path);
      MapView.loadMap(info);
    } catch (_) {
      MapView.draw();
    }
  }

  async function populateMaps() {
    if (state.simulation) return;
    try {
      const data = await AdevApi.fetchMaps();
      const sel = $('mapSelect');
      sel.innerHTML = '';
      (data.maps || []).forEach((m) => {
        const opt = document.createElement('option');
        opt.value = m.name + '.yaml';
        opt.textContent = m.name;
        sel.appendChild(opt);
      });
    } catch (_) { /* offline */ }
  }

  async function startSimulation() {
    state.simulation = true;
    updateConnBadge(true, true);
    setConnStatus('Simulation ON — planner path only, point follows line', true);
    try {
      if (!state.connected) await connect();
      await AdevApi.setSimulation(true);
    } catch (e) {
      state.simulation = false;
      updateConnBadge(state.connected, false);
      setConnStatus('Simulation failed: ' + e.message, false);
      $('toggleSim').classList.remove('on');
      return;
    }
    await loadDefaultMap();
  }

  async function stopSimulation() {
    state.simulation = false;
    try { await AdevApi.setSimulation(false); } catch (_) { /* */ }
    updateConnBadge(state.connected, false);
    setConnStatus('Simulation OFF — robot pose kept at last position', true);
  }

  function bindUI() {
    $('btnConnect').onclick = connect;

    $('robotSelect').onchange = () => {
      state.robot = $('robotSelect').value;
      loadDefaultMap();
    };

    $('mapSelect').onchange = () => loadMapByName($('mapSelect').value);

    $('btnLoadMap').onclick = () => loadMapByName($('mapSelect').value);

    $('btnLoadLocalMap').onclick = () => loadLocalMap();

    $('mapPgmFile').onchange = () => {
      if ($('mapFile').files[0] && $('mapPgmFile').files[0]) {
        loadLocalMap();
      }
    };

    $('toggleSim').onclick = async () => {
      const on = !$('toggleSim').classList.contains('on');
      $('toggleSim').classList.toggle('on', on);
      if (on) await startSimulation();
      else await stopSimulation();
    };

    $('btnEstop').onclick = async () => {
      state.estopActive = !state.estopActive;
      $('btnEstop').classList.toggle('active', state.estopActive);
      $('btnEstop').textContent = state.estopActive ? 'E-STOP ACTIVE' : 'EMERGENCY STOP';
      if (!state.simulation) {
        try { await AdevApi.estop(state.estopActive); } catch (_) { /* */ }
      }
    };

    $('btnMissionStart').onclick = async () => {
      const missionWps = buildMissionWaypoints();
      if (!missionWps) return;
      try {
        await AdevApi.setWaypoints(missionWps);
        const res = await AdevApi.startMission();
        if (res.ok === false) alert(res.message || 'Mission start failed');
      } catch (e) { alert('Mission start failed: ' + e.message); }
    };

    $('btnMissionStop').onclick = async () => {
      try { await AdevApi.stopMission(); } catch (_) { /* */ }
    };

    $('btnClearWp').onclick = () => {
      MapView.clearWaypoints();
      renderWaypointList([]);
    };

    $('btnSetYaw').onclick = () => {
      const yawDeg = parseFloat($('wpYaw').value) || 0;
      const wps = MapView.getWaypoints();
      if (wps.length) {
        wps[wps.length - 1].yaw = yawDeg * Math.PI / 180;
        wps[wps.length - 1].ignore_yaw = $('chkIgnoreYaw')?.checked || false;
        MapView.setWaypoints(wps);
        renderWaypointList(wps);
      }
    };

    $('toggleSequences')?.addEventListener('click', () => {
      state.useSequences = !$('toggleSequences').classList.contains('on');
      $('toggleSequences').classList.toggle('on', state.useSequences);
      $('sequencePanel')?.classList.toggle('hidden', !state.useSequences);
      localStorage.setItem('adev_use_sequences', state.useSequences ? '1' : '0');
    });

    $('btnSeqSave')?.addEventListener('click', () => {
      const name = ($('sequenceName')?.value || '').trim();
      if (!name) { alert('Enter a sequence name (e.g. Loop A)'); return; }
      if (!state.sequenceBuilder.length) { alert('Click waypoint numbers to build a chain first'); return; }
      state.sequences[name] = [...state.sequenceBuilder];
      saveSequences();
      $('sequenceSelect').value = name;
      alert(`Saved "${name}": ${state.sequenceBuilder.map((i) => i + 1).join(' → ')}`);
    });

    $('btnSeqClear')?.addEventListener('click', () => {
      state.sequenceBuilder = [];
      updateSeqChainPreview(MapView.getWaypoints());
    });

    $('btnSeqUndo')?.addEventListener('click', () => {
      if (state.sequenceBuilder.length) {
        state.sequenceBuilder.pop();
        updateSeqChainPreview(MapView.getWaypoints());
      }
    });

    $('btnSeqDelete')?.addEventListener('click', () => {
      const name = $('sequenceSelect')?.value;
      if (!name) { alert('Select a sequence to delete'); return; }
      if (!confirm(`Delete sequence "${name}"?`)) return;
      delete state.sequences[name];
      saveSequences();
    });

    document.querySelectorAll('.map-toolbar button').forEach((btn) => {
      btn.onclick = () => {
        document.querySelectorAll('.map-toolbar button').forEach((b) => b.classList.remove('active'));
        btn.classList.add('active');
        MapView.setMode(btn.dataset.mode);
      };
    });

    MapView.onWaypointAdd = (wps) => renderWaypointList(wps);

    MapView.onInitialPose = async (x, y) => {
      const yawDeg = parseFloat($('wpYaw').value) || 0;
      const yaw = yawDeg * Math.PI / 180;
      try { await AdevApi.setInitialPose(x, y, yaw); } catch (_) { /* */ }
    };
  }

  function setMapStatus(msg, ok) {
    const el = $('mapStatus');
    if (!el) return;
    el.textContent = msg;
    el.className = 'map-status' + (ok === true ? ' ok' : ok === false ? ' err' : '');
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
    if (!out.origin || out.origin.length < 2) {
      out.origin = [0, 0, 0];
    }
    return out;
  }

  async function readFileAsArrayBuffer(file) {
    try {
      return await file.arrayBuffer();
    } catch (e) {
      throw new Error(
        `Cannot read ${file.name}. On laptop run: chmod 644 "${file.name}"`);
    }
  }

  async function readFileAsText(file) {
    try {
      return await file.text();
    } catch (e) {
      throw new Error(
        `Cannot read ${file.name}. File may be locked (padlock icon). ` +
        `On laptop terminal: chmod 644 "${file.name}"`);
    }
  }

  async function loadLocalMap() {
    const yamlFile = $('mapFile').files[0];
    let pgmFile = $('mapPgmFile').files[0];

    if (!yamlFile) {
      setMapStatus('Select a .yaml map file first', false);
      alert('Select the .yaml file first');
      return;
    }

    try {
      const text = await readFileAsText(yamlFile);
      const yamlMeta = parseMapYaml(text);

      if (!pgmFile) {
        const stem = yamlFile.name.replace(/\.(yaml|yml)$/i, '');
        const pgmInput = $('mapPgmFile');
        const imageName = (yamlMeta.image || stem + '.pgm').toLowerCase();
        const match = [...pgmInput.files].find((f) => f.name.toLowerCase() === imageName)
          || [...pgmInput.files][0];
        if (match) pgmFile = match;
      }

      if (!pgmFile) {
        setMapStatus('Also select the .pgm file (same name as in yaml)', false);
        alert('Select the matching .pgm file.\n\nIn the yaml, image is: ' + (yamlMeta.image || '(not set)'));
        return;
      }

      const buf = await readFileAsArrayBuffer(pgmFile);
      MapView.loadMap({
        resolution: parseFloat(yamlMeta.resolution) || 0.05,
        origin: yamlMeta.origin,
        occupied_thresh: parseFloat(yamlMeta.occupied_thresh ?? 0.65),
        free_thresh: parseFloat(yamlMeta.free_thresh ?? 0.25),
        negate: parseInt(yamlMeta.negate ?? 0, 10),
        mode: yamlMeta.mode || 'trinary',
        pgm_bytes: new Uint8Array(buf),
      });

      setMapStatus(`Loaded: ${yamlFile.name} + ${pgmFile.name}`, true);
    } catch (e) {
      console.error(e);
      setMapStatus('Map error: ' + e.message, false);
      alert('Map load failed: ' + e.message);
    }
  }

  function init() {
    MapView.bindEvents();
    bindUI();
    loadSequences();
    const saved = localStorage.getItem('adev_pi_url');
    const url = saved || AdevApi.getDefaultUrl();
    $('piUrl').value = url;
    AdevApi.setBaseUrl(url);
    connect();
  }

  document.addEventListener('DOMContentLoaded', init);
})();
