/** Pick-place dashboard app — stickers + mission builder */
(() => {
  const LS_STICKERS = 'pp_dashboard_stickers';
  const LS_MISSIONS = 'pp_dashboard_missions';
  const LS_STEPS = 'pp_dashboard_steps';
  const LS_ROBOT = 'pp_dashboard_robot';

  let stickers = [];
  let steps = [];
  let camTimer = null;

  function $(id) { return document.getElementById(id); }

  function loadStickers() {
    try { stickers = JSON.parse(localStorage.getItem(LS_STICKERS) || '[]'); }
    catch (_) { stickers = []; }
  }

  function saveStickers() {
    localStorage.setItem(LS_STICKERS, JSON.stringify(stickers));
    PpMap.setStickers(stickers);
    renderStickers();
  }

  function loadSteps() {
    try { steps = JSON.parse(localStorage.getItem(LS_STEPS) || '[]'); }
    catch (_) { steps = []; }
  }

  function saveSteps() {
    localStorage.setItem(LS_STEPS, JSON.stringify(steps));
    renderSteps();
  }

  function savedMissions() {
    try { return JSON.parse(localStorage.getItem(LS_MISSIONS) || '{}'); }
    catch (_) { return {}; }
  }

  function writeMissions(obj) {
    localStorage.setItem(LS_MISSIONS, JSON.stringify(obj));
    refreshMissionSelect();
  }

  function renderStickers() {
    const ul = $('stickerList');
    const sel = $('stickerPick');
    ul.innerHTML = '';
    sel.innerHTML = '';
    stickers.forEach((s, i) => {
      const li = document.createElement('li');
      li.innerHTML = `<span><b>${s.name}</b> [${s.kind}] `
        + `${s.x.toFixed(2)}, ${s.y.toFixed(2)}, ${(s.yaw * 180 / Math.PI).toFixed(0)}°</span>`;
      const del = document.createElement('button');
      del.className = 'btn btn-ghost';
      del.textContent = '✕';
      del.onclick = () => { stickers.splice(i, 1); saveStickers(); };
      li.appendChild(del);
      ul.appendChild(li);

      const opt = document.createElement('option');
      opt.value = String(i);
      opt.textContent = `${s.name} (${s.kind})`;
      sel.appendChild(opt);
    });
  }

  function renderSteps() {
    const ul = $('stepList');
    ul.innerHTML = '';
    steps.forEach((s, i) => {
      const li = document.createElement('li');
      let detail = '';
      if (s.type === 'goto') {
        detail = `x=${s.x.toFixed(2)} y=${s.y.toFixed(2)} yaw=${(s.yaw * 180 / Math.PI).toFixed(0)}°`;
      } else if (s.type === 'align') {
        detail = `mode=${s.mode}` + (s.colors ? ` colors=${s.colors.join(',')}` : '');
      } else if (s.type === 'pick' || s.type === 'place') {
        detail = `elev=${s.elevator} ext=${s.extend} grip=${s.gripper}`;
      } else if (s.type === 'wait') {
        detail = `${s.sec}s`;
      }
      li.innerHTML = `<span class="step-type">${i + 1}. ${s.type}</span> ${detail}`;
      ul.appendChild(li);
    });
  }

  function refreshMissionSelect() {
    const sel = $('missionSelect');
    const all = savedMissions();
    sel.innerHTML = '<option value="">— saved —</option>';
    Object.keys(all).forEach((k) => {
      const opt = document.createElement('option');
      opt.value = k;
      opt.textContent = k;
      sel.appendChild(opt);
    });
  }

  function selectedSticker() {
    const i = Number($('stickerPick').value);
    return stickers[i] || null;
  }

  function buildTask() {
    return {
      version: 1,
      name: $('missionName').value || 'mission',
      steps: steps.slice(),
    };
  }

  function setConnected(ok, text) {
    $('connDot').style.background = ok ? '#2ecc71' : '#e63946';
    $('connText').textContent = text;
    $('connStatus').textContent = text;
  }

  function onTelemetry(t) {
    if (!t) return;
    setConnected(true, 'ONLINE · ' + (t.robot || ''));
    if (t.pose) {
      PpMap.setPose(t.pose);
      $('poseX').textContent = t.pose.x.toFixed(3);
      $('poseY').textContent = t.pose.y.toFixed(3);
      $('poseYaw').textContent = (t.pose.yaw * 180 / Math.PI).toFixed(1) + '°';
    }
    const r = t.ranges || [];
    $('rFront').textContent = r[0] != null ? Number(r[0]).toFixed(2) : '—';
    $('rBack').textContent = r[1] != null ? Number(r[1]).toFixed(2) : '—';
    $('rLeft').textContent = r[2] != null ? Number(r[2]).toFixed(2) : '—';
    $('rRight').textContent = r[3] != null ? Number(r[3]).toFixed(2) : '—';
    const m = t.mission || {};
    $('missionState').textContent = m.state || '—';
    $('missionStep').textContent = (m.step_index >= 0)
      ? `${m.step_index + 1}/${m.steps}` : '—';
    $('missionMsg').textContent = m.message || '—';
    const arm = t.arm || {};
    $('armBusy').textContent = arm.busy ? 'BUSY' : 'idle';
    $('armEnc').textContent = arm.encoder != null ? Number(arm.encoder).toFixed(0) : '—';
  }

  async function connect() {
    const url = $('piUrl').value.trim();
    PpApi.setBaseUrl(url);
    try {
      const st = await PpApi.status();
      setConnected(true, 'ONLINE · ' + (st.robot || 'pp'));
      PpApi.connectStream(onTelemetry);
      const maps = await PpApi.fetchMaps();
      const sel = $('mapSelect');
      sel.innerHTML = '';
      (maps.maps || []).forEach((m) => {
        const opt = document.createElement('option');
        opt.value = m.path;
        opt.textContent = m.name;
        sel.appendChild(opt);
      });
    } catch (err) {
      setConnected(false, 'OFFLINE: ' + err.message);
    }
  }

  function wireUi() {
    $('piUrl').value = PpApi.getBaseUrl();
    const savedRobot = localStorage.getItem(LS_ROBOT);
    if (savedRobot) $('robotSelect').value = savedRobot;
    $('robotSelect').onchange = () => {
      localStorage.setItem(LS_ROBOT, $('robotSelect').value);
    };

    document.querySelectorAll('.map-toolbar button[data-mode]').forEach((btn) => {
      btn.addEventListener('click', () => {
        document.querySelectorAll('.map-toolbar button[data-mode]').forEach((b) => b.classList.remove('active'));
        btn.classList.add('active');
        PpMap.setMode(btn.dataset.mode);
        if (btn.dataset.mode !== 'measure') PpMap.clearMeasure();
      });
    });

    $('btnFitMap').onclick = () => PpMap.fit();
    $('btnConnect').onclick = connect;
    $('btnEstop').onclick = async () => {
      try { await PpApi.estop(true); } catch (e) { alert(e.message); }
    };
    $('btnResetOdom').onclick = async () => {
      try { await PpApi.resetOdom(); } catch (e) { alert(e.message); }
    };

    $('btnLoadMap').onclick = async () => {
      const path = $('mapSelect').value;
      if (!path) return;
      try {
        const info = await PpApi.fetchMap(path);
        PpMap.setMap(info);
        $('mapStatus').textContent = 'Loaded ' + (info.yaml_path || path);
        $('mapMetaInfo').textContent =
          `res=${info.resolution} origin=[${(info.origin || []).join(', ')}]`;
      } catch (e) {
        $('mapStatus').textContent = e.message;
      }
    };

    $('btnLoadLocalMap').onclick = async () => {
      const yf = $('mapFile').files[0];
      const pf = $('mapPgmFile').files[0];
      if (!yf || !pf) {
        $('mapStatus').textContent = 'Select both yaml and pgm';
        return;
      }
      const yamlText = await yf.text();
      const pgmBuf = await pf.arrayBuffer();
      PpMap.setLocalMap(yamlText, pgmBuf);
      $('mapStatus').textContent = `Local ${yf.name} + ${pf.name}`;
    };

    PpMap.onStickerAdd((x, y, kind) => {
      if (kind !== 'sticker' && kind !== 'station') return;
      const yawDeg = Number($('stickerYaw').value) || 0;
      const name = $('stickerName').value || `${kind}_${stickers.length + 1}`;
      stickers.push({
        name, kind, x, y, yaw: yawDeg * Math.PI / 180,
      });
      $('stickerName').value = `${kind}_${stickers.length + 1}`;
      saveStickers();
    });

    $('btnClearStickers').onclick = () => {
      stickers = [];
      saveStickers();
    };

    $('btnAddGoto').onclick = () => {
      const s = selectedSticker();
      if (!s) { alert('Select a sticker'); return; }
      steps.push({
        type: 'goto', x: s.x, y: s.y, yaw: s.yaw,
        xy_tol: 0.02, yaw_tol: 0.05,
      });
      saveSteps();
    };
    $('btnAddAlignCobra').onclick = () => {
      steps.push({ type: 'align', mode: 'cobra', timeout_sec: 8, confirm_frames: 20 });
      saveSteps();
    };
    $('btnAddAlignCam').onclick = () => {
      const colors = ($('alignColors').value || 'red').split(',').map((c) => c.trim()).filter(Boolean);
      steps.push({ type: 'align', mode: 'camera', colors, timeout_sec: 8 });
      saveSteps();
    };
    $('btnAddPick').onclick = () => {
      steps.push({
        type: 'pick', elevator: 'down', extend: 'extend', gripper: 'close',
        elevator_after: 'up',
      });
      saveSteps();
    };
    $('btnAddPlace').onclick = () => {
      steps.push({
        type: 'place', elevator: 'down', extend: 'extend', gripper: 'open',
        elevator_after: 'up',
      });
      saveSteps();
    };
    $('btnAddWait').onclick = () => {
      steps.push({ type: 'wait', sec: 1.0 });
      saveSteps();
    };
    $('btnClearSteps').onclick = () => { steps = []; saveSteps(); };
    $('btnStepUndo').onclick = () => { steps.pop(); saveSteps(); };

    $('btnSaveMission').onclick = () => {
      const task = buildTask();
      const all = savedMissions();
      all[task.name] = task;
      writeMissions(all);
    };
    $('btnLoadMission').onclick = () => {
      const name = $('missionSelect').value;
      const all = savedMissions();
      if (!name || !all[name]) return;
      const task = all[name];
      $('missionName').value = task.name || name;
      steps = task.steps || [];
      saveSteps();
    };
    $('btnDelMission').onclick = () => {
      const name = $('missionSelect').value;
      const all = savedMissions();
      delete all[name];
      writeMissions(all);
    };
    $('btnExport').onclick = () => {
      const task = buildTask();
      const blob = new Blob([JSON.stringify(task, null, 2)], { type: 'application/json' });
      const a = document.createElement('a');
      a.href = URL.createObjectURL(blob);
      a.download = (task.name || 'mission') + '.json';
      a.click();
    };
    $('btnImport').onclick = () => $('missionFile').click();
    $('missionFile').onchange = async () => {
      const f = $('missionFile').files[0];
      if (!f) return;
      const task = JSON.parse(await f.text());
      $('missionName').value = task.name || f.name.replace(/\.json$/, '');
      steps = task.steps || [];
      saveSteps();
    };

    $('btnMissionStart').onclick = async () => {
      try {
        const task = buildTask();
        await PpApi.setTask(task);
        await PpApi.startMission();
      } catch (e) { alert(e.message); }
    };
    $('btnMissionStop').onclick = async () => {
      try { await PpApi.stopMission(); } catch (e) { alert(e.message); }
    };

    const arm = (cmd) => async () => {
      try { await PpApi.manipulatorCmd(cmd); } catch (e) { alert(e.message); }
    };
    $('btnArmUp').onclick = arm({ action: 'elevator', elevator: 'up' });
    $('btnArmDown').onclick = arm({ action: 'elevator', elevator: 'down' });
    $('btnLinExt').onclick = arm({ action: 'linear', extend: 'extend' });
    $('btnLinRet').onclick = arm({ action: 'linear', extend: 'retract' });
    $('btnGripOpen').onclick = arm({ action: 'gripper', gripper: 'open' });
    $('btnGripClose').onclick = arm({ action: 'gripper', gripper: 'close' });

    $('btnCamToggle').onclick = () => {
      $('camFloat').classList.toggle('hidden');
      if (!$('camFloat').classList.contains('hidden')) startCam();
      else stopCam();
    };
    $('btnCamClose').onclick = () => {
      $('camFloat').classList.add('hidden');
      stopCam();
    };
  }

  function startCam() {
    stopCam();
    camTimer = setInterval(() => {
      const img = $('visionPreview');
      img.src = `${PpApi.getBaseUrl()}/api/vision/preview?t=${Date.now()}`;
    }, 400);
  }
  function stopCam() {
    if (camTimer) { clearInterval(camTimer); camTimer = null; }
  }

  // Init
  loadStickers();
  loadSteps();
  PpMap.bindEvents();
  PpMap.setStickers(stickers);
  renderStickers();
  renderSteps();
  refreshMissionSelect();
  wireUi();
})();
