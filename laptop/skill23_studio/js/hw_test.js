/** Hardware Testing Software dashboard */
(() => {
  const $ = (id) => document.getElementById(id);
  const params = new URLSearchParams(window.location.search);
  const state = {
    robot: params.get('robot') || 'allterrain',
    connected: false,
    running: false,
    freeze: false,
    showExpected: false,
    notes: [],
    lastLive: null,
  };

  const GROUP_ORDER = [
    'battery', 'lidar', 'imu', 'titan', 'servo',
    'ultrasonic', 'ir', 'cobra', 'dio', 'encoder', 'light_tower',
  ];

  function note(line) {
    const stamp = new Date().toLocaleTimeString();
    state.notes.push(`${stamp} ${line}`);
    if (state.notes.length > 80) state.notes = state.notes.slice(-60);
    const el = $('hwLog');
    if (el) el.textContent = state.notes.join('\n');
  }

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
      connectRow.hidden = ok || params.get('from') === 'studio';
    }
  }

  function setRunning(on, message) {
    state.running = !!on;
    document.body.classList.toggle('ready-on', state.running);
    $('readyPill').textContent = state.running ? 'HW running' : 'Stopped';
    if (message) $('runHint').textContent = message;
  }

  function selectRobot(robot) {
    state.robot = robot;
    document.querySelectorAll('#robotChips .chip').forEach((b) => {
      b.classList.toggle('active', b.dataset.robot === robot);
    });
  }

  function fmtVal(v, unit) {
    if (v == null) return '—';
    if (typeof v === 'boolean') return v ? 'TRUE' : 'FALSE';
    if (typeof v === 'number') {
      const s = Number.isInteger(v) ? String(v) : v.toFixed(Math.abs(v) >= 10 ? 2 : 3);
      return unit ? `${s} ${unit}` : s;
    }
    return String(v);
  }

  function renderGroups(live) {
    const root = $('hwGroups');
    if (!root) return;
    const groups = (live && live.groups) || {};
    const keys = Object.keys(groups).sort((a, b) => {
      const ia = GROUP_ORDER.indexOf(a);
      const ib = GROUP_ORDER.indexOf(b);
      return (ia < 0 ? 99 : ia) - (ib < 0 ? 99 : ib);
    });
    if (!keys.length) {
      root.innerHTML = '<p class="hint">No live publishes yet — wait for Titan/lidars, or enable sensors in hw_params.</p>';
      return;
    }
    const html = keys.map((g) => {
      const rows = groups[g] || [];
      const liveN = rows.filter((r) => r.live).length;
      const cells = rows.map((r) => {
        const boolClass = typeof r.value === 'boolean'
          ? (r.value ? ' bool-true' : ' bool-false')
          : '';
        const stale = r.live ? '' : ' stale';
        return [
          `<div class="hw-cell${stale}${boolClass}" title="${r.topic}">`,
          `<div class="lbl">${r.label}</div>`,
          `<div class="val">${fmtVal(r.value, r.unit)}</div>`,
          `<div class="meta">${r.topic} · ${r.hz || 0} Hz · age ${r.age_s ?? '—'}s</div>`,
          '</div>',
        ].join('');
      }).join('');
      return [
        `<section class="hw-group" data-group="${g}">`,
        `<div class="hw-group-head"><h3>${g}</h3><span>${liveN}/${rows.length} live</span></div>`,
        `<div class="hw-grid">${cells}</div>`,
        '</section>',
      ].join('');
    }).join('');
    root.innerHTML = html;

    const liveTotal = live.live_channels || 0;
    $('statsLine').textContent = `Live: ${liveTotal} · robot ${live.robot || state.robot}`;
  }

  function onTelemetry(t) {
    if (!state.connected) setConnected(true);
    const stacks = t.stacks || {};
    const on = !!stacks.hw_test;
    if (on !== state.running) {
      setRunning(on, on
        ? 'HW stack up — showing only publishing topics.'
        : 'HW stack stopped.');
    }
    if (state.freeze) return;
    const live = t.hw_test;
    if (live && live.active) {
      state.lastLive = live;
      if (!state.showExpected) renderGroups(live);
    }
  }

  async function refreshLive() {
    if (!state.connected || state.freeze) return;
    try {
      const live = state.showExpected
        ? await StudioApi.hwTestChannels(state.robot)
        : await StudioApi.hwTestLive();
      if (state.showExpected && live.expected) {
        // Merge expected specs with live values for debugging missing pubs.
        const byId = {};
        const groups = ((live.live && live.live.groups) || {});
        Object.values(groups).forEach((rows) => {
          rows.forEach((r) => { byId[r.id] = r; });
        });
        const merged = { robot: live.robot, live_channels: 0, groups: {} };
        (live.expected || []).forEach((spec) => {
          const cur = byId[spec.id];
          const row = cur || {
            id: spec.id,
            group: spec.group,
            label: spec.label,
            topic: spec.topic,
            unit: spec.unit,
            value: null,
            age_s: null,
            hz: 0,
            live: false,
          };
          if (row.live) merged.live_channels += 1;
          merged.groups[spec.group] = merged.groups[spec.group] || [];
          merged.groups[spec.group].push(row);
        });
        renderGroups(merged);
      } else {
        state.lastLive = live;
        renderGroups(live);
      }
    } catch (e) {
      /* ignore poll errors */
    }
  }

  async function connect() {
    try {
      await StudioApi.status();
      setConnected(true);
      StudioApi.connectStream(onTelemetry);
      note(`Linked ${StudioApi.getBaseUrl()}`);
      setRunning(false, 'Select robot → Start HW. Stop other stacks (Nav2/OMS) first.');
    } catch (e) {
      setConnected(false);
      note(`Connect failed: ${e.message || e}`);
    }
  }

  document.querySelectorAll('#robotChips .chip').forEach((btn) => {
    btn.addEventListener('click', () => {
      if (state.running) {
        return alert('Stop HW before changing robot type.');
      }
      selectRobot(btn.dataset.robot);
    });
  });
  selectRobot(state.robot);

  $('btnHwStart').onclick = async () => {
    if (!state.connected) return alert('Connect from Studio first.');
    try {
      const res = await StudioApi.hwTestStart({
        robot: state.robot,
        enable_lidar: !!($('chkLidar') && $('chkLidar').checked),
      });
      setRunning(true, res.message || 'HW starting…');
      note(res.message || 'HW start');
      setTimeout(refreshLive, 2500);
    } catch (e) {
      alert(e.message || String(e));
      note(`Start failed: ${e.message || e}`);
    }
  };

  $('btnHwStop').onclick = async () => {
    try {
      const res = await StudioApi.hwTestStop();
      setRunning(false, res.message || 'Stopped');
      note(res.message || 'HW stop');
      $('hwGroups').innerHTML = '<p class="hint">Stopped.</p>';
    } catch (e) {
      alert(e.message || String(e));
    }
  };

  $('btnFreeze').onclick = () => {
    state.freeze = !state.freeze;
    $('btnFreeze').textContent = state.freeze ? 'Unfreeze' : 'Freeze UI';
    note(state.freeze ? 'UI frozen' : 'UI live');
  };

  $('btnShowStale').onclick = () => {
    state.showExpected = !state.showExpected;
    $('btnShowStale').textContent = state.showExpected ? 'Live only' : 'Show expected';
    refreshLive();
  };

  $('btnCopy').onclick = async () => {
    const payload = state.lastLive || await StudioApi.hwTestLive().catch(() => ({}));
    try {
      await navigator.clipboard.writeText(JSON.stringify(payload, null, 2));
      note('Snapshot copied to clipboard');
    } catch (e) {
      note(`Copy failed: ${e.message || e}`);
    }
  };

  $('btnClearLog').onclick = () => {
    state.notes = [];
    $('hwLog').textContent = '—';
  };

  if ($('btnConnect')) {
    $('btnConnect').onclick = () => {
      const url = ($('piUrl') && $('piUrl').value) || '';
      if (url) StudioApi.setBaseUrl(url);
      connect();
    };
  }

  // Linked from Studio — same pattern as OMS.
  if (params.get('pi')) {
    StudioApi.setBaseUrl(params.get('pi'));
  }
  if (params.get('from') === 'studio' || params.get('pi')) {
    connect();
  } else {
    setConnected(false);
  }

  setInterval(refreshLive, 500);
})();
