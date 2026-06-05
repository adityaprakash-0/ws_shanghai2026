/** Task Builder — stackable navigate / look_color / wait blocks → mission steps.
 *  Tasks are saved in the browser (localStorage) by name; Export/Import handles .json files.
 */
const TaskBuilder = (() => {
  const COLORS = ['red', 'orange', 'yellow', 'green', 'blue', 'purple', 'white', 'black'];
  const LS_TASKS = 'adev_tasks';
  let steps = [];
  let previewTimer = null;

  function $(id) { return document.getElementById(id); }

  function mapWaypoints() {
    try {
      return MapView.getWaypoints() || [];
    } catch (_) {
      return [];
    }
  }

  // ---------------------------------------------------------------- steps
  function getSteps() { return steps.map((s) => ({ ...s })); }

  function setSteps(next) {
    steps = Array.isArray(next) ? next.map((s) => ({ ...s })) : [];
    render();
  }

  function clear() {
    steps = [];
    render();
  }

  function addNavigate(wpIndex) {
    const wps = mapWaypoints();
    const wp = wps[wpIndex];
    if (!wp) {
      alert('Mark waypoints on the map first (+ Waypoint), then click a WP number');
      return;
    }
    steps.push({
      type: 'navigate',
      x: wp.x,
      y: wp.y,
      yaw: wp.yaw || 0,
      ignore_yaw: !!wp.ignore_yaw,
      label: `WP${wpIndex + 1}`,
    });
    render();
  }

  function addLookColor() {
    steps.push({
      type: 'look_color',
      colors: [($('taskQuickColor') && $('taskQuickColor').value) || 'red'],
      timeout_sec: 5,
      retry_count: 0,
      on_found: 'continue',
      on_miss: 'continue',
      label: 'Look color',
    });
    render();
  }

  function addWait() {
    steps.push({ type: 'wait', seconds: 2, label: 'Wait' });
    render();
  }

  function addAllWpThenLook(color) {
    const wps = mapWaypoints();
    if (!wps.length) {
      alert('Mark waypoints on the map first (+ Waypoint)');
      return;
    }
    const c = color || 'red';
    wps.forEach((wp, i) => {
      steps.push({
        type: 'navigate',
        x: wp.x,
        y: wp.y,
        yaw: wp.yaw || 0,
        ignore_yaw: !!wp.ignore_yaw,
        label: `WP${i + 1}`,
      });
      steps.push({
        type: 'look_color',
        colors: [c],
        timeout_sec: 5,
        retry_count: 0,
        on_found: 'continue',
        on_miss: 'continue',
        label: `Look ${c}`,
      });
    });
    render();
  }

  function move(i, dir) {
    const j = i + dir;
    if (j < 0 || j >= steps.length) return;
    const tmp = steps[i];
    steps[i] = steps[j];
    steps[j] = tmp;
    render();
  }

  function remove(i) {
    steps.splice(i, 1);
    render();
  }

  function buildTask() {
    const name = ($('taskName') && $('taskName').value.trim()) || 'task';
    return {
      version: 1,
      name,
      steps: steps.map((s) => {
        const out = { type: s.type };
        if (s.type === 'navigate') {
          out.x = s.x;
          out.y = s.y;
          out.yaw = s.yaw || 0;
          out.ignore_yaw = !!s.ignore_yaw;
          if (s.label) out.label = s.label;
        } else if (s.type === 'look_color') {
          out.colors = [...(s.colors || [])];
          out.timeout_sec = Number(s.timeout_sec) || 5;
          out.retry_count = Math.max(0, Number(s.retry_count) || 0);
          out.on_found = s.on_found || 'continue';
          out.on_miss = s.on_miss || 'continue';
          if (s.label) out.label = s.label;
        } else if (s.type === 'wait') {
          out.seconds = Number(s.seconds) || 2;
          if (s.label) out.label = s.label;
        }
        return out;
      }),
    };
  }

  // ------------------------------------------------ task library (browser)
  function loadTaskLibrary() {
    try {
      return JSON.parse(localStorage.getItem(LS_TASKS) || '{}');
    } catch (_) {
      return {};
    }
  }

  function refreshTaskSelect() {
    const sel = $('taskSelect');
    if (!sel) return;
    const lib = loadTaskLibrary();
    const current = sel.value;
    sel.innerHTML = '<option value="">— saved tasks —</option>' +
      Object.keys(lib).sort().map((n) => `<option value="${n}">${n}</option>`).join('');
    if (lib[current]) sel.value = current;
  }

  function saveTaskToBrowser() {
    const name = ($('taskName') && $('taskName').value.trim()) || '';
    if (!name) { alert('Enter a task name first (e.g. find_red_loop)'); return; }
    if (!steps.length) { alert('Add some blocks first'); return; }
    const lib = loadTaskLibrary();
    lib[name] = buildTask();
    localStorage.setItem(LS_TASKS, JSON.stringify(lib));
    refreshTaskSelect();
    const sel = $('taskSelect');
    if (sel) sel.value = name;
    setStatus(`Saved "${name}" in browser (${steps.length} blocks)`);
  }

  function loadTaskFromBrowser() {
    const sel = $('taskSelect');
    const name = sel && sel.value;
    if (!name) { alert('Pick a saved task from the dropdown first'); return; }
    const task = loadTaskLibrary()[name];
    if (!task) { alert(`Task "${name}" not found`); return; }
    if ($('taskName')) $('taskName').value = task.name || name;
    setSteps(task.steps || []);
    setStatus(`Loaded "${name}" (${steps.length} blocks)`);
  }

  function deleteTaskFromBrowser() {
    const sel = $('taskSelect');
    const name = sel && sel.value;
    if (!name) { alert('Pick a saved task to delete'); return; }
    const lib = loadTaskLibrary();
    delete lib[name];
    localStorage.setItem(LS_TASKS, JSON.stringify(lib));
    refreshTaskSelect();
    setStatus(`Deleted "${name}"`);
  }

  function setStatus(msg) {
    const el = $('taskStatus');
    if (el) el.textContent = msg;
  }

  // ------------------------------------------------------- file export/import
  function exportJson() {
    if (!steps.length) { alert('Add some blocks first'); return; }
    const task = buildTask();
    const blob = new Blob([JSON.stringify(task, null, 2)], { type: 'application/json' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = `${task.name.replace(/\s+/g, '_') || 'task'}.json`;
    a.click();
    URL.revokeObjectURL(a.href);
    setStatus(`Exported ${a.download} → your Downloads folder`);
  }

  function importJsonFile(file) {
    const reader = new FileReader();
    reader.onload = () => {
      try {
        const data = JSON.parse(reader.result);
        const list = data.steps || data;
        if ($('taskName') && data.name) $('taskName').value = data.name;
        setSteps(list);
        setStatus(`Imported ${file.name} (${steps.length} blocks)`);
      } catch (e) {
        alert('Invalid task JSON: ' + e.message);
      }
    };
    reader.readAsText(file);
  }

  // ----------------------------------------------------------------- render
  function refreshWaypoints() {
    const box = $('taskWpPicker');
    if (!box) return;
    const wps = mapWaypoints();
    if (!wps.length) {
      box.innerHTML = '<span class="seq-hint">No waypoints yet — use “+ Waypoint” on the map</span>';
      return;
    }
    box.innerHTML = wps.map((_, i) =>
      `<button type="button" data-wp="${i}" title="Add Navigate WP${i + 1} block">${i + 1}</button>`).join('');
    box.querySelectorAll('button').forEach((btn) => {
      btn.onclick = () => addNavigate(Number(btn.dataset.wp));
    });
  }

  function render() {
    refreshWaypoints();
    const box = $('taskBlocks');
    if (!box) return;
    if (!steps.length) {
      box.innerHTML = '<p class="seq-hint">No blocks yet. Click a WP number above, or “+ Look color” / “+ Wait”.</p>';
      return;
    }
    box.innerHTML = steps.map((s, i) => {
      if (s.type === 'navigate') {
        return `<div class="task-block nav" data-i="${i}">
          <div class="task-block-head">
            <span>${i + 1}. Navigate ${s.label || ''}</span>
            <span class="task-block-ops">
              <button type="button" data-act="up">↑</button>
              <button type="button" data-act="down">↓</button>
              <button type="button" data-act="del">✕</button>
            </span>
          </div>
          <div class="task-block-body">(${(s.x || 0).toFixed(2)}, ${(s.y || 0).toFixed(2)}) yaw=${((s.yaw || 0) * 180 / Math.PI).toFixed(0)}°</div>
        </div>`;
      }
      if (s.type === 'look_color') {
        const opts = COLORS.map((c) =>
          `<label class="task-color"><input type="checkbox" data-color="${c}" ${
            (s.colors || []).includes(c) ? 'checked' : ''}>${c}</label>`).join('');
        return `<div class="task-block cam" data-i="${i}">
          <div class="task-block-head">
            <span>${i + 1}. 📷 Look color</span>
            <span class="task-block-ops">
              <button type="button" data-act="up">↑</button>
              <button type="button" data-act="down">↓</button>
              <button type="button" data-act="del">✕</button>
            </span>
          </div>
          <div class="task-block-body">
            <div class="task-colors">${opts}</div>
            <label>Timeout <input type="number" data-field="timeout_sec" value="${s.timeout_sec || 5}" min="1" max="30" step="1" style="width:3.5rem">s</label>
            <label>Retries after no match
              <input type="number" data-field="retry_count" value="${Math.max(0, Number(s.retry_count) || 0)}" min="0" max="20" step="1" style="width:3.5rem">
            </label>
            <label>If found
              <select data-field="on_found">
                <option value="continue" ${s.on_found !== 'stop' ? 'selected' : ''}>continue</option>
                <option value="stop" ${s.on_found === 'stop' ? 'selected' : ''}>stop mission</option>
              </select>
            </label>
            <label>If miss
              <select data-field="on_miss">
                <option value="continue" ${s.on_miss !== 'stop' ? 'selected' : ''}>continue</option>
                <option value="stop" ${s.on_miss === 'stop' ? 'selected' : ''}>stop mission</option>
              </select>
            </label>
          </div>
        </div>`;
      }
      return `<div class="task-block wait" data-i="${i}">
        <div class="task-block-head">
          <span>${i + 1}. ⏱ Wait</span>
          <span class="task-block-ops">
            <button type="button" data-act="up">↑</button>
            <button type="button" data-act="down">↓</button>
            <button type="button" data-act="del">✕</button>
          </span>
        </div>
        <div class="task-block-body">
          <label>Seconds <input type="number" data-field="seconds" value="${s.seconds || 2}" min="0" max="60" step="0.5" style="width:3.5rem"></label>
        </div>
      </div>`;
    }).join('');

    box.querySelectorAll('.task-block').forEach((el) => {
      const i = Number(el.dataset.i);
      el.querySelectorAll('[data-act]').forEach((btn) => {
        btn.onclick = () => {
          const act = btn.dataset.act;
          if (act === 'up') move(i, -1);
          else if (act === 'down') move(i, 1);
          else if (act === 'del') remove(i);
        };
      });
      el.querySelectorAll('[data-color]').forEach((cb) => {
        cb.onchange = () => {
          const selected = [...el.querySelectorAll('[data-color]:checked')].map((c) => c.dataset.color);
          steps[i].colors = selected.length ? selected : ['red'];
        };
      });
      el.querySelectorAll('[data-field]').forEach((inp) => {
        inp.onchange = () => {
          const f = inp.dataset.field;
          if (f === 'retry_count') steps[i][f] = Math.max(0, parseInt(inp.value, 10) || 0);
          else if (f === 'timeout_sec' || f === 'seconds') steps[i][f] = parseFloat(inp.value) || 0;
          else steps[i][f] = inp.value;
        };
      });
    });
  }

  // ------------------------------------------------ floating camera panel
  function startPreview() {
    stopPreview();
    const img = $('visionPreview');
    if (!img) return;
    const tick = () => {
      img.src = `${AdevApi.getBaseUrl()}/api/vision/preview?t=${Date.now()}`;
    };
    tick();
    previewTimer = setInterval(tick, 400);
  }

  function stopPreview() {
    if (previewTimer) {
      clearInterval(previewTimer);
      previewTimer = null;
    }
  }

  function showCamPanel(show) {
    $('camFloat')?.classList.toggle('hidden', !show);
    $('btnCamToggle')?.classList.toggle('active', show);
    if (!show) {
      stopPreview();
      AdevApi.visionGate(false).catch(() => {});
    }
  }

  // -------------------------------------------------------------------- bind
  function bind() {
    $('btnTaskAddCam')?.addEventListener('click', () => addLookColor());
    $('btnTaskAddWait')?.addEventListener('click', () => addWait());
    $('btnTaskAllLook')?.addEventListener('click', () => {
      const c = ($('taskQuickColor') && $('taskQuickColor').value) || 'red';
      addAllWpThenLook(c);
    });
    $('btnTaskClear')?.addEventListener('click', () => clear());

    $('btnTaskSave')?.addEventListener('click', () => saveTaskToBrowser());
    $('btnTaskLoadSel')?.addEventListener('click', () => loadTaskFromBrowser());
    $('btnTaskDelete')?.addEventListener('click', () => deleteTaskFromBrowser());

    $('btnTaskExport')?.addEventListener('click', () => exportJson());
    $('btnTaskImport')?.addEventListener('click', () => $('taskFile')?.click());
    $('taskFile')?.addEventListener('change', (e) => {
      const f = e.target.files && e.target.files[0];
      if (f) importJsonFile(f);
      e.target.value = '';
    });

    $('toggleTaskBuilder')?.addEventListener('click', () => {
      const on = !$('toggleTaskBuilder').classList.contains('on');
      $('toggleTaskBuilder').classList.toggle('on', on);
      $('taskBuilderPanel')?.classList.toggle('hidden', !on);
      localStorage.setItem('adev_task_builder', on ? '1' : '0');
      if (on) render();
    });

    $('btnCamToggle')?.addEventListener('click', () => {
      const show = $('camFloat')?.classList.contains('hidden');
      showCamPanel(show);
    });
    $('btnCamClose')?.addEventListener('click', () => showCamPanel(false));
    $('btnVisionGateOn')?.addEventListener('click', async () => {
      try {
        await AdevApi.visionGate(true);
        startPreview();
      } catch (e) { alert('Camera: ' + e.message); }
    });
    $('btnVisionGateOff')?.addEventListener('click', async () => {
      stopPreview();
      try { await AdevApi.visionGate(false); } catch (_) { /* */ }
    });

    refreshTaskSelect();
    if (localStorage.getItem('adev_task_builder') === '1') {
      $('toggleTaskBuilder')?.classList.add('on');
      $('taskBuilderPanel')?.classList.remove('hidden');
      render();
    }
  }

  function isEnabled() {
    return !!$('toggleTaskBuilder')?.classList.contains('on') && steps.length > 0;
  }

  return {
    bind, getSteps, setSteps, clear, buildTask, isEnabled,
    refreshWaypoints, startPreview, stopPreview, render, COLORS,
  };
})();
