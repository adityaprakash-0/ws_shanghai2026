/** Case-nav map — ADEV-style grid / zoom / pan + green lidar rays */
const CnMap = (() => {
  let canvas, ctx;
  let mapData = null;
  let mapBitmap = null;
  let pose = null;
  let path = [];
  let scan = [];
  let waypoints = [];
  let dims = { length: 0.4, width: 0.4 };
  let mode = 'goal'; // 'goal' | 'pose'
  let onCommit = null;
  let draggingPose = false;
  let panning = false;
  let draft = null;
  let scale = 1;
  let ox = 0;
  let oy = 0;
  let fitPending = false;
  let userPanned = false;
  let followRobot = false;
  let lastPanX = 0;
  let lastPanY = 0;

  function init(canvasEl, commitCb) {
    canvas = canvasEl;
    ctx = canvas.getContext('2d');
    onCommit = commitCb;
    bindEvents();
  }

  function bindEvents() {
    canvas.addEventListener('mousedown', (ev) => {
      // Alt / middle / right = pan (ADEV-style)
      if (ev.button === 1 || ev.button === 2 || ev.altKey) {
        panning = true;
        userPanned = true;
        followRobot = false;
        lastPanX = ev.clientX;
        lastPanY = ev.clientY;
        ev.preventDefault();
        return;
      }
      if (!mapData || !onCommit || ev.button !== 0) return;
      const w = worldFromEvent(ev);
      if (!w) return;
      draggingPose = true;
      draft = { x: w.x, y: w.y, yaw: 0, mode };
      draw();
      ev.preventDefault();
    });

    window.addEventListener('mousemove', (ev) => {
      if (panning) {
        ox += ev.clientX - lastPanX;
        oy += ev.clientY - lastPanY;
        lastPanX = ev.clientX;
        lastPanY = ev.clientY;
        draw();
        return;
      }
      if (!draggingPose || !draft) return;
      const w = worldFromEvent(ev);
      if (!w) return;
      const dx = w.x - draft.x;
      const dy = w.y - draft.y;
      if (Math.hypot(dx, dy) > 0.02) draft.yaw = Math.atan2(dy, dx);
      draw();
    });

    window.addEventListener('mouseup', () => {
      if (panning) {
        panning = false;
        return;
      }
      if (!draggingPose || !draft) return;
      draggingPose = false;
      const final = { ...draft };
      draft = null;
      draw();
      if (onCommit) onCommit(final.x, final.y, final.mode, final.yaw);
    });

    canvas.addEventListener('contextmenu', (e) => e.preventDefault());

    canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      if (!mapData) return;
      userPanned = true;
      followRobot = false;
      const factor = e.deltaY > 0 ? 0.9 : 1.1;
      const rect = canvas.getBoundingClientRect();
      const mx = (e.clientX - rect.left) * (canvas.width / rect.width);
      const my = (e.clientY - rect.top) * (canvas.height / rect.height);
      // Zoom around cursor in world meters (ox/oy are canvas offsets for world)
      const wx = (mx - ox) / scale;
      const wy = (oy - my) / scale;
      scale *= factor;
      ox = mx - wx * scale;
      oy = my + wy * scale;
      draw();
    }, { passive: false });

    if (typeof ResizeObserver !== 'undefined' && canvas.parentElement) {
      new ResizeObserver(() => {
        if (!userPanned) scheduleFit(followRobot);
        else {
          resizeCanvasOnly();
          draw();
        }
      }).observe(canvas.parentElement);
    }
    window.addEventListener('resize', () => {
      if (!userPanned) scheduleFit(followRobot);
      else {
        resizeCanvasOnly();
        draw();
      }
    });

    const btnFit = document.getElementById('btnFitMap');
    if (btnFit) {
      btnFit.onclick = () => {
        userPanned = false;
        followRobot = false;
        fit(false);
        draw();
      };
    }
    const btnCenter = document.getElementById('btnCenterRobot');
    if (btnCenter) {
      btnCenter.onclick = () => {
        followRobot = true;
        userPanned = false;
        fit(true);
        draw();
      };
    }
  }

  function resizeCanvasOnly() {
    if (!canvas || !canvas.parentElement) return;
    const parent = canvas.parentElement;
    const vw = Math.max(320, parent.clientWidth - 8);
    const vh = Math.max(320, parent.clientHeight - 48);
    canvas.width = vw;
    canvas.height = vh;
  }

  function worldFromEvent(ev) {
    const rect = canvas.getBoundingClientRect();
    const px = (ev.clientX - rect.left) * (canvas.width / rect.width);
    const py = (ev.clientY - rect.top) * (canvas.height / rect.height);
    return worldFromPixel(px, py);
  }

  function worldFromPixel(px, py) {
    if (!mapData) return null;
    return {
      x: (px - ox) / scale,
      y: (oy - py) / scale,
    };
  }

  function setMode(m) {
    mode = m;
    draggingPose = false;
    draft = null;
    draw();
  }
  function setDims(d) { if (d) dims = d; }
  function setPose(p) {
    pose = p;
    if (followRobot && !userPanned && pose) fit(true);
    draw();
  }
  function setPath(p) { path = p || []; draw(); }
  function setScan(s) { scan = s || []; draw(); }
  function setWaypoints(wps) { waypoints = wps || []; draw(); }
  function getWaypoints() { return waypoints.slice(); }
  function clearWaypoints() { waypoints = []; draw(); }

  function setMap(payload) {
    mapData = payload;
    buildBitmap();
    userPanned = false;
    followRobot = false;
    updateMapMetaLabel();
    scheduleFit(false);
  }

  function updateMapMetaLabel() {
    const el = document.getElementById('mapMetaInfo');
    if (!el || !mapData) return;
    const mw = (mapData.width * mapData.resolution).toFixed(2);
    const mh = (mapData.height * mapData.resolution).toFixed(2);
    el.textContent =
      `${mapData.width}×${mapData.height} cells · ${mw}×${mh} m · ${mapData.resolution} m/cell`;
  }

  function buildBitmap() {
    if (!mapData) { mapBitmap = null; return; }
    const w = mapData.width;
    const h = mapData.height;
    const tmp = document.createElement('canvas');
    tmp.width = w;
    tmp.height = h;
    const g = tmp.getContext('2d');
    const img = g.createImageData(w, h);
    for (let j = 0; j < h; j++) {
      for (let i = 0; i < w; i++) {
        const v = mapData.data[j * w + i];
        let r, gr, b;
        if (v < 0) { r = gr = b = 160; }
        else if (v >= 50) { r = gr = b = 0; }
        else { r = gr = b = 255; }
        const canvasRow = (h - 1 - j);
        const o = (canvasRow * w + i) * 4;
        img.data[o] = r;
        img.data[o + 1] = gr;
        img.data[o + 2] = b;
        img.data[o + 3] = 255;
      }
    }
    g.putImageData(img, 0, 0);
    mapBitmap = tmp;
  }

  function scheduleFit(centerRobot) {
    if (fitPending) return;
    fitPending = true;
    requestAnimationFrame(() => {
      fitPending = false;
      fit(!!centerRobot);
      draw();
    });
  }

  function fit(centerRobot) {
    if (!canvas || !canvas.parentElement || !mapData) return;
    resizeCanvasOnly();
    const vw = canvas.width;
    const vh = canvas.height;
    const margin = 12;
    const res = mapData.resolution;
    const mw = mapData.width * res;
    const mh = mapData.height * res;
    scale = Math.min((vw - margin * 2) / mw, (vh - margin * 2) / mh);

    if (centerRobot && pose) {
      ox = vw / 2 - pose.x * scale;
      oy = vh / 2 + pose.y * scale;
      return;
    }
    ox = (vw - mw * scale) / 2 - mapData.origin.x * scale;
    oy = (vh + mh * scale) / 2 + mapData.origin.y * scale;
  }

  function drawRobot(x, y, yaw, fill, stroke) {
    const px = ox + x * scale;
    const py = oy - y * scale;
    const L = dims.length * scale;
    const W = dims.width * scale;
    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(-yaw);
    ctx.fillStyle = fill;
    ctx.fillRect(-L / 2, -W / 2, L, W);
    ctx.strokeStyle = stroke;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(L / 2 + 8, 0);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(L / 2 + 8, 0);
    ctx.lineTo(L / 2 + 2, -4);
    ctx.lineTo(L / 2 + 2, 4);
    ctx.closePath();
    ctx.fillStyle = stroke;
    ctx.fill();
    ctx.restore();
  }

  function drawGrid() {
    if (!mapData) return;
    const res = mapData.resolution;
    const x0 = mapData.origin.x;
    const y0 = mapData.origin.y;
    const x1 = x0 + mapData.width * res;
    const y1 = y0 + mapData.height * res;

    // Fine cell grid when zoomed in (like ADEV)
    if (scale / res >= 2) {
      ctx.strokeStyle = 'rgba(0, 0, 0, 0.06)';
      ctx.lineWidth = 1;
      for (let i = 0; i <= mapData.width; i++) {
        const x = x0 + i * res;
        ctx.beginPath();
        ctx.moveTo(ox + x * scale, oy - y0 * scale);
        ctx.lineTo(ox + x * scale, oy - y1 * scale);
        ctx.stroke();
      }
      for (let j = 0; j <= mapData.height; j++) {
        const y = y0 + j * res;
        ctx.beginPath();
        ctx.moveTo(ox + x0 * scale, oy - y * scale);
        ctx.lineTo(ox + x1 * scale, oy - y * scale);
        ctx.stroke();
      }
    }

    // Major ~0.5 m grid (India green tint like ADEV)
    const major = 0.5;
    ctx.strokeStyle = 'rgba(19, 136, 8, 0.22)';
    ctx.lineWidth = 1;
    const xStart = Math.ceil(x0 / major) * major;
    for (let x = xStart; x <= x1; x += major) {
      ctx.beginPath();
      ctx.moveTo(ox + x * scale, oy - y0 * scale);
      ctx.lineTo(ox + x * scale, oy - y1 * scale);
      ctx.stroke();
    }
    const yStart = Math.ceil(y0 / major) * major;
    for (let y = yStart; y <= y1; y += major) {
      ctx.beginPath();
      ctx.moveTo(ox + x0 * scale, oy - y * scale);
      ctx.lineTo(ox + x1 * scale, oy - y * scale);
      ctx.stroke();
    }
  }

  function drawScan() {
    if (!pose || !scan.length) return;
    const rx = ox + pose.x * scale;
    const ry = oy - pose.y * scale;
    ctx.strokeStyle = 'rgba(19, 136, 8, 0.55)';
    ctx.lineWidth = 1.2;
    scan.forEach((pt) => {
      const a = pose.yaw + pt.a;
      const wx = pose.x + pt.r * Math.cos(a);
      const wy = pose.y + pt.r * Math.sin(a);
      ctx.beginPath();
      ctx.moveTo(rx, ry);
      ctx.lineTo(ox + wx * scale, oy - wy * scale);
      ctx.stroke();
    });
  }

  function draw() {
    if (!ctx || !canvas) return;
    ctx.fillStyle = '#e4e8ec';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    if (!mapData || !mapBitmap) {
      ctx.fillStyle = '#666';
      ctx.font = '15px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('Load a map to begin', canvas.width / 2, canvas.height / 2);
      ctx.textAlign = 'start';
      return;
    }

    const res = mapData.resolution;
    const left = ox + mapData.origin.x * scale;
    const top = oy - (mapData.origin.y + mapData.height * res) * scale;
    const dw = mapData.width * res * scale;
    const dh = mapData.height * res * scale;
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(mapBitmap, left, top, dw, dh);

    drawGrid();

    if (path.length > 1) {
      ctx.strokeStyle = 'rgba(0, 0, 128, 0.85)';
      ctx.lineWidth = 3;
      ctx.beginPath();
      path.forEach((p, i) => {
        const sx = ox + p.x * scale;
        const sy = oy - p.y * scale;
        if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
      });
      ctx.stroke();
    }

    drawScan();

    waypoints.forEach((wp, i) => {
      drawRobot(wp.x, wp.y, wp.yaw || 0, 'rgba(232,93,76,0.55)', '#e85d4c');
      const sx = ox + wp.x * scale;
      const sy = oy - wp.y * scale;
      ctx.fillStyle = '#e85d4c';
      ctx.beginPath();
      ctx.arc(sx, sy - 14, 9, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = '#fff';
      ctx.font = 'bold 11px sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(String(i + 1), sx, sy - 14);
      ctx.textAlign = 'start';
      ctx.textBaseline = 'alphabetic';
    });

    if (pose) {
      drawRobot(pose.x, pose.y, pose.yaw, 'rgba(0,0,128,0.75)', '#ff9933');
    }

    if (draft) {
      const isGoal = draft.mode === 'goal';
      drawRobot(
        draft.x, draft.y, draft.yaw,
        isGoal ? 'rgba(232,93,76,0.75)' : 'rgba(255,153,51,0.75)',
        isGoal ? '#e85d4c' : '#ff9933');
      const deg = (draft.yaw * 180 / Math.PI).toFixed(0);
      ctx.fillStyle = '#000080';
      ctx.font = '12px sans-serif';
      ctx.fillText(
        `${isGoal ? 'GOAL' : 'POSE'} yaw ${deg}° (drag to aim)`,
        ox + draft.x * scale + 12,
        oy - draft.y * scale - 12);
    }
  }

  return {
    init, setMap, setPose, setPath, setScan, setWaypoints, getWaypoints,
    clearWaypoints, setMode, setDims, fit, draw,
  };
})();
