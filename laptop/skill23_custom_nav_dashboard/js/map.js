/** Custom-Nav map — ADEV-style grid / zoom / pan / Shift+drag rotate + green lidar */
const CnMap = (() => {
  let canvas, ctx;
  let mapData = null;
  let mapBitmap = null;
  let pose = null;
  let path = [];
  let plannerPath = [];
  let smoothedPath = [];
  let pathStyle = 'smoothed'; // 'normal' | 'smoothed'
  let scan = [];
  let waypoints = [];
  let dims = { length: 0.4, width: 0.4 };
  let mode = 'goal'; // 'goal' | 'pose' | 'sim_start' | 'sim_end'
  let onCommit = null;
  let onSimPoint = null;
  let onHoverCoords = null;
  let draggingPose = false;
  let panning = false;
  let rotating = false;
  let draft = null;
  let scale = 1;
  let ox = 0;
  let oy = 0;
  let viewRot = 0;
  let fitPending = false;
  let userPanned = false;
  let followRobot = false;
  let lastPanX = 0;
  let lastPanY = 0;
  let simStart = null;
  let simEnd = null;
  let simPath = [];
  let simPlannerPath = [];
  let simSmoothedPath = [];

  function init(canvasEl, commitCb) {
    canvas = canvasEl;
    ctx = canvas.getContext('2d');
    onCommit = commitCb;
    bindEvents();
  }

  function bindEvents() {
    canvas.addEventListener('mousedown', (ev) => {
      if (ev.shiftKey && ev.button === 0) {
        rotating = true;
        userPanned = true;
        followRobot = false;
        lastPanX = ev.clientX;
        lastPanY = ev.clientY;
        ev.preventDefault();
        return;
      }
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

      if (mode === 'sim_start' || mode === 'sim_end') {
        const pt = { x: +w.x.toFixed(3), y: +w.y.toFixed(3) };
        if (mode === 'sim_start') simStart = pt;
        else simEnd = pt;
        if (onSimPoint) onSimPoint(mode, pt, simStart, simEnd);
        draw();
        return;
      }

      draggingPose = true;
      draft = { x: w.x, y: w.y, yaw: 0, mode };
      draw();
      ev.preventDefault();
    });

    window.addEventListener('mousemove', (ev) => {
      if (rotating) {
        const rect = canvas.getBoundingClientRect();
        const cx = rect.left + rect.width / 2;
        const cy = rect.top + rect.height / 2;
        const a0 = Math.atan2(lastPanY - cy, lastPanX - cx);
        const a1 = Math.atan2(ev.clientY - cy, ev.clientX - cx);
        viewRot += a1 - a0;
        lastPanX = ev.clientX;
        lastPanY = ev.clientY;
        draw();
        return;
      }
      if (panning) {
        const dx = ev.clientX - lastPanX;
        const dy = ev.clientY - lastPanY;
        const c = Math.cos(-viewRot);
        const s = Math.sin(-viewRot);
        ox += dx * c - dy * s;
        oy += dx * s + dy * c;
        lastPanX = ev.clientX;
        lastPanY = ev.clientY;
        draw();
        return;
      }
      if (draggingPose && draft) {
        const w = worldFromEvent(ev);
        if (!w) return;
        const dx = w.x - draft.x;
        const dy = w.y - draft.y;
        if (Math.hypot(dx, dy) > 0.02) draft.yaw = Math.atan2(dy, dx);
        draw();
        return;
      }
      if (onHoverCoords && mapData) {
        const w = worldFromEvent(ev);
        if (w) onHoverCoords(w.x, w.y);
      }
    });

    window.addEventListener('mouseup', () => {
      if (panning) { panning = false; return; }
      if (rotating) { rotating = false; return; }
      if (!draggingPose || !draft) return;
      draggingPose = false;
      const final = { ...draft };
      draft = null;
      draw();
      if (onCommit) onCommit(final.x, final.y, final.mode, final.yaw);
    });

    canvas.addEventListener('mouseleave', () => {
      if (onHoverCoords) onHoverCoords(null, null);
    });

    canvas.addEventListener('contextmenu', (e) => e.preventDefault());

    canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      if (!mapData) return;
      userPanned = true;
      followRobot = false;
      const factor = e.deltaY > 0 ? 0.9 : 1.1;
      const rect = canvas.getBoundingClientRect();
      const cx = canvas.width / 2;
      const cy = canvas.height / 2;
      const sx = (e.clientX - rect.left) * (canvas.width / rect.width) - cx;
      const sy = (e.clientY - rect.top) * (canvas.height / rect.height) - cy;
      const cos = Math.cos(-viewRot);
      const sin = Math.sin(-viewRot);
      const mx = sx * cos - sy * sin + cx;
      const my = sx * sin + sy * cos + cy;
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
        viewRot = 0;
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

  function screenToOffset(ev) {
    const rect = canvas.getBoundingClientRect();
    const sx = (ev.clientX - rect.left) * (canvas.width / rect.width);
    const sy = (ev.clientY - rect.top) * (canvas.height / rect.height);
    const cx = canvas.width / 2;
    const cy = canvas.height / 2;
    const dx = sx - cx;
    const dy = sy - cy;
    const c = Math.cos(-viewRot);
    const s = Math.sin(-viewRot);
    return {
      px: dx * c - dy * s + cx,
      py: dx * s + dy * c + cy,
    };
  }

  function worldFromEvent(ev) {
    const { px, py } = screenToOffset(ev);
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
  function setPathLayers(planner, smoothed, style) {
    plannerPath = planner || [];
    smoothedPath = smoothed || [];
    if (style === 'normal' || style === 'smoothed') pathStyle = style;
    draw();
  }
  function setPathStyle(style) {
    if (style === 'normal' || style === 'smoothed') pathStyle = style;
    draw();
  }
  function setScan(s) { scan = s || []; draw(); }
  function setWaypoints(wps) { waypoints = wps || []; draw(); }
  function getWaypoints() { return waypoints.slice(); }
  function clearWaypoints() { waypoints = []; draw(); }

  function decodePgmBytes(bytes) {
    let i = 0;
    const readLine = () => {
      const start = i;
      while (i < bytes.length && bytes[i] !== 0x0a) i++;
      const line = new TextDecoder().decode(bytes.subarray(start, i)).trim();
      i++;
      return line;
    };
    let magic = readLine();
    while (magic.startsWith('#') || magic === '') magic = readLine();
    let dimsLine = readLine();
    while (dimsLine.startsWith('#') || dimsLine === '') dimsLine = readLine();
    const [w, h] = dimsLine.split(/\s+/).map(Number);
    let maxval = readLine();
    while (maxval.startsWith('#') || maxval === '') maxval = readLine();
    return { width: w, height: h, data: bytes.subarray(i, i + w * h) };
  }

  /** Convert laptop yaml+pgm into OccupancyGrid-like payload for setMap. */
  function setMapFromLocal(meta, pgmBytes) {
    const pgm = decodePgmBytes(pgmBytes);
    const w = pgm.width;
    const h = pgm.height;
    const occThresh = meta.occupied_thresh ?? 0.65;
    const freeThresh = meta.free_thresh ?? 0.25;
    const negate = meta.negate ? 1 : 0;
    const modeName = meta.mode || 'trinary';
    const data = new Array(w * h);
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const v = pgm.data[y * w + x];
        let occ;
        if (modeName === 'trinary') {
          if (v >= 250) occ = 0;
          else if (v <= 25) occ = 100;
          else occ = -1;
        } else {
          const p = negate ? v / 255.0 : (255 - v) / 255.0;
          if (p >= occThresh) occ = 100;
          else if (p <= freeThresh) occ = 0;
          else occ = -1;
        }
        // OccupancyGrid row 0 = bottom; PGM row 0 = top
        data[(h - 1 - y) * w + x] = occ;
      }
    }
    const origin = Array.isArray(meta.origin) ? meta.origin : [0, 0, 0];
    setMap({
      resolution: Number(meta.resolution) || 0.05,
      width: w,
      height: h,
      origin: { x: origin[0] || 0, y: origin[1] || 0 },
      data,
    });
  }

  function setMap(payload) {
    mapData = payload;
    buildBitmap();
    userPanned = false;
    followRobot = false;
    viewRot = 0;
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

    ctx.save();
    ctx.translate(canvas.width / 2, canvas.height / 2);
    ctx.rotate(viewRot);
    ctx.translate(-canvas.width / 2, -canvas.height / 2);

    const res = mapData.resolution;
    const left = ox + mapData.origin.x * scale;
    const top = oy - (mapData.origin.y + mapData.height * res) * scale;
    const dw = mapData.width * res * scale;
    const dh = mapData.height * res * scale;
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(mapBitmap, left, top, dw, dh);

    drawGrid();

    function strokePath(pts, color, width) {
      if (!pts || pts.length < 2) return;
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.beginPath();
      pts.forEach((p, i) => {
        const sx = ox + p.x * scale;
        const sy = oy - p.y * scale;
        if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
      });
      ctx.stroke();
    }

    // Live / mission paths: show both; selected is bold.
    const livePlanner = plannerPath.length > 1 ? plannerPath : path;
    const liveSmooth = smoothedPath.length > 1 ? smoothedPath : [];
    if (pathStyle === 'smoothed') {
      strokePath(livePlanner, 'rgba(0, 0, 128, 0.35)', 2);
      strokePath(liveSmooth.length ? liveSmooth : path, 'rgba(232, 93, 76, 0.95)', 3.5);
    } else {
      strokePath(liveSmooth, 'rgba(232, 93, 76, 0.35)', 2);
      strokePath(livePlanner.length ? livePlanner : path, 'rgba(0, 0, 128, 0.9)', 3.5);
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

    // Sim markers + preview paths (both styles when available)
    if (simPlannerPath.length > 1 || simSmoothedPath.length > 1 || simPath.length > 1) {
      const sp = simPlannerPath.length > 1 ? simPlannerPath : simPath;
      const ss = simSmoothedPath.length > 1 ? simSmoothedPath : [];
      if (pathStyle === 'smoothed') {
        strokePath(sp, 'rgba(0, 0, 128, 0.4)', 2);
        strokePath(ss.length ? ss : simPath, 'rgba(255, 153, 51, 0.95)', 3.5);
      } else {
        strokePath(ss, 'rgba(255, 153, 51, 0.4)', 2);
        strokePath(sp.length ? sp : simPath, 'rgba(0, 0, 128, 0.95)', 3.5);
      }
    }
    if (simStart) {
      const sx = ox + simStart.x * scale;
      const sy = oy - simStart.y * scale;
      ctx.fillStyle = '#138808';
      ctx.beginPath();
      ctx.arc(sx, sy, 9, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = '#fff';
      ctx.font = 'bold 11px sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText('S', sx, sy);
    }
    if (simEnd) {
      const sx = ox + simEnd.x * scale;
      const sy = oy - simEnd.y * scale;
      ctx.fillStyle = '#e63946';
      ctx.beginPath();
      ctx.arc(sx, sy, 9, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = '#fff';
      ctx.font = 'bold 11px sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText('E', sx, sy);
    }
    ctx.textAlign = 'start';
    ctx.textBaseline = 'alphabetic';

    ctx.restore();
    drawZeroDegreeArrow();
  }

  function drawZeroDegreeArrow() {
    const cx = 44;
    const cy = Math.max(70, canvas.height - 56);
    ctx.save();
    ctx.translate(cx, cy);
    ctx.fillStyle = 'rgba(255,255,255,0.94)';
    ctx.strokeStyle = '#000080';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(0, 0, 26, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.rotate(viewRot);
    ctx.strokeStyle = '#e63946';
    ctx.fillStyle = '#e63946';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(-12, 0);
    ctx.lineTo(12, 0);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(14, 0);
    ctx.lineTo(5, -6);
    ctx.lineTo(5, 6);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
    ctx.fillStyle = '#000080';
    ctx.font = 'bold 11px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('0°', cx, cy + 40);
    ctx.textAlign = 'start';
  }

  function setSimMarkers(start, end) {
    simStart = start || null;
    simEnd = end || null;
    draw();
  }
  function setSimPath(pts) {
    simPath = pts || [];
    draw();
  }
  function setSimPaths(planner, smoothed, active) {
    simPlannerPath = planner || [];
    simSmoothedPath = smoothed || [];
    simPath = active || (pathStyle === 'smoothed' ? simSmoothedPath : simPlannerPath) || [];
    draw();
  }
  function clearSimPlan() {
    simStart = null;
    simEnd = null;
    simPath = [];
    simPlannerPath = [];
    simSmoothedPath = [];
    draw();
  }

  return {
    init, setMap, setMapFromLocal, setPose, setPath, setPathLayers, setPathStyle,
    setScan, setWaypoints, getWaypoints,
    clearWaypoints, setMode, setDims, fit, draw,
    setSimMarkers, setSimPath, setSimPaths, clearSimPlan,
    set onSimPoint(fn) { onSimPoint = fn; },
    set onHoverCoords(fn) { onHoverCoords = fn; },
  };
})();
