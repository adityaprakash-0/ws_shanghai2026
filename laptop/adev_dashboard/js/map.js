/** Map renderer — RViz-style occupancy grid (YAML resolution, full-map fit) */
const MapView = (() => {
  const canvas = () => document.getElementById('mapCanvas');
  const ctx = () => canvas().getContext('2d');

  let meta = null;
  let pixels = null;
  let width = 0;
  let height = 0;
  let mapBitmap = null;
  let scale = 1;
  let offsetX = 0;
  let offsetY = 0;
  let viewRot = 0; // radians — view rotation about viewport center
  let pose = { x: 0, y: 0, yaw: 0 };
  let scan = [];
  let globalPath = [];
  let localPath = [];
  let routePath = [];
  let robotDims = { length: 0.43, width: 0.40 };
  let waypoints = [];
  let missionIndex = 0;
  let mode = 'waypoint';
  let measureStart = null;
  let measureEnd = null;
  let onWaypointAdd = null;
  let onInitialPose = null;
  let onSimPoint = null;
  let onHoverCoords = null;
  let showGrid = true;
  let followRobot = true;
  let userPanned = false;
  let lastVpW = 0;
  let lastVpH = 0;
  let fitPending = false;
  let resizeObserver = null;
  let simStart = null;
  let simEnd = null;
  let simPath = [];

  function worldToPixel(wx, wy) {
    if (!meta) return { px: 0, py: 0 };
    const res = meta.resolution;
    const ox = meta.origin[0];
    const oy = meta.origin[1];
    const px = (wx - ox) / res;
    const py = height - (wy - oy) / res;
    return { px, py };
  }

  function pixelToWorld(px, py) {
    if (!meta) return { x: 0, y: 0 };
    const res = meta.resolution;
    const ox = meta.origin[0];
    const oy = meta.origin[1];
    return {
      x: px * res + ox,
      y: (height - py) * res + oy,
    };
  }

  function screenToPixel(sx, sy) {
    const rect = canvas().getBoundingClientRect();
    const { w, h } = viewportSize();
    const cx = w / 2;
    const cy = h / 2;
    // Undo: viewport-center rotate → pan offset → scale
    let x = sx - rect.left - cx;
    let y = sy - rect.top - cy;
    const c = Math.cos(-viewRot);
    const s = Math.sin(-viewRot);
    const xr = x * c - y * s;
    const yr = x * s + y * c;
    return {
      px: (xr + cx - offsetX) / scale,
      py: (yr + cy - offsetY) / scale,
    };
  }

  function viewportSize() {
    const rect = canvas().getBoundingClientRect();
    return { w: rect.width, h: rect.height };
  }

  function decodePgm(base64) {
    const raw = atob(base64);
    const bytes = new Uint8Array(raw.length);
    for (let i = 0; i < raw.length; i++) bytes[i] = raw.charCodeAt(i);
    return decodePgmBytes(bytes);
  }

  function decodePgmBytes(bytes) {
    let i = 0;
    const readLine = () => {
      let start = i;
      while (i < bytes.length && bytes[i] !== 0x0a) i++;
      const line = new TextDecoder().decode(bytes.subarray(start, i)).trim();
      i++;
      return line;
    };

    let magic = readLine();
    while (magic.startsWith('#') || magic === '') magic = readLine();

    let dims = readLine();
    while (dims.startsWith('#') || dims === '') dims = readLine();
    const [w, h] = dims.split(/\s+/).map(Number);

    let maxval = readLine();
    while (maxval.startsWith('#') || maxval === '') maxval = readLine();

    const data = bytes.subarray(i, i + w * h);
    return { width: w, height: h, data };
  }

  function pixelColor(v) {
    if (meta.mode === 'trinary') {
      if (v >= 250) return [255, 255, 255];
      if (v <= 25) return [0, 0, 0];
      return [160, 160, 160];
    }
    const occProb = meta.negate ? v / 255.0 : (255 - v) / 255.0;
    if (occProb >= meta.occupied_thresh) return [0, 0, 0];
    if (occProb <= meta.free_thresh) return [255, 255, 255];
    return [160, 160, 160];
  }

  function buildMapBitmap() {
    const oc = document.createElement('canvas');
    oc.width = width;
    oc.height = height;
    const og = oc.getContext('2d');
    const img = og.createImageData(width, height);
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const v = pixels[y * width + x];
        const idx = (y * width + x) * 4;
        const [r, gr, b] = pixelColor(v);
        img.data[idx] = r;
        img.data[idx + 1] = gr;
        img.data[idx + 2] = b;
        img.data[idx + 3] = 255;
      }
    }
    og.putImageData(img, 0, 0);
    mapBitmap = oc;
  }

  function updateMapMetaLabel() {
    const el = document.getElementById('mapMetaInfo');
    if (!el || !meta) return;
    const mw = (width * meta.resolution).toFixed(2);
    const mh = (height * meta.resolution).toFixed(2);
    el.textContent = `${width}×${height} cells · ${mw}×${mh} m · ${meta.resolution} m/cell`;
  }

  function scheduleFit(centerRobot = false) {
    if (fitPending) return;
    fitPending = true;
    const tryFit = () => {
      const { w, h } = viewportSize();
      if (w > 10 && h > 10 && width > 0) {
        fitToView(centerRobot);
        draw();
        fitPending = false;
        return;
      }
      requestAnimationFrame(tryFit);
    };
    requestAnimationFrame(tryFit);
  }

  /** RViz-style: fit the entire occupancy grid into the viewport */
  function fitToView(centerRobot = false) {
    const { w: vw, h: vh } = viewportSize();
    if (!vw || !vh || !width) return;

    const margin = 12;
    scale = Math.min((vw - margin * 2) / width, (vh - margin * 2) / height);

    if (centerRobot && meta) {
      const { px: rx, py: ry } = worldToPixel(pose.x, pose.y);
      offsetX = vw / 2 - rx * scale;
      offsetY = vh / 2 - ry * scale;
      userPanned = false;
      return;
    }

    offsetX = (vw - width * scale) / 2;
    offsetY = (vh - height * scale) / 2;
    viewRot = 0;
  }

  function loadMap(mapInfo) {
    meta = {
      resolution: Number(mapInfo.resolution) || 0.05,
      origin: Array.isArray(mapInfo.origin) ? mapInfo.origin : [0, 0, 0],
      occupied_thresh: mapInfo.occupied_thresh ?? 0.65,
      free_thresh: mapInfo.free_thresh ?? 0.25,
      negate: mapInfo.negate ? 1 : 0,
      mode: mapInfo.mode || 'trinary',
    };
    let pgm;
    if (mapInfo.pgm_bytes) {
      pgm = decodePgmBytes(mapInfo.pgm_bytes);
    } else {
      pgm = decodePgm(mapInfo.pgm_base64);
    }
    width = pgm.width;
    height = pgm.height;
    pixels = pgm.data;
    buildMapBitmap();
    userPanned = false;
    followRobot = true;
    updateMapMetaLabel();
    scheduleFit(false);
  }

  function setWaypoints(wps, currentIdx = 0) {
    waypoints = wps;
    missionIndex = currentIdx;
    draw();
  }

  function setPose(p) {
    const moved = Math.hypot(p.x - pose.x, p.y - pose.y) > 0.01;
    pose = p;
    if (followRobot && !userPanned && moved) {
      fitToView(true);
    }
    draw();
  }

  function setScan(s) {
    scan = s || [];
    draw();
  }

  function setPaths(global, local, route) {
    globalPath = global || [];
    localPath = local || [];
    routePath = route || [];
    draw();
  }

  function setRobotDims(dims) {
    if (dims && dims.length && dims.width) robotDims = dims;
    draw();
  }

  function drawPathLine(g, pts, color, widthPx) {
    if (!pts || pts.length < 2 || !meta) return;
    g.strokeStyle = color;
    g.lineWidth = widthPx / scale;
    g.beginPath();
    pts.forEach((pt, i) => {
      const { px, py } = worldToPixel(pt.x, pt.y);
      if (i === 0) g.moveTo(px, py);
      else g.lineTo(px, py);
    });
    g.stroke();
  }

  function drawRobotBox(g, rx, ry) {
    if (!meta) return;
    const hl = (robotDims.length / meta.resolution) / 2;
    const hw = (robotDims.width / meta.resolution) / 2;
    g.save();
    g.translate(rx, ry);
    g.rotate(-pose.yaw);
    g.fillStyle = 'rgba(0, 0, 128, 0.75)';
    g.strokeStyle = '#ff9933';
    g.lineWidth = 2.5 / scale;
    g.fillRect(-hl, -hw, hl * 2, hw * 2);
    g.strokeRect(-hl, -hw, hl * 2, hw * 2);
    g.strokeStyle = '#138808';
    g.lineWidth = 2 / scale;
    g.beginPath();
    g.moveTo(hl, 0);
    g.lineTo(hl * 0.35, 0);
    g.stroke();
    g.restore();
  }

  function setMode(m) {
    mode = m;
    measureStart = null;
    measureEnd = null;
    updateMeasureUI();
    draw();
  }

  function getMode() { return mode; }

  function updateMeasureUI() {
    const el = document.getElementById('measureDist');
    if (!el) return;
    if (!measureStart || !measureEnd) {
      el.textContent = '—';
      return;
    }
    const a = pixelToWorld(measureStart.px, measureStart.py);
    const b = pixelToWorld(measureEnd.px, measureEnd.py);
    el.textContent = `${Math.hypot(b.x - a.x, b.y - a.y).toFixed(3)} m`;
  }

  function drawGrid(g) {
    if (!showGrid || !meta) return;

    const res = meta.resolution;
    const majorStep = Math.max(1, Math.round(0.5 / res));

    g.lineWidth = 1 / scale;

    if (scale >= 2) {
      g.strokeStyle = 'rgba(0, 0, 0, 0.06)';
      for (let x = 0; x <= width; x++) {
        g.beginPath();
        g.moveTo(x, 0);
        g.lineTo(x, height);
        g.stroke();
      }
      for (let y = 0; y <= height; y++) {
        g.beginPath();
        g.moveTo(0, y);
        g.lineTo(width, y);
        g.stroke();
      }
    }

    g.strokeStyle = 'rgba(19, 136, 8, 0.22)';
    for (let x = 0; x <= width; x += majorStep) {
      g.beginPath();
      g.moveTo(x, 0);
      g.lineTo(x, height);
      g.stroke();
    }
    for (let y = 0; y <= height; y += majorStep) {
      g.beginPath();
      g.moveTo(0, y);
      g.lineTo(width, y);
      g.stroke();
    }
  }

  function draw() {
    const c = canvas();
    const g = ctx();
    const dpr = window.devicePixelRatio || 1;
    const { w, h } = viewportSize();
    if (!w || !h) return;

    if (w !== lastVpW || h !== lastVpH) {
      lastVpW = w;
      lastVpH = h;
      c.width = Math.round(w * dpr);
      c.height = Math.round(h * dpr);
      g.setTransform(dpr, 0, 0, dpr, 0, 0);
      if (!userPanned && mapBitmap) {
        if (followRobot) fitToView(true);
        else fitToView(false);
      }
    }

    g.fillStyle = '#e4e8ec';
    g.fillRect(0, 0, w, h);

    if (!mapBitmap || !meta) {
      g.fillStyle = '#666';
      g.font = '15px sans-serif';
      g.textAlign = 'center';
      g.fillText('Load a map to begin', w / 2, h / 2);
      return;
    }

    g.save();
    // Pan/zoom in screen space, then rotate the whole map about the viewport center
    const cx = w / 2;
    const cy = h / 2;
    g.translate(cx, cy);
    g.rotate(viewRot);
    g.translate(-cx, -cy);
    g.translate(offsetX, offsetY);
    g.scale(scale, scale);
    g.imageSmoothingEnabled = false;
    g.drawImage(mapBitmap, 0, 0, width, height);

    drawGrid(g);

    drawPathLine(g, routePath, 'rgba(180, 0, 220, 0.9)', 3);
    drawPathLine(g, globalPath, 'rgba(0, 0, 128, 0.85)', 3);
    drawPathLine(g, localPath, 'rgba(19, 136, 8, 0.9)', 2);

    waypoints.forEach((wp, i) => {
      const { px, py } = worldToPixel(wp.x, wp.y);
      const isCurrent = i === missionIndex;
      g.fillStyle = isCurrent ? '#138808' : '#ff9933';
      g.beginPath();
      g.arc(px, py, 8 / scale, 0, Math.PI * 2);
      g.fill();
      g.strokeStyle = '#000080';
      g.lineWidth = 2 / scale;
      g.stroke();
      const len = 0.4 / meta.resolution;
      const yaw = wp.yaw || 0;
      g.strokeStyle = isCurrent ? '#138808' : '#ff9933';
      g.lineWidth = 2 / scale;
      g.beginPath();
      g.moveTo(px, py);
      g.lineTo(px + Math.cos(yaw) * len, py - Math.sin(yaw) * len);
      g.stroke();
    });

    const { px: rx, py: ry } = worldToPixel(pose.x, pose.y);

    if (scan.length) {
      g.strokeStyle = 'rgba(19, 136, 8, 0.55)';
      g.lineWidth = 1.2 / scale;
      scan.forEach((pt) => {
        const a = pose.yaw + pt.a;
        const wx = pose.x + pt.r * Math.cos(a);
        const wy = pose.y + pt.r * Math.sin(a);
        const { px, py } = worldToPixel(wx, wy);
        g.beginPath();
        g.moveTo(rx, ry);
        g.lineTo(px, py);
        g.stroke();
      });
    }

    drawRobotBox(g, rx, ry);

    if (measureStart && measureEnd) {
      g.strokeStyle = '#ff9933';
      g.lineWidth = 2.5 / scale;
      g.setLineDash([5 / scale, 4 / scale]);
      g.beginPath();
      g.moveTo(measureStart.px, measureStart.py);
      g.lineTo(measureEnd.px, measureEnd.py);
      g.stroke();
      g.setLineDash([]);
    }

    // Simulation start / end markers (map frame)
    if (simStart) {
      const { px, py } = worldToPixel(simStart.x, simStart.y);
      g.fillStyle = '#138808';
      g.beginPath();
      g.arc(px, py, 9 / scale, 0, Math.PI * 2);
      g.fill();
      g.fillStyle = '#fff';
      g.font = `${11 / scale}px sans-serif`;
      g.textAlign = 'center';
      g.textBaseline = 'middle';
      g.fillText('S', px, py);
    }
    if (simEnd) {
      const { px, py } = worldToPixel(simEnd.x, simEnd.y);
      g.fillStyle = '#e63946';
      g.beginPath();
      g.arc(px, py, 9 / scale, 0, Math.PI * 2);
      g.fill();
      g.fillStyle = '#fff';
      g.font = `${11 / scale}px sans-serif`;
      g.textAlign = 'center';
      g.textBaseline = 'middle';
      g.fillText('E', px, py);
    }
    if (simPath && simPath.length > 1) {
      drawPathLine(g, simPath, 'rgba(255, 153, 51, 0.95)', 3.5);
    }

    g.restore();

    // Screen-space: 0° arrow (map +X) rotates with viewRot
    drawZeroDegreeArrow(g, w, h);
  }

  function drawZeroDegreeArrow(g, w, h) {
    const cx = 44;
    const cy = Math.max(70, h - 56);
    g.save();
    g.translate(cx, cy);
    g.fillStyle = 'rgba(255,255,255,0.94)';
    g.strokeStyle = '#000080';
    g.lineWidth = 2;
    g.beginPath();
    g.arc(0, 0, 26, 0, Math.PI * 2);
    g.fill();
    g.stroke();
    // Canvas Y-down: viewRot matches map transform so +X follows the map
    g.rotate(viewRot);
    g.strokeStyle = '#e63946';
    g.fillStyle = '#e63946';
    g.lineWidth = 3;
    g.beginPath();
    g.moveTo(-12, 0);
    g.lineTo(12, 0);
    g.stroke();
    g.beginPath();
    g.moveTo(14, 0);
    g.lineTo(5, -6);
    g.lineTo(5, 6);
    g.closePath();
    g.fill();
    g.restore();
    g.fillStyle = '#000080';
    g.font = 'bold 11px sans-serif';
    g.textAlign = 'center';
    g.fillText('0°', cx, cy + 40);
  }

  function bindEvents() {
    const c = canvas();
    let dragging = false;   // Alt / middle / right = pan
    let rotating = false;   // Shift = rotate
    let lastX = 0;
    let lastY = 0;

    const parent = c.parentElement;
    if (parent && typeof ResizeObserver !== 'undefined') {
      resizeObserver = new ResizeObserver(() => {
        lastVpW = 0;
        if (!userPanned) {
          if (followRobot) fitToView(true);
          else fitToView(false);
        }
        draw();
      });
      resizeObserver.observe(parent);
    }

    c.addEventListener('contextmenu', (e) => e.preventDefault());

    c.addEventListener('mousedown', (e) => {
      if (e.shiftKey && e.button === 0) {
        rotating = true;
        userPanned = true;
        followRobot = false;
        lastX = e.clientX;
        lastY = e.clientY;
        return;
      }
      if (e.button === 1 || e.button === 2 || e.altKey) {
        dragging = true;
        userPanned = true;
        followRobot = false;
        lastX = e.clientX;
        lastY = e.clientY;
        return;
      }
      const { px, py } = screenToPixel(e.clientX, e.clientY);
      const w = pixelToWorld(px, py);

      if (mode === 'measure') {
        if (!measureStart) measureStart = { px, py };
        else { measureEnd = { px, py }; updateMeasureUI(); }
        draw();
        return;
      }

      if (mode === 'initial_pose') {
        if (onInitialPose) onInitialPose(w.x, w.y);
        return;
      }

      if (mode === 'sim_start' || mode === 'sim_end') {
        const pt = {
          x: parseFloat(w.x.toFixed(3)),
          y: parseFloat(w.y.toFixed(3)),
        };
        if (mode === 'sim_start') simStart = pt;
        else simEnd = pt;
        if (onSimPoint) onSimPoint(mode, pt, simStart, simEnd);
        draw();
        return;
      }

      const yaw = waypoints.length ? (waypoints[waypoints.length - 1].yaw || 0) : 0;
      const ignoreYaw = document.getElementById('chkIgnoreYaw')?.checked || false;
      waypoints.push({
        x: parseFloat(w.x.toFixed(3)),
        y: parseFloat(w.y.toFixed(3)),
        yaw,
        ignore_yaw: ignoreYaw,
      });
      if (onWaypointAdd) onWaypointAdd(waypoints);
      draw();
    });

    c.addEventListener('mousemove', (e) => {
      if (rotating) {
        const rect = c.getBoundingClientRect();
        const cx = rect.left + rect.width / 2;
        const cy = rect.top + rect.height / 2;
        const a0 = Math.atan2(lastY - cy, lastX - cx);
        const a1 = Math.atan2(e.clientY - cy, e.clientX - cx);
        viewRot += a1 - a0;
        lastX = e.clientX;
        lastY = e.clientY;
        draw();
      } else if (dragging) {
        const dx = e.clientX - lastX;
        const dy = e.clientY - lastY;
        // Offset is applied inside the rotation, so convert screen delta → offset space
        const c = Math.cos(-viewRot);
        const s = Math.sin(-viewRot);
        offsetX += dx * c - dy * s;
        offsetY += dx * s + dy * c;
        lastX = e.clientX;
        lastY = e.clientY;
        draw();
      } else if (mode === 'measure' && measureStart && !measureEnd) {
        const { px, py } = screenToPixel(e.clientX, e.clientY);
        measureEnd = { px, py };
        updateMeasureUI();
        draw();
      } else if (onHoverCoords && meta) {
        const { px, py } = screenToPixel(e.clientX, e.clientY);
        const wpt = pixelToWorld(px, py);
        onHoverCoords(wpt.x, wpt.y);
      }
    });

    const endDrag = () => { dragging = false; rotating = false; };
    c.addEventListener('mouseup', endDrag);
    c.addEventListener('mouseleave', () => {
      endDrag();
      if (onHoverCoords) onHoverCoords(null, null);
    });

    c.addEventListener('wheel', (e) => {
      e.preventDefault();
      userPanned = true;
      followRobot = false;
      const factor = e.deltaY > 0 ? 0.9 : 1.1;
      const rect = c.getBoundingClientRect();
      const { w, h } = viewportSize();
      const cx = w / 2;
      const cy = h / 2;
      // Cursor in screen space → undo rotation to reach offset space
      const sx = e.clientX - rect.left - cx;
      const sy = e.clientY - rect.top - cy;
      const cos = Math.cos(-viewRot);
      const sin = Math.sin(-viewRot);
      const mx = sx * cos - sy * sin + cx;
      const my = sx * sin + sy * cos + cy;
      offsetX = mx - (mx - offsetX) * factor;
      offsetY = my - (my - offsetY) * factor;
      scale *= factor;
      draw();
    }, { passive: false });

    const btnFit = document.getElementById('btnFitMap');
    if (btnFit) {
      btnFit.onclick = () => {
        userPanned = false;
        followRobot = false;
        fitToView(false);
        draw();
      };
    }
    const btnCenter = document.getElementById('btnCenterRobot');
    if (btnCenter) {
      btnCenter.onclick = () => {
        followRobot = true;
        userPanned = false;
        fitToView(true);
        draw();
      };
    }

    window.addEventListener('resize', () => {
      lastVpW = 0;
      if (followRobot && !userPanned) fitToView(true);
      else if (!userPanned) fitToView(false);
      draw();
    });
  }

  function clearWaypoints() {
    waypoints = [];
    draw();
  }

  function getWaypoints() { return [...waypoints]; }

  function setSimMarkers(start, end) {
    simStart = start || null;
    simEnd = end || null;
    draw();
  }

  function setSimPath(pts) {
    simPath = pts || [];
    draw();
  }

  function clearSimPlan() {
    simStart = null;
    simEnd = null;
    simPath = [];
    draw();
  }

  return {
    loadMap, setPose, setScan, setPaths, setRobotDims, setWaypoints, setMode, getMode,
    clearWaypoints, getWaypoints, bindEvents, draw, fitToView,
    setSimMarkers, setSimPath, clearSimPlan,
    set onWaypointAdd(fn) { onWaypointAdd = fn; },
    set onInitialPose(fn) { onInitialPose = fn; },
    set onSimPoint(fn) { onSimPoint = fn; },
    set onHoverCoords(fn) { onHoverCoords = fn; },
  };
})();
