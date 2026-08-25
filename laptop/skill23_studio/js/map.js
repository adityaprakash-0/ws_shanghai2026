/** Studio map canvas — grid, compass, footprint, edit tools + undo */
const StudioMap = (() => {
  let canvas, ctx;
  let mapData = null;
  let mapBitmap = null;
  let pose = null;
  let scan = [];
  let trail = [];
  let showTrail = true;
  let path = [];
  let plannerPath = [];
  let smoothedPath = [];
  let waypoints = [];
  let restrictions = [];
  /** Simulation overlay: up to two picked points + the returned path. */
  let simPoints = [];
  let simPath = [];
  let simStyle = 'planned';   // planned (dashed purple) | smoothed (solid teal)
  let tool = 'pan';
  let onCommit = null;
  let onEditCells = null;
  let onRestrict = null;
  let onBrushChange = null;
  let robotDims = { length: 0.43, width: 0.40 };

  let scale = 1, ox = 0, oy = 0, viewRot = 0;
  let fitPending = true;
  let panning = false, rotating = false, painting = false;
  let drafting = null;
  let lastPanX = 0, lastPanY = 0;
  /** Committed cells of the active stroke, as flat grid indices (dedup). */
  let paintSet = new Set();
  /** Sparse stroke centres — cheap brush preview instead of per-cell rects. */
  let strokePts = [];
  let lastStrokeCell = null;
  /** Box-fill draft in cells: {gx0,gy0,gx1,gy1}. */
  let boxDraft = null;
  /** Brush width in cells (1 = single cell). */
  let brushCells = 1;
  /** Cell under cursor for the brush preview. */
  let hoverCell = null;
  let undoStack = [];
  /** @type {{x0:number,y0:number,x1:number,y1:number}[]} */
  let measures = [];
  /** @type {{x0:number,y0:number,x1:number,y1:number}|null} */
  let measureDraft = null;

  function init(el, commitCb) {
    canvas = el;
    ctx = canvas.getContext('2d');
    onCommit = commitCb;
    bind();
    resize();
    window.addEventListener('resize', resize);
  }

  function resize() {
    if (!canvas || !canvas.parentElement) return;
    const r = canvas.parentElement.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.max(1, Math.floor(r.width * dpr));
    canvas.height = Math.max(1, Math.floor(r.height * dpr));
    canvas.style.width = `${r.width}px`;
    canvas.style.height = `${r.height}px`;
    fitPending = true;
    draw();
  }

  function setTool(t) {
    tool = t;
    hoverCell = null;
    draw();
  }
  function setOnEditCells(cb) { onEditCells = cb; }
  function setOnRestrict(cb) { onRestrict = cb; }
  function setOnBrushChange(cb) { onBrushChange = cb; }
  function setBrush(cells) {
    brushCells = Math.max(1, Math.min(101, Math.round(+cells || 1)));
    draw();
    return brushCells;
  }
  function getBrush() { return brushCells; }
  /** wall/erase (freehand or box) — tools that write occupancy cells. */
  function isPaintTool(t) {
    return t === 'wall' || t === 'erase' || t === 'wallbox' || t === 'erasebox';
  }
  function paintAction(t) {
    return (t === 'wall' || t === 'wallbox') ? 'wall' : 'erase';
  }
  function setDims(d) {
    if (d && d.length && d.width) robotDims = { length: +d.length, width: +d.width };
  }
  function setShowTrail(v) { showTrail = !!v; draw(); }
  function clearTrailLocal() { trail = []; draw(); }

  function setMap(m) {
    if (!m || !m.data) { mapData = null; mapBitmap = null; draw(); return; }
    const same = mapData
      && mapData.width === m.width
      && mapData.height === m.height
      && mapData.resolution === m.resolution
      && mapData.data.length === m.data.length;
    // Keep local paint edits if same grid and we're editing
    if (same && undoStack.length && mapData._locallyEdited) {
      draw();
      return;
    }
    mapData = {
      width: m.width,
      height: m.height,
      resolution: m.resolution,
      origin: m.origin,
      data: m.data.slice(),
      _locallyEdited: false,
    };
    buildBitmap();
    if (!same) fitPending = true;
    draw();
  }

  function occupancyToGray(v) {
    // OccupancyGrid: -1 unknown, 0 free, 100 occupied
    if (v < 0) return 175;           // unknown — grey (PGM 205)
    if (v >= 65) return 0;           // occupied — black
    if (v <= 25) return 255;         // free — white
    return Math.round(255 - (v / 100) * 255);
  }

  function buildBitmap() {
    const { width: w, height: h, data } = mapData;
    const img = ctx.createImageData(w, h);
    for (let i = 0; i < data.length; i++) {
      const c = occupancyToGray(data[i]);
      const o = i * 4;
      img.data[o] = c;
      img.data[o + 1] = c;
      img.data[o + 2] = c;
      img.data[o + 3] = 255;
    }
    const flipped = ctx.createImageData(w, h);
    for (let row = 0; row < h; row++) {
      const src = (h - 1 - row) * w * 4;
      const dst = row * w * 4;
      flipped.data.set(img.data.subarray(src, src + w * 4), dst);
    }
    const off = document.createElement('canvas');
    off.width = w;
    off.height = h;
    off.getContext('2d').putImageData(flipped, 0, 0);
    mapBitmap = off;
  }

  /** Square brush around (gx,gy) → flat indices into set. */
  function addBrushCells(gx, gy, set) {
    const lo = Math.floor((brushCells - 1) / 2);
    const hi = brushCells - 1 - lo;
    const w = mapData.width;
    const h = mapData.height;
    for (let dy = -lo; dy <= hi; dy++) {
      const y = gy + dy;
      if (y < 0 || y >= h) continue;
      for (let dx = -lo; dx <= hi; dx++) {
        const x = gx + dx;
        if (x < 0 || x >= w) continue;
        set.add(y * w + x);
      }
    }
  }

  /** Bresenham between samples so a fast drag still paints a solid stroke. */
  function addStrokeSegment(from, to, set) {
    let [x0, y0] = from;
    const [x1, y1] = to;
    const dx = Math.abs(x1 - x0);
    const dy = Math.abs(y1 - y0);
    const sx = x0 < x1 ? 1 : -1;
    const sy = y0 < y1 ? 1 : -1;
    let err = dx - dy;
    let guard = 0;
    for (;;) {
      addBrushCells(x0, y0, set);
      if ((x0 === x1 && y0 === y1) || ++guard > 20000) break;
      const e2 = 2 * err;
      if (e2 > -dy) { err -= dy; x0 += sx; }
      if (e2 < dx) { err += dx; y0 += sy; }
    }
  }

  function addBoxCells(gx0, gy0, gx1, gy1, set) {
    const w = mapData.width;
    const h = mapData.height;
    const x0 = Math.max(0, Math.min(gx0, gx1));
    const x1 = Math.min(w - 1, Math.max(gx0, gx1));
    const y0 = Math.max(0, Math.min(gy0, gy1));
    const y1 = Math.min(h - 1, Math.max(gy0, gy1));
    for (let y = y0; y <= y1; y++) {
      for (let x = x0; x <= x1; x++) set.add(y * w + x);
    }
  }

  function cellsFromSet(set) {
    const w = mapData.width;
    const out = [];
    set.forEach((idx) => out.push([idx % w, Math.floor(idx / w)]));
    return out;
  }

  /** Write cells into the local grid; returns the undo record (no undo push). */
  function applyCells(action, cells) {
    if (!mapData) return [];
    const target = action === 'wall' ? 100 : 0;
    const before = [];
    cells.forEach(([gx, gy]) => {
      if (gx < 0 || gy < 0 || gx >= mapData.width || gy >= mapData.height) return;
      const idx = gy * mapData.width + gx;
      // Skip no-ops: keeps undo small and the /api/map/edit payload light.
      if (mapData.data[idx] === target) return;
      before.push({ idx, val: mapData.data[idx], gx, gy });
      mapData.data[idx] = target;
    });
    if (!before.length) return before;
    mapData._locallyEdited = true;
    buildBitmap();
    if (onEditCells) {
      onEditCells(action, before.map((c) => [c.gx, c.gy]));
    }
    return before;
  }

  function paintCellsLocal(action, cells) {
    const before = applyCells(action, cells);
    if (!before.length) return;
    undoStack.push({ type: 'edit', cells: before });
    draw();
  }

  function restoreCells(cells) {
    if (!mapData || !cells || !cells.length) return;
    cells.forEach(({ idx, val }) => { mapData.data[idx] = val; });
    mapData._locallyEdited = undoStack.some(
      (u) => (u.type === 'edit') || (u.cells && u.cells.length));
    buildBitmap();
  }

  /** Index of the topmost zone containing the world point, else -1. */
  function restrictionIndexAt(wx, wy) {
    for (let i = restrictions.length - 1; i >= 0; i--) {
      const r = restrictions[i];
      const yaw = r.yaw || 0;
      const c = Math.cos(yaw);
      const s = Math.sin(yaw);
      const dx = wx - (r.cx || 0);
      const dy = wy - (r.cy || 0);
      const lx = c * dx + s * dy;
      const ly = -s * dx + c * dy;
      if (Math.abs(lx) <= (r.w || 0) / 2 && Math.abs(ly) <= (r.h || 0) / 2) return i;
    }
    return -1;
  }

  /** Occupied cells covered by a zone — what a burn turned black. */
  function occupiedCellsInRect(r) {
    if (!mapData) return [];
    const res = mapData.resolution;
    const yaw = r.yaw || 0;
    const c = Math.cos(yaw);
    const s = Math.sin(yaw);
    const hw = (r.w || 0) / 2;
    const hh = (r.h || 0) / 2;
    const reach = Math.hypot(hw, hh) + res;
    const gx0 = Math.floor((r.cx - reach - mapData.origin.x) / res);
    const gx1 = Math.ceil((r.cx + reach - mapData.origin.x) / res);
    const gy0 = Math.floor((r.cy - reach - mapData.origin.y) / res);
    const gy1 = Math.ceil((r.cy + reach - mapData.origin.y) / res);
    const out = [];
    for (let gy = Math.max(0, gy0); gy <= Math.min(mapData.height - 1, gy1); gy++) {
      for (let gx = Math.max(0, gx0); gx <= Math.min(mapData.width - 1, gx1); gx++) {
        const wx = mapData.origin.x + (gx + 0.5) * res;
        const wy = mapData.origin.y + (gy + 0.5) * res;
        const dx = wx - r.cx;
        const dy = wy - r.cy;
        const lx = c * dx + s * dy;
        const ly = -s * dx + c * dy;
        if (Math.abs(lx) > hw || Math.abs(ly) > hh) continue;
        // Only cells a burn could have blackened — leave unknown (-1) alone.
        if (mapData.data[gy * mapData.width + gx] === 100) out.push([gx, gy]);
      }
    }
    return out;
  }

  /** Delete the zone under the cursor and wipe the cells its burn blackened. */
  function removeRestrictionAt(wx, wy, { clearCells = true } = {}) {
    const idx = restrictionIndexAt(wx, wy);
    if (idx < 0) return null;
    const removed = restrictions[idx];
    const prev = restrictions.slice();
    restrictions = restrictions.filter((_, i) => i !== idx);
    // One undo entry restores the box and its pixels together.
    const cells = clearCells ? applyCells('erase', occupiedCellsInRect(removed)) : [];
    undoStack.push({ type: 'restrict', prev, cells });
    if (onRestrict) onRestrict(restrictions, { removed });
    draw();
    return removed;
  }

  function clearRestrictions({ clearCells = true } = {}) {
    if (!restrictions.length) return [];
    const prev = restrictions.slice();
    restrictions = [];
    let cells = [];
    if (clearCells && mapData) {
      const set = new Set();
      const w = mapData.width;
      prev.forEach((r) => {
        occupiedCellsInRect(r).forEach(([gx, gy]) => set.add(gy * w + gx));
      });
      cells = applyCells('erase', cellsFromSet(set));
    }
    undoStack.push({ type: 'restrict', prev, cells });
    if (onRestrict) onRestrict(restrictions, { cleared: prev });
    draw();
    return prev;
  }

  function undo() {
    const op = undoStack.pop();
    if (!op) return false;
    if (op.type === 'edit' && mapData) {
      restoreCells(op.cells);
      draw();
      return true;
    }
    if (op.type === 'restrict') {
      restoreCells(op.cells);
      restrictions = op.prev || [];
      // restored: the Pi already stored the removal — re-sync the zone list.
      if (onRestrict) onRestrict(restrictions, { restored: true });
      draw();
      return true;
    }
    return false;
  }

  function canUndo() { return undoStack.length > 0; }
  function clearUndo() { undoStack = []; }

  function setPose(p) { pose = p; draw(); }
  function setScan(s) { scan = s || []; draw(); }
  function setTrail(t) { trail = t || []; draw(); }
  function setPaths(p, pl, sm) {
    path = p || [];
    plannerPath = pl || [];
    smoothedPath = sm || [];
    draw();
  }
  function setWaypoints(w) { waypoints = w || []; draw(); }
  function setRestrictions(r) { restrictions = r || []; draw(); }
  function setSim(points, pathPts, style) {
    simPoints = points || [];
    simPath = pathPts || [];
    if (style) simStyle = style;
    draw();
  }

  function worldFromEvent(ev) {
    if (!mapData) return null;
    const rect = canvas.getBoundingClientRect();
    let sx = (ev.clientX - rect.left) * (canvas.width / rect.width);
    let sy = (ev.clientY - rect.top) * (canvas.height / rect.height);
    const cx = canvas.width / 2;
    const cy = canvas.height / 2;
    let dx = sx - cx - ox;
    let dy = sy - cy - oy;
    const c = Math.cos(-viewRot);
    const s = Math.sin(-viewRot);
    const lx = (dx * c - dy * s) / scale;
    const ly = (dx * s + dy * c) / scale;
    const gx = lx + mapData.width / 2;
    const row = ly + mapData.height / 2;
    const gy = mapData.height - 1 - row;
    return {
      x: mapData.origin.x + gx * mapData.resolution,
      y: mapData.origin.y + gy * mapData.resolution,
    };
  }

  function worldToScreen(wx, wy) {
    const gx = (wx - mapData.origin.x) / mapData.resolution;
    const gy = (wy - mapData.origin.y) / mapData.resolution;
    const row = mapData.height - 1 - gy;
    let dx = (gx - mapData.width / 2) * scale;
    let dy = (row - mapData.height / 2) * scale;
    const c = Math.cos(viewRot);
    const s = Math.sin(viewRot);
    const rx = dx * c - dy * s;
    const ry = dx * s + dy * c;
    return {
      x: canvas.width / 2 + ox + rx,
      y: canvas.height / 2 + oy + ry,
    };
  }

  function cellFromWorld(wx, wy) {
    const gx = Math.floor((wx - mapData.origin.x) / mapData.resolution);
    const gy = Math.floor((wy - mapData.origin.y) / mapData.resolution);
    return [gx, gy];
  }

  function fit() {
    if (!mapData) return;
    const pad = 40;
    const sx = (canvas.width - pad * 2) / mapData.width;
    const sy = (canvas.height - pad * 2) / mapData.height;
    scale = Math.min(sx, sy);
    ox = 0;
    oy = 0;
    viewRot = 0;
    fitPending = false;
  }

  function bind() {
    canvas.addEventListener('mousedown', (ev) => {
      if (ev.shiftKey && ev.button === 0) {
        rotating = true;
        lastPanX = ev.clientX; lastPanY = ev.clientY;
        return;
      }
      if (ev.button === 1 || ev.button === 2 || (tool === 'pan' && ev.button === 0)) {
        panning = true;
        lastPanX = ev.clientX; lastPanY = ev.clientY;
        return;
      }
      if (!mapData || ev.button !== 0) return;
      const w = worldFromEvent(ev);
      if (!w) return;

      if (tool === 'wall' || tool === 'erase') {
        painting = true;
        paintSet = new Set();
        const c0 = cellFromWorld(w.x, w.y);
        strokePts = [c0];
        lastStrokeCell = c0;
        addBrushCells(c0[0], c0[1], paintSet);
        draw();
        return;
      }
      if (tool === 'wallbox' || tool === 'erasebox') {
        const c0 = cellFromWorld(w.x, w.y);
        boxDraft = { gx0: c0[0], gy0: c0[1], gx1: c0[0], gy1: c0[1] };
        draw();
        return;
      }
      if (tool === 'restrictdel') {
        removeRestrictionAt(w.x, w.y);
        return;
      }
      if (tool === 'measure') {
        measureDraft = { x0: w.x, y0: w.y, x1: w.x, y1: w.y };
        draw();
        return;
      }
      if (tool === 'restrict') {
        drafting = {
          kind: 'restrict',
          x0: w.x, y0: w.y,
          x1: w.x, y1: w.y,
          yaw: 0,
        };
        draw();
        return;
      }
      if (tool === 'pose' || tool === 'goal' || tool === 'waypoint' || tool === 'simpt') {
        drafting = { kind: tool, x: w.x, y: w.y, yaw: 0 };
        draw();
      }
    });

    window.addEventListener('mousemove', (ev) => {
      if (rotating) {
        const rect = canvas.getBoundingClientRect();
        const cx = rect.left + rect.width / 2;
        const cy = rect.top + rect.height / 2;
        const a0 = Math.atan2(lastPanY - cy, lastPanX - cx);
        const a1 = Math.atan2(ev.clientY - cy, ev.clientX - cx);
        viewRot += a1 - a0;
        lastPanX = ev.clientX; lastPanY = ev.clientY;
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
        lastPanX = ev.clientX; lastPanY = ev.clientY;
        draw();
        return;
      }
      if (painting) {
        const w = worldFromEvent(ev);
        if (!w) return;
        const cell = cellFromWorld(w.x, w.y);
        if (lastStrokeCell
          && cell[0] === lastStrokeCell[0] && cell[1] === lastStrokeCell[1]) return;
        if (lastStrokeCell) addStrokeSegment(lastStrokeCell, cell, paintSet);
        else addBrushCells(cell[0], cell[1], paintSet);
        lastStrokeCell = cell;
        strokePts.push(cell);
        draw();
        return;
      }
      if (boxDraft) {
        const w = worldFromEvent(ev);
        if (!w) return;
        const cell = cellFromWorld(w.x, w.y);
        boxDraft.gx1 = cell[0];
        boxDraft.gy1 = cell[1];
        draw();
        return;
      }
      if (measureDraft) {
        const w = worldFromEvent(ev);
        if (!w) return;
        measureDraft.x1 = w.x;
        measureDraft.y1 = w.y;
        draw();
        return;
      }
      if (drafting) {
        const w = worldFromEvent(ev);
        if (!w) return;
        if (drafting.kind === 'restrict') {
          // Corner-to-corner: first click = one corner, drag = opposite corner
          drafting.x1 = w.x;
          drafting.y1 = w.y;
        } else {
          const dx = w.x - drafting.x;
          const dy = w.y - drafting.y;
          if (Math.hypot(dx, dy) > 0.02) drafting.yaw = Math.atan2(dy, dx);
        }
        draw();
        return;
      }
      // Idle: keep a brush footprint under the cursor so size is obvious.
      if (isPaintTool(tool) && mapData && ev.target === canvas) {
        const w = worldFromEvent(ev);
        if (!w) return;
        const cell = cellFromWorld(w.x, w.y);
        if (!hoverCell || hoverCell[0] !== cell[0] || hoverCell[1] !== cell[1]) {
          hoverCell = cell;
          draw();
        }
      } else if (hoverCell) {
        hoverCell = null;
        draw();
      }
    });

    window.addEventListener('mouseup', () => {
      if (panning) { panning = false; return; }
      if (rotating) { rotating = false; return; }
      if (painting) {
        painting = false;
        const action = paintAction(tool);
        const cells = cellsFromSet(paintSet);
        paintSet = new Set();
        strokePts = [];
        lastStrokeCell = null;
        paintCellsLocal(action, cells);
        return;
      }
      if (boxDraft) {
        const d = boxDraft;
        boxDraft = null;
        const action = paintAction(tool);
        const set = new Set();
        addBoxCells(d.gx0, d.gy0, d.gx1, d.gy1, set);
        paintCellsLocal(action, cellsFromSet(set));
        return;
      }
      if (measureDraft) {
        const d = measureDraft;
        measureDraft = null;
        const dist = Math.hypot(d.x1 - d.x0, d.y1 - d.y0);
        if (dist >= 0.005) {
          measures.push({ x0: d.x0, y0: d.y0, x1: d.x1, y1: d.y1 });
          if (measures.length > 12) measures = measures.slice(-12);
        }
        draw();
        return;
      }
      if (!drafting) return;
      const d = drafting;
      drafting = null;
      if (d.kind === 'restrict') {
        const minX = Math.min(d.x0, d.x1);
        const maxX = Math.max(d.x0, d.x1);
        const minY = Math.min(d.y0, d.y1);
        const maxY = Math.max(d.y0, d.y1);
        const w = Math.max(0.05, maxX - minX);
        const h = Math.max(0.05, maxY - minY);
        if (w < 0.05 && h < 0.05) { draw(); return; }
        const prev = restrictions.slice();
        const rect = {
          cx: (minX + maxX) / 2,
          cy: (minY + maxY) / 2,
          w, h,
          yaw: 0,
        };
        restrictions = restrictions.concat([rect]);
        undoStack.push({ type: 'restrict', prev });
        if (onRestrict) onRestrict(restrictions);
        draw();
        return;
      }
      draw();
      if (onCommit) onCommit(d.x, d.y, d.kind, d.yaw);
    });

    canvas.addEventListener('contextmenu', (e) => e.preventDefault());
    canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      if (!mapData) return;
      // Alt/Ctrl + wheel resizes the brush instead of zooming.
      if ((e.altKey || e.ctrlKey) && isPaintTool(tool)) {
        const step = brushCells < 10 ? 1 : Math.round(brushCells * 0.2);
        setBrush(brushCells + (e.deltaY > 0 ? -step : step));
        if (onBrushChange) onBrushChange(brushCells);
        return;
      }
      const factor = e.deltaY > 0 ? 0.9 : 1.1;
      scale = Math.max(0.5, Math.min(40, scale * factor));
      draw();
    }, { passive: false });
  }

  function drawPoly(pts, stroke, width) {
    if (!pts || pts.length < 2) return;
    ctx.beginPath();
    const s0 = worldToScreen(pts[0].x, pts[0].y);
    ctx.moveTo(s0.x, s0.y);
    for (let i = 1; i < pts.length; i++) {
      const s = worldToScreen(pts[i].x, pts[i].y);
      ctx.lineTo(s.x, s.y);
    }
    ctx.strokeStyle = stroke;
    ctx.lineWidth = width;
    ctx.stroke();
  }

  function drawGrid() {
    if (!mapData) return;
    const res = mapData.resolution;
    const w = mapData.width;
    const h = mapData.height;
    const oxm = mapData.origin.x;
    const oym = mapData.origin.y;
    const maxX = oxm + w * res;
    const maxY = oym + h * res;

    // Fixed world spacing (multiples of resolution) — never changes with zoom.
    // Minor = 1 cell; major ≈ 1 m (integer cells).
    const minorCells = 1;
    const majorCells = Math.max(1, Math.round(1.0 / res));
    const dpr = window.devicePixelRatio || 1;
    // Only skip drawing when a line would be sub-pixel (too dense to see).
    const minorPx = minorCells * scale;
    const majorPx = majorCells * scale;
    const drawMinor = minorPx >= 3 * dpr;
    const drawMajor = majorPx >= 2 * dpr;

    ctx.save();
    ctx.lineWidth = 1;
    const step = drawMinor ? minorCells : (drawMajor ? majorCells : 0);
    if (step > 0) {
      for (let i = 0; i <= w; i += step) {
        const x = oxm + i * res;
        const a = worldToScreen(x, oym);
        const b = worldToScreen(x, maxY);
        const isMaj = (i % majorCells) === 0;
        if (!drawMinor && !isMaj) continue;
        ctx.strokeStyle = isMaj ? 'rgba(0,0,128,0.30)' : 'rgba(0,0,128,0.12)';
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
      }
      for (let j = 0; j <= h; j += step) {
        const y = oym + j * res;
        const a = worldToScreen(oxm, y);
        const b = worldToScreen(maxX, y);
        const isMaj = (j % majorCells) === 0;
        if (!drawMinor && !isMaj) continue;
        ctx.strokeStyle = isMaj ? 'rgba(0,0,128,0.30)' : 'rgba(0,0,128,0.12)';
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
      }
    }
    ctx.restore();

    const pad = 10 * dpr;
    const fontPx = 11 * dpr;
    ctx.save();
    ctx.font = `${fontPx}px Outfit, sans-serif`;
    ctx.textAlign = 'left';
    ctx.textBaseline = 'bottom';
    const label = `cell ${res.toFixed(3)} m · major ${formatLen(majorCells * res)}`;
    const tw = ctx.measureText(label).width;
    ctx.fillStyle = 'rgba(255,255,255,0.85)';
    ctx.fillRect(pad - 4, canvas.height - pad - fontPx - 4, tw + 8, fontPx + 8);
    ctx.fillStyle = '#000080';
    ctx.fillText(label, pad, canvas.height - pad);
    ctx.restore();
  }

  function formatLen(m) {
    if (m >= 1) return `${m.toFixed(m >= 10 ? 0 : 2)} m`;
    if (m >= 0.01) return `${(m * 100).toFixed(m * 100 >= 10 ? 0 : 1)} cm`;
    return `${(m * 1000).toFixed(0)} mm`;
  }

  function drawMeasureSeg(m, accent) {
    const a = worldToScreen(m.x0, m.y0);
    const b = worldToScreen(m.x1, m.y1);
    const dist = Math.hypot(m.x1 - m.x0, m.y1 - m.y0);
    const dpr = window.devicePixelRatio || 1;
    ctx.save();
    ctx.strokeStyle = accent;
    ctx.fillStyle = accent;
    ctx.lineWidth = 2 * dpr;
    ctx.setLineDash([6 * dpr, 4 * dpr]);
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();
    ctx.setLineDash([]);
    for (const p of [a, b]) {
      ctx.beginPath();
      ctx.arc(p.x, p.y, 4 * dpr, 0, Math.PI * 2);
      ctx.fill();
    }
    const mx = (a.x + b.x) / 2;
    const my = (a.y + b.y) / 2;
    const cells = mapData ? dist / mapData.resolution : 0;
    const text = `${formatLen(dist)}  (${cells.toFixed(1)} cells)`;
    const fontPx = 12 * dpr;
    ctx.font = `bold ${fontPx}px Outfit, sans-serif`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    const tw = ctx.measureText(text).width;
    ctx.fillStyle = 'rgba(255,255,255,0.92)';
    ctx.fillRect(mx - tw / 2 - 6, my - fontPx / 2 - 4, tw + 12, fontPx + 8);
    ctx.strokeStyle = accent;
    ctx.lineWidth = 1;
    ctx.strokeRect(mx - tw / 2 - 6, my - fontPx / 2 - 4, tw + 12, fontPx + 8);
    ctx.fillStyle = '#000080';
    ctx.fillText(text, mx, my);
    ctx.restore();
  }

  function drawMeasures() {
    measures.forEach((m) => drawMeasureSeg(m, '#FF9933'));
    if (measureDraft) drawMeasureSeg(measureDraft, '#138808');
  }

  function clearMeasures() {
    measures = [];
    measureDraft = null;
    draw();
  }

  function headingTip(x, y, yaw, distM) {
    return worldToScreen(
      x + Math.cos(yaw || 0) * distM,
      y + Math.sin(yaw || 0) * distM);
  }

  function drawCompass() {
    // Always points to map +X (yaw 0) in map frame, accounting for viewRot
    const size = 36 * (window.devicePixelRatio || 1);
    const pad = 16 * (window.devicePixelRatio || 1);
    const cx = canvas.width - pad - size;
    const cy = pad + size;
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(viewRot); // cancel view rotation so 0° stays map-forward
    ctx.beginPath();
    ctx.arc(0, 0, size * 0.55, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(255,255,255,0.9)';
    ctx.fill();
    ctx.strokeStyle = '#000080';
    ctx.lineWidth = 2;
    ctx.stroke();
    // Arrow toward map +X (screen: right when viewRot=0; worldToScreen uses -sin for y so +X is +screenX)
    ctx.beginPath();
    ctx.moveTo(size * 0.45, 0);
    ctx.lineTo(-size * 0.15, -size * 0.22);
    ctx.lineTo(-size * 0.05, 0);
    ctx.lineTo(-size * 0.15, size * 0.22);
    ctx.closePath();
    ctx.fillStyle = '#FF9933';
    ctx.fill();
    ctx.fillStyle = '#000080';
    ctx.font = `bold ${11 * (window.devicePixelRatio || 1)}px Outfit, sans-serif`;
    ctx.textAlign = 'center';
    ctx.fillText('0°', 0, size * 0.85);
    ctx.restore();
  }

  function drawFootprint() {
    if (!pose || !mapData) return;
    const yaw = pose.yaw || 0;
    const hl = robotDims.length / 2;
    const hw = robotDims.width / 2;
    const c = Math.cos(yaw);
    const s = Math.sin(yaw);
    const corners = [
      [hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw],
    ].map(([lx, ly]) => ({
      x: pose.x + c * lx - s * ly,
      y: pose.y + s * lx + c * ly,
    }));
    ctx.beginPath();
    const p0 = worldToScreen(corners[0].x, corners[0].y);
    ctx.moveTo(p0.x, p0.y);
    for (let i = 1; i < 4; i++) {
      const p = worldToScreen(corners[i].x, corners[i].y);
      ctx.lineTo(p.x, p.y);
    }
    ctx.closePath();
    ctx.fillStyle = 'rgba(19,136,8,0.25)';
    ctx.fill();
    ctx.strokeStyle = '#138808';
    ctx.lineWidth = 2;
    ctx.stroke();

    const sc = worldToScreen(pose.x, pose.y);
    // Center mark
    ctx.fillStyle = '#000080';
    ctx.beginPath();
    ctx.arc(sc.x, sc.y, 4, 0, Math.PI * 2);
    ctx.fill();
    // Yaw arrow (map yaw: screen uses cos for x, -sin for y)
    const tip = worldToScreen(
      pose.x + Math.cos(yaw) * hl * 1.15,
      pose.y + Math.sin(yaw) * hl * 1.15);
    ctx.strokeStyle = '#FF9933';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(sc.x, sc.y);
    ctx.lineTo(tip.x, tip.y);
    ctx.stroke();
  }

  function draw() {
    if (!ctx) return;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = '#c8d0d8';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    if (!mapData || !mapBitmap) {
      drawCompass();
      return;
    }
    if (fitPending) fit();

    ctx.save();
    ctx.translate(canvas.width / 2 + ox, canvas.height / 2 + oy);
    ctx.rotate(viewRot);
    ctx.scale(scale, scale);
    ctx.imageSmoothingEnabled = false;
    ctx.translate(-mapData.width / 2, -mapData.height / 2);
    ctx.drawImage(mapBitmap, 0, 0);
    ctx.restore();

    drawGrid();

    if (showTrail) drawPoly(trail, 'rgba(255,153,51,0.85)', 2);
    drawPoly(plannerPath, 'rgba(0,0,128,0.45)', 2);
    drawPoly(smoothedPath.length ? smoothedPath : path, 'rgba(255,153,51,0.95)', 3);

    if (pose && scan.length) {
      const c = Math.cos(pose.yaw);
      const s = Math.sin(pose.yaw);
      ctx.fillStyle = 'rgba(19,136,8,0.75)';
      for (const p of scan) {
        const wx = pose.x + c * p.x - s * p.y;
        const wy = pose.y + s * p.x + c * p.y;
        const sc = worldToScreen(wx, wy);
        ctx.fillRect(sc.x - 1.5, sc.y - 1.5, 3, 3);
      }
    }

    for (const r of restrictions) {
      drawRect(r, 'rgba(198,40,40,0.35)', '#c62828');
    }
    if (drafting && drafting.kind === 'restrict') {
      const minX = Math.min(drafting.x0, drafting.x1);
      const maxX = Math.max(drafting.x0, drafting.x1);
      const minY = Math.min(drafting.y0, drafting.y1);
      const maxY = Math.max(drafting.y0, drafting.y1);
      drawRect(
        {
          cx: (minX + maxX) / 2,
          cy: (minY + maxY) / 2,
          w: Math.max(0.05, maxX - minX),
          h: Math.max(0.05, maxY - minY),
          yaw: 0,
        },
        'rgba(198,40,40,0.25)', '#c62828');
    }

    drawPaintPreview();

    waypoints.forEach((wp, i) => {
      const sc = worldToScreen(wp.x, wp.y);
      const yaw = wp.yaw || 0;
      const deg = Math.round((yaw * 180) / Math.PI);
      ctx.fillStyle = '#000080';
      ctx.beginPath();
      ctx.arc(sc.x, sc.y, 7, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = '#fff';
      ctx.font = '11px Outfit, sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(String(i + 1), sc.x, sc.y);
      const len = 0.35;
      const tip = headingTip(wp.x, wp.y, yaw, len);
      ctx.strokeStyle = '#FF9933';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(sc.x, sc.y);
      ctx.lineTo(tip.x, tip.y);
      ctx.stroke();
      // Angle label (degrees) next to the arrow tip
      const label = `${deg}°`;
      ctx.font = 'bold 11px Outfit, sans-serif';
      ctx.textAlign = 'left';
      ctx.textBaseline = 'bottom';
      const lx = tip.x + 6;
      const ly = tip.y - 4;
      ctx.fillStyle = 'rgba(255,255,255,0.92)';
      const tw = ctx.measureText(label).width;
      ctx.fillRect(lx - 2, ly - 11, tw + 4, 13);
      ctx.fillStyle = '#000080';
      ctx.fillText(label, lx, ly);
    });

    if (drafting && drafting.kind !== 'restrict') {
      const sc = worldToScreen(drafting.x, drafting.y);
      const deg = Math.round(((drafting.yaw || 0) * 180) / Math.PI);
      ctx.strokeStyle = '#FF9933';
      ctx.beginPath();
      ctx.arc(sc.x, sc.y, 10, 0, Math.PI * 2);
      ctx.stroke();
      const tip = headingTip(drafting.x, drafting.y, drafting.yaw, 0.40);
      ctx.beginPath();
      ctx.moveTo(sc.x, sc.y);
      ctx.lineTo(tip.x, tip.y);
      ctx.stroke();
      ctx.font = 'bold 11px Outfit, sans-serif';
      ctx.fillStyle = '#000080';
      ctx.textAlign = 'left';
      ctx.textBaseline = 'bottom';
      ctx.fillText(`${deg}°`, tip.x + 6, tip.y - 4);
    }

    drawSim();
    drawFootprint();
    drawMeasures();
    drawCompass();
  }

  function drawSim() {
    if (!mapData || (!simPoints.length && !simPath.length)) return;
    const dpr = window.devicePixelRatio || 1;
    if (simPath.length >= 2) {
      ctx.save();
      if (simStyle === 'smoothed') {
        drawPoly(simPath, 'rgba(0,137,123,0.95)', 3);
      } else {
        ctx.setLineDash([8 * dpr, 5 * dpr]);
        drawPoly(simPath, 'rgba(123,31,162,0.95)', 3);
      }
      ctx.restore();
    }
    simPoints.forEach((p, i) => {
      const sc = worldToScreen(p.x, p.y);
      ctx.beginPath();
      ctx.arc(sc.x, sc.y, 9, 0, Math.PI * 2);
      ctx.fillStyle = i === 0 ? '#138808' : '#c62828';
      ctx.fill();
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.fillStyle = '#fff';
      ctx.font = 'bold 11px Outfit, sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(i === 0 ? 'A' : 'B', sc.x, sc.y);
    });
  }

  /** Screen-space quad for a cell-aligned box (respects view rotation). */
  function fillCellBox(gx0, gy0, gx1, gy1, fill, stroke) {
    const res = mapData.resolution;
    const x0 = Math.min(gx0, gx1);
    const x1 = Math.max(gx0, gx1) + 1;
    const y0 = Math.min(gy0, gy1);
    const y1 = Math.max(gy0, gy1) + 1;
    const corners = [
      [x0, y0], [x1, y0], [x1, y1], [x0, y1],
    ].map(([gx, gy]) => worldToScreen(
      mapData.origin.x + gx * res,
      mapData.origin.y + gy * res));
    ctx.beginPath();
    ctx.moveTo(corners[0].x, corners[0].y);
    for (let i = 1; i < 4; i++) ctx.lineTo(corners[i].x, corners[i].y);
    ctx.closePath();
    if (fill) { ctx.fillStyle = fill; ctx.fill(); }
    if (stroke) {
      ctx.strokeStyle = stroke;
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }
  }

  function drawPaintPreview() {
    if (!mapData || !isPaintTool(tool)) return;
    const wallish = paintAction(tool) === 'wall';
    const fill = wallish ? 'rgba(0,0,0,0.5)' : 'rgba(255,255,255,0.72)';
    const edge = wallish ? 'rgba(0,0,128,0.9)' : 'rgba(198,40,40,0.9)';
    const lo = Math.floor((brushCells - 1) / 2);
    const hi = brushCells - 1 - lo;

    if (boxDraft) {
      fillCellBox(boxDraft.gx0, boxDraft.gy0, boxDraft.gx1, boxDraft.gy1, fill, edge);
      return;
    }
    if (painting && strokePts.length) {
      // Draw the brush square at each sample instead of every painted cell —
      // a wide brush over a long drag can cover >100k cells.
      strokePts.forEach(([gx, gy]) => {
        fillCellBox(gx - lo, gy - lo, gx + hi, gy + hi, fill, null);
      });
      return;
    }
    if (hoverCell) {
      fillCellBox(
        hoverCell[0] - lo, hoverCell[1] - lo,
        hoverCell[0] + hi, hoverCell[1] + hi,
        wallish ? 'rgba(0,0,0,0.18)' : 'rgba(255,255,255,0.35)', edge);
    }
  }

  function drawRect(r, fill, stroke) {
    const corners = [];
    const c = Math.cos(r.yaw || 0);
    const s = Math.sin(r.yaw || 0);
    const hw = (r.w || 0.2) / 2;
    const hh = (r.h || 0.2) / 2;
    for (const [lx, ly] of [[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]]) {
      corners.push({
        x: r.cx + c * lx - s * ly,
        y: r.cy + s * lx + c * ly,
      });
    }
    ctx.beginPath();
    const s0 = worldToScreen(corners[0].x, corners[0].y);
    ctx.moveTo(s0.x, s0.y);
    for (let i = 1; i < 4; i++) {
      const p = worldToScreen(corners[i].x, corners[i].y);
      ctx.lineTo(p.x, p.y);
    }
    ctx.closePath();
    ctx.fillStyle = fill;
    ctx.fill();
    ctx.strokeStyle = stroke;
    ctx.lineWidth = 2;
    ctx.stroke();
  }

  return {
    init, setMap, setPose, setScan, setTrail, setPaths, setSim,
    setWaypoints, setRestrictions, setTool, setOnEditCells, setOnRestrict,
    setDims, setShowTrail, clearTrailLocal, undo, canUndo, clearUndo,
    clearMeasures, resize, draw,
    setBrush, getBrush, setOnBrushChange,
    removeRestrictionAt, clearRestrictions, restrictionIndexAt,
  };
})();
