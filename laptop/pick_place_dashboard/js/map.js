/** Pick-place map — occupancy grid + object/station stickers */
const PpMap = (() => {
  const canvas = () => document.getElementById('mapCanvas');
  const ctx = () => canvas().getContext('2d');

  let meta = null;
  let width = 0;
  let height = 0;
  let mapBitmap = null;
  let scale = 1;
  let offsetX = 0;
  let offsetY = 0;
  let pose = { x: 0, y: 0, yaw: 0 };
  let stickers = [];
  let mode = 'sticker';
  let measureStart = null;
  let measureEnd = null;
  let onStickerAdd = null;
  let followRobot = false;
  let userPanned = false;

  function worldToPixel(wx, wy) {
    if (!meta) return { px: 0, py: 0 };
    const res = meta.resolution;
    return {
      px: (wx - meta.origin[0]) / res,
      py: height - (wy - meta.origin[1]) / res,
    };
  }

  function pixelToWorld(px, py) {
    if (!meta) return { x: 0, y: 0 };
    const res = meta.resolution;
    return {
      x: px * res + meta.origin[0],
      y: (height - py) * res + meta.origin[1],
    };
  }

  function screenToPixel(sx, sy) {
    const rect = canvas().getBoundingClientRect();
    return {
      px: (sx - rect.left - offsetX) / scale,
      py: (sy - rect.top - offsetY) / scale,
    };
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
    return { width: w, height: h, data: bytes.subarray(i, i + w * h) };
  }

  function decodePgm(base64) {
    const raw = atob(base64);
    const bytes = new Uint8Array(raw.length);
    for (let i = 0; i < raw.length; i++) bytes[i] = raw.charCodeAt(i);
    return decodePgmBytes(bytes);
  }

  function pixelColor(v) {
    if (v < 50) return [30, 30, 35];
    if (v > 200) return [232, 236, 240];
    return [120, 130, 145];
  }

  function buildBitmap(pgm) {
    width = pgm.width;
    height = pgm.height;
    const img = ctx().createImageData(width, height);
    for (let i = 0; i < pgm.data.length; i++) {
      const [r, g, b] = pixelColor(pgm.data[i]);
      const j = i * 4;
      img.data[j] = r; img.data[j + 1] = g; img.data[j + 2] = b; img.data[j + 3] = 255;
    }
    const off = document.createElement('canvas');
    off.width = width; off.height = height;
    off.getContext('2d').putImageData(img, 0, 0);
    mapBitmap = off;
  }

  function fit() {
    const c = canvas();
    const rect = c.getBoundingClientRect();
    c.width = rect.width;
    c.height = rect.height;
    if (!width || !height) return;
    scale = Math.min(rect.width / width, rect.height / height) * 0.95;
    offsetX = (rect.width - width * scale) / 2;
    offsetY = (rect.height - height * scale) / 2;
    userPanned = false;
    draw();
  }

  function setMap(info) {
    meta = info;
    buildBitmap(decodePgm(info.pgm_base64));
    fit();
  }

  function setLocalMap(yamlText, pgmBytes) {
    const lines = yamlText.split('\n');
    const metaObj = {};
    for (const line of lines) {
      const m = line.match(/^(\w+):\s*(.+)$/);
      if (!m) continue;
      let v = m[2].trim();
      if (v.startsWith('[')) {
        metaObj[m[1]] = JSON.parse(v.replace(/,/g, ','));
      } else if (!isNaN(Number(v))) {
        metaObj[m[1]] = Number(v);
      } else {
        metaObj[m[1]] = v.replace(/^["']|["']$/g, '');
      }
    }
    if (typeof metaObj.origin === 'string') {
      metaObj.origin = JSON.parse(metaObj.origin.replace(/\s+/g, ''));
    }
    meta = {
      resolution: Number(metaObj.resolution || 0.05),
      origin: metaObj.origin || [0, 0, 0],
      image: metaObj.image || '',
    };
    buildBitmap(decodePgmBytes(new Uint8Array(pgmBytes)));
    fit();
  }

  function setPose(p) {
    if (!p) return;
    pose = { x: p.x || 0, y: p.y || 0, yaw: p.yaw || 0 };
    draw();
  }

  function setStickers(list) {
    stickers = list || [];
    draw();
  }

  function getStickers() { return stickers.slice(); }

  function setMode(m) { mode = m; }

  function draw() {
    const c = canvas();
    const g = ctx();
    const rect = c.getBoundingClientRect();
    if (c.width !== rect.width || c.height !== rect.height) {
      c.width = rect.width;
      c.height = rect.height;
    }
    g.fillStyle = '#e4e8ec';
    g.fillRect(0, 0, c.width, c.height);
    if (!mapBitmap) return;

    g.save();
    g.translate(offsetX, offsetY);
    g.scale(scale, scale);
    g.drawImage(mapBitmap, 0, 0);

    // Stickers
    for (const s of stickers) {
      const { px, py } = worldToPixel(s.x, s.y);
      g.beginPath();
      g.arc(px, py, 7 / scale, 0, Math.PI * 2);
      g.fillStyle = s.kind === 'station' ? '#138808' : '#ff9933';
      g.fill();
      g.strokeStyle = '#000080';
      g.lineWidth = 1.5 / scale;
      g.stroke();
      // yaw tick
      g.beginPath();
      g.moveTo(px, py);
      g.lineTo(px + Math.cos(s.yaw || 0) * 14 / scale, py - Math.sin(s.yaw || 0) * 14 / scale);
      g.stroke();
      g.fillStyle = '#0a1628';
      g.font = `${11 / scale}px sans-serif`;
      g.fillText(s.name || '', px + 10 / scale, py - 8 / scale);
    }

    // Robot
    {
      const { px, py } = worldToPixel(pose.x, pose.y);
      g.save();
      g.translate(px, py);
      g.rotate(-pose.yaw);
      g.fillStyle = '#000080';
      g.beginPath();
      g.moveTo(12 / scale, 0);
      g.lineTo(-8 / scale, 7 / scale);
      g.lineTo(-8 / scale, -7 / scale);
      g.closePath();
      g.fill();
      g.restore();
    }

    // Measure
    if (measureStart && measureEnd) {
      const a = worldToPixel(measureStart.x, measureStart.y);
      const b = worldToPixel(measureEnd.x, measureEnd.y);
      g.strokeStyle = '#e63946';
      g.lineWidth = 2 / scale;
      g.beginPath();
      g.moveTo(a.px, a.py);
      g.lineTo(b.px, b.py);
      g.stroke();
    }
    g.restore();
  }

  function bindEvents() {
    const c = canvas();
    let dragging = false;
    let lastX = 0, lastY = 0;

    c.addEventListener('wheel', (e) => {
      e.preventDefault();
      const factor = e.deltaY < 0 ? 1.1 : 0.9;
      const rect = c.getBoundingClientRect();
      const mx = e.clientX - rect.left;
      const my = e.clientY - rect.top;
      offsetX = mx - (mx - offsetX) * factor;
      offsetY = my - (my - offsetY) * factor;
      scale *= factor;
      userPanned = true;
      draw();
    }, { passive: false });

    c.addEventListener('mousedown', (e) => {
      if (e.altKey || e.button === 1) {
        dragging = true;
        lastX = e.clientX;
        lastY = e.clientY;
        userPanned = true;
        return;
      }
      const { px, py } = screenToPixel(e.clientX, e.clientY);
      const w = pixelToWorld(px, py);
      if (mode === 'measure') {
        if (!measureStart) measureStart = w;
        else { measureEnd = w; }
        draw();
        return;
      }
      if (onStickerAdd) onStickerAdd(w.x, w.y, mode);
    });

    window.addEventListener('mousemove', (e) => {
      if (!dragging) return;
      offsetX += e.clientX - lastX;
      offsetY += e.clientY - lastY;
      lastX = e.clientX;
      lastY = e.clientY;
      draw();
    });
    window.addEventListener('mouseup', () => { dragging = false; });
    window.addEventListener('resize', () => fit());
  }

  function clearMeasure() {
    measureStart = null;
    measureEnd = null;
    draw();
  }

  function measureDistance() {
    if (!measureStart || !measureEnd) return null;
    const dx = measureEnd.x - measureStart.x;
    const dy = measureEnd.y - measureStart.y;
    return Math.hypot(dx, dy);
  }

  return {
    setMap, setLocalMap, setPose, setStickers, getStickers, setMode, fit, draw,
    bindEvents, onStickerAdd: (cb) => { onStickerAdd = cb; },
    clearMeasure, measureDistance, worldToPixel,
  };
})();
