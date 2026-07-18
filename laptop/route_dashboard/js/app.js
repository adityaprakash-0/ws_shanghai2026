/** Route Graph Builder — offline only, ADEV-style map UI */
(() => {
  const $ = (id) => document.getElementById(id);

  /** Remember which GeoJSON file was loaded so Save can overwrite it. */
  let loadedGraphName = '';
  let graphFileHandle = null;

  function setMapStatus(msg, ok) {
    const el = $('mapStatus');
    if (!el) return;
    el.textContent = msg || '';
    el.className = 'map-status' + (ok === true ? ' ok' : ok === false ? ' err' : '');
  }

  function setGraphStatus(msg, ok) {
    const el = $('graphFileStatus');
    if (!el) return;
    el.textContent = msg || '';
    el.className = 'map-status' + (ok === true ? ' ok' : ok === false ? ' err' : '');
  }

  function syncWallClearanceFromUI() {
    const input = $('wallClearanceM');
    if (!input) return;
    MapView.setWallClearance(input.value);
    const hint = $('wallClearanceHint');
    if (hint && MapView.hasMap()) {
      const cells = MapView.getClearanceCells();
      hint.textContent =
        `Edges/nodes stay ≥ ${MapView.getWallClearance().toFixed(2)} m from walls ` +
        `(${cells} cell${cells === 1 ? '' : 's'} at map resolution). Re-run autoconnect after changing.`;
    }
  }

  function refreshGraphView() {
    const snap = RouteGraph.snapshot();
    $('nodeCount').textContent = snap.nodes.length;
    $('edgeCount').textContent = snap.edges.length;
    MapView.setGraph(snap.nodes, snap.edges);
    const ul = $('nodeList');
    ul.innerHTML = snap.nodes.map((n) =>
      `<li><span>#${n.id} (${n.x.toFixed(2)}, ${n.y.toFixed(2)})</span></li>`).join('');
    syncWallClearanceFromUI();
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
    if (!out.origin || out.origin.length < 2) out.origin = [0, 0, 0];
    return out;
  }

  async function readFileAsArrayBuffer(file) {
    try {
      return await file.arrayBuffer();
    } catch (e) {
      throw new Error(`Cannot read ${file.name}. Run: chmod 644 "${file.name}"`);
    }
  }

  async function readFileAsText(file) {
    try {
      return await file.text();
    } catch (e) {
      throw new Error(`Cannot read ${file.name}. Run: chmod 644 "${file.name}"`);
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
        const imageName = (yamlMeta.image || stem + '.pgm').toLowerCase();
        pgmFile = [...$('mapPgmFile').files].find((f) => f.name.toLowerCase() === imageName)
          || $('mapPgmFile').files[0];
      }

      if (!pgmFile) {
        setMapStatus('Also select the .pgm file', false);
        alert('Select the matching .pgm file.\n\nYaml image: ' + (yamlMeta.image || '(not set)'));
        return;
      }

      const hadGraph = RouteGraph.snapshot().nodes.length > 0;
      if (hadGraph) {
        const keep = window.confirm(
          'Map reload normally clears the graph.\n\nOK = keep current nodes/edges\nCancel = clear graph');
        if (!keep) {
          RouteGraph.clear();
          loadedGraphName = '';
          graphFileHandle = null;
        }
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

      const stem = yamlFile.name.replace(/\.(yaml|yml)$/i, '');
      RouteGraph.setMapName(stem);
      refreshGraphView();
      setMapStatus(`Loaded: ${yamlFile.name} + ${pgmFile.name}`, true);
      $('connText').textContent = 'MAP LOADED';
    } catch (e) {
      console.error(e);
      setMapStatus('Map error: ' + e.message, false);
      alert('Map load failed: ' + e.message);
    }
  }

  function applyLoadedGraph(geojson, fileName) {
    const result = RouteGraph.fromGeoJSON(geojson);
    loadedGraphName = fileName || `${RouteGraph.getMapName()}_graph.geojson`;
    if (!RouteGraph.getMapName() || RouteGraph.getMapName() === 'map') {
      const stem = loadedGraphName.replace(/_graph\.geojson$/i, '').replace(/\.geojson$/i, '');
      if (stem) RouteGraph.setMapName(stem);
    }
    refreshGraphView();
    setGraphStatus(
      `Loaded ${loadedGraphName}: ${result.nodes} nodes, ${result.edges} edges`,
      true);
    setMapStatus(`Graph: ${result.nodes} nodes · ${result.edges} edges`, true);
    $('connText').textContent = 'GRAPH LOADED';
  }

  async function loadGraphFromInput() {
    const input = $('graphFile');
    const file = input && input.files && input.files[0];
    if (!file) {
      setGraphStatus('Select a .geojson file first', false);
      alert('Select a GeoJSON graph file, then click Load Graph');
      return;
    }
    try {
      const text = await readFileAsText(file);
      const geojson = JSON.parse(text);
      graphFileHandle = null;
      applyLoadedGraph(geojson, file.name);
      if (!MapView.hasMap()) {
        setGraphStatus(
          `Loaded ${file.name} (${RouteGraph.snapshot().nodes.length} nodes). ` +
          'Load yaml+pgm to see them on the map.',
          true);
      }
    } catch (e) {
      console.error(e);
      setGraphStatus('Graph error: ' + e.message, false);
      alert('Graph load failed: ' + e.message);
    }
  }

  /** Chrome/Edge: open with a writable handle so Save can overwrite in place. */
  async function loadGraphWithFilePicker() {
    if (!window.showOpenFilePicker) {
      return loadGraphFromInput();
    }
    try {
      const [handle] = await window.showOpenFilePicker({
        multiple: false,
        types: [{
          description: 'GeoJSON graph',
          accept: {
            'application/geo+json': ['.geojson'],
            'application/json': ['.json', '.geojson'],
          },
        }],
      });
      graphFileHandle = handle;
      const file = await handle.getFile();
      const text = await file.text();
      const geojson = JSON.parse(text);
      applyLoadedGraph(geojson, file.name);
      if (!MapView.hasMap()) {
        setGraphStatus(
          `Loaded ${file.name}. Load yaml+pgm to display on the map.`,
          true);
      }
    } catch (e) {
      if (e && e.name === 'AbortError') return;
      console.error(e);
      setGraphStatus('Graph error: ' + e.message, false);
      alert('Graph load failed: ' + e.message);
    }
  }

  async function saveGraphOverwrite() {
    const snap = RouteGraph.snapshot();
    if (!snap.nodes.length) {
      alert('No nodes to save — load or build a graph first');
      return;
    }

    const payload = JSON.stringify(RouteGraph.toGeoJSON(), null, 2);
    const name = loadedGraphName || `${RouteGraph.getMapName()}_graph.geojson`;

    try {
      if (graphFileHandle && typeof graphFileHandle.createWritable === 'function') {
        const writable = await graphFileHandle.createWritable();
        await writable.write(payload);
        await writable.close();
        setGraphStatus(`Saved (overwrote): ${name}`, true);
        setMapStatus(`Saved ${name}`, true);
        return;
      }

      if (window.showSaveFilePicker) {
        const handle = await window.showSaveFilePicker({
          suggestedName: name,
          types: [{
            description: 'GeoJSON graph',
            accept: { 'application/geo+json': ['.geojson'] },
          }],
        });
        graphFileHandle = handle;
        loadedGraphName = handle.name || name;
        const writable = await handle.createWritable();
        await writable.write(payload);
        await writable.close();
        setGraphStatus(`Saved: ${loadedGraphName}`, true);
        setMapStatus(`Saved ${loadedGraphName}`, true);
        return;
      }

      RouteGraph.exportDownload(name);
      setGraphStatus(
        `Downloaded ${name} — choose overwrite in the save dialog if prompted`,
        true);
      setMapStatus(`Downloaded ${name}`, true);
    } catch (e) {
      if (e && e.name === 'AbortError') return;
      console.error(e);
      setGraphStatus('Save failed: ' + e.message, false);
      alert('Save failed: ' + e.message);
    }
  }

  function bindUI() {
    document.querySelectorAll('.map-toolbar button[data-mode]').forEach((btn) => {
      btn.addEventListener('click', () => {
        document.querySelectorAll('.map-toolbar button[data-mode]').forEach((b) => {
          b.classList.remove('active');
        });
        btn.classList.add('active');
        MapView.setMode(btn.dataset.mode);
      });
    });

    MapView.onGraphNodeAdd = (x, y) => {
      if (!MapView.hasMap()) {
        alert('Load map first (yaml + pgm)');
        return;
      }
      syncWallClearanceFromUI();
      if (!MapView.isPointClear(x, y)) {
        const d = MapView.getWallClearance().toFixed(2);
        setMapStatus(`Node too close to wall — need ≥ ${d} m clearance`, false);
        alert(`This point is within ${d} m of a wall.\n\nPlace the node further from walls/corners.`);
        return;
      }
      RouteGraph.addNode(x, y, (x1, y1, x2, y2) => MapView.lineOfSight(x1, y1, x2, y2));
      refreshGraphView();
    };

    $('btnLoadLocalMap').onclick = loadLocalMap;
    $('mapPgmFile').onchange = () => {
      if ($('mapFile').files[0] && $('mapPgmFile').files[0]) loadLocalMap();
    };

    $('btnLoadGraph').onclick = () => {
      if (window.showOpenFilePicker && (!$('graphFile').files || !$('graphFile').files[0])) {
        loadGraphWithFilePicker();
      } else if ($('graphFile').files && $('graphFile').files[0]) {
        loadGraphFromInput();
      } else {
        loadGraphWithFilePicker();
      }
    };

    const graphInput = $('graphFile');
    if (graphInput) {
      graphInput.onchange = () => {
        if (graphInput.files && graphInput.files[0]) loadGraphFromInput();
      };
    }

    const saveHandler = () => { saveGraphOverwrite(); };
    if ($('btnSaveGraph')) $('btnSaveGraph').onclick = saveHandler;
    if ($('btnSaveGraphSide')) $('btnSaveGraphSide').onclick = saveHandler;

    $('btnUndo').onclick = () => {
      if (RouteGraph.undo()) {
        refreshGraphView();
        setMapStatus('Undid last node or edge', true);
      } else {
        setMapStatus('Nothing to undo', false);
      }
    };

    const wallClearanceInput = $('wallClearanceM');
    if (wallClearanceInput) {
      wallClearanceInput.addEventListener('change', () => {
        syncWallClearanceFromUI();
        if (!MapView.hasMap() || !RouteGraph.snapshot().edges.length) return;
        const n = RouteGraph.removeBlockedEdges(
          (x1, y1, x2, y2) => MapView.lineOfSight(x1, y1, x2, y2));
        refreshGraphView();
        if (n > 0) {
          setMapStatus(`Clearance updated — removed ${n} edge(s) too close to walls`, true);
        }
      });
    }

    $('btnPruneEdges').onclick = () => {
      if (!MapView.hasMap()) { alert('Load map first'); return; }
      syncWallClearanceFromUI();
      const n = RouteGraph.removeBlockedEdges(
        (x1, y1, x2, y2) => MapView.lineOfSight(x1, y1, x2, y2));
      refreshGraphView();
      const d = MapView.getWallClearance().toFixed(2);
      setMapStatus(`Removed ${n} edge(s) within ${d} m of walls`, true);
    };

    $('btnAutoconnect').onclick = () => {
      if (!MapView.hasMap()) { alert('Load map first'); return; }
      syncWallClearanceFromUI();
      const isClear = (x1, y1, x2, y2) => MapView.lineOfSight(x1, y1, x2, y2);
      RouteGraph.removeBlockedEdges(isClear);
      const { added, blocked } = RouteGraph.autoconnect(isClear);
      refreshGraphView();
      const d = MapView.getWallClearance().toFixed(2);
      if (added === 0 && RouteGraph.snapshot().nodes.length >= 2) {
        setMapStatus(
          `No edges added (${blocked} pairs blocked — walls or < ${d} m clearance). ` +
          'Place nodes in corridor centers or reduce clearance slightly.',
          false);
      } else {
        setMapStatus(
          `Autoconnect: +${added} edge(s), ${blocked} blocked (walls / ${d} m clearance)`,
          true);
      }
    };

    $('btnExportGraph').onclick = () => {
      if (!RouteGraph.snapshot().nodes.length) {
        alert('Add nodes first');
        return;
      }
      const name = loadedGraphName || `${RouteGraph.getMapName()}_graph.geojson`;
      RouteGraph.exportDownload(name);
      setMapStatus(`GeoJSON downloaded as ${name} — scp to Pi ~/skill23_ws/`, true);
    };

    $('btnClearGraph').onclick = () => {
      if (!RouteGraph.snapshot().nodes.length) return;
      if (!window.confirm('Clear all nodes and edges? This cannot be undone.')) return;
      RouteGraph.clear();
      loadedGraphName = '';
      graphFileHandle = null;
      refreshGraphView();
      setMapStatus('Graph cleared', true);
      setGraphStatus('', true);
    };

    const fitBtn = $('btnFitMap');
    if (fitBtn) fitBtn.onclick = () => MapView.fitToView && MapView.fitToView();
  }

  function init() {
    MapView.setMode('graph');
    syncWallClearanceFromUI();
    MapView.bindEvents();
    bindUI();
    refreshGraphView();
  }

  document.addEventListener('DOMContentLoaded', init);
})();
