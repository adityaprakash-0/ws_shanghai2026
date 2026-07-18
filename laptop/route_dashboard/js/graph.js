/** Local route graph — build on laptop, export GeoJSON for Pi */
const RouteGraph = (() => {
  let nodes = [];
  let edges = [];
  let nextNodeId = 0;
  let nextEdgeId = 0;
  let autoChain = false;
  let mapName = 'map';
  const undoStack = [];

  function setMapName(name) {
    mapName = name || 'map';
  }

  function clear() {
    nodes = [];
    edges = [];
    nextNodeId = 0;
    nextEdgeId = 0;
    undoStack.length = 0;
  }

  function findNode(id) {
    return nodes.find((n) => n.id === id);
  }

  function syncIdCounters() {
    nextNodeId = nodes.length ? Math.max(...nodes.map((n) => n.id)) + 1 : 0;
    nextEdgeId = edges.length ? Math.max(...edges.map((e) => e.id)) + 1 : 0;
  }

  function hasEdge(a, b) {
    return edges.some((e) =>
      (e.start === a && e.end === b) || (e.start === b && e.end === a));
  }

  function removeEdgesForNode(nodeId) {
    const removed = edges.filter((e) => e.start === nodeId || e.end === nodeId);
    edges = edges.filter((e) => e.start !== nodeId && e.end !== nodeId);
    return removed;
  }

  function addNode(x, y, isClear = null) {
    const id = nextNodeId++;
    const node = { id, x, y, name: String(id) };
    nodes.push(node);
    undoStack.push({ kind: 'node', id });

    if (autoChain && nodes.length > 1) {
      const prev = nodes[nodes.length - 2];
      if (!isClear || isClear(prev.x, prev.y, x, y)) {
        addEdge(prev.id, id, true, false);
      }
    }
    return node;
  }

  function addEdge(start, end, bidirectional = true, trackUndo = true) {
    if (start === end || hasEdge(start, end)) return null;
    const edge = { id: nextEdgeId++, start, end, bidirectional, auto: false };
    edges.push(edge);
    if (trackUndo) undoStack.push({ kind: 'edge', id: edge.id });
    return edge;
  }

  function removeEdgeById(edgeId) {
    edges = edges.filter((e) => e.id !== edgeId);
  }

  function undo() {
    if (!undoStack.length) return false;
    const entry = undoStack.pop();
    if (entry.kind === 'edge') {
      removeEdgeById(entry.id);
      syncIdCounters();
      return true;
    }
    if (entry.kind === 'node') {
      removeEdgesForNode(entry.id);
      nodes = nodes.filter((n) => n.id !== entry.id);
      syncIdCounters();
      return true;
    }
    return false;
  }

  function removeBlockedEdges(isClear) {
    let removed = 0;
    edges = edges.filter((e) => {
      const a = findNode(e.start);
      const b = findNode(e.end);
      if (!a || !b) return false;
      if (isClear(a.x, a.y, b.x, b.y)) return true;
      removed++;
      return false;
    });
    syncIdCounters();
    return removed;
  }

  function autoconnect(isClear) {
    removeBlockedEdges(isClear);
    let added = 0;
    let blocked = 0;
    for (let i = 0; i < nodes.length; i++) {
      for (let j = i + 1; j < nodes.length; j++) {
        const a = nodes[i];
        const b = nodes[j];
        if (hasEdge(a.id, b.id)) continue;
        if (isClear(a.x, a.y, b.x, b.y)) {
          const edge = addEdge(a.id, b.id, true, false);
          if (edge) {
            edge.auto = true;
            added++;
          }
        } else {
          blocked++;
        }
      }
    }
    return { added, blocked };
  }

  function toGeoJSON() {
    const features = [];
    nodes.forEach((n) => {
      features.push({
        type: 'Feature',
        geometry: { type: 'Point', coordinates: [n.x, n.y] },
        properties: { id: n.id, frame: 'map', name: n.name },
      });
    });
    let revId = nextEdgeId;
    edges.forEach((e) => {
      const sn = findNode(e.start);
      const en = findNode(e.end);
      if (!sn || !en) return;
      features.push({
        type: 'Feature',
        geometry: {
          type: 'MultiLineString',
          coordinates: [[[sn.x, sn.y], [en.x, en.y]]],
        },
        properties: { id: e.id, startid: e.start, endid: e.end },
      });
      if (e.bidirectional) {
        features.push({
          type: 'Feature',
          geometry: {
            type: 'MultiLineString',
            coordinates: [[[en.x, en.y], [sn.x, sn.y]]],
          },
          properties: { id: revId++, startid: e.end, endid: e.start },
        });
      }
    });
    return { type: 'FeatureCollection', features };
  }

  /**
   * Load FeatureCollection from route_dashboard / nav2_route GeoJSON.
   * Directed reverse edges collapse into one bidirectional undirected edge.
   */
  function fromGeoJSON(geojson) {
    if (!geojson || geojson.type !== 'FeatureCollection' || !Array.isArray(geojson.features)) {
      throw new Error('Not a GeoJSON FeatureCollection');
    }

    const loadedNodes = [];
    const directed = [];

    geojson.features.forEach((feat) => {
      if (!feat || !feat.geometry || !feat.properties) return;
      const g = feat.geometry;
      const p = feat.properties;

      if (g.type === 'Point' && p.id != null && Array.isArray(g.coordinates) && g.coordinates.length >= 2) {
        const id = Number(p.id);
        loadedNodes.push({
          id,
          x: Number(g.coordinates[0]),
          y: Number(g.coordinates[1]),
          name: p.name != null ? String(p.name) : String(id),
        });
        return;
      }

      if ((g.type === 'MultiLineString' || g.type === 'LineString') &&
          p.startid != null && p.endid != null) {
        directed.push({
          id: p.id != null ? Number(p.id) : null,
          start: Number(p.startid),
          end: Number(p.endid),
        });
      }
    });

    if (!loadedNodes.length) {
      throw new Error('GeoJSON has no Point nodes with id');
    }

    clear();
    nodes = loadedNodes.slice().sort((a, b) => a.id - b.id);

    const seen = new Set();
    directed.forEach((d) => {
      if (!findNode(d.start) || !findNode(d.end) || d.start === d.end) return;
      const lo = Math.min(d.start, d.end);
      const hi = Math.max(d.start, d.end);
      const key = `${lo}|${hi}`;
      if (seen.has(key)) return;
      seen.add(key);
      const edgeId = d.id != null ? d.id : nextEdgeId;
      edges.push({
        id: edgeId,
        start: d.start,
        end: d.end,
        bidirectional: true,
        auto: false,
      });
    });

    syncIdCounters();
    undoStack.length = 0;
    return { nodes: nodes.length, edges: edges.length };
  }

  function exportDownload(filename) {
    const geojson = toGeoJSON();
    const blob = new Blob([JSON.stringify(geojson, null, 2)], { type: 'application/json' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = filename || `${mapName}_graph.geojson`;
    a.click();
    URL.revokeObjectURL(a.href);
  }

  function snapshot() {
    return {
      nodes: nodes.map((n) => ({ ...n })),
      edges: edges.map((e) => ({
        id: e.id,
        auto: !!e.auto,
        x1: findNode(e.start)?.x,
        y1: findNode(e.start)?.y,
        x2: findNode(e.end)?.x,
        y2: findNode(e.end)?.y,
      })),
    };
  }

  return {
    setMapName,
    getMapName: () => mapName,
    clear,
    addNode,
    addEdge,
    autoconnect,
    removeBlockedEdges,
    undo,
    toGeoJSON,
    fromGeoJSON,
    exportDownload,
    snapshot,
    setAutoChain: (v) => { autoChain = v; },
    canUndo: () => undoStack.length > 0,
  };
})();
