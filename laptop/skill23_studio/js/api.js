/** Skill23 Studio API — Pi bridge :8083 */
const StudioApi = (() => {
  const DEFAULT_URL = 'http://10.42.0.1:8083';
  const LS_URL = 'skill23_studio_pi_url';
  let baseUrl = localStorage.getItem(LS_URL) || DEFAULT_URL;
  let eventSource = null;
  let pollTimer = null;
  let onTelemetry = null;

  function setBaseUrl(url) {
    baseUrl = String(url || '').replace(/\/$/, '');
    localStorage.setItem(LS_URL, baseUrl);
  }
  function getBaseUrl() { return baseUrl; }
  function getDefaultUrl() { return DEFAULT_URL; }

  async function request(path, options = {}) {
    const res = await fetch(`${baseUrl}${path}`, {
      headers: { 'Content-Type': 'application/json', ...(options.headers || {}) },
      ...options,
    });
    const data = await res.json().catch(() => ({}));
    if (!res.ok) throw new Error(data.error || data.message || res.statusText);
    return data;
  }

  const get = (p) => request(p);
  const post = (p, body = {}) => request(p, { method: 'POST', body: JSON.stringify(body) });

  function connectStream(cb) {
    onTelemetry = cb;
    disconnectStream();
    try {
      eventSource = new EventSource(`${baseUrl}/api/stream`);
      eventSource.onmessage = (ev) => {
        try { onTelemetry && onTelemetry(JSON.parse(ev.data)); } catch (_) {}
      };
      eventSource.onerror = () => {
        if (eventSource) { eventSource.close(); eventSource = null; }
        if (!pollTimer) {
          pollTimer = setInterval(async () => {
            try {
              const t = await get('/api/telemetry');
              onTelemetry && onTelemetry(t);
            } catch (_) {}
          }, 400);
        }
      };
    } catch (_) {
      pollTimer = setInterval(async () => {
        try {
          const t = await get('/api/telemetry');
          onTelemetry && onTelemetry(t);
        } catch (_) {}
      }, 400);
    }
  }

  function disconnectStream() {
    if (eventSource) { eventSource.close(); eventSource = null; }
    if (pollTimer) { clearInterval(pollTimer); pollTimer = null; }
  }

  return {
    setBaseUrl, getBaseUrl, getDefaultUrl, connectStream, disconnectStream,
    status: () => get('/api/status'),
    telemetry: () => get('/api/telemetry'),
    maps: () => get('/api/maps'),
    session: (body) => post('/api/session', body),
    mapDelete: (name) => post('/api/map/delete', { map_name: name }),
    mappingStart: (body) => post('/api/mapping/start', body),
    mappingStop: () => post('/api/mapping/stop'),
    mappingRemap: () => post('/api/mapping/remap'),
    mapSave: (body) => post('/api/map/save', body || {}),
    mapImport: (body) => post('/api/map/import', body),
    mapEdit: (ops) => post('/api/map/edit', { ops }),
    mapRestrictions: (rectangles, burn = false) =>
      post('/api/map/restrictions', { rectangles, burn }),
    teleop: (vx, vy, wz) => post('/api/teleop', { vx, vy, wz }),
    waypointsGet: () => get('/api/waypoints'),
    waypointsSave: (waypoints) => post('/api/waypoints', { waypoints }),
    restrictionsGet: () => get('/api/restrictions'),
    phase: (phase) => post('/api/phase', { phase }),
    deployStart: (body) => post('/api/deploy/start', body || {}),
    deployStop: () => post('/api/deploy/stop'),
    clearCostmaps: () => post('/api/nav2/clear_costmaps'),
    estop: (active) => post('/api/estop', { active }),
    setPose: (x, y, yaw) => post('/api/set_pose', { x, y, yaw }),
    setGoal: (x, y, yaw) => post('/api/goal', { x, y, yaw }),
    setGoals: (waypoints, sequence) => post('/api/goal', {
      waypoints,
      sequence: sequence || undefined,
    }),
    armMission: (waypoints, sequence) => post('/api/mission/arm', {
      waypoints,
      sequence: sequence || '',
    }),
    start: (body) => post('/api/start', body || {}),
    stop: (body) => post('/api/stop', body || {}),
    pathStyle: (style) => post('/api/path_style', { style }),
    relocStart: (motion) => post('/api/reloc/start', { motion }),
    relocStop: () => post('/api/reloc/stop'),
    relocThenTask: (motion, waypoints) => post('/api/reloc/then_task', {
      motion,
      waypoints: waypoints || undefined,
    }),
    trailClear: () => post('/api/trail/clear'),
    simPlan: (body) => post('/api/sim/plan', body),
    simClear: () => post('/api/sim/clear'),
    logs: () => get('/api/logs'),
    localizeStart: (body) => post('/api/localize/start', body || {}),
    localizeStop: () => post('/api/localize/stop'),
    cameraOn: () => post('/api/camera/on'),
    cameraOff: () => post('/api/camera/off'),
    cameraKill: () => post('/api/camera/kill'),
  };
})();
