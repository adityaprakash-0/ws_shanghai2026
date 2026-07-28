/** Case-nav dashboard API — port 8082 */
const CnApi = (() => {
  const DEFAULT_URL = 'http://10.42.0.1:8082';
  const LS_URL = 'cn_dashboard_pi_url';
  let baseUrl = localStorage.getItem(LS_URL) || DEFAULT_URL;
  let eventSource = null;
  let pollTimer = null;
  let onTelemetry = null;

  function setBaseUrl(url) {
    baseUrl = url.replace(/\/$/, '');
    localStorage.setItem(LS_URL, baseUrl);
  }
  function getBaseUrl() { return baseUrl; }
  function getDefaultUrl() { return DEFAULT_URL; }

  async function request(path, options = {}) {
    const res = await fetch(`${baseUrl}${path}`, {
      headers: { 'Content-Type': 'application/json', ...options.headers },
      ...options,
    });
    const data = await res.json().catch(() => ({}));
    if (!res.ok) throw new Error(data.message || data.error || res.statusText);
    return data;
  }

  async function status() { return request('/api/status'); }
  async function telemetry() { return request('/api/telemetry'); }
  async function fetchMaps() { return request('/api/maps'); }
  async function fetchMap(path) {
    return request(`/api/map?path=${encodeURIComponent(path || '')}`);
  }
  async function loadMap(path) {
    return request('/api/map/load', { method: 'POST', body: JSON.stringify({ path }) });
  }
  async function estop(active) {
    return request('/api/estop', { method: 'POST', body: JSON.stringify({ active }) });
  }
  async function setPose(x, y, yaw) {
    return request('/api/set_pose', {
      method: 'POST',
      body: JSON.stringify({ x, y, yaw }),
    });
  }
  async function setGoal(x, y, yaw = 0) {
    return request('/api/goal', {
      method: 'POST',
      body: JSON.stringify({ x, y, yaw }),
    });
  }
  async function setGoals(waypoints) {
    return request('/api/goal', {
      method: 'POST',
      body: JSON.stringify({ waypoints }),
    });
  }
  async function start() {
    return request('/api/start', { method: 'POST', body: '{}' });
  }
  async function stop() {
    return request('/api/stop', { method: 'POST', body: '{}' });
  }

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
              const t = await telemetry();
              onTelemetry && onTelemetry(t);
            } catch (_) {}
          }, 400);
        }
      };
    } catch (_) {
      pollTimer = setInterval(async () => {
        try {
          const t = await telemetry();
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
    setBaseUrl, getBaseUrl, getDefaultUrl,
    status, telemetry, fetchMaps, fetchMap, loadMap,
    estop, setPose, setGoal, setGoals, start, stop,
    connectStream, disconnectStream,
  };
})();
