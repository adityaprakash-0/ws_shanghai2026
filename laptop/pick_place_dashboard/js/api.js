/** Pick-place dashboard API — port 8081, separate localStorage keys */
const PpApi = (() => {
  const DEFAULT_URL = 'http://10.42.0.1:8081';
  const LS_URL = 'pp_dashboard_pi_url';
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
    return request(`/api/map?path=${encodeURIComponent(path)}`);
  }
  async function estop(active) {
    return request('/api/estop', { method: 'POST', body: JSON.stringify({ active }) });
  }
  async function setTask(task) {
    return request('/api/mission/task', { method: 'POST', body: JSON.stringify({ task }) });
  }
  async function startMission() {
    return request('/api/mission/start', { method: 'POST', body: '{}' });
  }
  async function stopMission() {
    return request('/api/mission/stop', { method: 'POST', body: '{}' });
  }
  async function resetOdom() {
    return request('/api/odom/reset', { method: 'POST', body: '{}' });
  }
  async function manipulatorCmd(cmd) {
    return request('/api/manipulator/cmd', { method: 'POST', body: JSON.stringify(cmd) });
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
    setBaseUrl, getBaseUrl, getDefaultUrl, status, telemetry, fetchMaps, fetchMap,
    estop, setTask, startMission, stopMission, resetOdom, manipulatorCmd,
    connectStream, disconnectStream,
  };
})();
