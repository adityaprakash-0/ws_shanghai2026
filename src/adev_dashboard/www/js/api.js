/** ADEV Dashboard API client — lightweight HTTP/SSE (no rosbridge) */
const AdevApi = (() => {
  const DEFAULT_URL = 'http://10.42.0.1:8080';
  let baseUrl = DEFAULT_URL;
  let eventSource = null;
  let pollTimer = null;
  let onTelemetry = null;

  function setBaseUrl(url) {
    baseUrl = url.replace(/\/$/, '');
  }

  function getBaseUrl() {
    return baseUrl;
  }

  function getDefaultUrl() {
    return DEFAULT_URL;
  }

  async function request(path, options = {}) {
    const res = await fetch(`${baseUrl}${path}`, {
      headers: { 'Content-Type': 'application/json', ...options.headers },
      ...options,
    });
    const data = await res.json().catch(() => ({}));
    if (!res.ok) {
      throw new Error(data.message || data.error || res.statusText);
    }
    return data;
  }

  async function status() {
    return request('/api/status');
  }

  async function telemetry() {
    return request('/api/telemetry');
  }

  async function fetchMaps() {
    return request('/api/maps');
  }

  async function fetchMap(path) {
    return request(`/api/map?path=${encodeURIComponent(path)}`);
  }

  async function estop(active) {
    return request('/api/estop', { method: 'POST', body: JSON.stringify({ active }) });
  }

  async function setWaypoints(waypoints) {
    return request('/api/mission/waypoints', {
      method: 'POST',
      body: JSON.stringify({ waypoints }),
    });
  }

  async function startMission() {
    return request('/api/mission/start', { method: 'POST', body: '{}' });
  }

  async function setSimulation(enabled) {
    return request('/api/simulation', {
      method: 'POST',
      body: JSON.stringify({ enabled }),
    });
  }

  async function stopMission() {
    return request('/api/mission/stop', { method: 'POST', body: '{}' });
  }

  async function setInitialPose(x, y, yaw) {
    return request('/api/initial_pose', {
      method: 'POST',
      body: JSON.stringify({ x, y, yaw }),
    });
  }

  function stopPolling() {
    if (pollTimer) {
      clearInterval(pollTimer);
      pollTimer = null;
    }
  }

  function startPolling(callback) {
    stopPolling();
    const tick = async () => {
      try {
        const data = await telemetry();
        callback(data);
      } catch (_) {
        callback({ _offline: true });
      }
    };
    tick();
    pollTimer = setInterval(tick, 300);
  }

  function connectStream(callback) {
    onTelemetry = callback;
    disconnectStream();

    let sseOk = false;
    try {
      eventSource = new EventSource(`${baseUrl}/api/stream`);
      eventSource.onmessage = (ev) => {
        sseOk = true;
        stopPolling();
        try {
          callback(JSON.parse(ev.data));
        } catch (_) { /* ignore */ }
      };
      eventSource.onerror = () => {
        if (eventSource) {
          eventSource.close();
          eventSource = null;
        }
        if (!sseOk) {
          startPolling(callback);
        } else {
          callback({ _offline: true });
          startPolling(callback);
        }
      };
    } catch (_) {
      startPolling(callback);
    }

    setTimeout(() => {
      if (!sseOk && !pollTimer) startPolling(callback);
    }, 1500);
  }

  function disconnectStream() {
    stopPolling();
    if (eventSource) {
      eventSource.close();
      eventSource = null;
    }
  }

  return {
    setBaseUrl, getBaseUrl, getDefaultUrl, status, telemetry, fetchMaps, fetchMap,
    estop, setWaypoints, startMission, stopMission, setInitialPose, setSimulation,
    connectStream, disconnectStream,
  };
})();
