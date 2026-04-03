const STATE_NAMES = ["IDLE", "REFERENCING", "REFERENCED", "MOVING_OPEN", "MOVING_CLOSE", "ERROR"];
const STATE_CLASSES = ["state-idle", "state-ref", "state-ref", "state-moving", "state-moving", "state-error"];
const MAX_RPM = 500;

let ws = null;

function connectWs() {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  ws = new WebSocket(`${proto}//${location.host}/ws`);

  const badge = document.getElementById("ws-status");

  ws.onopen = () => {
    badge.textContent = "Connected";
    badge.classList.add("connected");
  };

  ws.onclose = () => {
    badge.textContent = "Disconnected";
    badge.classList.remove("connected");
    setTimeout(connectWs, 2000);
  };

  ws.onerror = () => ws.close();

  ws.onmessage = (evt) => {
    try {
      const data = JSON.parse(evt.data);
      updateUI(data);
    } catch (e) { /* ignore parse errors */ }
  };
}

function updateUI(data) {
  // Wheels
  const w = data.wheel_status || {};
  setText("w-backend", w.backend || "--");
  setText("w-left", (w.left_rpm || 0).toFixed(0));
  setText("w-right", (w.right_rpm || 0).toFixed(0));
  setBar("bar-left", Math.abs(w.left_rpm || 0), MAX_RPM);
  setBar("bar-right", Math.abs(w.right_rpm || 0), MAX_RPM);

  // Actuators
  for (const name of ["lift", "brush", "bin_door"]) {
    const a = data[name] || {};
    const stateIdx = a.state || 0;
    const el = document.getElementById(`${name}-state`);
    if (el) {
      el.textContent = STATE_NAMES[stateIdx] || "UNKNOWN";
      el.className = "value " + (STATE_CLASSES[stateIdx] || "");
    }
    setText(`${name}-dir`, a.direction || "--");
    setText(`${name}-ref`, a.is_referenced ? "Yes" : "No");
  }
}

function setText(id, text) {
  const el = document.getElementById(id);
  if (el) el.textContent = text;
}

function setBar(id, value, max) {
  const el = document.getElementById(id);
  if (el) {
    const pct = Math.min(100, (value / max) * 100);
    el.style.width = pct + "%";
  }
}

async function cmd(node, action) {
  try {
    await fetch(`/api/command/${node}/${action}`, { method: "POST" });
  } catch (e) {
    console.error("Command failed:", e);
  }
}

async function switchBackend() {
  try {
    await fetch("/api/command/wheel_driver_node/switch_backend", { method: "POST" });
  } catch (e) {
    console.error("Switch failed:", e);
  }
}

connectWs();
