const STATE_NAMES = ["IDLE", "REFERENCING", "REFERENCED", "MOVING_OPEN", "MOVING_CLOSE", "ERROR"];
const STATE_CLASSES = ["state-idle", "state-ref", "state-ref", "state-moving", "state-moving", "state-error"];
const MAX_RPM = 500;

let ws = null;
let ctrlConfig = null;   // fetched from server
let activeInputs = [];    // live from WebSocket

// ─── WebSocket ───────────────────────────────────────────────────

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
      updateDashboard(data);
      activeInputs = (data.gamepad || {}).active_inputs || [];
      updateCtrlHighlights();
    } catch (e) { /* ignore */ }
  };
}

// ─── Dashboard ───────────────────────────────────────────────────

function updateDashboard(data) {
  const w = data.wheel_status || {};
  setText("w-backend", w.backend || "--");
  setText("w-left", (w.left_rpm || 0).toFixed(0));
  setText("w-right", (w.right_rpm || 0).toFixed(0));
  setBar("bar-left", Math.abs(w.left_rpm || 0), MAX_RPM);
  setBar("bar-right", Math.abs(w.right_rpm || 0), MAX_RPM);

  const estopBanner = document.getElementById("estop-banner");
  if (estopBanner) estopBanner.style.display = w.estopped ? "block" : "none";
  const card = document.getElementById("card-wheels");
  if (card) card.classList.toggle("estopped", !!w.estopped);
  const wErr = document.getElementById("w-error");
  if (wErr) {
    if (w.error) {
      wErr.textContent = w.error;
      wErr.style.color = "var(--err)";
    } else {
      wErr.textContent = "None";
      wErr.style.color = "";
    }
  }

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

  const gp = data.gamepad || {};
  const gpDot = document.getElementById("gp-dot");
  const gpLabel = document.getElementById("gp-label");
  if (gpDot && gpLabel) {
    if (gp.connected) {
      gpDot.classList.add("connected");
      gpLabel.textContent = gp.name || "Gamepad";
      gpLabel.style.color = "";
    } else {
      gpDot.classList.remove("connected");
      gpLabel.textContent = "Disconnected";
      gpLabel.style.color = "#8a8a9a";
    }
  }

  const rec = data.bag_recorder || {};
  const dot = document.getElementById("rec-dot");
  const label = document.getElementById("rec-label");
  if (dot && label) {
    if (rec.recording) {
      dot.classList.add("recording");
      label.textContent = "RECORDING";
      label.style.color = "#e94560";
    } else {
      dot.classList.remove("recording");
      label.textContent = "Idle";
      label.style.color = "";
    }
  }
  if (rec.recording) {
    const dur = rec.duration_sec || 0;
    const m = Math.floor(dur / 60);
    const s = Math.floor(dur % 60);
    setText("rec-duration", `${m}:${s.toString().padStart(2, "0")}`);
  } else {
    setText("rec-duration", "--");
  }
  setText("rec-frames", rec.frame_count != null ? rec.frame_count : "--");
  setText("rec-path", rec.bag_path || "--");
}

// ─── Controller Config ───────────────────────────────────────────

async function loadCtrlConfig() {
  const loading = document.getElementById("ctrl-loading");
  const table = document.getElementById("ctrl-table");
  try {
    const resp = await fetch("/api/controller-config");
    if (!resp.ok) throw new Error("HTTP " + resp.status);
    ctrlConfig = await resp.json();
    renderCtrlTable();
    if (loading) loading.style.display = "none";
    if (table) table.style.display = "";
  } catch (e) {
    console.error("Failed to load controller config:", e);
    if (loading) loading.textContent = "Failed to load config: " + e.message;
  }
}

function renderCtrlTable() {
  const tbody = document.getElementById("ctrl-tbody");
  if (!tbody || !ctrlConfig) return;

  tbody.innerHTML = "";
  const actions = ctrlConfig.available_actions || {};
  const buttons = ctrlConfig.button_names || [];
  const mapping = ctrlConfig.buttons || {};
  const axes = ctrlConfig.axes || {};

  // Axes (read-only rows)
  for (const [axisName, axisDesc] of Object.entries(axes)) {
    const tr = document.createElement("tr");
    tr.id = `ctrl-row-${axisName}`;
    tr.className = "axis-row";

    const tdDot = document.createElement("td");
    tdDot.className = "ctrl-dot-cell";
    const dot = document.createElement("span");
    dot.className = "ctrl-dot";
    dot.id = `ctrl-dot-${axisName}`;
    tdDot.appendChild(dot);
    tr.appendChild(tdDot);

    const tdName = document.createElement("td");
    tdName.className = "ctrl-btn-name axis-name";
    tdName.textContent = axisName;
    tr.appendChild(tdName);

    const tdAction = document.createElement("td");
    tdAction.className = "axis-action";
    tdAction.textContent = axisDesc;
    tr.appendChild(tdAction);

    tbody.appendChild(tr);
  }

  // Separator
  const sepTr = document.createElement("tr");
  sepTr.innerHTML = '<td colspan="3" class="ctrl-sep"></td>';
  tbody.appendChild(sepTr);

  // Configurable buttons
  for (const btn of buttons) {
    const tr = document.createElement("tr");
    tr.id = `ctrl-row-${btn}`;

    const tdDot = document.createElement("td");
    tdDot.className = "ctrl-dot-cell";
    const dot = document.createElement("span");
    dot.className = "ctrl-dot";
    dot.id = `ctrl-dot-${btn}`;
    tdDot.appendChild(dot);
    tr.appendChild(tdDot);

    const tdBtn = document.createElement("td");
    tdBtn.className = "ctrl-btn-name";
    tdBtn.textContent = btn;
    tr.appendChild(tdBtn);

    const tdAction = document.createElement("td");
    const sel = document.createElement("select");
    sel.id = `ctrl-sel-${btn}`;
    sel.dataset.btn = btn;
    for (const [actionId, actionLabel] of Object.entries(actions)) {
      const opt = document.createElement("option");
      opt.value = actionId;
      opt.textContent = actionLabel;
      if (mapping[btn] === actionId) opt.selected = true;
      sel.appendChild(opt);
    }
    tdAction.appendChild(sel);
    tr.appendChild(tdAction);

    tbody.appendChild(tr);
  }
}

function updateCtrlHighlights() {
  if (!ctrlConfig) return;
  const allNames = [
    ...Object.keys(ctrlConfig.axes || {}),
    ...(ctrlConfig.button_names || []),
  ];
  for (const name of allNames) {
    const dot = document.getElementById(`ctrl-dot-${name}`);
    const row = document.getElementById(`ctrl-row-${name}`);
    const isActive = activeInputs.includes(name);
    if (dot) dot.classList.toggle("active", isActive);
    if (row) row.classList.toggle("active", isActive);
  }
}

async function saveCtrlConfig() {
  if (!ctrlConfig) return;
  const buttons = ctrlConfig.button_names || [];
  const mapping = {};
  for (const btn of buttons) {
    const sel = document.getElementById(`ctrl-sel-${btn}`);
    mapping[btn] = sel ? sel.value : "none";
  }

  try {
    const resp = await fetch("/api/controller-config", {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ buttons: mapping }),
    });
    const result = await resp.json();
    if (result.success) {
      ctrlConfig.buttons = result.buttons;
      showCtrlMsg("Saved! Config applied to controller.", "ok");
    } else {
      showCtrlMsg("Save failed.", "err");
    }
  } catch (e) {
    showCtrlMsg("Network error: " + e.message, "err");
  }
}

async function saveCtrlDefaults() {
  if (!ctrlConfig) return;
  const buttons = ctrlConfig.button_names || [];
  const mapping = {};
  for (const btn of buttons) {
    const sel = document.getElementById(`ctrl-sel-${btn}`);
    mapping[btn] = sel ? sel.value : "none";
  }

  try {
    const resp = await fetch("/api/controller-config/defaults", {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ buttons: mapping }),
    });
    const result = await resp.json();
    if (result.success) {
      showCtrlMsg("Saved to repo defaults — commit to keep in git.", "ok");
    } else {
      showCtrlMsg("Failed: " + (result.message || "unknown error"), "err");
    }
  } catch (e) {
    showCtrlMsg("Network error: " + e.message, "err");
  }
}

async function resetCtrlConfig() {
  if (!ctrlConfig) return;
  const defaults = ctrlConfig.defaults || {};
  const buttons = ctrlConfig.button_names || [];
  for (const btn of buttons) {
    const sel = document.getElementById(`ctrl-sel-${btn}`);
    if (sel && defaults[btn]) sel.value = defaults[btn];
  }
  showCtrlMsg("Defaults restored (not saved yet — click Save to apply).", "warn");
}

function showCtrlMsg(text, type) {
  const el = document.getElementById("ctrl-msg");
  if (!el) return;
  el.textContent = text;
  el.className = "ctrl-msg " + (type || "");
  clearTimeout(el._timer);
  el._timer = setTimeout(() => { el.textContent = ""; el.className = "ctrl-msg"; }, 4000);
}

// ─── Tabs ────────────────────────────────────────────────────────

function initTabs() {
  const tabs = document.querySelectorAll(".tabs .tab");
  tabs.forEach(tab => {
    tab.addEventListener("click", () => {
      tabs.forEach(t => t.classList.remove("active"));
      tab.classList.add("active");
      document.querySelectorAll(".tab-content").forEach(c => c.classList.remove("active"));
      const target = document.getElementById("tab-" + tab.dataset.tab);
      if (target) target.classList.add("active");
    });
  });
}

// ─── Helpers ─────────────────────────────────────────────────────

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
  try { await fetch(`/api/command/${node}/${action}`, { method: "POST" }); }
  catch (e) { console.error("Command failed:", e); }
}

async function switchBackend() {
  try { await fetch("/api/command/wheel_driver_node/switch_backend", { method: "POST" }); }
  catch (e) { console.error("Switch failed:", e); }
}

async function startRecording() {
  try { await fetch("/api/recording/start", { method: "POST" }); }
  catch (e) { console.error("Start recording failed:", e); }
}

async function stopRecording() {
  try { await fetch("/api/recording/stop", { method: "POST" }); }
  catch (e) { console.error("Stop recording failed:", e); }
}

// ─── Init ────────────────────────────────────────────────────────

initTabs();
connectWs();
loadCtrlConfig();
