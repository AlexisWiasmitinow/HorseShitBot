const STATE_NAMES = ["IDLE", "REFERENCING", "REFERENCED", "MOVING_OPEN", "MOVING_CLOSE", "ERROR"];
const STATE_CLASSES = ["state-idle", "state-ref", "state-ref", "state-moving", "state-moving", "state-error"];
const MAX_RPM = 500;

let ws = null;
let ctrlConfig = null;   // fetched from server
let activeInputs = [];    // live from WebSocket
let lastGamepad = {};     // last gamepad status from WS
let _cachedBtInfo = { mac: "", battery: null, connected: false };

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
      lastGamepad = data.gamepad || {};
      activeInputs = lastGamepad.active_inputs || [];
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

  // Battery + BT info (from gamepad node WS or cached API data)
  const batt = gp.battery != null ? gp.battery : _cachedBtInfo.battery;
  const mac = gp.bt_mac || _cachedBtInfo.mac;
  const btConnected = gp.connected || _cachedBtInfo.connected;

  const battRow = document.getElementById("gp-battery-row");
  const battText = document.getElementById("gp-battery-text");
  const battIcon = document.getElementById("gp-battery-icon");
  if (battRow && battText) {
    if (mac || btConnected) {
      battRow.style.display = "";
      if (batt != null) {
        battText.textContent = batt + "%";
        const pct = batt;
        const cls = pct <= 15 ? "gp-batt-low" : pct <= 40 ? "gp-batt-med" : "gp-batt-ok";
        battIcon.className = "gp-battery-icon " + cls;
        battIcon.style.setProperty("--batt-pct", pct + "%");
      } else {
        battText.textContent = "N/A";
        battIcon.className = "gp-battery-icon";
        battIcon.style.setProperty("--batt-pct", "0%");
      }
    } else {
      battRow.style.display = "none";
    }
  }

  const btRow = document.getElementById("gp-bt-row");
  const btMac = document.getElementById("gp-bt-mac");
  const reconRow = document.getElementById("gp-reconnect-row");
  if (btRow && btMac) {
    if (mac) {
      btRow.style.display = "";
      btMac.textContent = mac;
    } else {
      btRow.style.display = "none";
    }
  }
  if (reconRow) {
    reconRow.style.display = (!gp.connected && mac) ? "" : "none";
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

  // Recordings tab controls
  const dotLg = document.getElementById("rec-dot-lg");
  const barLabel = document.getElementById("rec-bar-label");
  const barStart = document.getElementById("rec-bar-start");
  const barStop = document.getElementById("rec-bar-stop");
  if (dotLg && barLabel) {
    if (rec.recording) {
      dotLg.classList.add("recording");
      const dur = rec.duration_sec || 0;
      const m = Math.floor(dur / 60);
      const s = Math.floor(dur % 60);
      barLabel.textContent = "RECORDING";
      barLabel.style.color = "#e94560";
      setText("rec-bar-time", `${m}:${s.toString().padStart(2, "0")}`);
      setText("rec-bar-frames", `${rec.frame_count || 0} frames`);
      if (barStart) barStart.style.display = "none";
      if (barStop) barStop.style.display = "";
    } else {
      dotLg.classList.remove("recording");
      barLabel.textContent = "Idle";
      barLabel.style.color = "";
      setText("rec-bar-time", "--");
      setText("rec-bar-frames", "-- frames");
      if (barStart) barStart.style.display = "";
      if (barStop) barStop.style.display = "none";
    }
  }

  updateThermals(data.thermals || []);
}

// ─── Thermals ─────────────────────────────────────────────────────

const THERMAL_WARN_C = 65;
const THERMAL_CRIT_C = 85;
const THERMAL_MAX_C = 105;

function thermalClass(temp) {
  if (temp >= THERMAL_CRIT_C) return "thermal-crit";
  if (temp >= THERMAL_WARN_C) return "thermal-warn";
  return "thermal-ok";
}

function updateThermals(zones) {
  const container = document.getElementById("thermal-zones");
  if (!container) return;

  if (!zones || zones.length === 0) {
    container.innerHTML = '<span class="thermal-placeholder">No thermal zones detected</span>';
    return;
  }

  container.innerHTML = "";
  for (const z of zones) {
    const row = document.createElement("div");
    row.className = "thermal-row";

    const label = document.createElement("span");
    label.className = "thermal-label";
    label.textContent = z.type;
    label.title = z.zone;

    const barWrap = document.createElement("div");
    barWrap.className = "thermal-bar-wrap";
    const bar = document.createElement("div");
    bar.className = "thermal-bar " + thermalClass(z.temp_c);
    bar.style.width = Math.min(100, (z.temp_c / THERMAL_MAX_C) * 100) + "%";
    barWrap.appendChild(bar);

    const val = document.createElement("span");
    val.className = "thermal-value " + thermalClass(z.temp_c);
    val.textContent = z.temp_c.toFixed(1) + "°C";

    row.appendChild(label);
    row.appendChild(barWrap);
    row.appendChild(val);
    container.appendChild(row);
  }
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

// ─── Camera (MJPEG) ──────────────────────────────────────────────

let camStreaming = false;

function startCamera() {
  const fps = document.getElementById("cam-fps").value;
  const quality = document.getElementById("cam-quality").value;

  const colorImg = document.getElementById("cam-color");
  const depthImg = document.getElementById("cam-depth");
  const stateEl = document.getElementById("cam-state");

  const qs = `fps=${fps}&quality=${quality}`;
  colorImg.src = `/api/stream/color?${qs}`;
  depthImg.src = `/api/stream/depth?${qs}`;

  camStreaming = true;
  document.getElementById("cam-toggle").textContent = "Stop Stream";
  document.getElementById("cam-toggle").className = "danger";
  if (stateEl) { stateEl.textContent = "Streaming"; stateEl.className = "cam-state ok"; }

  colorImg.onerror = () => {
    if (stateEl) { stateEl.textContent = "Stream error"; stateEl.className = "cam-state err"; }
  };
}

function stopCamera() {
  const colorImg = document.getElementById("cam-color");
  const depthImg = document.getElementById("cam-depth");
  const stateEl = document.getElementById("cam-state");

  colorImg.src = "";
  depthImg.src = "";
  camStreaming = false;
  document.getElementById("cam-toggle").textContent = "Start Stream";
  document.getElementById("cam-toggle").className = "primary";
  if (stateEl) { stateEl.textContent = "Stopped"; stateEl.className = "cam-state"; }
}

function toggleCamera() {
  if (camStreaming) stopCamera();
  else startCamera();
}

// ─── Bag Topic Picker ────────────────────────────────────────────

let bagTopicData = null;

async function loadBagTopics() {
  try {
    const resp = await fetch("/api/bag-topics");
    if (!resp.ok) throw new Error("HTTP " + resp.status);
    bagTopicData = await resp.json();
    renderTopicPicker();
  } catch (e) {
    console.error("Failed to load bag topics:", e);
  }
}

function renderTopicPicker() {
  const container = document.getElementById("topic-groups");
  const hint = document.getElementById("topic-picker-hint");
  if (!container || !bagTopicData) return;

  const groups = bagTopicData.topic_groups || {};
  const selected = bagTopicData.selected_topics || [];
  const recording = bagTopicData.recording;

  if (hint) {
    hint.textContent = recording ? "(locked while recording)" : "";
  }

  container.innerHTML = "";
  for (const [groupName, topics] of Object.entries(groups)) {
    const group = document.createElement("div");
    group.className = "topic-group";

    const label = document.createElement("span");
    label.className = "topic-group-label";
    label.textContent = groupName;
    group.appendChild(label);

    for (const topic of topics) {
      const lbl = document.createElement("label");
      lbl.className = "topic-cb-label";
      const cb = document.createElement("input");
      cb.type = "checkbox";
      cb.value = topic;
      cb.checked = selected.includes(topic);
      cb.disabled = recording;
      cb.className = "topic-cb";
      lbl.appendChild(cb);
      const span = document.createElement("span");
      span.textContent = topic.split("/").pop();
      span.title = topic;
      lbl.appendChild(span);
      group.appendChild(lbl);
    }
    container.appendChild(group);
  }
}

async function saveBagTopics() {
  const checkboxes = document.querySelectorAll("#topic-groups .topic-cb:checked");
  const topics = Array.from(checkboxes).map(cb => cb.value);
  if (topics.length === 0) {
    showTopicMsg("Select at least one topic.", "err");
    return;
  }
  try {
    const resp = await fetch("/api/bag-topics", {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ topics }),
    });
    const result = await resp.json();
    if (result.success) {
      showTopicMsg(`Applied ${topics.length} topic(s). Takes effect on next recording.`, "ok");
    } else {
      showTopicMsg(result.error || "Failed", "err");
    }
  } catch (e) {
    showTopicMsg("Network error: " + e.message, "err");
  }
}

function showTopicMsg(text, type) {
  const el = document.getElementById("topic-msg");
  if (!el) return;
  el.textContent = text;
  el.className = "ctrl-msg " + (type || "");
  clearTimeout(el._timer);
  el._timer = setTimeout(() => { el.textContent = ""; el.className = "ctrl-msg"; }, 4000);
}

// ─── Recordings ──────────────────────────────────────────────────

let bagsData = null;
let bagsAutoRefresh = null;

async function loadBags() {
  const loading = document.getElementById("bags-loading");
  const table = document.getElementById("bags-table");
  const empty = document.getElementById("bags-empty");
  try {
    const resp = await fetch("/api/bags");
    if (!resp.ok) throw new Error("HTTP " + resp.status);
    bagsData = await resp.json();
    renderBags();
    if (loading) loading.style.display = "none";
    if (bagsData.bags.length === 0) {
      if (table) table.style.display = "none";
      if (empty) empty.style.display = "";
    } else {
      if (table) table.style.display = "";
      if (empty) empty.style.display = "none";
    }
  } catch (e) {
    console.error("Failed to load bags:", e);
    if (loading) loading.textContent = "Failed to load: " + e.message;
  }
}

function formatBytes(bytes) {
  if (bytes === 0) return "0 B";
  const units = ["B", "KB", "MB", "GB"];
  const i = Math.min(Math.floor(Math.log(bytes) / Math.log(1024)), units.length - 1);
  const val = bytes / Math.pow(1024, i);
  return val.toFixed(i === 0 ? 0 : 1) + " " + units[i];
}

function formatDate(isoStr) {
  try {
    const d = new Date(isoStr);
    return d.toLocaleDateString(undefined, { month: "short", day: "numeric", hour: "2-digit", minute: "2-digit" });
  } catch {
    return isoStr;
  }
}

function renderBags() {
  const tbody = document.getElementById("bags-tbody");
  const summary = document.getElementById("bags-summary");
  if (!tbody || !bagsData) return;

  tbody.innerHTML = "";
  const bags = bagsData.bags || [];

  if (summary) {
    summary.textContent = `${bags.length} recording${bags.length !== 1 ? "s" : ""} · ${formatBytes(bagsData.total_bytes)}`;
  }

  // Disk usage bar
  const diskWrap = document.getElementById("disk-bar-wrap");
  if (diskWrap && bagsData.disk_total) {
    diskWrap.style.display = "";
    const total = bagsData.disk_total;
    const free = bagsData.disk_free;
    const used = bagsData.disk_used;
    const bagBytes = bagsData.total_bytes;
    const otherUsed = used - bagBytes;

    const pctBags = Math.max(0.5, (bagBytes / total) * 100);
    const pctOther = Math.max(0.5, (otherUsed / total) * 100);

    document.getElementById("disk-bar-bags").style.width = pctBags + "%";
    document.getElementById("disk-bar-other").style.width = pctOther + "%";
    document.getElementById("disk-label-used").textContent =
      `Bags: ${formatBytes(bagBytes)} · Other: ${formatBytes(otherUsed)} · Total: ${formatBytes(total)}`;
    document.getElementById("disk-label-free").textContent =
      `${formatBytes(free)} free`;
  }

  for (const bag of bags) {
    const tr = document.createElement("tr");

    const tdName = document.createElement("td");
    tdName.className = "bag-name";
    tdName.textContent = bag.name;
    tr.appendChild(tdName);

    const tdDate = document.createElement("td");
    tdDate.textContent = formatDate(bag.modified);
    tr.appendChild(tdDate);

    const tdSize = document.createElement("td");
    tdSize.textContent = formatBytes(bag.size_bytes);
    tr.appendChild(tdSize);

    const tdFiles = document.createElement("td");
    tdFiles.className = "bag-files";
    const exts = [...new Set((bag.files || []).map(f => f.includes(".") ? "." + f.split(".").pop() : f))];
    tdFiles.textContent = `${(bag.files || []).length} files (${exts.join(", ")})`;
    tr.appendChild(tdFiles);

    const tdActions = document.createElement("td");
    tdActions.className = "bag-actions";

    const dlBtn = document.createElement("a");
    dlBtn.href = `/api/bags/${encodeURIComponent(bag.name)}/download`;
    dlBtn.className = "bag-dl";
    dlBtn.textContent = "Download";
    dlBtn.setAttribute("download", "");
    tdActions.appendChild(dlBtn);

    const delBtn = document.createElement("button");
    delBtn.className = "danger bag-del";
    delBtn.textContent = "Delete";
    delBtn.onclick = () => deleteBag(bag.name);
    tdActions.appendChild(delBtn);

    tr.appendChild(tdActions);
    tbody.appendChild(tr);
  }
}

async function deleteBag(name) {
  if (!confirm(`Delete recording "${name}"?\nThis cannot be undone.`)) return;
  try {
    const resp = await fetch(`/api/bags/${encodeURIComponent(name)}`, { method: "DELETE" });
    const result = await resp.json();
    if (result.success || resp.ok) {
      loadBags();
    } else {
      alert("Delete failed: " + (result.error || "unknown error"));
    }
  } catch (e) {
    alert("Network error: " + e.message);
  }
}

function startBagsAutoRefresh() {
  if (bagsAutoRefresh) return;
  bagsAutoRefresh = setInterval(loadBags, 3000);
}

function stopBagsAutoRefresh() {
  if (bagsAutoRefresh) {
    clearInterval(bagsAutoRefresh);
    bagsAutoRefresh = null;
  }
}

// ─── Network ─────────────────────────────────────────────────────

let netData = null;
let netConfigIface = "";

async function loadNetwork() {
  const loading = document.getElementById("net-loading");
  const container = document.getElementById("net-interfaces");
  try {
    const resp = await fetch("/api/network");
    if (!resp.ok) throw new Error("HTTP " + resp.status);
    netData = await resp.json();
    renderNetInterfaces();
    if (loading) loading.style.display = "none";
  } catch (e) {
    console.error("Failed to load network:", e);
    if (loading) loading.textContent = "Failed to load: " + e.message;
  }
}

function renderNetInterfaces() {
  const container = document.getElementById("net-interfaces");
  const wifiSection = document.getElementById("net-wifi-section");
  if (!container || !netData) return;
  container.innerHTML = "";

  const ifaces = netData.interfaces || [];
  let hasWifi = false;

  let wifiIface = null;

  for (const iface of ifaces) {
    if (iface.type === "wifi") { hasWifi = true; wifiIface = iface; }

    const card = document.createElement("div");
    card.className = "card net-iface-card" + (iface.connected ? " net-connected" : "");

    const icon = iface.type === "wifi" ? "&#x1F4F6;" : "&#x1F50C;";
    const statusDot = iface.connected ? '<span class="net-dot on"></span>' : '<span class="net-dot off"></span>';

    let statusText = iface.connected ? "Connected" : "Disconnected";
    if (iface.ap_mode) statusText = "Access Point";

    let info = `
      <div class="net-iface-header">
        <span class="net-iface-icon">${icon}</span>
        <span class="net-iface-name">${iface.name}</span>
        ${statusDot}
        <span class="net-iface-status">${statusText}</span>
        <button class="net-cfg-btn" onclick="openNetConfig('${iface.name}')">Configure</button>
      </div>
      <div class="net-iface-details">`;

    if (iface.ap_mode) {
      info += `<div class="kv"><span class="label">Mode</span><span class="value net-ap-badge">Access Point</span></div>`;
      info += `<div class="kv"><span class="label">AP SSID</span><span class="value">${iface.ap_ssid || iface.wifi_ssid || "--"}</span></div>`;
      if (iface.ip4) {
        info += `<div class="kv"><span class="label">IP Address</span><span class="value">${iface.ip4}</span></div>`;
      }
    } else {
      if (iface.connected && iface.ip4) {
        info += `<div class="kv"><span class="label">IP Address</span><span class="value">${iface.ip4}</span></div>`;
      }
      if (iface.method) {
        const mLabel = iface.method === "auto" ? "DHCP" : (iface.method === "shared" ? "Shared" : "Static");
        info += `<div class="kv"><span class="label">Mode</span><span class="value">${mLabel}</span></div>`;
      }
      if (iface.gateway) {
        info += `<div class="kv"><span class="label">Gateway</span><span class="value">${iface.gateway}</span></div>`;
      }
      if (iface.dns) {
        info += `<div class="kv"><span class="label">DNS</span><span class="value">${iface.dns}</span></div>`;
      }
      if (iface.type === "wifi" && iface.wifi_ssid) {
        info += `<div class="kv"><span class="label">WiFi SSID</span><span class="value">${iface.wifi_ssid}</span></div>`;
        info += `<div class="kv"><span class="label">Signal</span><span class="value">${iface.wifi_signal}%</span></div>`;
      }
    }
    if (iface.mac) {
      info += `<div class="kv"><span class="label">MAC</span><span class="value" style="font-size:0.75rem">${iface.mac}</span></div>`;
    }
    if (iface.dhcp_server_active) {
      info += `<div class="kv"><span class="label">DHCP Server</span><span class="value net-dhcpd-on">Active (${iface.dhcp_range_start} - ${iface.dhcp_range_end})</span></div>`;
    }

    info += `</div>`;
    card.innerHTML = info;
    container.appendChild(card);
  }

  if (wifiSection) {
    wifiSection.style.display = hasWifi ? "" : "none";
  }

  // Show/hide wifi client vs AP sections
  const apSection = document.getElementById("net-ap-section");
  const clientSection = document.getElementById("net-wifi-client");
  if (wifiIface && wifiIface.ap_mode) {
    if (clientSection) clientSection.style.display = "none";
    if (apSection) {
      apSection.style.display = "";
      document.getElementById("ap-current-ssid").textContent = wifiIface.ap_ssid || wifiIface.wifi_ssid || "--";
      document.getElementById("ap-current-ip").textContent = wifiIface.ip4 || "10.0.0.1";
    }
  } else {
    if (clientSection) clientSection.style.display = "";
    if (apSection) apSection.style.display = "none";
    if (hasWifi) scanWifi();
  }
}

function openNetConfig(ifaceName) {
  netConfigIface = ifaceName;
  const modal = document.getElementById("net-config-modal");
  const title = document.getElementById("net-cfg-title");
  if (title) title.textContent = `Configure ${ifaceName}`;

  const iface = (netData?.interfaces || []).find(i => i.name === ifaceName);
  const modeSelect = document.getElementById("net-cfg-mode");
  if (modeSelect && iface) {
    modeSelect.value = iface.method === "manual" ? "manual" : "auto";
  }
  if (iface && iface.method === "manual" && iface.ip4) {
    document.getElementById("net-cfg-ip").value = iface.ip4;
    document.getElementById("net-cfg-gateway").value = iface.gateway || "";
    document.getElementById("net-cfg-dns").value = iface.dns || "";
  } else {
    document.getElementById("net-cfg-ip").value = "";
    document.getElementById("net-cfg-gateway").value = "";
    document.getElementById("net-cfg-dns").value = "";
  }

  const dhcpdCb = document.getElementById("net-cfg-dhcpd");
  if (dhcpdCb && iface) {
    dhcpdCb.checked = iface.dhcp_server_active;
    document.getElementById("net-cfg-dhcpd-start").value = iface.dhcp_range_start || "";
    document.getElementById("net-cfg-dhcpd-end").value = iface.dhcp_range_end || "";
  }

  onNetModeChange();
  onDhcpdToggle();
  showNetMsg("");
  if (modal) modal.style.display = "flex";
}

function closeNetConfig() {
  const modal = document.getElementById("net-config-modal");
  if (modal) modal.style.display = "none";
}

function onNetModeChange() {
  const mode = document.getElementById("net-cfg-mode").value;
  const fields = document.getElementById("net-cfg-static-fields");
  if (fields) fields.style.display = mode === "manual" ? "" : "none";
}

function onDhcpdToggle() {
  const checked = document.getElementById("net-cfg-dhcpd").checked;
  const fields = document.getElementById("net-cfg-dhcpd-fields");
  if (fields) fields.style.display = checked ? "" : "none";
}

async function applyNetConfig() {
  const mode = document.getElementById("net-cfg-mode").value;
  const iface = netConfigIface;
  if (!iface) return;

  showNetMsg("Applying...", "warn");

  try {
    let result;
    if (mode === "auto") {
      const resp = await fetch(`/api/network/interface/${encodeURIComponent(iface)}/dhcp`, {
        method: "PUT",
      });
      result = await resp.json();
    } else {
      const ip = document.getElementById("net-cfg-ip").value.trim();
      if (!ip) { showNetMsg("IP address is required.", "err"); return; }
      const resp = await fetch(`/api/network/interface/${encodeURIComponent(iface)}/static`, {
        method: "PUT",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          ip: ip,
          prefix: parseInt(document.getElementById("net-cfg-prefix").value) || 24,
          gateway: document.getElementById("net-cfg-gateway").value.trim(),
          dns: document.getElementById("net-cfg-dns").value.trim(),
        }),
      });
      result = await resp.json();
    }

    if (!result.success) {
      showNetMsg(result.message || "Failed", "err");
      return;
    }
    showNetMsg(result.message || "Applied!", "ok");

    // DHCP server config
    if (mode === "manual") {
      const dhcpdEnabled = document.getElementById("net-cfg-dhcpd").checked;
      const dhcpResp = await fetch(`/api/network/interface/${encodeURIComponent(iface)}/dhcp-server`, {
        method: "PUT",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          enabled: dhcpdEnabled,
          range_start: document.getElementById("net-cfg-dhcpd-start").value.trim(),
          range_end: document.getElementById("net-cfg-dhcpd-end").value.trim(),
        }),
      });
      const dhcpResult = await dhcpResp.json();
      if (!dhcpResult.success) {
        showNetMsg(result.message + " | DHCP server: " + dhcpResult.message, "warn");
      } else if (dhcpdEnabled) {
        showNetMsg(result.message + " | " + dhcpResult.message, "ok");
      }
    }

    setTimeout(loadNetwork, 2000);
  } catch (e) {
    showNetMsg("Network error: " + e.message, "err");
  }
}

function showNetMsg(text, type) {
  const el = document.getElementById("net-cfg-msg");
  if (!el) return;
  el.textContent = text;
  el.className = "ctrl-msg " + (type || "");
}

// ── WiFi ──

async function scanWifi() {
  const btn = document.getElementById("wifi-scan-btn");
  const status = document.getElementById("wifi-scan-status");
  if (btn) btn.disabled = true;
  if (status) { status.textContent = "Scanning..."; status.className = "ctrl-hint"; }

  try {
    const resp = await fetch("/api/network/wifi/scan");
    if (!resp.ok) throw new Error("HTTP " + resp.status);
    const data = await resp.json();
    renderWifiList(data.networks || []);
    if (status) { status.textContent = `${(data.networks || []).length} networks found`; }
  } catch (e) {
    console.error("WiFi scan failed:", e);
    if (status) { status.textContent = "Scan failed: " + e.message; status.className = "ctrl-hint err"; }
  } finally {
    if (btn) btn.disabled = false;
  }
}

function renderWifiList(networks) {
  const container = document.getElementById("wifi-list");
  if (!container) return;
  container.innerHTML = "";

  if (networks.length === 0) {
    container.innerHTML = '<div class="ctrl-hint">No WiFi networks found.</div>';
    return;
  }

  for (const net of networks) {
    const row = document.createElement("div");
    row.className = "wifi-row" + (net.in_use ? " wifi-active" : "");

    const bars = Math.min(4, Math.max(1, Math.ceil(net.signal / 25)));
    let signalBars = "";
    for (let i = 1; i <= 4; i++) {
      signalBars += `<span class="wifi-bar ${i <= bars ? "on" : ""}"></span>`;
    }

    row.innerHTML = `
      <div class="wifi-signal">${signalBars}</div>
      <span class="wifi-ssid">${net.ssid}</span>
      <span class="wifi-security">${net.security || "Open"}</span>
      ${net.in_use
        ? '<span class="wifi-connected-badge">Connected</span>'
        : `<button class="wifi-connect-btn" onclick="openWifiConnect('${net.ssid.replace(/'/g, "\\'")}', '${net.security || ""}')">Connect</button>`
      }
    `;
    container.appendChild(row);
  }
}

function openWifiConnect(ssid, security) {
  const panel = document.getElementById("wifi-connect-panel");
  document.getElementById("wifi-ssid").value = ssid;
  const pwInput = document.getElementById("wifi-password");
  pwInput.value = "";
  pwInput.type = "password";
  const pwField = document.getElementById("wifi-password-field");
  const needsPw = security && security !== "--" && security !== "" &&
                  !security.toLowerCase().includes("open");
  if (pwField) pwField.style.display = needsPw ? "" : "none";
  showWifiMsg("");
  if (panel) {
    panel.style.display = "";
    if (needsPw) pwInput.focus();
  }
}

function closeWifiPanel() {
  const panel = document.getElementById("wifi-connect-panel");
  if (panel) panel.style.display = "none";
}

function toggleWifiPw() {
  const pw = document.getElementById("wifi-password");
  if (!pw) return;
  const show = pw.type === "password";
  pw.type = show ? "text" : "password";
  const btn = pw.parentElement.querySelector(".net-pw-toggle");
  if (btn) btn.textContent = show ? "Hide" : "Show";
}

async function connectWifi() {
  const ssid = document.getElementById("wifi-ssid").value;
  const password = document.getElementById("wifi-password").value;
  if (!ssid) return;

  const btn = document.getElementById("wifi-connect-btn");
  if (btn) btn.disabled = true;
  showWifiMsg("Connecting...", "warn");
  try {
    const resp = await fetch("/api/network/wifi/connect", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ ssid, password }),
    });
    const result = await resp.json();
    if (result.success) {
      showWifiMsg(result.message || "Connected!", "ok");
      closeWifiPanel();
      setTimeout(() => { loadNetwork(); }, 3000);
    } else {
      showWifiMsg(result.message || "Failed to connect", "err");
    }
  } catch (e) {
    showWifiMsg("Network error: " + e.message, "err");
  } finally {
    if (btn) btn.disabled = false;
  }
}

function showWifiMsg(text, type) {
  const el = document.getElementById("wifi-msg");
  if (!el) return;
  el.textContent = text;
  el.className = "ctrl-msg " + (type || "");
}

// ── Access Point ──

function openApSetup() {
  const panel = document.getElementById("ap-setup-panel");
  if (panel) panel.style.display = "";
  showApMsg("");
}

function closeApSetup() {
  const panel = document.getElementById("ap-setup-panel");
  if (panel) panel.style.display = "none";
}

function getWifiIfaceName() {
  if (!netData) return "";
  const wifi = (netData.interfaces || []).find(i => i.type === "wifi");
  return wifi ? wifi.name : "";
}

async function startAp() {
  const iface = getWifiIfaceName();
  if (!iface) { showApMsg("No WiFi interface found", "err"); return; }

  const ssid = document.getElementById("ap-ssid").value.trim();
  const password = document.getElementById("ap-password").value;
  const band = document.getElementById("ap-band").value;
  if (!ssid) { showApMsg("SSID is required", "err"); return; }
  if (password && password.length < 8) { showApMsg("Password must be at least 8 characters (or leave empty for open)", "err"); return; }

  const btn = document.getElementById("ap-start-btn");
  if (btn) btn.disabled = true;
  showApMsg("Starting access point...", "warn");

  try {
    const resp = await fetch(`/api/network/interface/${encodeURIComponent(iface)}/ap`, {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ enabled: true, ssid, password, band }),
    });
    const result = await resp.json();
    if (result.success) {
      showApMsg(result.message || "AP started!", "ok");
      closeApSetup();
      setTimeout(loadNetwork, 3000);
    } else {
      showApMsg(result.message || "Failed to start AP", "err");
    }
  } catch (e) {
    showApMsg("Network error: " + e.message, "err");
  } finally {
    if (btn) btn.disabled = false;
  }
}

async function stopAp() {
  const iface = getWifiIfaceName();
  if (!iface) return;

  showApMsg("Stopping access point...", "warn");
  try {
    const resp = await fetch(`/api/network/interface/${encodeURIComponent(iface)}/ap`, {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ enabled: false }),
    });
    const result = await resp.json();
    if (result.success) {
      showApMsg(result.message || "AP stopped", "ok");
      setTimeout(loadNetwork, 3000);
    } else {
      showApMsg(result.message || "Failed", "err");
    }
  } catch (e) {
    showApMsg("Network error: " + e.message, "err");
  }
}

function showApMsg(text, type) {
  const el = document.getElementById("ap-msg");
  if (!el) return;
  el.textContent = text;
  el.className = "ctrl-msg " + (type || "");
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

      if (tab.dataset.tab === "recordings") {
        loadBagTopics();
        loadBags();
        startBagsAutoRefresh();
      } else {
        stopBagsAutoRefresh();
      }
      if (tab.dataset.tab === "network") {
        loadNetwork();
      }
      if (tab.dataset.tab !== "camera" && camStreaming) {
        stopCamera();
      }
    });
  });
}

// ─── Gamepad Bluetooth Reconnect ─────────────────────────────────

async function reconnectGamepad() {
  const btn = document.getElementById("gp-reconnect-btn");
  const msg = document.getElementById("gp-reconnect-msg");
  if (btn) btn.disabled = true;
  if (msg) { msg.textContent = "Connecting..."; msg.className = "ctrl-msg warn"; }

  try {
    const mac = lastGamepad.bt_mac || _cachedBtInfo.mac || "";
    const resp = await fetch("/api/bluetooth/reconnect", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ mac }),
    });
    const result = await resp.json();
    if (result.success) {
      if (msg) { msg.textContent = "Connected!"; msg.className = "ctrl-msg ok"; }
    } else {
      if (msg) { msg.textContent = result.message || "Failed"; msg.className = "ctrl-msg err"; }
    }
  } catch (e) {
    if (msg) { msg.textContent = "Error: " + e.message; msg.className = "ctrl-msg err"; }
  } finally {
    if (btn) btn.disabled = false;
    clearTimeout(msg?._timer);
    if (msg) msg._timer = setTimeout(() => { msg.textContent = ""; }, 5000);
  }
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

async function pollBtGamepad() {
  try {
    const resp = await fetch("/api/bluetooth/gamepad");
    if (resp.ok) {
      const data = await resp.json();
      if (data.found) {
        _cachedBtInfo = {
          mac: data.mac || "",
          battery: data.battery,
          connected: data.connected || false,
        };
      }
    }
  } catch (e) { /* ignore */ }
}

initTabs();
connectWs();
loadCtrlConfig();
pollBtGamepad();
setInterval(pollBtGamepad, 15000);

// Camera slider readouts
const _fpsSlider = document.getElementById("cam-fps");
const _fpsVal = document.getElementById("cam-fps-val");
if (_fpsSlider && _fpsVal) {
  _fpsSlider.addEventListener("input", () => { _fpsVal.textContent = _fpsSlider.value; });
}
const _qualSlider = document.getElementById("cam-quality");
const _qualVal = document.getElementById("cam-quality-val");
if (_qualSlider && _qualVal) {
  _qualSlider.addEventListener("input", () => { _qualVal.textContent = _qualSlider.value; });
}
