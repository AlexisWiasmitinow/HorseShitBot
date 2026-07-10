const STATE_NAMES = ["IDLE", "REFERENCING", "REFERENCED", "MOVING_OPEN", "MOVING_CLOSE", "ERROR"];
const STATE_CLASSES = ["state-idle", "state-ref", "state-ref", "state-moving", "state-moving", "state-error"];
const METERS_PER_TURN = 0.093333;
const MAX_SPEED_MS = 3.0;
function rpmToMs(rpm) { return rpm * METERS_PER_TURN / 60.0; }

function updateCurrent(side, amps, limit) {
  const el = document.getElementById("w-current-" + side);
  const bar = document.getElementById("bar-current-" + side);
  if (!el) return;
  if (amps == null) {
    el.textContent = "--";
    el.style.color = "";
    if (bar) { bar.style.width = "0%"; bar.className = "bar-fill bar-current"; }
    return;
  }
  const abs = Math.abs(amps);
  el.textContent = abs.toFixed(2);
  const pct = Math.min(100, (abs / limit) * 100);
  const high = abs > limit * 0.8;
  const crit = abs > limit * 0.95;
  el.style.color = crit ? "var(--err)" : high ? "var(--warn, #f0c929)" : "";
  if (bar) {
    bar.style.width = pct.toFixed(1) + "%";
    bar.className = "bar-fill bar-current" + (crit ? " danger" : high ? " warn" : "");
  }
}

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
  const badgeText = badge?.querySelector(".connection-text");

  ws.onopen = () => {
    if (badgeText) badgeText.textContent = "Connected";
    else if (badge) badge.textContent = "Connected";
    badge?.classList.add("connected");
  };

  ws.onclose = () => {
    if (badgeText) badgeText.textContent = "Disconnected";
    else if (badge) badge.textContent = "Disconnected";
    badge?.classList.remove("connected");
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
  const leftMs = rpmToMs(w.left_rpm || 0);
  const rightMs = rpmToMs(w.right_rpm || 0);
  setText("w-left", leftMs.toFixed(2));
  setText("w-right", rightMs.toFixed(2));
  setBar("bar-left", Math.abs(leftMs), MAX_SPEED_MS);
  setBar("bar-right", Math.abs(rightMs), MAX_SPEED_MS);

  // Motor currents
  const currentLim = (w.diag && w.diag.current_lim) ? w.diag.current_lim : 20;
  updateCurrent("left", w.current_left, currentLim);
  updateCurrent("right", w.current_right, currentLim);

  const estopBanner = document.getElementById("estop-banner");
  if (estopBanner) estopBanner.style.display = w.estopped ? "flex" : "none";
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

  // Hardware status
  const hw = data.mks_bus || {};
  const wBackend = (w.backend || "").toLowerCase();
  const odriveEl = document.getElementById("hw-odrive");
  if (odriveEl) {
    if (wBackend === "odrive") {
      const hasErr = !!w.error;
      const diag = w.diag || {};
      const vbus = diag.vbus || 0;
      const noPower = vbus > 0 && vbus < 10;
      let label = noPower ? "No motor power" : hasErr ? "Error" : "Connected";
      const parts = [];
      if (vbus) parts.push(vbus.toFixed(1) + "V");
      if (diag.fet_temp_left != null || diag.fet_temp_right != null) {
        const tl = diag.fet_temp_left, tr = diag.fet_temp_right;
        if (tl != null && tr != null) parts.push(`FET ${tl}/${tr}°C`);
        else if (tl != null) parts.push(`FET L ${tl}°C`);
        else parts.push(`FET R ${tr}°C`);
      }
      if (parts.length) label += " · " + parts.join(" · ");
      odriveEl.textContent = label;
      odriveEl.style.color = noPower ? "var(--warn, #f0c929)" : hasErr ? "var(--err)" : "var(--ok, #4ecca3)";
    } else if (wBackend === "none" && w.error) {
      odriveEl.textContent = "Disconnected";
      odriveEl.style.color = "var(--err)";
    } else {
      odriveEl.textContent = wBackend === "mks" ? "Not active" : "--";
      odriveEl.style.color = "";
    }
  }
  const mksBusEl = document.getElementById("hw-mks-bus");
  if (mksBusEl) {
    if (hw.bus_connected) {
      mksBusEl.textContent = "Connected";
      mksBusEl.style.color = "var(--ok, #4ecca3)";
    } else if (hw.bus_connected === false) {
      mksBusEl.textContent = "Disconnected";
      mksBusEl.style.color = "var(--err)";
    } else {
      mksBusEl.textContent = "--";
      mksBusEl.style.color = "";
    }
  }
  const motorsEl = document.getElementById("hw-motors");
  if (motorsEl && hw.motors) {
    const motorNames = {
      "1": "Left Track",
      "2": "Right Track",
      "3": "Lift A",
      "4": "Brush",
      "5": "Lift B",
      "6": "Door",
    };
    const info = hw.motor_info || {};
    for (const [id, online] of Object.entries(hw.motors)) {
      const mi = info[id] || {};
      let block = document.getElementById("hw-motor-" + id);
      if (!block) {
        block = document.createElement("div");
        block.id = "hw-motor-" + id;
        block.className = "motor-block";
        block.innerHTML =
          `<div class="kv"><span class="label">Motor ${id} (${motorNames[id] || "?"})</span><span class="value" id="hw-m${id}-status">--</span></div>` +
          `<div class="motor-current-row">` +
            `<label>Run<input type="number" id="hw-m${id}-run" class="motor-cur-input" min="10" max="5200" step="100" oninput="_motorDirty[${id}]=true"></label>` +
            `<span class="jog-unit">mA</span>` +
            `<label>Hold<input type="number" id="hw-m${id}-hold" class="motor-cur-input" min="10" max="100" step="10" oninput="_motorDirty[${id}]=true"></label>` +
            `<span class="jog-unit">%</span>` +
            `<button class="primary" onclick="setMotorCurrent(${id})">Set</button>` +
          `</div>`;
        motorsEl.appendChild(block);
      }
      const statusEl = document.getElementById("hw-m" + id + "-status");
      if (statusEl) {
        statusEl.textContent = online ? "Online" : "Offline";
        statusEl.style.color = online ? "var(--ok, #4ecca3)" : "var(--err)";
      }
      const runInput = document.getElementById("hw-m" + id + "-run");
      const holdInput = document.getElementById("hw-m" + id + "-hold");
      if (!_motorDirty[id]) {
        if (runInput && mi.run_current_ma != null)
          runInput.value = mi.run_current_ma;
        if (holdInput && mi.hold_current_pct != null)
          holdInput.value = mi.hold_current_pct;
      }
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

  for (const profile of ["perception", "mapping"]) {
    const rec = data[profile + "_recorder"] || {};
    const dot = document.getElementById(`rec-dot-${profile}`);
    const label = document.getElementById(`rec-label-${profile}`);
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
      setText(`rec-duration-${profile}`, `${m}:${s.toString().padStart(2, "0")}`);
    } else {
      setText(`rec-duration-${profile}`, "--");
    }
    setText(`rec-frames-${profile}`, rec.frame_count != null ? rec.frame_count : "--");

    // Recordings tab controls
    const dotLg = document.getElementById(`rec-dot-lg-${profile}`);
    const barLabel = document.getElementById(`rec-bar-label-${profile}`);
    const barStart = document.getElementById(`rec-bar-start-${profile}`);
    const barStop = document.getElementById(`rec-bar-stop-${profile}`);
    if (dotLg && barLabel) {
      if (rec.recording) {
        dotLg.classList.add("recording");
        const dur = rec.duration_sec || 0;
        const m = Math.floor(dur / 60);
        const s = Math.floor(dur % 60);
        barLabel.textContent = "RECORDING";
        barLabel.style.color = "var(--red)";
        setText(`rec-bar-time-${profile}`, `${m}:${s.toString().padStart(2, "0")}`);
        setText(`rec-bar-frames-${profile}`, `${rec.frame_count || 0} frames`);
        if (barStart) barStart.style.display = "none";
        if (barStop) barStop.style.display = "";
      } else {
        dotLg.classList.remove("recording");
        barLabel.textContent = "Idle";
        barLabel.style.color = "";
        setText(`rec-bar-time-${profile}`, "--");
        setText(`rec-bar-frames-${profile}`, "-- frames");
        if (barStart) barStart.style.display = "";
        if (barStop) barStop.style.display = "none";
      }
    }
  }

  updateThermals(data.thermals || []);

  // Lidar preview is controlled only by the preview button. The backend may
  // keep the physical lidar node running for navigation, which must not reopen
  // the UI preview after the operator has stopped it.
  const lidar = data.lidar || {};
  const lidarPts = data.lidar_points;
  if (lidarScanning && Array.isArray(lidarPts)) {
    lidarPoints = lidarPts;
  }

  const statsEl = document.getElementById("lidar-stats");
  if (statsEl && lidar.connected != null) {
    const parts = [];
    if (!lidar.connected) parts.push("Sensor disconnected");
    else {
      parts.push("Sensor ready");
      if (lidarScanning) parts.push(`${lidar.point_count || lidarPoints.length || 0} pts`);
      if (lidar.firmware) parts.push(`fw ${lidar.firmware}`);
    }
    statsEl.textContent = parts.join(" · ");
  }

  updateMotionPreview(data, leftMs, rightMs);
}

function _finiteNumber(...values) {
  for (const value of values) {
    if (value == null || value === "") continue;
    const number = Number(value);
    if (Number.isFinite(number)) return number;
  }
  return null;
}

function _getPath(source, path) {
  let value = source;
  for (const key of path) {
    if (value == null || typeof value !== "object") return undefined;
    value = value[key];
  }
  return value;
}

function _setMotionValue(id, value, digits = 2) {
  const el = document.getElementById(id);
  if (!el) return;
  el.textContent = Number.isFinite(value) ? value.toFixed(digits) : "--";
}

function _quaternionToRollPitch(orientation) {
  if (!orientation) return { roll: null, pitch: null };
  const x = _finiteNumber(orientation.x, orientation.qx);
  const y = _finiteNumber(orientation.y, orientation.qy);
  const z = _finiteNumber(orientation.z, orientation.qz);
  const w = _finiteNumber(orientation.w, orientation.qw);
  if ([x, y, z, w].some(v => v == null)) return { roll: null, pitch: null };

  const sinrCosp = 2 * (w * x + y * z);
  const cosrCosp = 1 - 2 * (x * x + y * y);
  const roll = Math.atan2(sinrCosp, cosrCosp) * 180 / Math.PI;

  const sinp = 2 * (w * y - z * x);
  const pitch = (Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp)) * 180 / Math.PI;
  return { roll, pitch };
}

function updateMotionPreview(data, leftWheelMs, rightWheelMs) {
  const imu = data.imu || data.imu_data || data.imu_status || data.imu_raw || {};
  const odom = data.odom || data.odometry || data.robot_odom || {};

  const linearSpeed = _finiteNumber(
    _getPath(odom, ["twist", "twist", "linear", "x"]),
    _getPath(odom, ["twist", "linear", "x"]),
    _getPath(odom, ["linear_velocity", "x"]),
    odom.linear_speed,
    data.linear_velocity,
    (Number.isFinite(leftWheelMs) && Number.isFinite(rightWheelMs))
      ? (leftWheelMs + rightWheelMs) / 2
      : null
  );

  const angularVelocity = imu.angular_velocity || imu.gyro || imu.angular || {};
  const linearAcceleration = imu.linear_acceleration || imu.acceleration || imu.accel || {};
  const yawRate = _finiteNumber(
    angularVelocity.z,
    imu.gyro_z,
    imu.angular_velocity_z,
    _getPath(odom, ["twist", "twist", "angular", "z"]),
    _getPath(odom, ["twist", "angular", "z"]),
    odom.angular_velocity
  );
  const accelX = _finiteNumber(linearAcceleration.x, imu.accel_x, imu.linear_acceleration_x);
  const accelY = _finiteNumber(linearAcceleration.y, imu.accel_y, imu.linear_acceleration_y);

  const orientation = imu.orientation || imu.quaternion || imu.pose?.orientation || null;
  const quaternionAngles = _quaternionToRollPitch(orientation);
  const roll = _finiteNumber(
    imu.roll_deg,
    quaternionAngles.roll,
    Number.isFinite(Number(imu.roll_rad)) ? Number(imu.roll_rad) * 180 / Math.PI : null
  );
  const pitch = _finiteNumber(
    imu.pitch_deg,
    quaternionAngles.pitch,
    Number.isFinite(Number(imu.pitch_rad)) ? Number(imu.pitch_rad) * 180 / Math.PI : null
  );

  _setMotionValue("motion-linear-speed", linearSpeed);
  _setMotionValue("motion-yaw-rate", yawRate, 3);
  _setMotionValue("imu-accel-x", accelX);
  _setMotionValue("imu-accel-y", accelY);
  _setMotionValue("imu-roll", roll, 1);
  _setMotionValue("imu-pitch", pitch, 1);

  const imuLive = [yawRate, accelX, accelY, roll, pitch].some(Number.isFinite);
  const status = document.getElementById("imu-preview-status");
  if (status) {
    status.textContent = imuLive ? "IMU live" : "Waiting for /imu/data_raw";
    status.className = "motion-status" + (imuLive ? " live" : "");
  }
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

  const sidebarTemp = document.getElementById("sidebar-jetson-temp");
  if (!zones || zones.length === 0) {
    container.innerHTML = '<span class="thermal-placeholder">No thermal zones detected</span>';
    if (sidebarTemp) sidebarTemp.textContent = "No thermal data";
    return;
  }

  const representative = zones.find(z => /cpu|gpu|soc|thermal/i.test(z.type || "")) || zones[0];
  if (sidebarTemp && representative?.temp_c != null) {
    sidebarTemp.textContent = `${Number(representative.temp_c).toFixed(1)}°C`;
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

function setCameraPlaceholder(stream, visible, message = "Start preview to see feed") {
  const placeholder = document.getElementById(`cam-${stream}-empty`);
  if (!placeholder) return;
  placeholder.classList.toggle("hidden", !visible);
  const messageEl = placeholder.querySelector("span");
  if (messageEl) messageEl.textContent = message;
}

function startCamera() {
  const fps = document.getElementById("cam-fps")?.value || "10";
  const quality = document.getElementById("cam-quality")?.value || "50";
  const colorImg = document.getElementById("cam-color");
  const depthImg = document.getElementById("cam-depth");
  const stateEl = document.getElementById("cam-state");
  const toggle = document.getElementById("cam-toggle");
  if (!colorImg || !depthImg || !toggle) return;

  const qs = `fps=${encodeURIComponent(fps)}&quality=${encodeURIComponent(quality)}&_=${Date.now()}`;
  setCameraPlaceholder("color", true, "Connecting to color stream...");
  setCameraPlaceholder("depth", true, "Connecting to depth stream...");

  colorImg.onload = () => setCameraPlaceholder("color", false);
  depthImg.onload = () => setCameraPlaceholder("depth", false);
  colorImg.onerror = () => {
    setCameraPlaceholder("color", true, "Color stream unavailable");
    if (stateEl) { stateEl.textContent = "Stream error"; stateEl.className = "cam-state err"; }
  };
  depthImg.onerror = () => {
    setCameraPlaceholder("depth", true, "Depth stream unavailable");
    if (stateEl) { stateEl.textContent = "Stream error"; stateEl.className = "cam-state err"; }
  };

  colorImg.src = `/api/stream/color?${qs}`;
  depthImg.src = `/api/stream/depth?${qs}`;
  camStreaming = true;
  toggle.innerHTML = '<span class="button-play">■</span> Stop Preview';
  toggle.className = "danger compact";
  if (stateEl) { stateEl.textContent = "Streaming"; stateEl.className = "cam-state ok"; }
}

function stopCamera() {
  const colorImg = document.getElementById("cam-color");
  const depthImg = document.getElementById("cam-depth");
  const stateEl = document.getElementById("cam-state");
  const toggle = document.getElementById("cam-toggle");

  if (colorImg) {
    colorImg.onload = null;
    colorImg.onerror = null;
    colorImg.removeAttribute("src");
  }
  if (depthImg) {
    depthImg.onload = null;
    depthImg.onerror = null;
    depthImg.removeAttribute("src");
  }

  setCameraPlaceholder("color", true);
  setCameraPlaceholder("depth", true);
  camStreaming = false;
  if (toggle) {
    toggle.innerHTML = '<span class="button-play">▶</span> Start Preview';
    toggle.className = "primary compact";
  }
  if (stateEl) { stateEl.textContent = "Stopped"; stateEl.className = "cam-state"; }
}

function toggleCamera() {
  if (camStreaming) stopCamera();
  else startCamera();
}

// ─── Bag Topic Picker ────────────────────────────────────────────

const _bagTopicData = {};

async function loadBagTopics() {
  const profiles = ["perception", "mapping"];
  for (const profile of profiles) {
    const container = document.getElementById(`topic-groups-${profile}`);
    if (container) container.innerHTML = '<div class="topic-load-state">Loading topics…</div>';
  }

  await Promise.all(profiles.map(async profile => {
    try {
      const resp = await fetch(`/api/bag-topics/${profile}`);
      if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
      _bagTopicData[profile] = await resp.json();
    } catch (e) {
      console.error(`Failed to load ${profile} bag topics:`, e);
      _bagTopicData[profile] = { _error: e.message };
    }
    renderTopicPicker(profile);
  }));
}

function _normaliseTopicGroups(data) {
  const raw = data.topic_groups ?? data.groups ?? data.available_topics ?? data.topics ?? {};
  if (Array.isArray(raw)) return { Topics: raw };
  if (!raw || typeof raw !== "object") return {};

  const groups = {};
  for (const [name, value] of Object.entries(raw)) {
    if (Array.isArray(value)) groups[name] = value;
    else if (typeof value === "string") groups[name] = [value];
    else if (value && Array.isArray(value.topics)) groups[name] = value.topics;
  }
  return groups;
}

function renderTopicPicker(profile) {
  const container = document.getElementById(`topic-groups-${profile}`);
  const hint = document.getElementById(`topic-picker-hint-${profile}`);
  const data = _bagTopicData[profile];
  if (!container) return;

  if (!data) {
    container.innerHTML = '<div class="topic-load-state">Loading topics…</div>';
    return;
  }
  if (data._error) {
    container.innerHTML = `<div class="topic-load-state error">Could not load topics: ${data._error}</div>`;
    return;
  }

  const groups = _normaliseTopicGroups(data);
  const selected = data.selected_topics || data.selected || data.enabled_topics || [];
  const recording = Boolean(data.recording);

  if (hint) hint.textContent = recording ? "Locked while recording" : "";

  if (Object.keys(groups).length === 0 && selected.length > 0) groups.Selected = selected;
  if (Object.keys(groups).length === 0) {
    container.innerHTML = '<div class="topic-load-state error">No topics were returned by the recorder API.</div>';
    return;
  }

  container.innerHTML = "";
  for (const [groupName, topics] of Object.entries(groups)) {
    const group = document.createElement("div");
    group.className = "topic-group";

    const label = document.createElement("span");
    label.className = "topic-group-label";
    label.textContent = groupName;
    group.appendChild(label);

    for (const topicValue of topics) {
      const topic = typeof topicValue === "string"
        ? topicValue
        : (topicValue?.name || topicValue?.topic || "");
      if (!topic) continue;

      const lbl = document.createElement("label");
      lbl.className = "topic-cb-label";
      lbl.dataset.topic = topic;
      const cb = document.createElement("input");
      cb.type = "checkbox";
      cb.value = topic;
      cb.checked = selected.includes(topic) || Boolean(topicValue?.selected);
      cb.disabled = recording;
      cb.className = "topic-cb";
      lbl.appendChild(cb);

      const span = document.createElement("span");
      span.textContent = topic.split("/").filter(Boolean).pop() || topic;
      span.title = topic;
      lbl.appendChild(span);


      group.appendChild(lbl);
    }
    container.appendChild(group);
  }
}

async function saveBagTopics(profile) {
  const checkboxes = document.querySelectorAll(`#topic-groups-${profile} .topic-cb:checked`);
  const topics = Array.from(checkboxes).map(cb => cb.value);
  if (topics.length === 0) {
    showTopicMsg(profile, "Select at least one topic.", "err");
    return;
  }
  try {
    const resp = await fetch(`/api/bag-topics/${profile}`, {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ topics }),
    });
    const result = await resp.json();
    if (result.success) {
      showTopicMsg(profile, `Applied ${topics.length} topic(s). Takes effect on next recording.`, "ok");
    } else {
      showTopicMsg(profile, result.error || "Failed", "err");
    }
  } catch (e) {
    showTopicMsg(profile, "Network error: " + e.message, "err");
  }
}

function showTopicMsg(profile, text, type) {
  const el = document.getElementById(`topic-msg-${profile}`);
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
  const empty = document.getElementById("bags-empty");
  const isInitialLoad = !bagsData;
  if (loading) {
    loading.style.display = isInitialLoad ? "" : "none";
    loading.textContent = "Loading...";
  }
  try {
    const resp = await fetch("/api/bags");
    if (!resp.ok) throw new Error("HTTP " + resp.status);
    bagsData = await resp.json();
    renderBags();
    if (loading) loading.style.display = "none";
  } catch (e) {
    console.error("Failed to load bags:", e);
    if (loading) loading.textContent = "Failed to load: " + e.message;
  }
}

function formatBytes(bytes) {
  const safeBytes = Number(bytes) || 0;
  if (safeBytes === 0) return "0 B";
  const units = ["B", "KB", "MB", "GB", "TB"];
  const i = Math.min(Math.floor(Math.log(safeBytes) / Math.log(1024)), units.length - 1);
  const val = safeBytes / Math.pow(1024, i);
  return val.toFixed(i === 0 ? 0 : 1) + " " + units[i];
}

function _bagDateValue(bag) {
  const value = bag.modified || bag.created || bag.date || bag.timestamp || 0;
  const parsed = new Date(value).getTime();
  return Number.isFinite(parsed) ? parsed : 0;
}

function formatDate(isoStr) {
  try {
    const d = new Date(isoStr);
    if (Number.isNaN(d.getTime())) return isoStr || "--";
    return d.toLocaleDateString(undefined, {
      year: "numeric",
      month: "short",
      day: "numeric",
      hour: "2-digit",
      minute: "2-digit",
    });
  } catch {
    return isoStr || "--";
  }
}

function _bagProfile(bag) {
  const explicit = String(bag.profile || bag.type || bag.category || "").toLowerCase();
  const name = String(bag.name || "").toLowerCase();
  if (explicit.includes("perception") || name.startsWith("perception") || name.includes("_perception")) return "perception";
  if (explicit.includes("mapping") || name.startsWith("mapping") || name.includes("_mapping")) return "mapping";

  const topics = Array.isArray(bag.topics) ? bag.topics.join(" ").toLowerCase() : "";
  if (/camera|image_raw|depth/.test(topics)) return "perception";
  if (/scan|odom|\/tf|imu/.test(topics)) return "mapping";
  return "mapping";
}

function _bagDateObject(bag) {
  const value = bag.modified || bag.created || bag.date || bag.timestamp;
  const date = new Date(value);
  return Number.isNaN(date.getTime()) ? null : date;
}

function _localDateKey(date) {
  if (!date) return "unknown";
  const year = date.getFullYear();
  const month = String(date.getMonth() + 1).padStart(2, "0");
  const day = String(date.getDate()).padStart(2, "0");
  return `${year}-${month}-${day}`;
}

function _formatBagDay(date) {
  if (!date) return "Unknown date";
  const today = new Date();
  const yesterday = new Date(today);
  yesterday.setDate(today.getDate() - 1);
  const key = _localDateKey(date);
  const prefix = key === _localDateKey(today)
    ? "Today"
    : key === _localDateKey(yesterday)
      ? "Yesterday"
      : date.toLocaleDateString(undefined, { weekday: "long" });
  return `${prefix}, ${date.toLocaleDateString(undefined, {
    year: "numeric",
    month: "long",
    day: "numeric",
  })}`;
}

function _formatBagTime(bag) {
  const date = _bagDateObject(bag);
  return date
    ? date.toLocaleTimeString(undefined, { hour: "2-digit", minute: "2-digit", second: "2-digit" })
    : "--";
}

function _createBagTableRow(bag) {
  const tr = document.createElement("tr");

  const tdTime = document.createElement("td");
  tdTime.className = "bag-time";
  tdTime.textContent = _formatBagTime(bag);
  tr.appendChild(tdTime);

  const tdName = document.createElement("td");
  tdName.className = "bag-name";
  tdName.textContent = bag.name || "Unnamed recording";
  tdName.title = bag.name || "";
  tr.appendChild(tdName);

  const tdSize = document.createElement("td");
  tdSize.textContent = formatBytes(bag.size_bytes);
  tr.appendChild(tdSize);

  const tdFiles = document.createElement("td");
  tdFiles.className = "bag-files";
  const files = bag.files || [];
  const exts = [...new Set(files.map(f => f.includes(".") ? "." + f.split(".").pop() : f))];
  tdFiles.textContent = `${files.length} files${exts.length ? ` (${exts.join(", ")})` : ""}`;
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
  return tr;
}

function _renderBagRows(profile, bags) {
  const container = document.getElementById(`bags-list-${profile}`);
  const empty = document.getElementById(`bags-empty-${profile}`);
  const count = document.getElementById(`bags-count-${profile}`);
  if (!container || !empty) return;

  container.innerHTML = "";
  const label = `${bags.length} recording${bags.length === 1 ? "" : "s"}`;
  if (count) count.textContent = label;
  empty.style.display = bags.length ? "none" : "block";
  if (!bags.length) return;

  const groups = new Map();
  for (const bag of bags) {
    const date = _bagDateObject(bag);
    const key = _localDateKey(date);
    if (!groups.has(key)) groups.set(key, { date, bags: [] });
    groups.get(key).bags.push(bag);
  }

  for (const { date, bags: dayBags } of groups.values()) {
    const section = document.createElement("section");
    section.className = "bag-date-section";

    const heading = document.createElement("div");
    heading.className = "bag-date-heading";
    const title = document.createElement("h4");
    title.textContent = _formatBagDay(date);
    const dayCount = document.createElement("span");
    dayCount.textContent = `${dayBags.length} bag${dayBags.length === 1 ? "" : "s"}`;
    heading.append(title, dayCount);

    const shell = document.createElement("div");
    shell.className = "table-shell";
    const table = document.createElement("table");
    table.className = "bags-table dated-bags-table";
    table.innerHTML = '<thead><tr><th>Time</th><th>Name</th><th>Size</th><th>Files</th><th></th></tr></thead>';
    const tbody = document.createElement("tbody");
    for (const bag of dayBags) tbody.appendChild(_createBagTableRow(bag));
    table.appendChild(tbody);
    shell.appendChild(table);

    section.append(heading, shell);
    container.appendChild(section);
  }
}

function clearBagDateFilter() {
  const input = document.getElementById("bags-date-filter");
  if (input) input.value = "";
  renderBags();
}

function renderBags() {
  if (!bagsData) return;
  const summary = document.getElementById("bags-summary");
  const allBags = [...(bagsData.bags || [])];
  const selectedDate = document.getElementById("bags-date-filter")?.value || "";
  const clearButton = document.getElementById("bags-clear-date");
  if (clearButton) clearButton.disabled = !selectedDate;

  const sortDirection = document.getElementById("bags-sort")?.value || "desc";
  allBags.sort((a, b) => sortDirection === "asc"
    ? _bagDateValue(a) - _bagDateValue(b)
    : _bagDateValue(b) - _bagDateValue(a));

  const bags = selectedDate
    ? allBags.filter(bag => _localDateKey(_bagDateObject(bag)) === selectedDate)
    : allBags;

  if (summary) {
    const filteredText = selectedDate ? `${bags.length} of ${allBags.length}` : `${bags.length}`;
    const visibleBytes = bags.reduce((sum, bag) => sum + (Number(bag.size_bytes) || 0), 0);
    summary.textContent = `${filteredText} recording${bags.length === 1 ? "" : "s"} · ${formatBytes(visibleBytes)}`;
  }

  const diskWrap = document.getElementById("disk-bar-wrap");
  if (diskWrap && bagsData.disk_total) {
    diskWrap.style.display = "";
    const total = Number(bagsData.disk_total) || 1;
    const free = Number(bagsData.disk_free) || 0;
    const used = Number(bagsData.disk_used) || 0;
    const bagBytes = Number(bagsData.total_bytes) || 0;
    const otherUsed = Math.max(0, used - bagBytes);

    document.getElementById("disk-bar-bags").style.width = Math.max(0.5, (bagBytes / total) * 100) + "%";
    document.getElementById("disk-bar-other").style.width = Math.max(0.5, (otherUsed / total) * 100) + "%";
    document.getElementById("disk-label-used").textContent =
      `Bags: ${formatBytes(bagBytes)} · Other: ${formatBytes(otherUsed)} · Total: ${formatBytes(total)}`;
    document.getElementById("disk-label-free").textContent = `${formatBytes(free)} free`;
  } else if (diskWrap) {
    diskWrap.style.display = "none";
  }

  const perception = bags.filter(bag => _bagProfile(bag) === "perception");
  const mapping = bags.filter(bag => _bagProfile(bag) === "mapping");
  _renderBagRows("perception", perception);
  _renderBagRows("mapping", mapping);

  const globalEmpty = document.getElementById("bags-empty");
  if (globalEmpty) {
    globalEmpty.style.display = bags.length ? "none" : "block";
    globalEmpty.textContent = selectedDate
      ? "No recordings were found on the selected date."
      : "No recordings found.";
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

async function _parseJsonResponse(resp) {
  const text = await resp.text();
  try {
    return JSON.parse(text);
  } catch {
    return { success: false, message: text || `HTTP ${resp.status}` };
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
      result = await _parseJsonResponse(resp);
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
      result = await _parseJsonResponse(resp);
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
      const dhcpResult = await _parseJsonResponse(dhcpResp);
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

// ─── Lidar ───────────────────────────────────────────────────────

let lidarScanning = false;
let lidarPoints = [];
let lidarAnimFrame = null;

let lidarRequestPending = false;

function _setLidarPreviewUi(active, message = active ? "Previewing" : "Stopped", error = false) {
  const toggle = document.getElementById("lidar-toggle");
  const status = document.getElementById("lidar-status");
  if (toggle) {
    toggle.innerHTML = active
      ? '<span class="button-play">■</span> Stop Preview'
      : '<span class="button-play">▶</span> Start Preview';
    toggle.className = (active ? "danger" : "primary") + " compact";
    toggle.disabled = lidarRequestPending;
  }
  if (status) {
    status.textContent = message;
    status.className = "lidar-status" + (error ? " err" : active ? " ok" : "");
  }
}

async function startLidar() {
  if (lidarScanning || lidarRequestPending) return;
  lidarRequestPending = true;
  _setLidarPreviewUi(false, "Starting...");

  try {
    const resp = await fetch("/api/lidar/start", { method: "POST" });
    const text = await resp.text();
    let result = {};
    try { result = text ? JSON.parse(text) : {}; } catch { result = {}; }
    if (!resp.ok || result.success === false) {
      throw new Error(result.message || `HTTP ${resp.status}`);
    }

    lidarScanning = true;
    lidarPoints = [];
    _setLidarPreviewUi(true, "Previewing");
    if (!lidarAnimFrame) lidarAnimFrame = requestAnimationFrame(drawLidar);
  } catch (e) {
    lidarScanning = false;
    lidarPoints = [];
    _setLidarPreviewUi(false, e.message || "Failed to start", true);
    drawLidar();
  } finally {
    lidarRequestPending = false;
    const toggle = document.getElementById("lidar-toggle");
    if (toggle) toggle.disabled = false;
  }
}

async function stopLidar() {
  if (lidarRequestPending) return;

  // Stop the UI immediately. WebSocket lidar.scanning status is deliberately
  // ignored, so the preview cannot turn itself back on.
  lidarScanning = false;
  lidarPoints = [];
  if (lidarAnimFrame) {
    cancelAnimationFrame(lidarAnimFrame);
    lidarAnimFrame = null;
  }
  _setLidarPreviewUi(false, "Stopped");
  drawLidar();

  lidarRequestPending = true;
  const toggle = document.getElementById("lidar-toggle");
  if (toggle) toggle.disabled = true;
  try {
    await fetch("/api/lidar/stop", { method: "POST" });
  } catch (e) {
    console.warn("Lidar stop request failed; preview remains stopped:", e);
  } finally {
    lidarRequestPending = false;
    if (toggle) toggle.disabled = false;
  }
}

function toggleLidar() {
  if (lidarScanning) stopLidar();
  else startLidar();
}

function drawLidar() {
  const canvas = document.getElementById("lidar-canvas");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  const W = canvas.width, H = canvas.height;
  const cx = W / 2, cy = H / 2;
  const rangeM = parseFloat(document.getElementById("lidar-range")?.value || "8");
  const rangeMax = rangeM * 1000;
  const radius = Math.min(cx, cy) - 20;

  ctx.fillStyle = "#0a0e17";
  ctx.fillRect(0, 0, W, H);

  // Distance rings
  const ringCount = 4;
  ctx.strokeStyle = "#1a2035";
  ctx.lineWidth = 1;
  ctx.font = "11px monospace";
  ctx.fillStyle = "#3a4565";
  for (let i = 1; i <= ringCount; i++) {
    const r = (radius / ringCount) * i;
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.stroke();
    const mLabel = ((rangeM / ringCount) * i).toFixed(1) + "m";
    ctx.fillText(mLabel, cx + r + 3, cy - 3);
  }

  // Cross-hairs
  ctx.strokeStyle = "#1a2035";
  ctx.beginPath();
  ctx.moveTo(cx, 10); ctx.lineTo(cx, H - 10);
  ctx.moveTo(10, cy); ctx.lineTo(W - 10, cy);
  ctx.stroke();

  // Robot marker
  ctx.fillStyle = "#e94560";
  ctx.beginPath();
  ctx.arc(cx, cy, 4, 0, Math.PI * 2);
  ctx.fill();

  // Points
  if (lidarPoints.length > 0) {
    ctx.fillStyle = "#4ecca3";
    for (const pt of lidarPoints) {
      const angleDeg = pt[0];
      const distMm = pt[1];
      if (distMm <= 0 || distMm > rangeMax) continue;
      const angleRad = (angleDeg - 90) * Math.PI / 180;
      const r = (distMm / rangeMax) * radius;
      const px = cx + r * Math.cos(angleRad);
      const py = cy + r * Math.sin(angleRad);
      ctx.fillRect(px - 1.5, py - 1.5, 3, 3);
    }
  }

  // Forward arrow
  ctx.save();
  ctx.strokeStyle = "#4ecca3";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(cx, cy - 8);
  ctx.lineTo(cx, 24);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(cx - 5, 30);
  ctx.lineTo(cx, 22);
  ctx.lineTo(cx + 5, 30);
  ctx.fillStyle = "#4ecca3";
  ctx.fill();
  ctx.restore();

  // Direction labels
  ctx.fillStyle = "#4ecca3";
  ctx.font = "bold 11px monospace";
  ctx.textAlign = "center";
  ctx.fillText("FWD", cx, 14);
  ctx.fillStyle = "#4ecca366";
  ctx.font = "11px monospace";
  ctx.fillText("BACK", cx, H - 6);
  ctx.textAlign = "left";
  ctx.fillText("LEFT", 2, cy - 3);
  ctx.textAlign = "right";
  ctx.fillText("RIGHT", W - 2, cy - 3);
  ctx.textAlign = "left";

  if (!lidarScanning && lidarPoints.length === 0) {
    ctx.fillStyle = "#d0d5dd";
    ctx.font = "16px ui-sans-serif, system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("Lidar preview stopped", cx, cy + 34);
    ctx.textAlign = "left";
  }

  if (lidarScanning) lidarAnimFrame = requestAnimationFrame(drawLidar);
}

// ─── Tabs ────────────────────────────────────────────────────────

const PAGE_META = {
  dashboard: {
    title: "Dashboard",
    subtitle: "Monitor robot systems and hardware status",
    icon: '<rect x="3" y="3" width="7" height="7" rx="1"/><rect x="14" y="3" width="7" height="7" rx="1"/><rect x="3" y="14" width="7" height="7" rx="1"/><rect x="14" y="14" width="7" height="7" rx="1"/>'
  },
  recordings: {
    title: "Recordings",
    subtitle: "Configure topics, preview sensors, and record data",
    icon: '<circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="3" fill="currentColor" stroke="none"/><path d="M12 2v2M12 20v2M2 12h2M20 12h2"/>'
  },
  "recorded-bags": {
    title: "Recorded Bags",
    subtitle: "Manage saved ROS bag recordings",
    icon: '<path d="M3 7.5A2.5 2.5 0 0 1 5.5 5H10l2 2h6.5A2.5 2.5 0 0 1 21 9.5v8A2.5 2.5 0 0 1 18.5 20h-13A2.5 2.5 0 0 1 3 17.5z"/>'
  },
  controller: {
    title: "Controller",
    subtitle: "Inspect input and configure button assignments",
    icon: '<path d="M8 8h8a5 5 0 0 1 4.8 3.6l1 3.4a3 3 0 0 1-5.2 2.8L14.8 16H9.2l-1.8 1.8A3 3 0 0 1 2.2 15l1-3.4A5 5 0 0 1 8 8z"/><path d="M8 11v4M6 13h4M16 12h.01M18 14h.01"/>'
  },
  network: {
    title: "Network",
    subtitle: "Manage robot connectivity and access-point settings",
    icon: '<path d="M5 12.5a10 10 0 0 1 14 0"/><path d="M8.5 16a5 5 0 0 1 7 0"/><path d="M12 20h.01"/>'
  }
};

function updatePageHeading(tabName) {
  const meta = PAGE_META[tabName] || PAGE_META.dashboard;
  setText("page-title", meta.title);
  setText("page-subtitle", meta.subtitle);
  const svg = document.getElementById("page-heading-svg");
  if (svg) svg.innerHTML = meta.icon;
}

function initTabs() {
  const tabs = document.querySelectorAll(".tabs .tab");
  tabs.forEach(tab => {
    tab.addEventListener("click", () => {
      const tabName = tab.dataset.tab;
      tabs.forEach(t => t.classList.remove("active"));
      tab.classList.add("active");
      document.querySelectorAll(".tab-content").forEach(c => c.classList.remove("active"));
      document.getElementById("tab-" + tabName)?.classList.add("active");
      updatePageHeading(tabName);
      if (window.matchMedia("(max-width: 900px)").matches) closeMobileSidebar();

      if (tabName === "recordings") {
        loadBagTopics();
        drawLidar();
      }

      if (tabName === "recorded-bags") {
        if (bagsData) renderBags();
        else loadBags();
        stopBagsAutoRefresh();
      } else {
        stopBagsAutoRefresh();
      }

      if (tabName === "network") loadNetwork();
      if (tabName !== "recordings") {
        if (camStreaming) stopCamera();
        if (lidarScanning) stopLidar();
      }
    });
  });
}


// ─── Collapsible navigation and floating topic menus ─────────────

const _sidebarMedia = window.matchMedia("(max-width: 900px)");

function updateSidebarAria() {
  const toggle = document.getElementById("sidebar-toggle");
  if (!toggle) return;
  const expanded = _sidebarMedia.matches
    ? document.body.classList.contains("sidebar-open")
    : !document.body.classList.contains("sidebar-collapsed");
  toggle.setAttribute("aria-expanded", String(expanded));
}

function closeMobileSidebar() {
  document.body.classList.remove("sidebar-open");
  updateSidebarAria();
}

function toggleSidebar() {
  if (_sidebarMedia.matches) {
    document.body.classList.toggle("sidebar-open");
  } else {
    document.body.classList.toggle("sidebar-collapsed");
  }
  updateSidebarAria();
}

function syncSidebarMode() {
  if (_sidebarMedia.matches) {
    document.body.classList.remove("sidebar-collapsed");
    document.body.classList.remove("sidebar-open");
  } else {
    document.body.classList.remove("sidebar-open");
  }
  updateSidebarAria();
}

function initSidebar() {
  document.getElementById("sidebar-toggle")?.addEventListener("click", toggleSidebar);
  document.getElementById("sidebar-backdrop")?.addEventListener("click", closeMobileSidebar);
  _sidebarMedia.addEventListener?.("change", syncSidebarMode);
  document.addEventListener("keydown", event => {
    if (event.key === "Escape") {
      closeMobileSidebar();
      document.querySelectorAll(".topic-checklist[open]").forEach(menu => { menu.open = false; });
    }
  });
  syncSidebarMode();
}

function initTopicDropdowns() {
  const menus = Array.from(document.querySelectorAll(".topic-checklist"));
  for (const menu of menus) {
    menu.open = false;
    menu.addEventListener("toggle", () => {
      if (!menu.open) return;
      for (const other of menus) {
        if (other !== menu) other.open = false;
      }
    });
  }

  document.addEventListener("pointerdown", event => {
    for (const menu of menus) {
      if (menu.open && !menu.contains(event.target)) menu.open = false;
    }
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

async function resumeEstop() {
  // /wheel_driver_node/stop is the action that clears the e-stop flag
  // (gentle stop + recovery, see _srv_stop in wheel_driver_node.py).
  await cmd("wheel_driver_node", "stop");
}

async function switchBackend() {
  try { await fetch("/api/command/wheel_driver_node/switch_backend", { method: "POST" }); }
  catch (e) { console.error("Switch failed:", e); }
}

async function saveMotorDefaults() {
  const msg = document.getElementById("hw-save-msg");
  try {
    const resp = await fetch("/api/mks/save_current_defaults", { method: "POST" });
    const r = await resp.json();
    if (msg) {
      msg.textContent = r.success ? "Saved" : (r.message || "Failed");
      msg.style.color = r.success ? "var(--ok)" : "var(--err)";
      setTimeout(() => { msg.textContent = ""; }, 3000);
    }
  } catch (e) {
    if (msg) { msg.textContent = "Failed"; msg.style.color = "var(--err)"; }
  }
}

async function scanMotors() {
  // Fire the manual MKS bus scan. Status is published asynchronously on
  // /mks_bus/status and will refresh the motor list within a second.
  const msg = document.getElementById("hw-save-msg");
  try {
    const resp = await fetch("/api/mks/scan", { method: "POST" });
    const r = await resp.json();
    if (msg) {
      msg.textContent = r.success ? "Scanning…" : (r.message || "Failed");
      msg.style.color = r.success ? "var(--ok)" : "var(--err)";
      setTimeout(() => { msg.textContent = ""; }, 3000);
    }
  } catch (e) {
    if (msg) { msg.textContent = "Failed"; msg.style.color = "var(--err)"; }
  }
}

const _motorDirty = {};

async function setMotorCurrent(motorId) {
  const runEl = document.getElementById("hw-m" + motorId + "-run");
  const holdEl = document.getElementById("hw-m" + motorId + "-hold");
  const run = runEl ? parseInt(runEl.value) : 0;
  const hold = holdEl ? parseInt(holdEl.value) : 0;
  if (!run && !hold) return;
  _motorDirty[motorId] = true;
  try {
    await fetch("/api/mks/set_current", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ motor_id: motorId, run_current_ma: run || 0, hold_current_pct: hold || 0 }),
    });
  } catch (e) { console.error("Set current failed:", e); }
}

async function jogMotor(motorId, inputId, speedInputId, accInputId) {
  const input = document.getElementById(inputId);
  if (!input) return;
  const turns = parseFloat(input.value);
  if (isNaN(turns) || turns === 0) return;
  const payload = { motor_id: motorId, turns: turns };
  if (speedInputId) {
    const spdEl = document.getElementById(speedInputId);
    if (spdEl) payload.speed_rpm = parseInt(spdEl.value) || 300;
  }
  if (accInputId) {
    const accEl = document.getElementById(accInputId);
    if (accEl) payload.accel = parseInt(accEl.value) || 3;
  }
  try {
    await fetch("/api/mks/move_turns", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
  } catch (e) { console.error("Jog failed:", e); }
}

async function startRecording(profile) {
  try { await fetch(`/api/recording/${profile}/start`, { method: "POST" }); }
  catch (e) { console.error(`Start ${profile} recording failed:`, e); }
}

async function stopRecording(profile) {
  try { await fetch(`/api/recording/${profile}/stop`, { method: "POST" }); }
  catch (e) { console.error(`Stop ${profile} recording failed:`, e); }
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
initSidebar();
initTopicDropdowns();
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

const _lidarRange = document.getElementById("lidar-range");
const _lidarRangeVal = document.getElementById("lidar-range-val");
if (_lidarRange && _lidarRangeVal) {
  _lidarRange.addEventListener("input", () => { _lidarRangeVal.textContent = Number(_lidarRange.value).toFixed(1) + " m"; });
}