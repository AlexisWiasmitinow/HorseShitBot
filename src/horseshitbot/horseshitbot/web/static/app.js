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

const _recorderUiState = {
  perception: { recording: false, duration_sec: 0, frame_count: 0 },
  mapping: { recording: false, duration_sec: 0, frame_count: 0 },
};

// A user start/stop command is authoritative for the UI until the opposite
// command is requested. This avoids flicker from stale/interleaved status
// messages while the recorder process is starting or stopping.
const _recorderIntent = {
  perception: null,
  mapping: null,
};

const _recorderLastTrueAt = {
  perception: 0,
  mapping: 0,
};

const _recorderTrueConfirmations = {
  perception: 0,
  mapping: 0,
};

function resolveRecorderState(profile, payload) {
  const current = _recorderUiState[profile] || {
    recording: false,
    duration_sec: 0,
    frame_count: 0,
  };

  const incoming =
    payload && typeof payload === "object"
      ? { ...current, ...payload }
      : { ...current };

  const receivedAt = Number(payload?._received_at || 0);
  const statusIsFresh =
    !receivedAt || (Date.now() / 1000 - receivedAt) < 5.0;

  const reported =
    statusIsFresh && payload && typeof payload.recording === "boolean"
      ? payload.recording
      : null;

  const intent = _recorderIntent[profile];

  if (intent !== null) {
    incoming.recording = intent;
    if (reported === true) {
      _recorderLastTrueAt[profile] = Date.now();
      _recorderTrueConfirmations[profile] += 1;
    }
  } else if (reported === true) {
    _recorderTrueConfirmations[profile] += 1;

    // One initial true packet is not enough to mark a recorder active.
    const confirmed =
      current.recording || _recorderTrueConfirmations[profile] >= 2;

    incoming.recording = confirmed;
    if (confirmed) _recorderLastTrueAt[profile] = Date.now();
  } else if (reported === false) {
    _recorderTrueConfirmations[profile] = 0;

    const recentlyTrue =
      Date.now() - (_recorderLastTrueAt[profile] || 0) < 3000;

    incoming.recording = current.recording && recentlyTrue;
  } else if (!statusIsFresh && intent === null) {
    incoming.recording = false;
    _recorderTrueConfirmations[profile] = 0;
  }

  _recorderUiState[profile] = incoming;
  return incoming;
}

function setRecordingButtonState(profile, recording) {
  const button = document.getElementById(`rec-bar-toggle-${profile}`);
  const text = document.getElementById(`rec-bar-toggle-text-${profile}`);
  const icon = document.getElementById(`rec-bar-toggle-icon-${profile}`);

  if (!button) return;

  button.classList.toggle("on", recording);
  button.classList.toggle("off", !recording);
  button.setAttribute("aria-pressed", recording ? "true" : "false");
  button.title = recording
    ? "Recording is active. Press to stop."
    : "Recording is inactive. Press to start.";

  if (text) text.textContent = recording ? "Stop Recording" : "Start Recording";
  if (icon) icon.textContent = recording ? "■" : "◎";
}

function setRecordingButtonBusy(profile, busy) {
  const button = document.getElementById(`rec-bar-toggle-${profile}`);
  if (!button) return;
  button.disabled = busy;
  button.classList.toggle("is-busy", busy);
}

function applyOptimisticRecorderState(profile, recording) {
  _recorderIntent[profile] = recording;

  const state = _recorderUiState[profile] || {};
  state.recording = recording;
  state.duration_sec = recording ? (state.duration_sec || 0) : 0;
  _recorderUiState[profile] = state;

  const dot = document.getElementById(`rec-dot-lg-${profile}`);
  const label = document.getElementById(`rec-bar-label-${profile}`);

  dot?.classList.toggle("recording", recording);
  if (label) {
    label.textContent = recording ? "RECORDING" : "Idle";
    label.style.color = recording ? "var(--red)" : "";
  }

  setRecordingButtonState(profile, recording);
}

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
    updateDashboardConnection(true);
  };

  ws.onclose = () => {
    if (badgeText) badgeText.textContent = "Disconnected";
    else if (badge) badge.textContent = "Disconnected";
    badge?.classList.remove("connected");
    updateDashboardConnection(false);
    setTimeout(connectWs, 2000);
  };

  ws.onerror = () => ws.close();

  ws.onmessage = (evt) => {
    try {
      const data = JSON.parse(evt.data);
      dashboardRuntime.telemetryLastSeenAt = Date.now();
      dashboardRuntime.telemetryConnected = true;
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
            `<label class="motor-field">` +
              `<span>Run current</span>` +
              `<span class="motor-input-wrap">` +
                `<input type="number" id="hw-m${id}-run" class="motor-cur-input" min="10" max="5200" step="100" oninput="_motorDirty[${id}]=true">` +
                `<small>mA</small>` +
              `</span>` +
            `</label>` +
            `<label class="motor-field">` +
              `<span>Hold current</span>` +
              `<span class="motor-input-wrap">` +
                `<input type="number" id="hw-m${id}-hold" class="motor-cur-input" min="10" max="100" step="10" oninput="_motorDirty[${id}]=true">` +
                `<small>%</small>` +
              `</span>` +
            `</label>` +
            `<button class="primary motor-set-button" onclick="setMotorCurrent(${id})">Set</button>` +
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

  updateControllerGamepadCard({
    connected: Boolean(gp.connected || _cachedBtInfo.connected),
    name: gp.connected ? (gp.name || "Gamepad connected") : "",
    mac,
    battery: batt,
  });

  for (const profile of ["perception", "mapping"]) {
    const rec = resolveRecorderState(profile, data[profile + "_recorder"]);
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
      } else {
        dotLg.classList.remove("recording");
        barLabel.textContent = "Idle";
        barLabel.style.color = "";
        setText(`rec-bar-time-${profile}`, "--");
        setText(`rec-bar-frames-${profile}`, "-- frames");
      }
      setRecordingButtonState(profile, rec.recording);
    }
  }

  updateMotionTelemetry(data);
  updateThermals(data.thermals || []);

  // Lidar
  const lidar = data.lidar || {};
  const lidarPts = data.lidar_points;

  // The robot may publish lidar points continuously for ROS. The web preview is
  // independent: ignore those points unless the user explicitly started it.
  if (lidarScanning && Array.isArray(lidarPts)) {
    lidarPoints = lidarPts;
  } else if (!lidarScanning && lidarPoints.length) {
    lidarPoints = [];
    drawLidar();
  }

  const lidarStatusEl = document.getElementById("lidar-status");
  if (lidarStatusEl && !lidarScanning) {
    lidarStatusEl.textContent = lidar.connected === false ? "Disconnected" : "Stopped";
    lidarStatusEl.className = lidar.connected === false ? "lidar-status err" : "lidar-status";
  }

  const statsEl = document.getElementById("lidar-stats");
  if (statsEl && lidar.connected != null) {
    const parts = [];
    if (!lidar.connected) {
      parts.push("Disconnected");
    } else if (lidarScanning) {
      parts.push(`${lidar.point_count || lidarPoints.length || 0} pts`);
      if (lidar.firmware) parts.push(`fw ${lidar.firmware}`);
    } else {
      parts.push("Sensor ready");
      if (lidar.firmware) parts.push(`fw ${lidar.firmware}`);
    }
    statsEl.textContent = parts.join(" · ");
  }
  updateDashboardOverview(data);
}

// ─── Motion / IMU ─────────────────────────────────────────────────

function _firstFinite(...values) {
  for (const value of values) {
    if (value == null || value === "") continue;
    const number = Number(value);
    if (Number.isFinite(number)) return number;
  }
  return null;
}

function _motionText(id, value, digits = 2) {
  const el = document.getElementById(id);
  if (!el) return;
  el.textContent = Number.isFinite(value) ? value.toFixed(digits) : "--";
}

function _quaternionToRollPitch(q) {
  if (!q) return { roll: null, pitch: null };
  const x = _firstFinite(q.x, q[0]);
  const y = _firstFinite(q.y, q[1]);
  const z = _firstFinite(q.z, q[2]);
  const w = _firstFinite(q.w, q[3]);
  if ([x, y, z, w].some(v => v == null)) return { roll: null, pitch: null };

  const sinrCosp = 2 * (w * x + y * z);
  const cosrCosp = 1 - 2 * (x * x + y * y);
  const roll = Math.atan2(sinrCosp, cosrCosp) * 180 / Math.PI;

  const sinp = 2 * (w * y - z * x);
  const pitch = (Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp)) * 180 / Math.PI;
  return { roll, pitch };
}

function updateMotionTelemetry(data) {
  const wheel = data.wheel_status || {};
  const left = rpmToMs(wheel.left_rpm || 0);
  const right = rpmToMs(wheel.right_rpm || 0);
  const linearSpeed = (left + right) / 2;

  const imu = data.imu || data.imu_data || data.imu_status || {};
  const angular = imu.angular_velocity || imu.gyro || imu.angular || {};
  const accel = imu.linear_acceleration || imu.acceleration || imu.accel || {};
  const orientation = imu.orientation || imu.quaternion || imu.q || null;

  const yawRate = _firstFinite(
    angular.z,
    imu.angular_velocity_z,
    imu.gyro_z,
    imu.yaw_rate
  );
  const accelX = _firstFinite(accel.x, imu.accel_x, imu.linear_acceleration_x);
  const accelY = _firstFinite(accel.y, imu.accel_y, imu.linear_acceleration_y);

  const converted = _quaternionToRollPitch(orientation);
  const roll = _firstFinite(imu.roll, imu.roll_deg, converted.roll);
  const pitch = _firstFinite(imu.pitch, imu.pitch_deg, converted.pitch);

  _motionText("motion-linear-speed", linearSpeed);
  _motionText("motion-yaw-rate", yawRate);
  _motionText("motion-accel-x", accelX);
  _motionText("motion-accel-y", accelY);
  _motionText("motion-roll", roll, 1);
  _motionText("motion-pitch", pitch, 1);

  const state = document.getElementById("imu-state");
  if (state) {
    const hasImu = [yawRate, accelX, accelY, roll, pitch].some(Number.isFinite);
    state.textContent = hasImu ? "Live IMU data" : "Waiting for /imu/data_raw";
    state.className = "motion-state" + (hasImu ? " ok" : "");
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
  const columnsRoot = document.getElementById("ctrl-columns");
  if (!columnsRoot || !ctrlConfig) return;

  columnsRoot.innerHTML = "";

  const actions = ctrlConfig.available_actions || {};
  const buttons = ctrlConfig.button_names || [];
  const mapping = ctrlConfig.buttons || {};
  const axes = ctrlConfig.axes || {};

  const entries = [];

  for (const [axisName, axisDesc] of Object.entries(axes)) {
    entries.push({
      name: axisName,
      readOnly: true,
      action: axisDesc,
    });
  }

  for (const buttonName of buttons) {
    entries.push({
      name: buttonName,
      readOnly: false,
      selected: mapping[buttonName] || "none",
    });
  }

  const splitAt = Math.ceil(entries.length / 2);
  const groups = [
    entries.slice(0, splitAt),
    entries.slice(splitAt),
  ];

  for (const group of groups) {
    const column = document.createElement("section");
    column.className = "controller-assignment-column";

    const header = document.createElement("div");
    header.className = "controller-assignment-column-header";
    header.innerHTML = "<span></span><strong>Button</strong><strong>Action</strong>";
    column.appendChild(header);

    for (const entry of group) {
      const row = document.createElement("div");
      row.className =
        "controller-assignment-row" +
        (entry.readOnly ? " axis-row" : "");
      row.id = `ctrl-row-${entry.name}`;

      const dot = document.createElement("span");
      dot.className = "ctrl-dot";
      dot.id = `ctrl-dot-${entry.name}`;

      const name = document.createElement("strong");
      name.className = "controller-assignment-name";
      name.textContent = entry.name;

      const actionCell = document.createElement("div");
      actionCell.className = "controller-assignment-action";

      if (entry.readOnly) {
        const description = document.createElement("span");
        description.className = "axis-action";
        description.textContent = entry.action;
        actionCell.appendChild(description);
      } else {
        const select = document.createElement("select");
        select.id = `ctrl-sel-${entry.name}`;
        select.dataset.btn = entry.name;

        for (const [actionId, actionLabel] of Object.entries(actions)) {
          const option = document.createElement("option");
          option.value = actionId;
          option.textContent = actionLabel;
          option.selected = entry.selected === actionId;
          select.appendChild(option);
        }

        actionCell.appendChild(select);
      }

      row.append(dot, name, actionCell);
      column.appendChild(row);
    }

    columnsRoot.appendChild(column);
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
  const image = document.getElementById(`cam-${stream}`);
  if (placeholder) {
    placeholder.classList.toggle("hidden", !visible);
    const messageEl = placeholder.querySelector("span");
    if (messageEl) messageEl.textContent = message;
  }
  // Empty <img> elements show a broken-image glyph in Chromium. Keep the
  // element invisible until the MJPEG stream has emitted its first frame.
  if (image) image.classList.toggle("stream-visible", !visible);
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
  toggle.className = "danger compact preview-toggle";
  if (stateEl) { stateEl.textContent = "Streaming"; stateEl.className = "cam-state ok"; }
  syncDashboardSensorControls();
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
    toggle.className = "compact preview-toggle";
  }
  if (stateEl) { stateEl.textContent = "Stopped"; stateEl.className = "cam-state"; }
  syncDashboardSensorControls();
}

function toggleCamera() {
  if (camStreaming) stopCamera();
  else startCamera();
}

// ─── Bag Topic Picker ────────────────────────────────────────────

const _bagTopicData = {};

async function loadBagTopics() {
  for (const profile of ["perception", "mapping"]) {
    try {
      const resp = await fetch(`/api/bag-topics/${profile}`);
      if (!resp.ok) throw new Error("HTTP " + resp.status);
      _bagTopicData[profile] = await resp.json();
    } catch (e) {
      console.error(`Failed to load ${profile} bag topics:`, e);
    }
  }
  renderTopicPicker("perception");
  renderTopicPicker("mapping");
}

function updateTopicCounts(profile) {
  const data = _bagTopicData[profile] || {};
  const groups = data.topic_groups || {};
  const allTopics = new Set(Object.values(groups).flat());
  const rendered = document.querySelectorAll(`#topic-groups-${profile} .topic-cb`);
  let enabled;

  if (rendered.length) {
    enabled = Array.from(rendered).filter(cb => cb.checked).length;
  } else {
    enabled = new Set(data.selected_topics || []).size;
  }

  const total = allTopics.size;
  const disabled = Math.max(0, total - enabled);
  setText(`topic-count-total-${profile}`, total || "--");
  setText(`topic-count-enabled-${profile}`, total ? enabled : "--");
  setText(`topic-count-disabled-${profile}`, total ? disabled : "--");
}

function renderTopicPicker(profile) {
  const container = document.getElementById(`topic-groups-${profile}`);
  const hint = document.getElementById(`topic-picker-hint-${profile}`);
  const data = _bagTopicData[profile];
  if (!container || !data) return;

  const groups = data.topic_groups || {};
  const selected = data.selected_topics || [];
  const recording = data.recording;

  if (hint) {
    hint.textContent = recording ? "(locked while recording)" : "";
  }

  closeTopicChecklist(profile);

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
      cb.addEventListener("change", () => updateTopicCounts(profile));
      lbl.appendChild(cb);
      const span = document.createElement("span");
      span.textContent = topic.split("/").pop();
      span.title = topic;
      lbl.appendChild(span);
      group.appendChild(lbl);
    }
    container.appendChild(group);
  }
  updateTopicCounts(profile);
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
      if (_bagTopicData[profile]) _bagTopicData[profile].selected_topics = topics;
      updateTopicCounts(profile);
      showTopicMsg(profile, `Applied ${topics.length} topic(s). Takes effect on next recording.`, "ok");
      closeTopicChecklist(profile);
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

// ─── Recorded bags ───────────────────────────────────────────────

let bagsData = null;
let bagsLoading = false;

async function loadBags() {
  if (bagsLoading) return;
  bagsLoading = true;

  const loading = document.getElementById("bags-loading");
  const refresh = document.getElementById("bags-refresh");
  if (loading) {
    loading.style.display = "";
    loading.textContent = bagsData ? "Refreshing recordings..." : "Loading recordings...";
  }
  if (refresh) refresh.disabled = true;

  try {
    const resp = await fetch("/api/bags", { cache: "no-store" });
    if (!resp.ok) throw new Error("HTTP " + resp.status);
    bagsData = await resp.json();
    renderBags();

    const updated = document.getElementById("bags-last-updated");
    if (updated) {
      updated.textContent = "Last refreshed " + new Date().toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" });
    }
    if (loading) loading.style.display = "none";
  } catch (e) {
    console.error("Failed to load bags:", e);
    if (loading) {
      loading.style.display = "";
      loading.textContent = "Failed to load recordings: " + e.message;
    }
  } finally {
    bagsLoading = false;
    if (refresh) refresh.disabled = false;
  }
}

function formatBytes(bytes) {
  const numeric = Number(bytes || 0);
  if (numeric === 0) return "0 B";
  const units = ["B", "KB", "MB", "GB", "TB"];
  const i = Math.min(Math.floor(Math.log(numeric) / Math.log(1024)), units.length - 1);
  const val = numeric / Math.pow(1024, i);
  return val.toFixed(i === 0 ? 0 : 1) + " " + units[i];
}

function bagTimestamp(bag) {
  const raw = bag.modified || bag.created || bag.date || bag.timestamp || 0;
  const timestamp = new Date(raw).getTime();
  return Number.isFinite(timestamp) ? timestamp : 0;
}

function bagDateKey(bag) {
  const timestamp = bagTimestamp(bag);
  if (!timestamp) return "unknown";
  const d = new Date(timestamp);
  const y = d.getFullYear();
  const m = String(d.getMonth() + 1).padStart(2, "0");
  const day = String(d.getDate()).padStart(2, "0");
  return `${y}-${m}-${day}`;
}

function formatBagDateHeading(key) {
  if (key === "unknown") return "Unknown date";
  const d = new Date(key + "T12:00:00");
  return d.toLocaleDateString(undefined, {
    weekday: "short",
    year: "numeric",
    month: "short",
    day: "numeric",
  });
}

function formatBagTime(bag) {
  const timestamp = bagTimestamp(bag);
  if (!timestamp) return "--:--";
  return new Date(timestamp).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" });
}

function inferBagProfile(bag) {
  const explicit = String(bag.profile || bag.type || bag.category || "").toLowerCase();
  const name = String(bag.name || "").toLowerCase();
  const text = `${explicit} ${name}`;
  if (/perception|camera|color|depth|image/.test(text)) return "perception";
  if (/mapping|map_|map-|lidar|scan|odom|imu|slam|navigation/.test(text)) return "mapping";
  return "other";
}

function renderBags() {
  if (!bagsData) return;

  const allBags = Array.isArray(bagsData.bags) ? bagsData.bags.slice() : [];
  const dateFilter = document.getElementById("bags-date-filter")?.value || "";
  const direction = document.getElementById("bags-sort")?.value === "asc" ? 1 : -1;
  const filtered = allBags
    .filter(bag => !dateFilter || bagDateKey(bag) === dateFilter)
    .sort((a, b) => direction * (bagTimestamp(a) - bagTimestamp(b)));

  const summary = document.getElementById("bags-summary");
  if (summary) {
    const visibleText = dateFilter ? `${filtered.length} shown of ${allBags.length}` : `${allBags.length} recording${allBags.length === 1 ? "" : "s"}`;
    summary.textContent = `${visibleText} · ${formatBytes(bagsData.total_bytes)}`;
  }

  renderDiskUsage();

  const buckets = { perception: [], mapping: [], other: [] };
  for (const bag of filtered) buckets[inferBagProfile(bag)].push(bag);

  renderBagProfile("perception", buckets.perception);
  renderBagProfile("mapping", buckets.mapping);
  renderBagProfile("other", buckets.other);

  const otherCard = document.getElementById("bags-other-card");
  if (otherCard) otherCard.style.display = buckets.other.length ? "" : "none";

  const grid = document.getElementById("bags-profile-grid");
  const empty = document.getElementById("bags-empty");
  if (filtered.length) {
    if (grid) grid.style.display = "grid";
    if (empty) empty.style.display = "none";
  } else {
    if (grid) grid.style.display = "none";
    if (empty) empty.style.display = "";
  }
}

function renderDiskUsage() {
  const diskWrap = document.getElementById("disk-bar-wrap");
  if (!diskWrap || !bagsData?.disk_total) return;

  diskWrap.style.display = "";
  const total = Number(bagsData.disk_total || 0);
  const free = Number(bagsData.disk_free || 0);
  const used = Number(bagsData.disk_used || Math.max(0, total - free));
  const bagBytes = Number(bagsData.total_bytes || 0);
  const otherUsed = Math.max(0, used - bagBytes);

  const pctBags = total ? (bagBytes / total) * 100 : 0;
  const pctOther = total ? (otherUsed / total) * 100 : 0;
  const bagsBar = document.getElementById("disk-bar-bags");
  const otherBar = document.getElementById("disk-bar-other");
  if (bagsBar) bagsBar.style.width = pctBags + "%";
  if (otherBar) otherBar.style.width = pctOther + "%";
  setText("disk-label-used", `Bags: ${formatBytes(bagBytes)} · Other: ${formatBytes(otherUsed)} · Total: ${formatBytes(total)}`);
  setText("disk-label-free", `${formatBytes(free)} free`);
}

function renderBagProfile(profile, bags) {
  const container = document.getElementById(`bags-${profile}`);
  const count = document.getElementById(`bags-count-${profile}`);
  if (!container) return;

  if (count) count.textContent = `${bags.length} recording${bags.length === 1 ? "" : "s"}`;
  container.innerHTML = "";

  if (!bags.length) {
    const empty = document.createElement("div");
    empty.className = "bag-profile-empty";
    empty.textContent = "No recordings for this date.";
    container.appendChild(empty);
    return;
  }

  const groups = new Map();
  for (const bag of bags) {
    const key = bagDateKey(bag);
    if (!groups.has(key)) groups.set(key, []);
    groups.get(key).push(bag);
  }

  for (const [dateKey, dateBags] of groups) {
    const group = document.createElement("section");
    group.className = "bag-date-group";

    const heading = document.createElement("div");
    heading.className = "bag-date-heading";
    const date = document.createElement("span");
    date.textContent = formatBagDateHeading(dateKey);
    const amount = document.createElement("span");
    amount.textContent = `${dateBags.length} bag${dateBags.length === 1 ? "" : "s"}`;
    heading.append(date, amount);
    group.appendChild(heading);

    const table = document.createElement("table");
    table.className = "bag-date-table";
    const tbody = document.createElement("tbody");
    for (const bag of dateBags) tbody.appendChild(buildBagRow(bag));
    table.appendChild(tbody);
    group.appendChild(table);
    container.appendChild(group);
  }
}

function buildBagRow(bag) {
  const tr = document.createElement("tr");

  const nameCell = document.createElement("td");
  nameCell.className = "bag-name-cell";
  const name = document.createElement("span");
  name.className = "bag-name";
  name.textContent = bag.name || "Unnamed recording";
  name.title = bag.name || "";
  const time = document.createElement("span");
  time.className = "bag-time";
  time.textContent = formatBagTime(bag);
  nameCell.append(name, time);

  const metaCell = document.createElement("td");
  metaCell.className = "bag-meta-cell";
  const fileCount = Array.isArray(bag.files) ? bag.files.length : 0;
  metaCell.textContent = `${formatBytes(bag.size_bytes)} · ${fileCount} file${fileCount === 1 ? "" : "s"}`;

  const actionCell = document.createElement("td");
  actionCell.className = "bag-action-cell";
  const actions = document.createElement("div");
  actions.className = "bag-actions";

  const download = document.createElement("a");
  download.href = `/api/bags/${encodeURIComponent(bag.name)}/download`;
  download.className = "bag-dl";
  download.innerHTML = '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M12 3v12"></path><path d="m7 10 5 5 5-5"></path><path d="M5 20h14"></path></svg>';
  download.setAttribute("download", "");
  download.setAttribute("aria-label", `Download ${bag.name || "recording"}`);
  download.title = "Download";

  const remove = document.createElement("button");
  remove.className = "danger bag-del";
  remove.type = "button";
  remove.innerHTML = '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M4 7h16"></path><path d="M9 7V4h6v3"></path><path d="m7 7 1 13h8l1-13"></path><path d="M10 11v5M14 11v5"></path></svg>';
  remove.setAttribute("aria-label", `Delete ${bag.name || "recording"}`);
  remove.title = "Delete";
  remove.onclick = () => deleteBag(bag.name);

  actions.append(download, remove);
  actionCell.appendChild(actions);
  tr.append(nameCell, metaCell, actionCell);
  return tr;
}

function clearBagDateFilter() {
  const input = document.getElementById("bags-date-filter");
  if (input) input.value = "";
  renderBags();
}

async function deleteBag(name) {
  if (!confirm(`Delete recording "${name}"?
This cannot be undone.`)) return;
  try {
    const resp = await fetch(`/api/bags/${encodeURIComponent(name)}`, { method: "DELETE" });
    const result = await resp.json();
    if (result.success || resp.ok) {
      await loadBags();
    } else {
      alert("Delete failed: " + (result.error || "unknown error"));
    }
  } catch (e) {
    alert("Network error: " + e.message);
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

async function startLidar() {
  try {
    const resp = await fetch("/api/lidar/start", { method: "POST" });
    const r = await resp.json();
    if (!r.success) {
      const el = document.getElementById("lidar-status");
      if (el) { el.textContent = r.message || "Failed"; el.className = "lidar-status err"; }
      return;
    }
  } catch (e) { console.error("Lidar start failed:", e); return; }

  lidarScanning = true;
  document.getElementById("lidar-toggle").innerHTML = '<span class="button-play">■</span> Stop Preview';
  document.getElementById("lidar-toggle").className = "danger compact preview-toggle";
  const el = document.getElementById("lidar-status");
  if (el) { el.textContent = "Scanning"; el.className = "lidar-status ok"; }
  if (!lidarAnimFrame) lidarAnimFrame = requestAnimationFrame(drawLidar);
  syncDashboardSensorControls();
}

async function stopLidar() {
  try { await fetch("/api/lidar/stop", { method: "POST" }); } catch (e) { /* ignore */ }

  lidarScanning = false;
  lidarPoints = [];
  document.getElementById("lidar-toggle").innerHTML = '<span class="button-play">▶</span> Start Preview';
  document.getElementById("lidar-toggle").className = "compact preview-toggle";
  const el = document.getElementById("lidar-status");
  if (el) { el.textContent = "Stopped"; el.className = "lidar-status"; }
  if (lidarAnimFrame) { cancelAnimationFrame(lidarAnimFrame); lidarAnimFrame = null; }
  drawLidar();
  syncDashboardSensorControls();
}

function toggleLidar() {
  if (lidarScanning) stopLidar();
  else startLidar();
}

function drawLidar() {
  const canvas = document.getElementById("lidar-canvas");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  canvas.parentElement?.classList.toggle("is-stopped", !lidarScanning);
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

  // Points are drawn only while the browser preview is explicitly active.
  if (lidarScanning && lidarPoints.length > 0) {
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

  if (!lidarScanning) {
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
    icon: '<circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3"/>'
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
      document.body.classList.toggle("dashboard-active", tabName === "dashboard");
      closeTopicChecklists();

      if (tabName === "dashboard") {
        resetDashboardComponentsClosed();
        resetDashboardHardwareClosed();
        loadDashboardBagStats();
        updateDashboardClock();
      }

      if (tabName === "recordings") {
        loadBagTopics();
        drawLidar();
      }

      // Recorded bags refresh only when the page is opened or Refresh is pressed.
      // There is intentionally no timer, so the list never jumps every few seconds.
      if (tabName === "recorded-bags" && !bagsData) loadBags();

      if (tabName === "network") loadNetwork();
      // Camera and LiDAR stay active while switching between tabs.
      // They stop only when the user explicitly toggles them off.
      if (window.matchMedia("(max-width: 900px)").matches) closeSidebar();
    });
  });
}

function closeTopicChecklist(profile) {
  const checklist = document.querySelector(`.topic-checklist[data-topic-profile="${profile}"]`);
  if (!checklist) return;
  const trigger = checklist.querySelector(".topic-checklist-trigger");
  const panel = checklist.querySelector(".topic-checklist-body");
  checklist.classList.remove("is-open");
  if (trigger) trigger.setAttribute("aria-expanded", "false");
  if (panel) panel.hidden = true;
}

function closeTopicChecklists(exceptProfile = null) {
  document.querySelectorAll(".topic-checklist[data-topic-profile]").forEach(checklist => {
    const profile = checklist.dataset.topicProfile;
    if (profile !== exceptProfile) closeTopicChecklist(profile);
  });
}

function toggleTopicChecklist(profile) {
  const checklist = document.querySelector(`.topic-checklist[data-topic-profile="${profile}"]`);
  if (!checklist) return;
  const trigger = checklist.querySelector(".topic-checklist-trigger");
  const panel = checklist.querySelector(".topic-checklist-body");
  if (!trigger || !panel) return;

  const shouldOpen = panel.hidden;
  closeTopicChecklists(profile);
  checklist.classList.toggle("is-open", shouldOpen);
  trigger.setAttribute("aria-expanded", String(shouldOpen));
  panel.hidden = !shouldOpen;
}

function initTopicChecklists() {
  document.querySelectorAll(".topic-checklist[data-topic-profile]").forEach(checklist => {
    const profile = checklist.dataset.topicProfile;
    closeTopicChecklist(profile);
    const trigger = checklist.querySelector(".topic-checklist-trigger");
    trigger?.addEventListener("click", event => {
      event.preventDefault();
      event.stopPropagation();
      toggleTopicChecklist(profile);
    });
  });

  document.addEventListener("click", event => {
    if (!event.target.closest(".topic-checklist")) closeTopicChecklists();
  });
  document.addEventListener("keydown", event => {
    if (event.key === "Escape") closeTopicChecklists();
  });
}

function syncSidebarAria() {
  const toggle = document.getElementById("sidebar-toggle");
  if (!toggle) return;
  const mobile = window.matchMedia("(max-width: 900px)").matches;
  const expanded = mobile
    ? document.body.classList.contains("sidebar-open")
    : !document.body.classList.contains("sidebar-collapsed");
  toggle.setAttribute("aria-expanded", String(expanded));
}

function closeSidebar() {
  document.body.classList.remove("sidebar-open");
  syncSidebarAria();
}

function toggleSidebar() {
  const mobile = window.matchMedia("(max-width: 900px)").matches;
  if (mobile) {
    document.body.classList.toggle("sidebar-open");
  } else {
    document.body.classList.toggle("sidebar-collapsed");
    try {
      localStorage.setItem("hsb-sidebar-collapsed", document.body.classList.contains("sidebar-collapsed") ? "1" : "0");
    } catch (_) { /* storage is optional */ }
  }
  syncSidebarAria();
}

function initSidebar() {
  const toggle = document.getElementById("sidebar-toggle");
  const close = document.getElementById("sidebar-close");
  const backdrop = document.getElementById("sidebar-backdrop");

  if (!window.matchMedia("(max-width: 900px)").matches) {
    try {
      document.body.classList.toggle("sidebar-collapsed", localStorage.getItem("hsb-sidebar-collapsed") === "1");
    } catch (_) { /* storage is optional */ }
  }

  toggle?.addEventListener("click", toggleSidebar);
  close?.addEventListener("click", closeSidebar);
  backdrop?.addEventListener("click", closeSidebar);

  window.addEventListener("resize", () => {
    if (!window.matchMedia("(max-width: 900px)").matches) {
      document.body.classList.remove("sidebar-open");
    }
    syncSidebarAria();
  });
  syncSidebarAria();
}

// ─── Gamepad Bluetooth Reconnect ─────────────────────────────────

function updateControllerGamepadCard(info = {}) {
  const connected = Boolean(info.connected);
  const name = document.getElementById("controller-gamepad-name");
  const battery = document.getElementById("controller-gamepad-battery");
  const mac = document.getElementById("controller-gamepad-mac");
  const button = document.getElementById("gp-reconnect-btn");
  const icon = document.getElementById("controller-status-icon");

  if (name) {
    if (connected) {
      name.textContent = info.name || "Gamepad connected";
    } else if (info.mac) {
      name.textContent = "Gamepad disconnected";
    } else {
      name.textContent = "No paired gamepad found";
    }
  }

  if (battery) {
    battery.textContent =
      info.battery == null ? "Not reported" : `${info.battery}%`;
  }
  if (mac) mac.textContent = info.mac || "Not reported";

  if (icon) {
    icon.textContent = connected ? "✓" : "!";
    icon.classList.toggle("connected", connected);
    icon.classList.toggle("disconnected", !connected);
  }

  if (button) {
    button.dataset.connected = connected ? "1" : "0";
    button.textContent = connected ? "Disconnect gamepad" : "Connect gamepad";
    button.classList.toggle("connect-attention", !connected);
    button.classList.toggle("disconnect-action", connected);
    button.title = connected
      ? "Disconnect the currently connected Bluetooth gamepad"
      : "Connect the most recently paired Bluetooth gamepad";
  }
}

async function controllerGamepadButtonAction() {
  const connected = document.getElementById("gp-reconnect-btn")?.dataset.connected === "1";
  if (connected) {
    await disconnectGamepad();
  } else {
    await reconnectGamepad();
  }
}

async function disconnectGamepad() {
  const btn = document.getElementById("gp-reconnect-btn");
  const msg = document.getElementById("gp-reconnect-msg");
  const mac = _cachedBtInfo?.mac || "";

  if (btn) btn.disabled = true;
  if (msg) { msg.textContent = "Disconnecting…"; msg.className = "ctrl-msg"; }

  try {
    const resp = await fetch("/api/bluetooth/disconnect", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ mac }),
    });
    const result = await resp.json();
    if (!resp.ok || !result.success) {
      throw new Error(result.message || `HTTP ${resp.status}`);
    }
    if (msg) { msg.textContent = "Disconnected"; msg.className = "ctrl-msg ok"; }
    await pollBtGamepad();
  } catch (e) {
    if (msg) { msg.textContent = e.message || "Disconnect failed"; msg.className = "ctrl-msg err"; }
  } finally {
    if (btn) btn.disabled = false;
  }
}

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
      await pollBtGamepad();
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


// ─── Dashboard overview ───────────────────────────────────────────

const dashboardRuntime = {
  startedAt: Date.now(),
  lastMessageAt: Date.now(),
  telemetryLastSeenAt: 0,
  telemetryConnected: false,
  disconnectTimer: null,
  runtimeSeconds: 0,
  distanceMeters: 0,
  systemUptimeSeconds: null,
  cpuHistory: [],
  gpuHistory: [],
  systemMetrics: {},
  systemHealthLastSuccess: 0,
  systemHealthError: "",
  lastData: null,
  bagStatsLoadedAt: 0,
};

function openDashboardTab(tabName) {
  const tab = document.querySelector(`.tabs .tab[data-tab="${tabName}"]`);
  if (tab) tab.click();
}

function setDashboardSensorBusy(sensor, busy) {
  const row = document.getElementById(`dash-${sensor}-row`);
  if (!row) return;
  row.classList.toggle("is-busy", busy);
  row.disabled = busy;
}

async function toggleDashboardSensor(sensor) {
  if (sensor === "camera") {
    setDashboardSensorBusy("camera", true);
    try {
      toggleCamera();
    } finally {
      setDashboardSensorBusy("camera", false);
      syncDashboardSensorControls();
    }
    return;
  }

  if (sensor === "lidar") {
    setDashboardSensorBusy("lidar", true);
    try {
      if (lidarScanning) await stopLidar();
      else await startLidar();
    } finally {
      setDashboardSensorBusy("lidar", false);
      syncDashboardSensorControls();
    }
  }
}

function syncDashboardSensorControls() {
  const states = {
    camera: Boolean(camStreaming),
    lidar: Boolean(lidarScanning),
  };

  for (const [sensor, active] of Object.entries(states)) {
    const row = document.getElementById(`dash-${sensor}-row`);
    const toggle = document.getElementById(`dash-${sensor}-switch`);

    row?.classList.toggle("is-on", active);
    toggle?.classList.toggle("is-on", active);
    row?.setAttribute("aria-pressed", String(active));
  }

  dashboardText("dash-camera-status", camStreaming ? "ON" : "OFF");
  dashboardClass("dash-camera-status", camStreaming ? "ok" : "");
  dashboardText(
    "dash-camera-detail",
    camStreaming ? "Web preview running" : "Web preview off"
  );

  dashboardText("dash-lidar-status", lidarScanning ? "ON" : "OFF");
  dashboardClass("dash-lidar-status", lidarScanning ? "ok" : "");
  dashboardText(
    "dash-lidar-detail",
    lidarScanning ? "Web scan preview running" : "Web preview off"
  );
}

function toggleDashboardImuDetails() {
  const details = document.getElementById("dashboard-imu-details");
  const row = document.getElementById("dash-imu-row");
  if (!details || !row) return;
  const open = details.hidden;
  details.hidden = !open;
  row.setAttribute("aria-expanded", String(open));
  row.classList.toggle("is-open", open);
}

function scrollDashboardSection(id) {
  document.getElementById(id)?.scrollIntoView({ behavior: "smooth", block: "start" });
}

function toggleDashboardSection(bodyId, button) {
  const body = document.getElementById(bodyId);
  if (!body) return;

  const open = body.hidden;
  body.hidden = !open;
  button?.setAttribute("aria-expanded", String(open));

  if (bodyId === "dashboard-components-body") {
    document.querySelectorAll(
      "#dashboard-components-body .dashboard-component-detail"
    ).forEach(detail => {
      detail.hidden = !open;
    });
  }

  if (bodyId === "dashboard-hardware-body") {
    document.querySelectorAll(
      "#dashboard-hardware-body .dashboard-diagnostic-panel"
    ).forEach(panel => {
      panel.hidden = !open;
    });
  }

  try {
    sessionStorage.setItem(
      `hsb-dashboard-section:${bodyId}`,
      open ? "open" : "closed"
    );
  } catch (_) { /* optional */ }
}

function resetDashboardComponentsClosed() {
  const body = document.getElementById("dashboard-components-body");
  const button = document.querySelector(
    "#dashboard-components-section .dashboard-accordion-heading"
  );

  if (body) body.hidden = true;
  button?.setAttribute("aria-expanded", "false");

  document.querySelectorAll(
    "#dashboard-components-body .dashboard-component-detail"
  ).forEach(detail => {
    detail.hidden = true;
  });

  try {
    sessionStorage.removeItem(
      "hsb-dashboard-section:dashboard-components-body"
    );
  } catch (_) { /* optional */ }
}

function resetDashboardHardwareClosed() {
  const body = document.getElementById("dashboard-hardware-body");
  const button = document.querySelector(
    "#dashboard-hardware-section .dashboard-accordion-heading"
  );

  if (body) body.hidden = true;
  button?.setAttribute("aria-expanded", "false");

  document.querySelectorAll(
    "#dashboard-hardware-body .dashboard-diagnostic-panel"
  ).forEach(panel => {
    panel.hidden = true;
  });

  try {
    sessionStorage.removeItem(
      "hsb-dashboard-section:dashboard-hardware-body"
    );
    sessionStorage.removeItem("hsb-dashboard-diagnostic");
  } catch (_) { /* optional */ }
}

function restoreDashboardSections() {
  // Both grouped sections intentionally start collapsed on every page load.
  resetDashboardComponentsClosed();
  resetDashboardHardwareClosed();
}

function closeDashboardDetails() {
  const body = document.getElementById("dashboard-components-body");
  const heading = document.querySelector(
    "#dashboard-components-section .dashboard-accordion-heading"
  );

  if (body) body.hidden = true;
  heading?.setAttribute("aria-expanded", "false");

  document.querySelectorAll("#dashboard-components-body .dashboard-component-detail")
    .forEach(detail => {
      detail.hidden = true;
    });

  try {
    sessionStorage.setItem(
      "hsb-dashboard-section:dashboard-components-body",
      "closed"
    );
  } catch (_) { /* optional */ }
}

function toggleDashboardDetail(name) {
  const body = document.getElementById("dashboard-components-body");
  const heading = document.querySelector(
    "#dashboard-components-section .dashboard-accordion-heading"
  );
  const slot = document.querySelector(
    `.dashboard-component-slot[data-component="${name}"]`
  );

  if (body) body.hidden = false;
  heading?.setAttribute("aria-expanded", "true");

  document.querySelectorAll("#dashboard-components-body .dashboard-component-detail")
    .forEach(detail => {
      detail.hidden = false;
    });

  try {
    sessionStorage.setItem(
      "hsb-dashboard-section:dashboard-components-body",
      "open"
    );
  } catch (_) { /* optional */ }

  slot?.scrollIntoView({ behavior: "smooth", block: "nearest" });
}

function selectDashboardDiagnostic(name) {
  const body = document.getElementById("dashboard-hardware-body");
  const accordionButton = document.querySelector(
    "#dashboard-hardware-section .dashboard-accordion-heading"
  );
  const target = document.getElementById(`dashboard-diagnostic-${name}`);

  if (body) body.hidden = false;
  accordionButton?.setAttribute("aria-expanded", "true");

  document.querySelectorAll(
    "#dashboard-hardware-body .dashboard-diagnostic-panel"
  ).forEach(panel => {
    panel.hidden = false;
  });

  try {
    sessionStorage.setItem(
      "hsb-dashboard-section:dashboard-hardware-body",
      "open"
    );
  } catch (_) { /* optional */ }

  target?.scrollIntoView({ behavior: "smooth", block: "nearest" });
}

function closeDashboardDiagnostic() {
  resetDashboardHardwareClosed();
}

function restoreDashboardDiagnostic() {
  // Hardware Diagnostics intentionally starts collapsed.
  resetDashboardHardwareClosed();
}

function dashboardNumber(...values) {
  for (const value of values) {
    if (value == null || value === "") continue;
    const number = Number(value);
    if (Number.isFinite(number)) return number;
  }
  return null;
}

function dashboardText(id, text) {
  const el = document.getElementById(id);
  if (el) el.textContent = text;
}

function dashboardClass(id, state) {
  const el = document.getElementById(id);
  if (!el) return;
  el.classList.remove("ok", "warn", "err");
  if (state) el.classList.add(state);
}

function dashboardService(id, state, title = null) {
  dashboardClass(id, state);
  if (title) document.getElementById(id)?.setAttribute("title", title);
}

function dashboardServiceNote(id, text) {
  dashboardText(`${id}-note`, text);
}

function dashboardSensor(statusId, detailId, state, status, detail) {
  dashboardText(statusId, status);
  dashboardText(detailId, detail);
  dashboardClass(statusId, state);
}

function dashboardMarker(name, state, text) {
  dashboardClass(`marker-${name}-dot`, state);
  const marker = document.getElementById(`marker-${name}-dot`)?.closest(".robot-marker");
  if (marker) {
    marker.classList.remove("ok", "warn", "err");
    if (state) marker.classList.add(state);
  }
  dashboardText(`marker-${name}-state`, text);
}

function dashboardBadge(id, state, text) {
  dashboardText(id, text);
  dashboardClass(id, state);
}

function dashboardProgress(id, value, max = 100) {
  const bar = document.getElementById(id);
  if (!bar) return;
  const number = Number(value);
  bar.style.width = Number.isFinite(number)
    ? `${Math.max(0, Math.min(100, number / max * 100))}%`
    : "0%";
}

function dashboardSparkline(id, history, value) {
  if (Number.isFinite(value)) {
    history.push(Math.max(0, Math.min(100, value)));
    while (history.length > 24) history.shift();
  }

  const line = document.getElementById(id);
  if (!line) return;

  if (!history.length) {
    line.setAttribute("points", "");
    return;
  }

  const width = 120;
  const height = 28;
  const step = width / Math.max(1, history.length - 1);
  const points = history.map((v, index) => {
    const x = index * step;
    const y = height - 3 - (v / 100) * (height - 6);
    return `${x.toFixed(1)},${y.toFixed(1)}`;
  });
  line.setAttribute("points", points.join(" "));
}

function actuatorDashboardState(actuator, label) {
  const index = Number(actuator?.state || 0);
  const name = STATE_NAMES[index] || "UNKNOWN";
  if (index === 5 || actuator?.error) return { css: "err", badge: "ERROR", detail: name };
  if (index === 1 || index === 3 || index === 4) return { css: "warn", badge: name, detail: name };
  if (actuator?.is_referenced || index === 2) return { css: "ok", badge: "REFERENCED", detail: name };
  if (index === 0) return { css: "", badge: "IDLE", detail: name };
  return { css: "", badge: label.toUpperCase(), detail: name };
}

function updateDashboardConnection(connected) {
  if (dashboardRuntime.disconnectTimer) {
    clearTimeout(dashboardRuntime.disconnectTimer);
    dashboardRuntime.disconnectTimer = null;
  }

  dashboardRuntime.telemetryConnected = connected;

  if (connected) {
    dashboardService(
      "dash-service-network",
      "ok",
      "Dashboard WebSocket connected"
    );
    dashboardService("dash-service-ros", "ok");
    dashboardText("dash-service-network-note", "Dashboard connected");
    dashboardText("dash-service-ros-note", "Telemetry connected");

    const ready = document.getElementById("dashboard-readiness");
    if (!dashboardRuntime.lastData) {
      ready?.classList.remove("ready", "error");
      ready?.classList.add("warning");
      dashboardText("dashboard-ready-title", "WAITING FOR ROBOT TELEMETRY");
      dashboardText(
        "dashboard-ready-subtitle",
        "The dashboard connection is open; waiting for the first robot status packet."
      );
    } else {
      updateDashboardOverview(dashboardRuntime.lastData);
    }
    return;
  }

  dashboardService(
    "dash-service-network",
    "warn",
    "Dashboard WebSocket reconnecting"
  );
  dashboardService("dash-service-ros", "warn");
  dashboardText("dash-service-network-note", "Reconnecting");
  dashboardText("dash-service-ros-note", "Telemetry interrupted");

  // A brief WebSocket restart should not label the whole robot as offline.
  // Wait five seconds before showing a persistent connection warning.
  dashboardRuntime.disconnectTimer = setTimeout(() => {
    if (ws?.readyState === WebSocket.OPEN) return;

    const hostMetricsAreLive =
      Date.now() - dashboardRuntime.systemHealthLastSuccess < 7000;
    const ready = document.getElementById("dashboard-readiness");

    ready?.classList.remove("ready", "error");
    ready?.classList.add("warning");

    dashboardText(
      "dashboard-ready-title",
      hostMetricsAreLive
        ? "HOST ONLINE · LIVE TELEMETRY DISCONNECTED"
        : "DASHBOARD LIVE DATA DISCONNECTED"
    );
    dashboardText(
      "dashboard-ready-subtitle",
      hostMetricsAreLive
        ? "The Jetson and REST metrics endpoint are reachable, but /ws robot telemetry is reconnecting."
        : "The browser cannot currently receive the /ws robot-status stream. This does not by itself prove that the robot is powered off."
    );
  }, 5000);
}


let _systemHealthPolling = false;

async function pollSystemHealth() {
  if (_systemHealthPolling) return;
  _systemHealthPolling = true;

  const state = document.getElementById("system-health-state");
  const started = performance.now();

  try {
    const response = await fetch("/api/system-health", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);

    const metrics = await response.json();
    metrics.network_latency_ms = Math.max(0, performance.now() - started);

    dashboardRuntime.systemMetrics = metrics;
    dashboardRuntime.systemHealthLastSuccess = Date.now();
    dashboardRuntime.systemHealthError = "";

    if (state) {
      state.textContent = "Live";
      state.className = "system-health-state ok";
    }

    if (dashboardRuntime.lastData) {
      updateDashboardOverview(dashboardRuntime.lastData);
    }
  } catch (error) {
    dashboardRuntime.systemHealthError = error.message || String(error);
    if (state) {
      state.textContent = "Host metrics unavailable";
      state.className = "system-health-state err";
      state.title = dashboardRuntime.systemHealthError;
    }
  } finally {
    _systemHealthPolling = false;
  }
}

function dashboardMetricObject(data) {
  const fromSocket =
    data.system_metrics ||
    data.system ||
    data.metrics ||
    data.host ||
    {};
  return {
    ...fromSocket,
    ...(dashboardRuntime.systemMetrics || {}),
  };
}

function updateDashboardOverview(data) {
  dashboardRuntime.lastData = data;
  const now = Date.now();
  const dt = Math.min(2, Math.max(0, (now - dashboardRuntime.lastMessageAt) / 1000));
  dashboardRuntime.lastMessageAt = now;

  const wheel = data.wheel_status || {};
  const left = rpmToMs(wheel.left_rpm || 0);
  const right = rpmToMs(wheel.right_rpm || 0);
  const speed = Math.abs((left + right) / 2);
  if (speed > 0.03) {
    dashboardRuntime.runtimeSeconds += dt;
    dashboardRuntime.distanceMeters += speed * dt;
  }

  const metrics = dashboardMetricObject(data);
  const hw = data.mks_bus || {};
  const lidar = data.lidar || {};
  const camera = data.camera || data.camera_status || {};
  const imu = data.imu || data.imu_data || data.imu_status || {};
  const gamepad = data.gamepad || {};
  const thermals = Array.isArray(data.thermals) ? data.thermals : [];

  const cpu = dashboardNumber(metrics.cpu_percent, metrics.cpu_usage, metrics.cpu);
  const gpu = dashboardNumber(metrics.gpu_percent, metrics.gpu_usage, metrics.gpu);
  const memUsed = dashboardNumber(metrics.memory_used_gb, metrics.mem_used_gb, metrics.memory_used);
  const memTotal = dashboardNumber(metrics.memory_total_gb, metrics.mem_total_gb, metrics.memory_total);
  const storageUsed = dashboardNumber(metrics.storage_used_gb, metrics.disk_used_gb);
  const storageTotal = dashboardNumber(metrics.storage_total_gb, metrics.disk_total_gb);
  const battery = dashboardNumber(metrics.battery_percent, metrics.battery, data.battery?.percent);
  const driveBusVoltage = dashboardNumber(wheel.diag?.vbus);
  const latency = dashboardNumber(metrics.network_latency_ms, metrics.latency_ms);
  const uptime = dashboardNumber(metrics.uptime_sec, metrics.uptime_seconds, data.uptime_sec);
  if (uptime != null) dashboardRuntime.systemUptimeSeconds = uptime;

  const representativeThermal =
    thermals.find(zone => /cpu|gpu|soc|thermal/i.test(zone.type || "")) ||
    thermals[0];
  const temperature = representativeThermal
    ? dashboardNumber(representativeThermal.temp_c)
    : dashboardNumber(metrics.temperature_c, metrics.temperature);

  dashboardText("dash-cpu-value", cpu == null ? "N/A" : `${cpu.toFixed(0)}%`);
  dashboardText(
    "dash-gpu-value",
    gpu == null
      ? (metrics.gpu_available === false ? "N/A" : "Loading")
      : `${gpu.toFixed(0)}%`
  );
  dashboardSparkline("dash-cpu-line", dashboardRuntime.cpuHistory, cpu);
  dashboardSparkline("dash-gpu-line", dashboardRuntime.gpuHistory, gpu);

  if (memUsed != null && memTotal != null && memTotal > 0) {
    dashboardText("dash-memory-value", `${memUsed.toFixed(1)} / ${memTotal.toFixed(1)} GB`);
    dashboardProgress("dash-memory-bar", memUsed, memTotal);
  } else {
    dashboardText("dash-memory-value", "N/A");
    dashboardProgress("dash-memory-bar", null);
  }

  if (storageUsed != null && storageTotal != null && storageTotal > 0) {
    dashboardText("dash-storage-value", `${storageUsed.toFixed(1)} / ${storageTotal.toFixed(1)} GB`);
    dashboardProgress("dash-storage-bar", storageUsed, storageTotal);
  } else {
    dashboardText("dash-storage-value", "N/A");
    dashboardProgress("dash-storage-bar", null);
  }

  if (battery != null) {
    dashboardText("dash-battery-value", `${battery.toFixed(0)}%`);
    dashboardProgress("dash-battery-bar", battery);
  } else if (driveBusVoltage != null) {
    dashboardText("dash-battery-value", `${driveBusVoltage.toFixed(1)} V bus`);
    dashboardProgress("dash-battery-bar", driveBusVoltage, 60);
  } else {
    dashboardText("dash-battery-value", "Not reported");
    dashboardProgress("dash-battery-bar", null);
  }

  dashboardText(
    "dash-temperature-value",
    temperature == null ? "N/A" : `${temperature.toFixed(1)}°C`
  );
  dashboardProgress("dash-temperature-bar", temperature, 100);
  const tempValue = document.getElementById("dash-temperature-value");
  if (tempValue) {
    tempValue.style.color =
      temperature != null && temperature >= 65
        ? "var(--dashboard-orange)"
        : "";
  }

  dashboardText(
    "dash-latency-value",
    latency == null ? "N/A" : `${latency.toFixed(0)} ms`
  );
  dashboardProgress(
    "dash-latency-bar",
    latency == null ? null : Math.max(0, 100 - latency),
    100
  );

  const healthState = document.getElementById("system-health-state");
  if (healthState && dashboardRuntime.systemHealthLastSuccess) {
    const ageSeconds = Math.max(
      0,
      Math.round((Date.now() - dashboardRuntime.systemHealthLastSuccess) / 1000)
    );
    healthState.textContent = ageSeconds <= 2 ? "Live" : `Updated ${ageSeconds}s ago`;
    healthState.className = "system-health-state ok";
  }

  const wheelBackend = String(wheel.backend || "").toUpperCase();
  const wheelError = Boolean(wheel.error || wheel.estopped);
  const wheelReady = Boolean(wheel.backend && !wheelError);
  dashboardBadge("dash-wheels-state", wheelError ? "err" : wheelReady ? "ok" : "", wheelError ? "ERROR" : wheelReady ? "READY" : "CHECKING");
  dashboardText("dash-wheels-sub", `Drive: ${wheelBackend || "--"}`);
  dashboardMarker("wheels", wheelError ? "err" : wheelReady ? "ok" : "", wheelError ? "Error" : wheelReady ? "Ready" : "Checking");

  const brushState = actuatorDashboardState(data.brush || {}, "Brush");
  const liftState = actuatorDashboardState(data.lift || {}, "Lift");
  const binState = actuatorDashboardState(data.bin_door || {}, "Bin Door");

  dashboardBadge("dash-brush-state", brushState.css, brushState.badge);
  dashboardBadge("dash-lift-state", liftState.css, liftState.badge);
  dashboardBadge("dash-bin-state", binState.css, binState.badge);
  dashboardText("dash-brush-sub", `State: ${brushState.detail}`);
  dashboardText("dash-lift-sub", `State: ${liftState.detail}`);
  dashboardText("dash-bin-sub", `State: ${binState.detail}`);
  dashboardMarker("brush", brushState.css, brushState.badge === "REFERENCED" ? "Ready" : brushState.badge);
  dashboardMarker("lift", liftState.css, liftState.badge);
  dashboardMarker("bin", binState.css, binState.badge === "REFERENCED" ? "Ready" : binState.badge);

  // Sensor availability and browser preview are intentionally separate.
  // "Online" describes the robot-side sensor. ON/OFF in the Sensors card
  // describes only this browser's preview.
  const cameraConnected =
    camera.connected === true ||
    camera.streaming === true ||
    camera.active === true ||
    camStreaming;
  const cameraKnown =
    camera.connected != null ||
    camera.streaming != null ||
    camera.active != null ||
    camStreaming;
  const cameraState = cameraKnown ? (cameraConnected ? "ok" : "err") : "";
  dashboardService("dash-service-camera", cameraState);
  dashboardServiceNote(
    "dash-service-camera",
    cameraKnown ? (cameraConnected ? "Online" : "Unavailable") : "Not reported"
  );
  dashboardText(
    "dash-camera-health",
    cameraKnown
      ? (cameraConnected ? "Hardware online" : "Hardware unavailable")
      : "Hardware status not reported"
  );
  dashboardMarker(
    "camera",
    cameraState,
    camStreaming
      ? "Preview on"
      : cameraConnected ? "Online · preview off"
      : cameraKnown ? "Unavailable" : "Status unknown"
  );

  const lidarKnown = lidar.connected != null;
  const lidarConnected = lidar.connected === true;
  const lidarState = lidarKnown ? (lidarConnected ? "ok" : "err") : "";
  const lidarRate = dashboardNumber(lidar.hz, lidar.frequency, lidar.scan_rate);
  dashboardService("dash-service-lidar", lidarState);
  dashboardServiceNote(
    "dash-service-lidar",
    lidarKnown ? (lidarConnected ? "Online" : "Offline") : "Not reported"
  );
  dashboardText(
    "dash-lidar-health",
    lidarKnown
      ? (lidarConnected
          ? `Hardware online${lidarRate != null ? ` · ${lidarRate.toFixed(0)} Hz` : ""}`
          : "Hardware offline")
      : "Hardware status not reported"
  );
  dashboardMarker(
    "lidar",
    lidarState,
    lidarScanning
      ? "Preview on"
      : lidarConnected ? "Online · preview off"
      : lidarKnown ? "Offline" : "Status unknown"
  );

  syncDashboardSensorControls();

  const angular = imu.angular_velocity || imu.gyro || {};
  const accel = imu.linear_acceleration || imu.acceleration || imu.accel || {};
  const imuAvailable = [
    angular.x, angular.y, angular.z,
    accel.x, accel.y, accel.z,
    imu.angular_velocity_z, imu.accel_x, imu.accel_y
  ].some(value => Number.isFinite(Number(value)));
  dashboardService("dash-service-imu", imuAvailable ? "ok" : "");
  dashboardServiceNote(
    "dash-service-imu",
    imuAvailable ? "Publishing" : "No data"
  );
  dashboardSensor(
    "dash-imu-status",
    "dash-imu-detail",
    imuAvailable ? "ok" : "",
    imuAvailable ? "LIVE" : "--",
    imuAvailable
      ? `${dashboardNumber(imu.hz, imu.frequency) || 200} Hz · ROS managed`
      : "Waiting for /imu/data_raw"
  );

  const dashOrientation = imu.orientation || imu.quaternion || imu.q || null;
  const dashAngles = _quaternionToRollPitch(dashOrientation);
  const dashYawRate = _firstFinite(angular.z, imu.angular_velocity_z, imu.gyro_z, imu.yaw_rate);
  const dashAccelX = _firstFinite(accel.x, imu.accel_x, imu.linear_acceleration_x);
  const dashAccelY = _firstFinite(accel.y, imu.accel_y, imu.linear_acceleration_y);
  const dashRoll = _firstFinite(imu.roll, imu.roll_deg, dashAngles.roll);
  const dashPitch = _firstFinite(imu.pitch, imu.pitch_deg, dashAngles.pitch);
  dashboardText("dash-imu-yaw", dashYawRate == null ? "--" : dashYawRate.toFixed(2));
  dashboardText("dash-imu-accel-x", dashAccelX == null ? "--" : dashAccelX.toFixed(2));
  dashboardText("dash-imu-accel-y", dashAccelY == null ? "--" : dashAccelY.toFixed(2));
  dashboardText("dash-imu-roll", dashRoll == null ? "--" : dashRoll.toFixed(1));
  dashboardText("dash-imu-pitch", dashPitch == null ? "--" : dashPitch.toFixed(1));

  const mksKnown = hw.bus_connected != null;
  const mksConnected = hw.bus_connected === true;
  dashboardService("dash-service-mks", mksKnown ? (mksConnected ? "ok" : "err") : "");
  dashboardServiceNote(
    "dash-service-mks",
    mksKnown ? (mksConnected ? "Online" : "Offline") : "Not reported"
  );
  dashboardText(
    "sidebar-mks-state",
    mksKnown ? (mksConnected ? "Connected" : "Disconnected") : "Checking"
  );

  const motors = hw.motors && typeof hw.motors === "object" ? hw.motors : {};
  const motorStates = Object.values(motors);
  const motorTotal = motorStates.length;
  const motorsOnline = motorStates.filter(Boolean).length;
  const motorsOffline = Math.max(0, motorTotal - motorsOnline);
  dashboardText(
    "dash-motor-diagnostics-summary",
    motorTotal
      ? `${motorsOnline}/${motorTotal} motors online`
      : "Run a motor-bus scan"
  );
  dashboardText("diag-motors-online", motorTotal ? String(motorsOnline) : "--");
  dashboardText("diag-motors-offline", motorTotal ? String(motorsOffline) : "--");

  const wheelDiag = wheel.diag || {};
  const wheelBackendName = String(wheel.backend || "none");
  const vbus = dashboardNumber(wheelDiag.vbus);
  dashboardText(
    "dash-drive-diagnostics-summary",
    wheel.error
      ? `${wheelBackendName.toUpperCase()} · error`
      : `${wheelBackendName.toUpperCase()} · no reported error`
  );
  dashboardText("diag-drive-backend", wheelBackendName.toUpperCase());
  dashboardText("diag-drive-vbus", vbus == null ? "Not reported" : `${vbus.toFixed(1)} V`);
  dashboardText("diag-drive-error", wheel.error || "None");

  dashboardService(
    "dash-service-gamepad",
    gamepad.connected ? "ok" : gamepad.connected === false ? "warn" : ""
  );
  dashboardServiceNote(
    "dash-service-gamepad",
    gamepad.connected ? "Connected" : gamepad.connected === false ? "Disconnected" : "Not reported"
  );

  const maxThermal = thermals.reduce((max, zone) => {
    const temp = Number(zone.temp_c);
    return Number.isFinite(temp) ? Math.max(max, temp) : max;
  }, -Infinity);
  dashboardText(
    "dash-thermal-diagnostics-summary",
    thermals.length
      ? `${thermals.length} zones · max ${maxThermal.toFixed(1)}°C`
      : "Waiting for thermal telemetry"
  );
  dashboardText("diag-thermal-count", thermals.length ? String(thermals.length) : "--");
  dashboardText(
    "diag-thermal-max",
    Number.isFinite(maxThermal) ? `${maxThermal.toFixed(1)}°C` : "--"
  );

  const batteryReported = battery != null;
  const leftCurrent = dashboardNumber(w.current_left);
  const rightCurrent = dashboardNumber(w.current_right);
  dashboardText(
    "dash-power-diagnostics-summary",
    batteryReported
      ? `${battery.toFixed(0)}% battery`
      : vbus != null ? `${vbus.toFixed(1)} V drive bus` : "No power telemetry"
  );
  dashboardText(
    "diag-power-battery",
    batteryReported ? `${battery.toFixed(0)}%` : "Not reported"
  );
  dashboardText(
    "diag-power-vbus",
    vbus == null ? "Not reported" : `${vbus.toFixed(1)} V`
  );
  dashboardText(
    "diag-power-left-current",
    leftCurrent == null ? "Not reported" : `${Math.abs(leftCurrent).toFixed(2)} A`
  );
  dashboardText(
    "diag-power-right-current",
    rightCurrent == null ? "Not reported" : `${Math.abs(rightCurrent).toFixed(2)} A`
  );
  dashboardText(
    "diag-power-note",
    batteryReported
      ? "Battery telemetry is available from the current WebSocket payload."
      : "No dedicated battery percentage is present. ODrive voltage/current values are shown when available."
  );

  const diagnosticIssues = [];

  if (wheel.estopped) {
    diagnosticIssues.push({
      key: "estop",
      severity: "critical",
      subsystem: "Wheel drive",
      title: "Emergency stop active",
      summary: "The wheel driver reports that the E-stop latch is active.",
      impact: "All commanded wheel motion is blocked.",
      recommendation: "Confirm that the area is safe, then clear the E-stop using Resume or the mapped gamepad action.",
      evidence: [
        ["E-stop flag", "true"],
        ["Backend", wheelBackendName.toUpperCase()],
        ["Wheel error", wheel.error || "None"]
      ],
      actions: [
        { label: "Resume", command: "resume-estop" },
        { label: "Drive diagnostics", command: "diagnostic-drive" }
      ]
    });
  }

  if (wheel.error) {
    diagnosticIssues.push({
      key: "wheel-error",
      severity: "critical",
      subsystem: "Wheel drive",
      title: "Wheel-driver error",
      summary: String(wheel.error),
      impact: "Wheel motion may be unavailable or unsafe.",
      recommendation: "Inspect the active backend, bus voltage and driver state before moving the robot.",
      evidence: [
        ["Backend", wheelBackendName.toUpperCase()],
        ["Driver error", String(wheel.error)],
        ["Bus voltage", vbus == null ? "Not reported" : `${vbus.toFixed(1)} V`]
      ],
      actions: [
        { label: "Drive diagnostics", command: "diagnostic-drive" }
      ]
    });
  }

  if (mksKnown && !mksConnected) {
    diagnosticIssues.push({
      key: "mks-offline",
      severity: "critical",
      subsystem: "Motor bus",
      title: "MKS motor bus disconnected",
      summary: "The motor-bus connection is reported offline.",
      impact: "Lift, brush and bin-door motor status or commands may be unavailable.",
      recommendation: "Scan the bus and inspect per-motor online states.",
      evidence: [
        ["Bus connected", "No"],
        ["Motors reported", String(motorTotal)],
        ["Motors online", String(motorsOnline)]
      ],
      actions: [
        { label: "Motor diagnostics", command: "diagnostic-motors" },
        { label: "Scan bus", command: "scan-motors" }
      ]
    });
  }

  if (lidarKnown && !lidarConnected) {
    diagnosticIssues.push({
      key: "lidar-offline",
      severity: "warning",
      subsystem: "LiDAR",
      title: "LiDAR hardware offline",
      summary: "The robot reports that the LiDAR sensor is not connected.",
      impact: "Mapping and scan preview are unavailable.",
      recommendation: "Check LiDAR power and the driver node, then retry the preview.",
      evidence: [
        ["Connected", "No"],
        ["Firmware", lidar.firmware || "Not reported"],
        ["Web preview", lidarScanning ? "On" : "Off"]
      ],
      actions: [
        { label: "Open Recordings", command: "tab-recordings" }
      ]
    });
  }

  if (gamepad.connected === false) {
    const btMac = gamepad.bt_mac || _cachedBtInfo.mac || "Not reported";
    const batteryText =
      gamepad.battery != null
        ? `${gamepad.battery}%`
        : _cachedBtInfo.battery != null ? `${_cachedBtInfo.battery}%` : "Not reported";

    diagnosticIssues.push({
      key: "gamepad-offline",
      severity: "warning",
      subsystem: "Gamepad",
      title: "Gamepad disconnected",
      summary: "The dashboard received an explicit disconnected state from the gamepad subsystem.",
      impact: "Manual driving and mapped actuator commands may be unavailable. Autonomous ROS functions are not necessarily affected.",
      recommendation: "Open Controller to inspect mappings. Reconnect Bluetooth when a saved MAC address is available.",
      evidence: [
        ["Connected", "No"],
        ["Device", gamepad.name || "Not detected"],
        ["Bluetooth MAC", btMac],
        ["Battery", batteryText],
        ["Active inputs", (gamepad.active_inputs || []).join(", ") || "None"]
      ],
      actions: [
        { label: "Open Controller", command: "tab-controller" },
        ...(btMac !== "Not reported"
          ? [{ label: "Reconnect", command: "reconnect-gamepad" }]
          : [])
      ]
    });
  }

  for (const zone of thermals) {
    const temp = Number(zone.temp_c);
    if (!Number.isFinite(temp) || temp < 65) continue;
    diagnosticIssues.push({
      key: `thermal-${zone.zone || zone.type || diagnosticIssues.length}`,
      severity: temp >= 85 ? "critical" : "warning",
      subsystem: "Jetson thermals",
      title: temp >= 85 ? "Critical thermal-zone temperature" : "Thermal-zone warning",
      summary: `${zone.type || zone.zone || "Thermal zone"} is ${temp.toFixed(1)}°C.`,
      impact: temp >= 85
        ? "The Jetson may throttle or shut down to protect itself."
        : "Sustained load may cause thermal throttling.",
      recommendation: "Inspect all thermal zones and verify airflow, fans and ambient conditions.",
      evidence: [
        ["Zone", zone.zone || "Not reported"],
        ["Type", zone.type || "Not reported"],
        ["Temperature", `${temp.toFixed(1)}°C`],
        ["Warning threshold", "65°C"],
        ["Critical threshold", "85°C"]
      ],
      actions: [
        { label: "Thermal diagnostics", command: "diagnostic-thermal" }
      ]
    });
  }

  dashboardRuntime.alerts = diagnosticIssues;

  const ready = document.getElementById("dashboard-readiness");
  const readinessButton = document.getElementById("readiness-diagnostics-button");
  ready?.classList.remove("ready", "warning", "error");

  const criticalCount = diagnosticIssues.filter(issue => issue.severity === "critical").length;
  const warningCount = diagnosticIssues.filter(issue => issue.severity === "warning").length;
  const firstIssue = diagnosticIssues[0];

  if (criticalCount) {
    ready?.classList.add("error");
    dashboardText("dashboard-ready-title", "ATTENTION REQUIRED");
    dashboardText(
      "dashboard-ready-subtitle",
      `${firstIssue.title} — ${firstIssue.impact}`
    );
  } else if (warningCount) {
    ready?.classList.add("warning");
    dashboardText("dashboard-ready-title", "READY WITH WARNING");
    dashboardText(
      "dashboard-ready-subtitle",
      `${firstIssue.title} — ${firstIssue.impact}`
    );
  } else if (ws?.readyState === WebSocket.OPEN) {
    ready?.classList.add("ready");
    dashboardText("dashboard-ready-title", "READY FOR OPERATION");
    dashboardText("dashboard-ready-subtitle", "All reported critical systems are online");
  }

  if (readinessButton) {
    readinessButton.hidden = diagnosticIssues.length === 0;
    readinessButton.textContent =
      criticalCount
        ? `Review ${criticalCount} critical issue${criticalCount === 1 ? "" : "s"}`
        : `Review ${warningCount} warning${warningCount === 1 ? "" : "s"}`;
  }

  renderDashboardAlerts(diagnosticIssues);
  updateDashboardClock();
}

function diagnosticSeverityText(severity) {
  return severity === "critical" ? "CRITICAL" : severity === "warning" ? "WARNING" : "INFO";
}

function renderDashboardAlerts(alerts) {
  const container = document.getElementById("dashboard-alert-list");
  const viewButton = document.getElementById("dashboard-view-diagnostics");
  if (!container) return;

  container.innerHTML = "";
  if (viewButton) {
    viewButton.disabled = alerts.length === 0;
    viewButton.textContent = alerts.length
      ? `View ${alerts.length} issue${alerts.length === 1 ? "" : "s"}`
      : "No issues";
  }

  if (!alerts.length) {
    container.innerHTML =
      '<div class="dashboard-no-alerts"><span>✓</span><div><strong>No active alerts</strong><small>All reported systems are operating normally</small></div></div>';
    closeDashboardDiagnostics();
    return;
  }

  for (const issue of alerts.slice(0, 4)) {
    const item = document.createElement("article");
    item.className = `dashboard-alert-item ${issue.severity}`;
    item.dataset.issueKey = issue.key;

    const icon = document.createElement("span");
    icon.className = "alert-severity-icon";
    icon.textContent = issue.severity === "critical" ? "×" : "!";

    const copy = document.createElement("div");
    copy.className = "alert-copy";
    const subsystem = document.createElement("small");
    subsystem.textContent = issue.subsystem;
    const title = document.createElement("strong");
    title.textContent = issue.title;
    const summary = document.createElement("p");
    summary.textContent = issue.summary;
    copy.append(subsystem, title, summary);

    const inspect = document.createElement("button");
    inspect.type = "button";
    inspect.className = "alert-inspect-button";
    inspect.textContent = "Inspect";
    inspect.onclick = () => openDashboardDiagnostics(issue.key);

    item.append(icon, copy, inspect);
    container.appendChild(item);
  }
}

function findDashboardIssue(issueKey = null) {
  const alerts = dashboardRuntime.alerts || [];
  if (!alerts.length) return null;
  return alerts.find(issue => issue.key === issueKey) || alerts[0];
}

function openDashboardDiagnostics(issueKey = null) {
  const issue = findDashboardIssue(issueKey);
  const panel = document.getElementById("dashboard-issue-diagnostics");
  if (!panel) return;

  if (!issue) {
    panel.hidden = true;
    scrollDashboardSection("dashboard-alerts-card");
    return;
  }

  panel.hidden = false;
  panel.dataset.issueKey = issue.key;
  dashboardText("issue-severity-label", diagnosticSeverityText(issue.severity));
  dashboardText("issue-diagnostic-title", issue.title);
  dashboardText("issue-diagnostic-summary", issue.summary);
  dashboardText("issue-diagnostic-impact", issue.impact);
  dashboardText("issue-diagnostic-action-text", issue.recommendation);

  const severityLabel = document.getElementById("issue-severity-label");
  if (severityLabel) severityLabel.className = `issue-severity-label ${issue.severity}`;

  const evidenceContainer = document.getElementById("issue-diagnostic-evidence");
  if (evidenceContainer) {
    evidenceContainer.innerHTML = "";
    for (const [label, value] of issue.evidence || []) {
      const row = document.createElement("div");
      const key = document.createElement("span");
      key.textContent = label;
      const val = document.createElement("strong");
      val.textContent = value;
      row.append(key, val);
      evidenceContainer.appendChild(row);
    }
  }

  const actions = document.getElementById("issue-action-buttons");
  if (actions) {
    actions.innerHTML = "";
    for (const action of issue.actions || []) {
      const button = document.createElement("button");
      button.type = "button";
      button.textContent = action.label;
      button.onclick = () => runDashboardDiagnosticAction(action.command);
      if (/resume|reconnect|scan/i.test(action.label)) button.classList.add("primary");
      actions.appendChild(button);
    }
  }

  panel.scrollIntoView({ behavior: "smooth", block: "nearest" });
}

function closeDashboardDiagnostics() {
  const panel = document.getElementById("dashboard-issue-diagnostics");
  if (panel) panel.hidden = true;
}

async function runDashboardDiagnosticAction(command) {
  switch (command) {
    case "resume-estop":
      await resumeEstop();
      break;
    case "diagnostic-drive":
      selectDashboardDiagnostic("drive", null, true);
      break;
    case "diagnostic-motors":
      selectDashboardDiagnostic("motors", null, true);
      break;
    case "diagnostic-thermal":
      selectDashboardDiagnostic("thermal", null, true);
      break;
    case "scan-motors":
      selectDashboardDiagnostic("motors", null, true);
      await scanMotors();
      break;
    case "reconnect-gamepad":
      await reconnectGamepad();
      break;
    case "tab-controller":
      openDashboardTab("controller");
      break;
    case "tab-recordings":
      openDashboardTab("recordings");
      break;
  }
}

function formatDashboardDuration(totalSeconds) {
  const seconds = Math.max(0, Math.floor(Number(totalSeconds) || 0));
  const days = Math.floor(seconds / 86400);
  const hours = Math.floor((seconds % 86400) / 3600);
  const minutes = Math.floor((seconds % 3600) / 60);
  if (days) return `${days}d ${hours}h`;
  if (hours) return `${hours}h ${minutes}m`;
  return `${minutes}m`;
}

function updateDashboardClock() {
  const sessionSeconds = (Date.now() - dashboardRuntime.startedAt) / 1000;
  const uptime = dashboardRuntime.systemUptimeSeconds ?? sessionSeconds;
  const uptimeText = formatDashboardDuration(uptime);
  dashboardText("dash-uptime", uptimeText);
  dashboardText("sidebar-uptime", uptimeText);
  dashboardText("dash-runtime", formatDashboardDuration(dashboardRuntime.runtimeSeconds));

  const distance = dashboardRuntime.distanceMeters;
  dashboardText("dash-distance", distance >= 1000 ? `${(distance / 1000).toFixed(2)} km` : `${distance.toFixed(0)} m`);
}

async function loadDashboardBagStats(force = false) {
  const now = Date.now();
  if (!force && now - dashboardRuntime.bagStatsLoadedAt < 30000) return;
  dashboardRuntime.bagStatsLoadedAt = now;

  try {
    const response = await fetch("/api/bags", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const data = await response.json();
    const bags = Array.isArray(data.bags) ? data.bags.length : 0;
    dashboardText("dash-bags-recorded", String(bags));

    const free = Number(data.disk_free || 0);
    const total = Number(data.disk_total || 0);
    if (free > 0) {
      const pct = total > 0 ? Math.round(free / total * 100) : null;
      dashboardText("dash-storage-remaining", `${formatBytes(free)}${pct == null ? "" : ` (${pct}%)`}`);
    } else {
      dashboardText("dash-storage-remaining", "--");
    }

    const metrics = dashboardRuntime.lastData ? dashboardMetricObject(dashboardRuntime.lastData) : {};
    if (!dashboardNumber(metrics.storage_total_gb, metrics.disk_total_gb) && total > 0) {
      const used = Number(data.disk_used || Math.max(0, total - free));
      dashboardText("dash-storage-value", `${formatBytes(used)} / ${formatBytes(total)}`);
      dashboardProgress("dash-storage-bar", used, total);
    }
  } catch (error) {
    console.debug("Dashboard bag statistics unavailable:", error);
  }
}

function initDashboardPage() {
  document.body.classList.add("dashboard-active");
  restoreDashboardSections();
  restoreDashboardDiagnostic();
  syncDashboardSensorControls();
  updateDashboardConnection(ws?.readyState === WebSocket.OPEN);
  updateDashboardClock();
  loadDashboardBagStats();
  pollSystemHealth();
  setInterval(updateDashboardClock, 1000);
  setInterval(pollSystemHealth, 2000);
  setInterval(() => {
    if (document.body.classList.contains("dashboard-active")) loadDashboardBagStats();
  }, 60000);
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

async function sidebarWheelCommand(action) {
  const stopButton = document.getElementById("sidebar-stop-button");
  const estopButton = document.getElementById("sidebar-estop-button");
  const message = document.getElementById("sidebar-emergency-message");
  const isEstop = action === "stop_fast";

  stopButton?.setAttribute("disabled", "disabled");
  estopButton?.setAttribute("disabled", "disabled");

  if (message) {
    message.textContent = isEstop ? "Sending E-STOP…" : "Sending stop…";
    message.className = "sidebar-emergency-message pending";
  }

  try {
    const response = await fetch(
      `/api/command/wheel_driver_node/${action}`,
      { method: "POST" }
    );

    const result = await response.json().catch(() => ({}));
    if (!response.ok || result.success === false) {
      throw new Error(result.message || `HTTP ${response.status}`);
    }

    if (message) {
      message.textContent = isEstop ? "E-STOP sent" : "Stop sent";
      message.className = "sidebar-emergency-message sent";
    }
  } catch (error) {
    console.error("Sidebar wheel command failed:", error);
    if (message) {
      message.textContent = "Command failed";
      message.className = "sidebar-emergency-message failed";
      message.title = error.message || String(error);
    }
  } finally {
    window.setTimeout(() => {
      stopButton?.removeAttribute("disabled");
      estopButton?.removeAttribute("disabled");
    }, 650);

    window.setTimeout(() => {
      if (message) {
        message.textContent = "";
        message.className = "sidebar-emergency-message";
      }
    }, 2600);
  }
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

async function toggleRecording(profile, event = null) {
  // Only a genuine user click may start or stop a bag.
  if (event && event.isTrusted === false) return;

  const current = Boolean(_recorderUiState[profile]?.recording);
  if (current) {
    await stopRecording(profile);
  } else {
    await startRecording(profile);
  }
}

async function startRecording(profile) {
  const previousIntent = _recorderIntent[profile];
  const previousState = Boolean(_recorderUiState[profile]?.recording);

  applyOptimisticRecorderState(profile, true);
  setRecordingButtonBusy(profile, true);

  try {
    const response = await fetch(`/api/recording/${profile}/start`, {
      method: "POST",
    });
    const result = await response.json().catch(() => ({}));

    if (!response.ok || result.success === false) {
      throw new Error(result.message || `HTTP ${response.status}`);
    }

    // Keep the explicit ON intent. The button remains red until the user
    // presses it again to stop recording.
    _recorderIntent[profile] = true;
  } catch (error) {
    _recorderIntent[profile] = previousIntent;
    applyOptimisticRecorderState(profile, previousState);
    if (previousIntent === null) _recorderIntent[profile] = null;
    console.error(`Start ${profile} recording failed:`, error);
  } finally {
    setRecordingButtonBusy(profile, false);
  }
}

async function stopRecording(profile) {
  const previousIntent = _recorderIntent[profile];
  const previousState = Boolean(_recorderUiState[profile]?.recording);

  applyOptimisticRecorderState(profile, false);
  setRecordingButtonBusy(profile, true);

  try {
    const response = await fetch(`/api/recording/${profile}/stop`, {
      method: "POST",
    });
    const result = await response.json().catch(() => ({}));

    if (!response.ok || result.success === false) {
      throw new Error(result.message || `HTTP ${response.status}`);
    }

    // Keep the explicit OFF intent. The button remains blue.
    _recorderIntent[profile] = false;
  } catch (error) {
    _recorderIntent[profile] = previousIntent;
    applyOptimisticRecorderState(profile, previousState);
    if (previousIntent === null) _recorderIntent[profile] = null;
    console.error(`Stop ${profile} recording failed:`, error);
  } finally {
    setRecordingButtonBusy(profile, false);
  }
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
        updateControllerGamepadCard({
          connected: data.connected || false,
          name: data.connected ? (data.name || "Gamepad connected") : "",
          mac: data.mac || "",
          battery: data.battery,
        });
      } else {
        _cachedBtInfo = { mac: "", battery: null, connected: false };
        updateControllerGamepadCard({
          connected: false,
          name: "No paired gamepad found",
          mac: "",
          battery: null,
        });
      }
    }
  } catch (e) { /* ignore */ }
}

initSidebar();
initDashboardPage();
initTopicChecklists();
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

const _lidarRange = document.getElementById("lidar-range");
const _lidarRangeVal = document.getElementById("lidar-range-val");
if (_lidarRange && _lidarRangeVal) {
  _lidarRange.addEventListener("input", () => { _lidarRangeVal.textContent = Number(_lidarRange.value).toFixed(0); });
}