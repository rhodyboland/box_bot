/* BOXBOT UI. All control requests require an explicit user action. */
"use strict";

const element = (id) => document.getElementById(id);
const map = L.map("map", { zoomControl: true, attributionControl: true }).setView([-27.4, 153.0], 20);
L.tileLayer("http://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}", {
  maxZoom: 21,
  attribution: "Imagery © Google",
}).addTo(map);
const plannedPath = L.polyline([], { color: "#69c4ff", weight: 4, opacity: 0.95 }).addTo(map);
const controllerPath = L.polyline([], { color: "#a9f4df", weight: 2, dashArray: "5 7" }).addTo(map);
const travelledPath = L.polyline([], { color: "#f472d0", weight: 3, opacity: 0.95 }).addTo(map);
const waypointLine = L.polyline([], { color: "#ffad4c", weight: 2, dashArray: "6 6" }).addTo(map);
const waypointPins = L.layerGroup().addTo(map);
let waypoints = [];
let routeSelecting = false;
let routeRestored = false;
const targetArrow = L.polyline([], { color: "#ffad4c", weight: 3, dashArray: "4 5" }).addTo(map);
const robot = L.marker([0, 0], {
  icon: L.divIcon({
    className: "robot-icon",
    html: '<div class="robot-shell"><div class="robot-arrow"></div></div>',
    iconSize: [28, 28], iconAnchor: [14, 14],
  }),
}).bindTooltip("BOXBOT");

let state = {};
let connected = false;
let follow = true;
let centered = false;
let selecting = false;
let target = null;
let targetMarker = null;
let requestBusy = false;
let toastTimer;
let filterEdited = false;
let recoveryEdited = false;
let tuningDirty = false;
const tuningInputs = new Map();
const tuningEdited = new Set();

function geographic(point) {
  const projection = state.projection;
  if (!projection || !point) return null;
  const dx = point[0] - projection.x;
  const dy = point[1] - projection.y;
  return [projection.latitude + dx * projection.x_lat + dy * projection.y_lat,
    projection.longitude + dx * projection.x_lon + dy * projection.y_lon];
}

function format(value, digits = 1, suffix = "") {
  return Number.isFinite(value) ? value.toFixed(digits) + suffix : "—";
}

function showToast(message) {
  element("toast").textContent = message;
  element("toast").hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { element("toast").hidden = true; }, 5000);
}

function badge(id, text, level) {
  element(id).textContent = text;
  element(id).className = "badge " + (level || "");
}

function robotBearing() {
  if (!state.pose || !state.projection) return 0;
  const pose = state.pose;
  const from = geographic([pose.x, pose.y]);
  const to = geographic([pose.x + Math.cos(pose.yaw), pose.y + Math.sin(pose.yaw)]);
  const east = (to[1] - from[1]) * Math.cos(from[0] * Math.PI / 180);
  const north = to[0] - from[0];
  return (Math.atan2(east, north) * 180 / Math.PI + 360) % 360;
}

const ObstacleOverlay = L.Layer.extend({
  onAdd(activeMap) {
    this.map = activeMap;
    this.canvas = L.DomUtil.create("canvas", "obstacle-overlay");
    activeMap.getPanes().overlayPane.appendChild(this.canvas);
    activeMap.on("moveend zoomend resize", this.redraw, this);
    this.redraw();
  },
  onRemove(activeMap) {
    activeMap.off("moveend zoomend resize", this.redraw, this);
    this.canvas.remove();
  },
  redraw() {
    if (!this.map) return;
    const size = this.map.getSize();
    const origin = this.map.containerPointToLayerPoint([0, 0]);
    L.DomUtil.setPosition(this.canvas, origin);
    this.canvas.width = size.x;
    this.canvas.height = size.y;
    const context = this.canvas.getContext("2d");
    if (!connected || !state.projection || !element("show-obstacles").checked) return;
    if ((state.components?.costmap?.age ?? 99) > 3) return;
    const center = this.map.getCenter();
    const metersPerPixel = 156543.03392 * Math.cos(center.lat * Math.PI / 180) / (2 ** this.map.getZoom());
    const width = Math.max(3, (state.resolution || 0.1) / metersPerPixel);
    for (const point of state.obstacles || []) {
      const position = this.map.latLngToContainerPoint(geographic(point));
      if (position.x < -width || position.x > size.x + width || position.y < -width || position.y > size.y + width) continue;
      context.fillStyle = point[2] >= 95 ? "#ffb253e6" : "#ffb25365";
      context.fillRect(position.x - width / 2, position.y - width / 2, width, width);
    }
  },
});
const obstacleOverlay = new ObstacleOverlay().addTo(map);

function updateMap() {
  const pose = state.pose;
  const position = pose && geographic([pose.x, pose.y]);
  if (position) {
    robot.setLatLng(position);
    if (!map.hasLayer(robot)) robot.addTo(map);
    const bearing = robotBearing();
    robot.getElement().querySelector(".robot-arrow").style.transform = `rotate(${bearing}deg)`;
    robot.setOpacity(connected && pose.age < 2 ? 1 : 0.4);
    if (!centered || follow) {
      map.panTo(position, { animate: centered, duration: 0.4 });
      centered = true;
    }
    element("position").textContent = `${position[0].toFixed(6)}, ${position[1].toFixed(6)}`;
    element("bearing").textContent = format(bearing, 0, "°");
  }
  if (state.projection) {
    plannedPath.setLatLngs((state.plan || []).map(geographic));
    controllerPath.setLatLngs((state.controller_plan || []).map(geographic));
    const segments = [];
    for (const point of state.trail || []) {
      if (!segments.length || point[2]) segments.push([]);
      segments[segments.length - 1].push(geographic(point));
    }
    travelledPath.setLatLngs(element("show-trail").checked ? segments : []);
  }
  const notice = !connected ? "Connection lost — telemetry is stale" :
    !state.projection ? "Waiting for localization and GPS map alignment…" :
      pose?.age > 2 ? "Robot localization is stale" :
        routeSelecting ? "Click the map to add waypoints in order" :
          selecting ? "Click the map to place a target pose" : "";
  element("map-notice").textContent = notice;
  element("map-notice").hidden = !notice;
  obstacleOverlay.redraw();
}

function describe(name, component) {
  if (component.level === "stale") return "Telemetry unavailable";
  if (name.startsWith("gps")) {
    let detail = `${component.fix || "GNSS"} · ${component.satellites ?? "—"} satellites · ±${format(component.accuracy, 2, " m")}`;
    if (Number.isFinite(component.epoch_age)) detail += ` · epoch ${format(component.epoch_age, 2, "s")}`;
    if (Number.isFinite(component.usb_usage)) detail += ` · USB ${component.usb_usage}%`;
    return detail;
  }
  if (name === "heading") return `${format(component.baseline, 3, " m")} baseline · ±${format(component.accuracy, 2, "°")}`;
  if (name === "hoverboard") return `${format(component.vx, 2, " m/s")} · ${format(component.wz, 2, " rad/s")}`;
  if (name === "realsense") return `${(component.points || 0).toLocaleString()} filtered obstacle points`;
  if (name === "imu") return `Roll ${format(component.roll, 1, "°")} · pitch ${format(component.pitch, 1, "°")}`;
  if (name === "costmap") return `${(component.occupied || 0).toLocaleString()} occupied / inflated cells`;
  return component.message || component.state || "Receiving telemetry";
}

function updateSystems() {
  const expected = [
    ["drive_controller", "Drive controller"], ["hoverboard", "Hoverboard telemetry"],
    ["boxbot_hub/hoverboard_power", "Hoverboard power sense"],
    ["gps_rover", "Rover GNSS"], ["gps_base", "Moving-base GNSS"],
    ["heading", "Moving-baseline heading"], ["realsense", "RealSense + ground filter"],
    ["costmap", "Local obstacle map"], ["imu", "BNO085 IMU"],
    ["transforms", "Frame transforms"],
    ["localization", "Global localization"], ["boxbot_hub/link", "ESP32 hub"],
    ["boxbot_hub/bms0", "Battery telemetry link"], ["planner_server", "Path planner"],
    ["controller_server", "Navigation controller"], ["bt_navigator", "BT navigator"],
    ["behavior_server", "Behavior server"], ["velocity_smoother", "Velocity smoother"],
  ];
  const fragment = document.createDocumentFragment();
  let online = 0;
  for (const [name, label] of expected) {
    const component = state.components?.[name] || { level: "stale", age: null };
    if (component.level === "ok" && connected) online += 1;
    const row = document.createElement("div");
    row.className = "system";
    const dot = document.createElement("span");
    dot.className = "dot " + (connected ? component.level : "stale");
    const body = document.createElement("div");
    const title = document.createElement("div");
    title.className = "system-name";
    title.textContent = label;
    const detail = document.createElement("div");
    detail.className = "system-detail";
    detail.textContent = describe(name, component);
    const age = document.createElement("span");
    age.className = "system-age";
    age.textContent = format(component.age, 1, "s");
    body.append(title, detail);
    row.append(dot, body, age);
    fragment.append(row);
  }
  element("systems").replaceChildren(fragment);
  element("systems-count").textContent = `${online} / ${expected.length} online`;
}

function updateBattery() {
  const battery = state.battery || {};
  const component = state.components?.battery;
  const fresh = connected && component?.level === "ok";
  badge("battery-badge", fresh ? "LIVE BMS" : "STALE", fresh ? "ok" : "warn");
  const percent = Number.isFinite(battery.percentage) && battery.percentage >= 0 ? battery.percentage * 100 : null;
  element("battery-percent").replaceChildren(document.createTextNode(format(percent, 0)));
  const symbol = document.createElement("span");
  symbol.textContent = "%";
  element("battery-percent").append(symbol);
  element("battery-fill").style.width = `${Math.max(0, Math.min(100, percent || 0))}%`;
  element("battery-fill").style.background = percent < 20 ? "var(--red)" : "var(--green)";
  element("battery-direction").textContent = !fresh ? "Awaiting fresh BMS data" : battery.current > 0.05 ? "Charging" : battery.current < -0.05 ? "Discharging" : "Low current";
  element("voltage").textContent = format(battery.voltage, 2, " V");
  element("current").textContent = format(battery.current, 2, " A");
  element("temperature").textContent = format(battery.temperature, 0, " °C");
  element("cell-voltages").replaceChildren(...(battery.cells || []).map((voltage, index) => {
    const cell = document.createElement("span");
    cell.textContent = format(voltage, 3);
    cell.title = `Cell ${index + 1} · ${format(voltage, 3, " V")}`;
    return cell;
  }));
}

let previewUrl = null;
let previewRequest = null;

function clearPreview() {
  element("camera-preview").hidden = true;
  element("camera-preview").removeAttribute("src");
  if (previewUrl) URL.revokeObjectURL(previewUrl);
  previewUrl = null;
}

element("show-camera").addEventListener("change", () => {
  if (!element("show-camera").checked) {
    previewRequest?.abort();
    clearPreview();
    element("camera-preview-status").textContent = "Preview off · 320 px · 2 fps when enabled";
  }
});

async function pollPreview() {
  if (element("show-camera").checked && !document.hidden) {
    previewRequest = new AbortController();
    const timer = setTimeout(() => previewRequest?.abort(), 2500);
    try {
      const response = await fetch("/api/camera.jpg", { cache: "no-store", signal: previewRequest.signal });
      if (!response.ok) throw new Error("Waiting for a fresh RGB frame…");
      const blob = await response.blob();
      if (element("show-camera").checked && !document.hidden) {
        const oldUrl = previewUrl;
        previewUrl = URL.createObjectURL(blob);
        element("camera-preview").src = previewUrl;
        element("camera-preview").hidden = false;
        if (oldUrl) URL.revokeObjectURL(oldUrl);
        element("camera-preview-status").textContent = "Live · 320 px · up to 2 fps";
      }
    } catch (error) {
      if (element("show-camera").checked) {
        clearPreview();
        element("camera-preview-status").textContent = error.name === "AbortError" ? "Camera preview connection timed out" : error.message;
      }
    } finally {
      clearTimeout(timer);
      previewRequest = null;
    }
  }
  setTimeout(pollPreview, 500);
}
pollPreview();

function updateTraction() {
  const traction = state.traction || {};
  const metrics = traction.metrics || {};
  const recovery = traction.recovery;
  const toggle = element("traction-recovery-enabled");
  const attempts = element("recovery-attempts");
  if (!recoveryEdited && recovery) attempts.value = recovery.max_attempts;
  attempts.disabled = !connected || !recovery || requestBusy;
  element("apply-recovery-attempts").disabled = attempts.disabled || !recoveryEdited ||
    !Number.isInteger(Number(attempts.value)) || Number(attempts.value) < 0 || Number(attempts.value) > 20;
  toggle.checked = Boolean(recovery?.enabled);
  toggle.disabled = !connected || !recovery || requestBusy;
  element("traction-recovery-description").textContent = recovery ?
    `After confirmation + ${format(recovery.delay, 1)} s · ${format(recovery.distance, 1)} m at ${format(recovery.speed, 1)} m/s · stored-map clearance and Nav2 collision checks` : "";
  element("traction-recovery-status").textContent = recovery ?
    `${recovery.enabled ? "Enabled" : "Disabled"} · ${recovery.message} ` +
    (Number.isFinite(recovery.countdown) ? `Recovery in ${format(recovery.countdown, 1)} s. ` : "") +
    `Consecutive attempts ${recovery.attempts}/${recovery.max_attempts || "∞"}` : "Waiting for recovery settings.";
  const labels = { idle: "IDLE", monitoring: "MONITORING", limited_data: "LIMITED",
    insufficient_data: "WAITING", telemetry_fault: "TELEMETRY FAULT" };
  const status = traction.state || "insufficient_data";
  const label = status.startsWith("suspected_") ? "SUSPECTED " + status.slice(10).replaceAll("_", " ").toUpperCase() :
    status.startsWith("checking_") ? "CHECKING" : labels[status];
  badge("traction-badge", connected ? label : "STALE", connected ? traction.level : "warn");
  let message = connected ? traction.message : "Telemetry is stale.";
  if (status.startsWith("checking_") || status.startsWith("suspected_")) {
    message += ` Evidence ${format(traction.duration, 1)} / ${format(traction.dwell, 1)} s.`;
  }
  element("traction-state").textContent = message || "Waiting for wheel, gyro and GNSS measurements.";
  for (const [key, id] of [["command_vx", "command-vx"], ["wheel_vx", "wheel-vx"],
    ["gps_speed", "gps-speed"], ["gps_accuracy", "gps-accuracy"]]) {
    element(`traction-${id}`).textContent = format(metrics[key], 3, " m/s");
  }
  for (const [key, id] of [["command_wz", "command-wz"], ["wheel_wz", "wheel-wz"], ["gyro_wz", "gyro-wz"]]) {
    const rate = Number.isFinite(metrics[key]) ? metrics[key] * 180 / Math.PI : null;
    element(`traction-${id}`).textContent = format(rate, 1, " °/s");
  }
  element("traction-log").textContent = traction.log_path ? `CSV · ${traction.log_path}` : "CSV recording unavailable";
}

element("recovery-attempts").addEventListener("input", () => { recoveryEdited = true; updateTraction(); });
element("apply-recovery-attempts").addEventListener("click", () => {
  if (!element("apply-recovery-attempts").disabled) control("recovery", { max_attempts: Number(element("recovery-attempts").value) });
});

element("traction-recovery-enabled").addEventListener("change", (event) => {
  control("recovery", { enabled: event.target.checked });
});

function updateGroundFilter() {
  const settings = state.ground_filter || {};
  const live = connected && Number.isFinite(settings.min_obstacle_height) && settings.age < 6;
  const slider = element("obstacle-height");
  const draft = Number(slider.value) / 100;
  if (live && (!filterEdited || Math.abs(draft - settings.min_obstacle_height) < 0.0001)) {
    slider.value = Math.round(settings.min_obstacle_height * 100);
    filterEdited = false;
  }
  slider.disabled = !live || settings.pending;
  element("obstacle-height-value").textContent = `${slider.value} cm`;
  badge("filter-live", live ? `${Math.round(settings.min_obstacle_height * 100)} CM LIVE` : "UNAVAILABLE", live ? "ok" : "warn");
  element("apply-filter").disabled = !live || !filterEdited || settings.pending || requestBusy;
  element("filter-status").textContent = settings.pending ? "Applying height threshold…" : settings.error ||
    (live ? `Ground plane ${format(settings.ground_height, 3, " m")} · live cutoff ${Math.round(settings.min_obstacle_height * 100)} cm` :
      "Waiting for the live ground filter settings.");
}

element("obstacle-height").addEventListener("input", () => { filterEdited = true; updateGroundFilter(); });
element("apply-filter").addEventListener("click", () => {
  if (!element("apply-filter").disabled) control("filter", { min_obstacle_height: Number(element("obstacle-height").value) / 100 });
});

function navigationReady() {
  return connected && state.projection && state.pose?.age < 2 &&
    ["bt_navigator", "planner_server", "controller_server"].every((name) =>
      state.components?.[name]?.state === "active" && state.components[name].age < 3);
}

function tuningDraft() {
  return Object.fromEntries([...tuningInputs].map(([key, input]) => [key, tuningEdited.has(key) ? Number(input.value) : input.original]));
}

function updateTuning() {
  const tuning = state.tuning || {};
  const schema = tuning.schema || {};
  const speed = (name) => {
    const component = state.components?.[name];
    return format(connected && component?.age <= 1 ? component.vx : null, 2, " m/s");
  };
  element("tuning-speeds").textContent =
    `Nav2 command ${speed("navigation_velocity")} · Drive command ${speed("velocity")} · Wheel speed ${speed("hoverboard")}`;
  if (!tuningInputs.size && Object.keys(schema).length) {
    const container = element("tuning-fields");
    for (const [key, setting] of Object.entries(schema)) {
      const row = document.createElement("div");
      row.className = "tuning-field";
      const label = document.createElement("label");
      label.htmlFor = `tuning-${key}`;
      label.textContent = `${setting.label} (${setting.unit})`;
      const input = document.createElement("input");
      input.id = `tuning-${key}`;
      input.type = "number";
      input.min = setting.min;
      input.max = setting.max;
      input.step = setting.step;
      input.addEventListener("input", () => { tuningEdited.add(key); tuningDirty = true; updateTuning(); });
      const readout = document.createElement("small");
      row.append(label, input, readout);
      container.append(row);
      input.readout = readout;
      tuningInputs.set(key, input);
    }
  }
  const live = tuning.values || {};
  const saved = tuning.saved?.values || {};
  for (const [key, input] of tuningInputs) {
    if (!tuningDirty) {
      const value = live[key] ?? saved[key];
      input.original = value;
      input.value = Number.isFinite(value) ? Number(value.toFixed(3)) : "";
    }
    input.disabled = !connected || tuning.busy || requestBusy;
    input.readout.textContent = `Live ${format(live[key], 3)} · Saved ${format(saved[key], 3)}`;
    input.parentElement.classList.toggle("changed", tuningDirty && Number.isFinite(live[key]) &&
      Math.abs((tuningEdited.has(key) ? Number(input.value) : input.original)-live[key]) > 0.000001);
  }
  const draft = tuningDraft();
  const valid = tuningInputs.size && Object.entries(schema).every(([key, spec]) =>
    Number.isFinite(draft[key]) && draft[key] >= spec.min && draft[key] <= spec.max);
  if (tuningDirty && tuning.ready && !tuning.busy && Object.keys(schema).every((key) =>
      Math.abs(draft[key]-live[key]) < 0.000001)) { tuningDirty = false; tuningEdited.clear(); }
  const unavailable = !connected || requestBusy || tuning.busy;
  element("tuning-try").disabled = unavailable || !tuning.can_apply || !valid || (!tuningDirty && !tuning.block_navigation);
  element("tuning-save").disabled = unavailable || !tuning.ready || tuningDirty || !tuning.trial || !tuning.saved?.revision || Boolean(tuning.config_error);
  element("tuning-load").disabled = unavailable || !tuning.saved?.revision;
  element("tuning-reset").disabled = unavailable || !tuningInputs.size;
  badge("tuning-badge", tuning.busy ? "UPDATING" : !tuning.ready ? "WAITING" : tuning.error ? "ERROR" : tuning.trial ? "TRIAL ACTIVE" : "SAVED",
    tuning.error ? "error" : tuning.trial ? "warn" : "ok");
  element("tuning-status").textContent = tuning.error || tuning.config_error ||
    (tuning.busy ? tuning.message : tuning.blocked_reason || tuning.message) || "Waiting for live Nav2 parameters.";
  element("tuning-file").textContent = tuning.config_path ? `Save destination · ${tuning.config_path}` : "";
}

element("tuning-try").addEventListener("click", () => {
  if (!element("tuning-try").disabled) control("tuning", { mode: "try", values: tuningDraft() });
});
element("tuning-save").addEventListener("click", () => {
  if (!element("tuning-save").disabled) control("tuning", {
    mode: "save", values: state.tuning.values, revision: state.tuning.saved.revision });
});
element("tuning-load").addEventListener("click", () => {
  tuningEdited.clear();
  for (const [key, input] of tuningInputs) {
    input.original = state.tuning.saved.values[key];
    input.value = Number(input.original.toFixed(3));
  }
  tuningDirty = true;
  updateTuning();
});
element("tuning-reset").addEventListener("click", () => { tuningDirty = false; tuningEdited.clear(); updateTuning(); });

function recoveryPresentation(recovery) {
  if (recovery?.state === "failed") return { message: recovery.message, label: "RECOVERY STOPPED", level: "error" };
  if (recovery?.busy) return { message: `Recovery · ${recovery.message}`, label: "RECOVERING", level: "warn" };
  if (Number.isFinite(recovery?.countdown)) return {
    message: `Slip confirmed · recovery in ${format(recovery.countdown, 1)} s`, label: "SLIP", level: "warn" };
  return null;
}

function updateControls() {
  const navigation = state.navigation || {};
  updateGroundFilter();
  updateTraction();
  updateTuning();
  if (!routeRestored && state.navigation) {
    routeRestored = true;
    if (navigation.mode === "waypoints") {
      waypoints = (navigation.targets || []).map((point) => ({ ...point }));
      renderRoute();
    }
  }
  const waypointReady = state.components?.waypoint_follower?.state === "active" && state.components.waypoint_follower.age < 3;
  element("route-send").disabled = !waypoints.length || !waypointReady || !navigationReady() || navigation.busy || requestBusy;
  element("route-undo").disabled = !waypoints.length || navigation.busy;
  element("route-clear").disabled = !waypoints.length || navigation.busy;
  element("route-pick").disabled = !!navigation.busy;
  element("clear-trail").disabled = !connected || requestBusy;
  for (const input of element("waypoint-list").querySelectorAll("input, button")) input.disabled = !!navigation.busy;
  waypointPins.eachLayer((pin) => {
    if (navigation.busy) pin.dragging.disable(); else pin.dragging.enable();
  });
  element("route-progress").textContent = navigation.mode === "waypoints" && navigation.busy ?
    `Following waypoint ${(navigation.current_waypoint || 0) + 1} of ${navigation.total_waypoints}` :
    "Up to 50 waypoints · 300 m per leg · stops on a failed waypoint";
  element("send").disabled = !target || !navigationReady() || navigation.busy || requestBusy;
  element("cancel").disabled = !state.csrf || requestBusy;
  const recovery = recoveryPresentation(state.traction?.recovery);
  const canceling = navigation.busy && navigation.state?.startsWith("Cancel");
  element("mission-state").textContent = recovery?.message || navigation.state || "Waiting for ROS";
  badge("nav-badge", recovery?.label || (canceling ? "CANCELING" : navigation.busy ? "ACTIVE" : navigationReady() ? "READY" : "WAITING"),
    recovery?.level || (canceling ? "warn" : navigation.busy ? "ok" : ""));
  if (state.traction?.recovery?.busy) element("route-progress").textContent =
    `Waypoint mission paused for traction recovery · ${state.traction.recovery.message}`;
  element("distance").textContent = navigation.busy ? format(navigation.distance, 1, " m") : "—";
  element("eta").textContent = navigation.busy && navigation.eta > 0 ? `${navigation.eta} s` : "—";
  element("speed").textContent = format(state.components?.hoverboard?.vx, 2, " m/s");
}

function updateTarget() {
  if (!target) return;
  const latlng = [target.latitude, target.longitude];
  targetMarker.setLatLng(latlng);
  const bearing = target.heading * Math.PI / 180;
  const scale = 2 / 6378137;
  targetArrow.setLatLngs([latlng, [target.latitude + scale * Math.cos(bearing) * 180 / Math.PI,
    target.longitude + scale * Math.sin(bearing) / Math.cos(target.latitude * Math.PI / 180) * 180 / Math.PI]]);
  element("target-position").textContent = `${target.latitude.toFixed(7)}, ${target.longitude.toFixed(7)}`;
  element("target-editor").hidden = false;
  element("heading").value = Math.round(target.heading);
  element("heading-number").value = Math.round(target.heading);
  updateControls();
}

function selectTarget(enabled) {
  if (enabled) { routeSelecting = false; element("route-pick").classList.remove("active"); }
  selecting = enabled;
  element("pick").classList.toggle("active", enabled);
  map.getContainer().style.cursor = enabled ? "crosshair" : "";
  updateMap();
  if (enabled && !state.projection) showToast("Waiting for GPS map alignment");
}

map.on("click", (event) => {
  if (routeSelecting && state.projection && !state.navigation?.busy) {
    if (waypoints.length >= 50) { showToast("Route is limited to 50 waypoints"); return; }
    const point = { latitude: event.latlng.lat, longitude: event.latlng.lng, heading: Math.round(robotBearing()) };
    const previous = waypoints[waypoints.length - 1];
    if (previous && !previous.headingEdited) {
      const east = (point.longitude - previous.longitude) * Math.cos(previous.latitude * Math.PI / 180);
      const north = point.latitude - previous.latitude;
      previous.heading = (Math.round(Math.atan2(east, north) * 180 / Math.PI) + 360) % 360;
    }
    waypoints.push(point);
    renderRoute();
    updateControls();
    return;
  }
  if (!selecting || !state.projection) return;
  target = { latitude: event.latlng.lat, longitude: event.latlng.lng, heading: Math.round(robotBearing()) };
  if (!targetMarker) {
    targetMarker = L.marker(event.latlng, { draggable: true,
      icon: L.divIcon({ className: "goal-icon", html: '<span class="goal-pin"></span>', iconSize: [24, 24], iconAnchor: [12, 24] }),
    }).addTo(map);
    targetMarker.on("dragend", () => {
      const position = targetMarker.getLatLng();
      target.latitude = position.lat;
      target.longitude = position.lng;
      updateTarget();
    });
  }
  selectTarget(false);
  updateTarget();
});
function renderRoute() {
  waypointPins.clearLayers();
  waypointLine.setLatLngs(waypoints.map((point) => [point.latitude, point.longitude]));
  const rows = waypoints.map((point, index) => {
    const pin = L.marker([point.latitude, point.longitude], { draggable: !state.navigation?.busy,
      icon: L.divIcon({ className: "waypoint-pin", html: `<span>${index + 1}</span>`, iconSize: [26, 26], iconAnchor: [13, 13] }),
    }).addTo(waypointPins);
    pin.on("dragend", () => {
      const position = pin.getLatLng();
      point.latitude = position.lat;
      point.longitude = position.lng;
      renderRoute();
    });
    const row = document.createElement("li");
    const label = document.createElement("span");
    label.textContent = `${index + 1}. ${point.latitude.toFixed(6)}, ${point.longitude.toFixed(6)}`;
    const heading = document.createElement("input");
    heading.type = "number"; heading.min = "0"; heading.max = "359"; heading.value = Math.round(point.heading);
    heading.setAttribute("aria-label", `Waypoint ${index + 1} final compass heading`);
    heading.addEventListener("change", () => {
      point.heading = Math.max(0, Math.min(359, Number(heading.value) || 0));
      point.headingEdited = true;
      heading.value = point.heading;
    });
    const remove = document.createElement("button");
    remove.textContent = "×";
    remove.setAttribute("aria-label", `Remove waypoint ${index + 1}`);
    remove.addEventListener("click", () => { waypoints.splice(index, 1); renderRoute(); updateControls(); });
    row.append(label, heading, document.createTextNode("°"), remove);
    return row;
  });
  element("waypoint-list").replaceChildren(...rows);
  element("route-count").textContent = `${waypoints.length} POINTS`;
}

element("route-pick").addEventListener("click", () => {
  const enabled = !routeSelecting;
  selectTarget(false);
  routeSelecting = enabled;
  element("route-pick").classList.toggle("active", enabled);
  map.getContainer().style.cursor = enabled ? "crosshair" : "";
  updateMap();
});
element("route-undo").addEventListener("click", () => { waypoints.pop(); renderRoute(); updateControls(); });
element("route-clear").addEventListener("click", () => { waypoints = []; renderRoute(); updateControls(); });
element("show-trail").addEventListener("change", updateMap);
element("clear-trail").addEventListener("click", () => control("clear_trail", {}));
element("route-send").addEventListener("click", () => {
  if (!element("route-send").disabled) {
    routeSelecting = false;
    element("route-pick").classList.remove("active");
    map.getContainer().style.cursor = "";
    control("waypoints", { waypoints });
  }
});

map.on("dragstart", () => { follow = false; element("follow").classList.remove("active"); });
element("follow").addEventListener("click", () => { follow = !follow; element("follow").classList.toggle("active", follow); updateMap(); });
element("pick").addEventListener("click", () => selectTarget(!selecting));
element("choose-target").addEventListener("click", () => { selectTarget(true); element("map").scrollIntoView({ behavior: "smooth", block: "center" }); });
element("show-obstacles").addEventListener("change", () => obstacleOverlay.redraw());
element("fit").addEventListener("click", () => {
  const bounds = L.latLngBounds([]);
  for (const layer of [plannedPath, travelledPath, waypointLine]) {
    const layerBounds = layer.getBounds();
    if (layerBounds.isValid()) bounds.extend(layerBounds);
  }
  if (bounds.isValid()) {
    follow = false;
    element("follow").classList.remove("active");
    map.fitBounds(bounds, { padding: [50, 70], maxZoom: 20 });
  }
});
for (const id of ["heading", "heading-number"]) {
  element(id).addEventListener("input", () => {
    if (target) { target.heading = Math.max(0, Math.min(359, Number(element(id).value) || 0)); updateTarget(); }
  });
}

async function control(action, payload) {
  requestBusy = true;
  updateControls();
  try {
    const response = await fetch(`/api/${action}`, {
      method: "POST", headers: { "Content-Type": "application/json", "X-Boxbot-Token": state.csrf },
      body: JSON.stringify(payload), signal: AbortSignal.timeout(5000),
    });
    const body = await response.json();
    if (!response.ok) throw new Error(body.error || "Robot control request failed");
    if (action === "recovery" && state.traction?.recovery) {
      Object.assign(state.traction.recovery, body);
      recoveryEdited = false;
    }
    const messages = { tuning: payload?.mode === "save" ? "Checking live values before saving" : "Applying trial settings", recovery: "Recovery settings updated", cancel: "Cancel requested", goal: "Target pose submitted", waypoints: "Waypoint route submitted", clear_trail: "Travelled track cleared", filter: "Height threshold update requested" };
    showToast(messages[action]);
  } catch (error) {
    showToast(error.message);
  } finally {
    requestBusy = false;
    updateControls();
  }
}
element("send").addEventListener("click", () => { if (target && navigationReady()) control("goal", target); });
element("cancel").addEventListener("click", () => control("cancel", {}));

async function poll() {
  try {
    const response = await fetch("/api/state", { cache: "no-store", signal: AbortSignal.timeout(2500) });
    if (!response.ok) throw new Error("Bridge unavailable");
    state = await response.json();
    connected = Number(response.headers.get("X-Boxbot-Snapshot-Age")) < 3;
    element("connection-dot").className = connected ? "dot ok" : "dot error";
    element("connection-text").textContent = connected ? "LIVE CONNECTION" : "ROS BRIDGE STALE";
    element("clock").textContent = new Date(state.time * 1000).toLocaleTimeString();
    const events = (state.events || []).slice().reverse();
    element("events").replaceChildren(...events.map((event) => {
      const row = document.createElement("li");
      const timestamp = document.createElement("time");
      timestamp.textContent = new Date(event.time * 1000).toLocaleTimeString();
      row.append(timestamp, document.createTextNode(event.message));
      return row;
    }));
    if (!events.length) element("events").textContent = "Ready for navigation events";
    const memory = state.system?.memory;
    element("system-load").textContent = memory ? `${((memory.MemTotal - memory.MemAvailable) / 1024).toFixed(1)} GB RAM used` : "—";
  } catch (error) {
    connected = false;
    element("connection-dot").className = "dot error";
    element("connection-text").textContent = "DISCONNECTED";
  }
  updateMap();
  updateSystems();
  updateBattery();
  updateControls();
  setTimeout(poll, 500);
}
poll();
