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
  }
  const notice = !connected ? "Connection lost — telemetry is stale" :
    !state.projection ? "Waiting for localization and GPS map alignment…" :
      pose?.age > 2 ? "Robot localization is stale" :
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

function navigationReady() {
  return connected && state.projection && state.pose?.age < 2 &&
    ["bt_navigator", "planner_server", "controller_server"].every((name) =>
      state.components?.[name]?.state === "active" && state.components[name].age < 3);
}

function updateControls() {
  const navigation = state.navigation || {};
  element("send").disabled = !target || !navigationReady() || navigation.busy || requestBusy;
  element("cancel").disabled = !state.csrf || requestBusy;
  element("mission-state").textContent = navigation.state || "Waiting for ROS";
  badge("nav-badge", navigation.busy ? "ACTIVE" : navigationReady() ? "READY" : "WAITING", navigation.busy ? "ok" : "");
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
  selecting = enabled;
  element("pick").classList.toggle("active", enabled);
  map.getContainer().style.cursor = enabled ? "crosshair" : "";
  updateMap();
  if (enabled && !state.projection) showToast("Waiting for GPS map alignment");
}

map.on("click", (event) => {
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
map.on("dragstart", () => { follow = false; element("follow").classList.remove("active"); });
element("follow").addEventListener("click", () => { follow = !follow; element("follow").classList.toggle("active", follow); updateMap(); });
element("pick").addEventListener("click", () => selectTarget(!selecting));
element("choose-target").addEventListener("click", () => { selectTarget(true); element("map").scrollIntoView({ behavior: "smooth", block: "center" }); });
element("show-obstacles").addEventListener("change", () => obstacleOverlay.redraw());
element("fit").addEventListener("click", () => {
  if (plannedPath.getLatLngs().length) {
    follow = false;
    element("follow").classList.remove("active");
    map.fitBounds(plannedPath.getBounds(), { padding: [50, 70], maxZoom: 20 });
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
    showToast(action === "cancel" ? "Cancel requested" : "Target pose submitted");
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
