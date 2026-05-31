/**
 * Diagram generator for the Precision Farming Robot 2.0 docs.
 *
 * Produces theme-aware inline SVG (using CSS classes defined in index.html)
 * and writes a JSON map of {key: svgString}. A second pass injects each SVG
 * into the corresponding markdown file between marker comments.
 *
 *   node docs/diagrams/gen.mjs
 *
 * Re-run any time the diagram definitions below change.
 */
import { writeFileSync, readFileSync } from "fs";
import { fileURLToPath } from "url";
import { dirname, join } from "path";

const __dir = dirname(fileURLToPath(import.meta.url));
const CONTENT = join(__dir, "..", "content");

// ── tiny SVG helpers ──────────────────────────────────────────────────────
const esc = (s) => String(s).replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");

function box(x, y, w, h, lines, cls = "node") {
  // lines: [{t, cls}] centered vertically inside the box
  const cx = x + w / 2;
  const total = lines.length;
  const lineH = 16;
  const startY = y + h / 2 - ((total - 1) * lineH) / 2 + 4;
  const text = lines
    .map((l, i) => `<text class="${l.cls || "t"}" x="${cx}" y="${startY + i * lineH}" text-anchor="middle">${esc(l.t)}</text>`)
    .join("");
  return `<rect class="${cls}" x="${x}" y="${y}" width="${w}" height="${h}" rx="9"/>${text}`;
}

// orthogonal-ish edge with optional mid label. pts = [[x,y],...]
function edge(pts, { cls = "edge", marker = "ar", label = null, lx = null, ly = null } = {}) {
  const d = "M" + pts.map((p) => p.join(",")).join(" L");
  let out = `<path class="${cls}" d="${d}" marker-end="url(#${marker})"/>`;
  if (label) {
    const w = label.length * 6 + 8;
    out += `<rect class="elabel-bg" x="${lx - 2}" y="${ly - 11}" width="${w}" height="14"/>`;
    out += `<text class="elabel" x="${lx}" y="${ly}">${esc(label)}</text>`;
  }
  return out;
}

function defs() {
  const mk = (id, fill) =>
    `<marker id="${id}" markerWidth="9" markerHeight="9" refX="7" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8 Z" fill="${fill}"/></marker>`;
  return `<defs>${mk("ar", "var(--text-dim)")}${mk("ar-a", "var(--accent)")}${mk("ar-b", "var(--accent-2)")}${mk("ar-w", "#d29922")}</defs>`;
}

function svg(vbW, vbH, label, body, caption) {
  return (
    `<div class="diagram">\n` +
    `<svg viewBox="0 0 ${vbW} ${vbH}" role="img" aria-label="${esc(label)}">\n` +
    defs() +
    "\n" +
    body +
    `\n</svg>\n` +
    `<div class="d-cap">${esc(caption)}</div>\n` +
    `</div>`
  );
}

const D = {};

/* ───────────────────────────── 1. SYSTEM ARCHITECTURE (overview) ───────── */
{
  let b = "";
  // Robot enclosure
  b += `<rect class="group-box" x="20" y="20" width="430" height="360" rx="12"/>`;
  b += `<text class="t-grp" x="40" y="44">Field robot</text>`;

  b += box(45, 60, 175, 56, [{ t: "Arduino Uno", cls: "t" }, { t: "Motor Shield · 4 DC · 2 servo", cls: "t-dim" }], "node-accent");
  b += box(45, 150, 175, 56, [{ t: "Arduino Nano", cls: "t" }, { t: "MPU IMU · laser (A0)", cls: "t-dim" }], "node-accent");
  b += box(45, 240, 175, 56, [{ t: "Camera", cls: "t" }, { t: "V4L2 / CSI", cls: "t-dim" }], "node");

  b += box(270, 130, 160, 110, [
    { t: "Raspberry Pi 4B", cls: "t" },
    { t: "Ubuntu 24.04", cls: "t-dim" },
    { t: "ROS2 Jazzy", cls: "t-dim" },
  ], "node-blue");

  // internal links
  b += edge([[220, 88], [270, 88], [270, 150]], { cls: "edge", marker: "ar", label: "USB serial 6-byte", lx: 226, ly: 80 });
  b += edge([[220, 178], [268, 178]], { cls: "edge", marker: "ar", label: "A:/G: · LASER", lx: 224, ly: 170 });
  b += edge([[220, 268], [270, 268], [270, 240]], { cls: "edge", marker: "ar", label: "frames", lx: 226, ly: 286 });

  // links out
  b += box(560, 40, 180, 56, [{ t: "Mobile App", cls: "t" }, { t: "MQTT telemetry", cls: "t-dim" }], "node");
  b += box(560, 160, 180, 56, [{ t: "Desktop Client", cls: "t" }, { t: "Qt6 digital twin", cls: "t-dim" }], "node");
  b += box(560, 280, 180, 56, [{ t: "CV scripts", cls: "t" }, { t: "YOLO train / infer", cls: "t-dim" }], "node");

  b += edge([[430, 165], [495, 165], [495, 68], [556, 68]], { cls: "edge-blue", marker: "ar-b", label: "MQTT 1883", lx: 452, ly: 130 });
  b += edge([[430, 185], [556, 185]], { cls: "edge-blue", marker: "ar-b", label: "ROS2 DDS", lx: 452, ly: 178 });
  b += edge([[430, 220], [495, 220], [495, 305], [556, 305]], { cls: "edge", marker: "ar", label: "weights", lx: 452, ly: 250 });

  D.systemArch = svg(
    760, 400,
    "System architecture: robot, Raspberry Pi, and the three clients",
    b,
    "System architecture — two Arduinos + camera feed the Raspberry Pi ROS2 stack, which serves the mobile app (MQTT), desktop client (ROS2 DDS), and offline CV tooling."
  );
}

/* ───────────────────────────── 2. END-TO-END DATA FLOW (overview) ──────── */
{
  let b = "";
  const row = (x, y, w, lines, cls) => box(x, y, w, 50, lines, cls);
  // pipeline left→right
  b += row(20, 30, 150, [{ t: "webcam_node", cls: "t" }], "node-accent");
  b += row(230, 30, 175, [{ t: "yolo_detection", cls: "t" }], "node-blue");
  b += row(465, 30, 160, [{ t: "mqtt_bridge", cls: "t" }], "node-blue");
  b += row(685, 30, 150, [{ t: "Mobile App", cls: "t" }], "node");
  b += edge([[170, 55], [226, 55]], { cls: "edge-accent", marker: "ar-a", label: "color_jpeg", lx: 172, ly: 47 });
  b += edge([[405, 55], [461, 55]], { cls: "edge-blue", marker: "ar-b", label: "results/img", lx: 408, ly: 47 });
  b += edge([[625, 55], [681, 55]], { cls: "edge-blue", marker: "ar-b", label: "robot/*", lx: 630, ly: 47 });

  // motion path
  b += row(20, 150, 150, [{ t: "/cmd_vel", cls: "t" }], "node");
  b += row(230, 150, 175, [{ t: "spi_controller_bridge", cls: "t" }], "node-accent");
  b += row(465, 150, 160, [{ t: "Arduino Uno", cls: "t" }], "node");
  b += row(685, 150, 150, [{ t: "4 DC motors", cls: "t" }], "node-accent");
  b += edge([[170, 175], [226, 175]], { cls: "edge", marker: "ar" });
  b += edge([[405, 175], [461, 175]], { cls: "edge-accent", marker: "ar-a", label: "6-byte serial", lx: 408, ly: 167 });
  b += edge([[625, 175], [681, 175]], { cls: "edge-accent", marker: "ar-a", label: "PWM", lx: 636, ly: 167 });

  // sensing path
  b += row(20, 270, 150, [{ t: "Arduino Nano", cls: "t" }], "node");
  b += row(230, 270, 175, [{ t: "imu_node", cls: "t" }], "node-accent");
  b += row(465, 270, 160, [{ t: "robot_controller", cls: "t" }], "node");
  b += row(685, 270, 150, [{ t: "/odom · /imu/data", cls: "t-dim" }], "node");
  b += edge([[170, 295], [226, 295]], { cls: "edge", marker: "ar", label: "A:/G:", lx: 176, ly: 287 });
  b += edge([[405, 295], [461, 295]], { cls: "edge", marker: "ar", label: "/imu/data", lx: 408, ly: 287 });
  b += edge([[625, 295], [681, 295]], { cls: "edge", marker: "ar", label: "/cmd_vel_safe", lx: 628, ly: 287 });

  D.dataFlow = svg(
    855, 340,
    "End-to-end data flow across three pipelines",
    b,
    "Three pipelines: vision (top), motion (middle), and sensing/safety (bottom). Green edges carry actuation/camera data; blue edges carry vision & MQTT."
  );
}

/* ───────────────────────────── 3. ROS2 NODE GRAPH (ros2-stack) ─────────── */
{
  let b = "";
  b += box(30, 30, 160, 54, [{ t: "webcam_node", cls: "t" }, { t: "camera_sensor", cls: "t-dim" }], "node-accent");
  b += box(330, 30, 180, 54, [{ t: "yolo_detection_node", cls: "t" }, { t: "yolo_detection", cls: "t-dim" }], "node-blue");
  b += box(660, 130, 180, 54, [{ t: "mqtt_bridge_node", cls: "t" }, { t: "mqtt_bridge", cls: "t-dim" }], "node-blue");
  b += box(700, 30, 120, 54, [{ t: "MQTT broker", cls: "t" }], "node-warn");

  b += box(30, 190, 190, 54, [{ t: "spi_controller_bridge", cls: "t" }, { t: "motor_control", cls: "t-dim" }], "node-accent");
  b += box(360, 190, 150, 54, [{ t: "Arduino Uno", cls: "t" }], "node");
  b += box(30, 330, 160, 54, [{ t: "imu_node", cls: "t" }, { t: "imu_sensor", cls: "t-dim" }], "node-accent");
  b += box(360, 330, 150, 54, [{ t: "Arduino Nano", cls: "t" }], "node");

  // camera edges
  b += edge([[190, 50], [326, 50]], { cls: "edge-accent", marker: "ar-a", label: "/camera/color_jpeg", lx: 196, ly: 42 });
  b += edge([[110, 84], [110, 130], [320, 130]], { cls: "edge", marker: "ar", label: "/camera/raw (gray4)", lx: 130, ly: 124 });

  // yolo -> mqtt
  b += edge([[510, 50], [600, 50], [600, 145], [656, 145]], { cls: "edge-blue", marker: "ar-b", label: "/camera/detection", lx: 520, ly: 42 });
  b += edge([[510, 64], [580, 64], [580, 165], [656, 165]], { cls: "edge-blue", marker: "ar-b", label: "/detections/results", lx: 520, ly: 100 });

  // cmd_vel -> bridge -> Uno
  b += `<text class="t-dim" x="30" y="180">/cmd_vel ►</text>`;
  b += edge([[220, 216], [356, 216]], { cls: "edge-accent", marker: "ar-a", label: "serial 6-byte", lx: 244, ly: 208 });

  // Nano -> imu_node
  b += edge([[360, 356], [194, 356]], { cls: "edge", marker: "ar", label: "serial A:/G:", lx: 228, ly: 348 });
  // imu_node -> /imu/data -> mqtt
  b += edge([[190, 340], [620, 340], [620, 188]], { cls: "edge", marker: "ar", label: "/imu/data", lx: 300, ly: 332 });
  // laser back
  b += edge([[110, 384], [110, 410], [435, 410], [435, 386]], { cls: "edge", marker: "ar", label: "/laser/set · /laser/cmd", lx: 140, ly: 406 });
  // mqtt -> broker
  b += edge([[750, 130], [750, 86]], { cls: "edge-blue", marker: "ar-b", label: "MQTT", lx: 758, ly: 112 });

  D.ros2Graph = svg(
    870, 430,
    "ROS2 node dataflow graph",
    b,
    "Production node graph from robot.launch.py plus the YOLO node. Green = motor/camera path, blue = vision/MQTT path."
  );
}

/* ───────────────────────────── 4. DESKTOP STARTUP FLOW (desktop) ───────── */
{
  let b = "";
  b += box(300, 20, 220, 48, [{ t: "src/main.cpp", cls: "t" }], "node");
  b += box(300, 110, 220, 48, [{ t: "Application", cls: "t" }, { t: "facade · wiring", cls: "t-dim" }], "node-accent");
  b += edge([[410, 68], [410, 106]], { cls: "edge", marker: "ar" });

  b += box(40, 220, 220, 56, [{ t: "ROS2Interface", cls: "t" }, { t: "pub/sub · thread boundary", cls: "t-dim" }], "node-blue");
  b += box(300, 220, 220, 56, [{ t: "DigitalTwin", cls: "t" }, { t: "modes · state · sim", cls: "t-dim" }], "node");
  b += box(560, 220, 220, 56, [{ t: "MainWindow + WidgetManager", cls: "t" }, { t: "menus · docks · widgets", cls: "t-dim" }], "node-accent");

  b += edge([[410, 158], [150, 158], [150, 216]], { cls: "edge", marker: "ar" });
  b += edge([[410, 158], [410, 216]], { cls: "edge", marker: "ar" });
  b += edge([[410, 158], [670, 158], [670, 216]], { cls: "edge", marker: "ar" });

  D.desktopFlow = svg(
    820, 300,
    "Desktop client startup and dependency-injection flow",
    b,
    "Startup order: main.cpp boots Application, which initialises ROS2Interface → DigitalTwin → MainWindow + WidgetManager in dependency order."
  );
}

// ── inject into markdown between markers ──────────────────────────────────
const injections = [
  { file: "01-overview.md", key: "systemArch", marker: "SYSTEM-ARCH" },
  { file: "01-overview.md", key: "dataFlow", marker: "DATA-FLOW" },
  { file: "05-ros2-stack.md", key: "ros2Graph", marker: "ROS2-GRAPH" },
  { file: "07-desktop-client.md", key: "desktopFlow", marker: "DESKTOP-FLOW" },
];

for (const inj of injections) {
  const path = join(CONTENT, inj.file);
  let md = readFileSync(path, "utf8");
  const begin = `<!-- DIAGRAM:${inj.marker}:BEGIN -->`;
  const end = `<!-- DIAGRAM:${inj.marker}:END -->`;
  const block = `${begin}\n${D[inj.key]}\n${end}`;
  const re = new RegExp(`${begin}[\\s\\S]*?${end}`);
  if (re.test(md)) {
    md = md.replace(re, block);
    writeFileSync(path, md);
    console.log(`updated ${inj.file} :: ${inj.marker}`);
  } else {
    console.log(`!! marker not found in ${inj.file} :: ${inj.marker} (add ${begin} ... ${end})`);
  }
}
console.log("done.");
