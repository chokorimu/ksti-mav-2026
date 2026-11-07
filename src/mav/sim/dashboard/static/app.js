// MAV sim dashboard — browser frontend.
//
// Plain ES module: no framework, no bundler, no build step. three.js r160 (MIT)
// is vendored next to this file and imported as an ES module. Nothing is
// loaded from a CDN — the robot runs offline.
//
// ---------------------------------------------------------------------------
// HTTP contract (see src/mav/sim/dashboard/state.py and server.py)
// ---------------------------------------------------------------------------
// GET  /state.json   {"ready":bool, "step", "t", "scenario", "robot_state",
//                     "checkpoints", "pose":{"x","y","theta"}, "lookahead_m",
//                     "true_offset_m",
//                     "measurement":{"offset_m","curvature_inv_m","confidence",
//                                    "n_points","lookahead_m"} | null,
//                     "debug":{"h","w","roi_top","reason","sigma","threshold",
//                              "light_frac","dark_frac","polarity","mask_pixels",
//                              "n_bands_with_pixels","band_points":[[row,x,px]],
//                              "fit":{"a","b","c"},"resid","lookahead_row",
//                              "lookahead_x"},
//                     "command":{"speed","curvature"},
//                     "metrics":{"detected","lost","departures","max_abs_offset",
//                                "mean_abs_offset","stop_frames","detection_rate",
//                                "distance_m","time_s","crashed","finished"},
//                     "fps"}
//                     (when ready is false this is only {"ready":false})
// GET  /lane.json    {"ready", "scenario", "lane":[[x,y]], "trail":[[x,y,theta]],
//                     "lane_width_m", "corridor_m", "pose", "lookahead_m"}
// GET  /frame.jpg    JPEG of the newest camera frame, optional ?w=<px>
// GET  /scenarios    {"scenarios":[{"name","description"}]}
// POST /control      {"action":"start"|"emergency"|"reset"|"scenario",
//                     "scenario":"<name>"}  ->  {"ok":true,"action","scenario"}
//                     or {"ok":false,"error":"..."} with HTTP 400
//
// World conventions (src/mav/sim/world.py): X right, Y forward, heading theta
// left-positive, curvature > 0 = left. Offsets are +right. Scene coordinates
// ARE world coordinates, so no axis remapping happens anywhere below.
// ---------------------------------------------------------------------------

import * as THREE from './three.module.js';

// ===========================================================================
// small helpers
// ===========================================================================

const $ = (id) => document.getElementById(id);

const isNum = (v) => typeof v === 'number' && Number.isFinite(v);
const isStr = (v) => typeof v === 'string';
const numOr = (v, d) => (isNum(v) ? v : d);

function fmt(v, digits = 2, dash = '--') {
  return isNum(v) ? v.toFixed(digits) : dash;
}
function fmtS(v, digits = 2, dash = '--') {
  return isNum(v) ? (v >= 0 ? '+' : '') + v.toFixed(digits) : dash;
}
function fmtInt(v, dash = '--') {
  return isNum(v) ? String(Math.round(v)) : dash;
}
const pct = (v) => (isNum(v) ? (100 * v).toFixed(1) + '%' : '--');

/** fetch() that throws on a non-2xx status, so callers see one error path. */
async function getJSON(url) {
  const r = await fetch(url, { cache: 'no-store' });
  if (!r.ok) throw new Error(`${url} -> HTTP ${r.status}`);
  return r.json();
}

/** One self-restarting poll loop. A failed tick never kills the loop. */
function pollLoop(periodMs, tick) {
  let timer = 0;
  let stopped = false;
  const run = async () => {
    if (stopped) return;
    try {
      await tick();
    } catch (err) {
      console.warn('[dashboard]', err);
    }
    if (!stopped) timer = setTimeout(run, periodMs);
  };
  run();
  return () => { stopped = true; clearTimeout(timer); };
}

// ===========================================================================
// DOM references
// ===========================================================================

const el = {
  chipReady: $('chip-ready'),
  chipState: $('chip-state'),
  chipCp: $('chip-cp'),
  chipScenario: $('chip-scenario'),
  chipFps: $('chip-fps'),

  arenaView: $('arena-view'),
  arenaCanvas: $('arena'),
  arenaWaiting: $('arena-waiting'),
  hudPose: $('hud-pose'),
  btnFit: $('btn-fit'),

  camWrap: $('cam-wrap'),
  frameImg: $('frame'),
  overlay: $('overlay'),
  camMsg: $('cam-msg'),
  pxm: $('pxm'),
  lkRow: $('lk-row'),
  lkX: $('lk-x'),
  lkTrue: $('lk-true'),

  btnStart: $('btn-start'),
  btnEmergency: $('btn-emergency'),
  btnReset: $('btn-reset'),
  selScenario: $('sel-scenario'),
  controlReply: $('control-reply'),

  nums: $('nums'),
};

// ===========================================================================
// shared state between the three poll loops
// ===========================================================================

const latest = {
  state: null,      // /state.json payload
  lane: null,       // /lane.json payload
  frameW: 640,      // from debug.w (never hardcoded as a constant: updated live)
  frameH: 360,      // from debug.h
  pxPerM: null,     // derived once from the first good frame (see below)
};

// ===========================================================================
// PANEL 1 — 3D top-view arena (three.js)
// ===========================================================================

const CAR_W = 0.18;          // body width  (m, along X)
const CAR_L = 0.28;          // body length (m, along Y)
const WHEEL_BASE = 0.15;     // m — matches the vehicle params used for steering
const WHEEL_R = 0.032;
const WHEEL_W = 0.018;
const TRACK = 0.13;          // axle track width
const TRAIL_MAX = 400;       // runner caps the trail at 400 poses

const COL = {
  floor: 0x0d1117,
  grid: 0x1a2330,
  gridMajor: 0x24344a,
  laneRibbon: 0x1f6feb,
  laneLine: 0x6ea8fe,
  corridor: 0x8b5cf6,
  trailNew: 0x4dd0e1,
  trailOld: 0x14414d,
  car: 0xe6edf6,
  nose: 0x4dd0e1,
  wheel: 0x1b2430,
  look: 0x4dd0e1,
  meas: 0x4ade80,
  true: 0xf0b429,
};

const arena = {
  ready: false,
  fitMode: false,       // false = follow the car, true = fit the whole track
  viewH: 7.0,           // world metres visible vertically (follow mode)
  laneSig: '',
};

let renderer;
let scene;
let camera;
let carGroup;
let wheelFL;
let wheelFR;
let wheelRL;
let wheelRR;
let laneLine;
let laneRibbon;
let corridorRibbon;
let trailLine;
let lookRing;
let lookStem;
let measMarker;
let trueMarker;

function initArena() {
  const canvas = el.arenaCanvas;

  renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setClearColor(0x080b11, 1);
  // Cap the pixel ratio: the Jetson drives a modest GPU and the arena is flat.
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));

  scene = new THREE.Scene();

  // Orthographic camera looking straight down -Z. Screen right = +X,
  // screen up = +Y, so scene coordinates are world coordinates directly.
  camera = new THREE.OrthographicCamera(-5, 5, 5, -5, 0.1, 200);
  camera.position.set(0, 0, 60);
  camera.up.set(0, 1, 0);
  camera.lookAt(0, 0, 0);

  scene.add(buildFloor());
  scene.add(buildLaneRibbons());
  scene.add(buildCentreLine());
  scene.add(buildTrail());
  scene.add(buildLookahead());
  carGroup = buildCar();
  scene.add(carGroup);

  resizeArena();
  window.addEventListener('resize', resizeArena);
  requestAnimationFrame(renderArena);
}

function buildFloor() {
  const g = new THREE.Group();

  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(200, 200),
    new THREE.MeshBasicMaterial({ color: COL.floor })
  );
  floor.rotation.x = -Math.PI / 2;   // lay the XZ plane down into XY
  floor.position.z = -0.03;
  g.add(floor);

  // GridHelper is built in XZ; rotate it into XY as well.
  const grid = new THREE.GridHelper(60, 60, COL.gridMajor, COL.grid);
  grid.rotation.x = Math.PI / 2;
  grid.position.z = -0.02;
  g.add(grid);

  return g;
}

/**
 * Ribbons are flat triangle strips offset from the centreline by +-half width.
 * Rebuilt only when the lane polyline actually changes (4 Hz poll, cheap).
 */
function buildRibbon(points, halfWidth, color, opacity, z) {
  const geo = new THREE.BufferGeometry();
  const n = points.length;
  if (n < 2) return new THREE.Mesh(geo, new THREE.MeshBasicMaterial());

  const pos = new Float32Array(n * 2 * 3);
  const idx = [];
  for (let i = 0; i < n; i++) {
    // Central-difference tangent, then the in-plane normal.
    const a = points[Math.max(0, i - 1)];
    const b = points[Math.min(n - 1, i + 1)];
    let tx = b[0] - a[0];
    let ty = b[1] - a[1];
    const len = Math.hypot(tx, ty) || 1;
    tx /= len;
    ty /= len;
    const nx = -ty;   // left normal
    const ny = tx;
    const px = points[i][0];
    const py = points[i][1];
    pos.set([px + nx * halfWidth, py + ny * halfWidth, z], i * 6);
    pos.set([px - nx * halfWidth, py - ny * halfWidth, z], i * 6 + 3);
    if (i > 0) {
      const k = (i - 1) * 2;
      idx.push(k, k + 1, k + 3, k, k + 3, k + 2);
    }
  }
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setIndex(idx);
  geo.computeBoundingSphere();

  return new THREE.Mesh(
    geo,
    new THREE.MeshBasicMaterial({
      color, transparent: true, opacity,
      side: THREE.DoubleSide, depthWrite: false,
    })
  );
}

function buildLaneRibbons() {
  const g = new THREE.Group();
  // Filled in by updateLaneGeometry(); empty placeholders for now.
  corridorRibbon = new THREE.Mesh(
    new THREE.BufferGeometry(),
    new THREE.MeshBasicMaterial({
      color: COL.corridor, transparent: true, opacity: 0.10,
      side: THREE.DoubleSide, depthWrite: false,
    })
  );
  laneRibbon = new THREE.Mesh(
    new THREE.BufferGeometry(),
    new THREE.MeshBasicMaterial({
      color: COL.laneRibbon, transparent: true, opacity: 0.30,
      side: THREE.DoubleSide, depthWrite: false,
    })
  );
  corridorRibbon.position.z = 0.0;
  laneRibbon.position.z = 0.004;
  g.add(corridorRibbon, laneRibbon);
  return g;
}

function buildCentreLine() {
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(0), 3));
  laneLine = new THREE.Line(
    geo,
    new THREE.LineBasicMaterial({ color: COL.laneLine, transparent: true, opacity: 0.9 })
  );
  laneLine.position.z = 0.008;
  return laneLine;
}

function buildTrail() {
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(0), 3));
  geo.setAttribute('color', new THREE.BufferAttribute(new Float32Array(0), 3));
  trailLine = new THREE.Line(
    geo,
    new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.95 })
  );
  trailLine.position.z = 0.012;
  trailLine.frustumCulled = false;
  return trailLine;
}

function buildLookahead() {
  const g = new THREE.Group();

  lookRing = new THREE.Mesh(
    new THREE.RingGeometry(0.085, 0.125, 28),
    new THREE.MeshBasicMaterial({
      color: COL.look, transparent: true, opacity: 0.85,
      side: THREE.DoubleSide, depthWrite: false,
    })
  );
  lookRing.position.z = 0.02;

  lookStem = new THREE.Line(
    new THREE.BufferGeometry().setAttribute(
      'position', new THREE.BufferAttribute(new Float32Array(6), 3)),
    new THREE.LineDashedMaterial({
      color: COL.look, transparent: true, opacity: 0.4, dashSize: 0.08, gapSize: 0.06,
    })
  );
  lookStem.position.z = 0.016;
  lookStem.frustumCulled = false;

  // Measured offset: filled disc. True offset: hollow ring.
  measMarker = new THREE.Mesh(
    new THREE.CircleGeometry(0.05, 20),
    new THREE.MeshBasicMaterial({
      color: COL.meas, transparent: true, opacity: 0.95, depthWrite: false,
    })
  );
  measMarker.position.z = 0.024;

  trueMarker = new THREE.Mesh(
    new THREE.RingGeometry(0.04, 0.07, 20),
    new THREE.MeshBasicMaterial({
      color: COL.true, transparent: true, opacity: 0.95,
      side: THREE.DoubleSide, depthWrite: false,
    })
  );
  trueMarker.position.z = 0.024;

  g.add(lookRing, lookStem, measMarker, trueMarker);
  return g;
}

function buildCar() {
  const g = new THREE.Group();

  const body = new THREE.Mesh(
    new THREE.BoxGeometry(CAR_W, CAR_L, 0.045),
    new THREE.MeshBasicMaterial({ color: COL.car })
  );
  body.position.z = 0.05;
  g.add(body);

  // Nose arrow: a flat triangle pointing along local +Y (the car's forward
  // axis, since forward = (-sin(theta), cos(theta))).
  const noseShape = new THREE.Shape();
  noseShape.moveTo(0, CAR_L / 2 + 0.075);
  noseShape.lineTo(0.062, CAR_L / 2 - 0.02);
  noseShape.lineTo(-0.062, CAR_L / 2 - 0.02);
  noseShape.closePath();
  const nose = new THREE.Mesh(
    new THREE.ShapeGeometry(noseShape),
    new THREE.MeshBasicMaterial({ color: COL.nose, side: THREE.DoubleSide })
  );
  nose.position.z = 0.056;
  g.add(nose);

  // One wheel = a thin disc whose axle runs along the car's lateral (X) axis.
  // A cylinder's axis is +Y, so rotate it a quarter turn about Z.
  const wheelGeo = new THREE.CylinderGeometry(WHEEL_R, WHEEL_R, WHEEL_W, 14);
  const wheelMat = new THREE.MeshBasicMaterial({ color: COL.wheel });

  const mkWheel = (x, y, steerable) => {
    // Outer group carries the steering angle; the inner mesh carries the
    // axle orientation. rotation.z of the disc is invisible (it is symmetric
    // about its own axis), so this is purely about tilting the wheel plane.
    const pivot = new THREE.Group();
    pivot.position.set(x, y, 0.05);
    const disc = new THREE.Mesh(wheelGeo, wheelMat);
    disc.rotation.z = Math.PI / 2;
    pivot.add(disc);
    g.add(pivot);
    return steerable ? pivot : null;
  };

  wheelFL = mkWheel(+TRACK / 2, +WHEEL_BASE / 2, true);
  wheelFR = mkWheel(-TRACK / 2, +WHEEL_BASE / 2, true);
  wheelRL = mkWheel(+TRACK / 2, -WHEEL_BASE / 2, false);
  wheelRR = mkWheel(-TRACK / 2, -WHEEL_BASE / 2, false);

  return g;
}

function resizeArena() {
  const w = Math.max(1, el.arenaView.clientWidth);
  const h = Math.max(1, el.arenaView.clientHeight);
  renderer.setSize(w, h, false);
}

/** Centre the orthographic frustum on (cx, cy) showing viewH metres of height. */
function applyView(cx, cy, viewH) {
  const aspect = Math.max(0.2, el.arenaView.clientWidth / Math.max(1, el.arenaView.clientHeight));
  const halfH = Math.max(0.4, viewH) / 2;
  const halfW = halfH * aspect;
  camera.left = -halfW;
  camera.right = halfW;
  camera.top = halfH;
  camera.bottom = -halfH;
  camera.position.set(cx, cy, 60);
  camera.up.set(0, 1, 0);
  camera.lookAt(cx, cy, 0);
  camera.updateProjectionMatrix();
}

function laneSignature(lane) {
  if (!Array.isArray(lane) || lane.length < 2) return '';
  const a = lane[0];
  const b = lane[lane.length - 1];
  return `${lane.length}:${a[0]},${a[1]}:${b[0]},${b[1]}`;
}

function updateLaneGeometry(laneData) {
  const lane = Array.isArray(laneData && laneData.lane) ? laneData.lane : [];
  const sig = laneSignature(lane);
  if (sig && sig === arena.laneSig) return;   // unchanged: keep the buffers
  arena.laneSig = sig;

  const pts = lane.filter((p) => Array.isArray(p) && isNum(p[0]) && isNum(p[1]));
  const halfLane = numOr(laneData && laneData.lane_width_m, 0.5) / 2;
  const halfCorr = numOr(laneData && laneData.corridor_m, 0.6) / 2;

  // Ribbons
  const ribbon = buildRibbon(pts, halfLane, COL.laneRibbon, 0.3, 0);
  laneRibbon.geometry.dispose();
  laneRibbon.geometry = ribbon.geometry;

  const corr = buildRibbon(pts, halfCorr, COL.corridor, 0.1, 0);
  corridorRibbon.geometry.dispose();
  corridorRibbon.geometry = corr.geometry;

  // Centreline
  const pos = new Float32Array(pts.length * 3);
  pts.forEach((p, i) => pos.set([p[0], p[1], 0.008], i * 3));
  laneLine.geometry.dispose();
  const lg = new THREE.BufferGeometry();
  lg.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  laneLine.geometry = lg;
}

function updateTrail(trail) {
  const pts = (Array.isArray(trail) ? trail : [])
    .filter((p) => Array.isArray(p) && isNum(p[0]) && isNum(p[1]))
    .slice(-TRAIL_MAX);
  if (pts.length < 2) {
    trailLine.visible = false;
    return;
  }
  trailLine.visible = true;

  const n = pts.length;
  const pos = new Float32Array(n * 3);
  const col = new Float32Array(n * 3);
  const cNew = new THREE.Color(COL.trailNew);
  const cOld = new THREE.Color(COL.trailOld);
  const tmp = new THREE.Color();
  for (let i = 0; i < n; i++) {
    pos.set([pts[i][0], pts[i][1], 0.012], i * 3);
    // Colour by recency: index 0 = oldest, n-1 = newest.
    const f = n > 1 ? i / (n - 1) : 1;
    tmp.copy(cOld).lerp(cNew, f);
    col.set([tmp.r, tmp.g, tmp.b], i * 3);
  }
  trailLine.geometry.dispose();
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  g.setAttribute('color', new THREE.BufferAttribute(col, 3));
  trailLine.geometry = g;
}

function updateArena(state, laneData) {
  arena.ready = !!(state && state.ready) || !!(laneData && laneData.ready);
  el.arenaWaiting.classList.toggle('show', !arena.ready);

  if (laneData && laneData.ready) updateLaneGeometry(laneData);
  if (laneData && Array.isArray(laneData.trail)) updateTrail(laneData.trail);

  // Pose: prefer the fresher /state.json, fall back to /lane.json.
  const pose = (state && state.pose) || (laneData && laneData.pose) || null;
  const x = numOr(pose && pose.x, 0);
  const y = numOr(pose && pose.y, 0);
  const theta = numOr(pose && pose.theta, 0);

  carGroup.position.set(x, y, 0);
  carGroup.rotation.z = theta;   // forward = (-sin, cos) — matches the world

  // Front-wheel steering: steer = atan(wheel_base * curvature).
  const curvature = numOr(state && state.command && state.command.curvature, 0);
  const steer = Math.atan(WHEEL_BASE * curvature);
  if (wheelFL) wheelFL.rotation.z = steer;   // +steer turns left (-X)
  if (wheelFR) wheelFR.rotation.z = steer;

  // Lookahead markers: base point ahead of the car along its heading, then
  // shifted laterally by the offset (+right).
  const look = numOr(
    state && state.lookahead_m,
    numOr(laneData && laneData.lookahead_m, 1.0)
  );
  const fx = -Math.sin(theta);
  const fy = Math.cos(theta);
  const rx = Math.cos(theta);   // right vector
  const ry = Math.sin(theta);
  const bx = x + fx * look;
  const by = y + fy * look;

  lookRing.position.set(bx, by, 0.02);
  const stem = lookStem.geometry.getAttribute('position');
  stem.setXYZ(0, x, y, 0.016);
  stem.setXYZ(1, bx, by, 0.016);
  stem.needsUpdate = true;
  lookStem.computeLineDistances();

  const meas = state && state.measurement;
  const measOff = meas ? numOr(meas.offset_m, null) : null;
  const trueOff = isNum(state && state.true_offset_m) ? state.true_offset_m : null;

  measMarker.visible = measOff !== null;
  if (measMarker.visible) measMarker.position.set(bx + rx * measOff, by + ry * measOff, 0.024);

  trueMarker.visible = trueOff !== null;
  if (trueMarker.visible) trueMarker.position.set(bx + rx * trueOff, by + ry * trueOff, 0.024);

  el.hudPose.textContent =
    `x ${fmtS(x, 2)}  y ${fmtS(y, 2)}  theta ${fmtS(theta, 2)} rad  look ${fmt(look, 2)} m`;
}

function renderArena() {
  requestAnimationFrame(renderArena);
  if (!arena.ready) {
    renderer.render(scene, camera);
    return;
  }

  const laneData = latest.lane;
  const state = latest.state;
  const pose = (state && state.pose) || (laneData && laneData.pose) || { x: 0, y: 0 };

  if (arena.fitMode) {
    // Fit the whole track (plus the car) with a small margin.
    const lane = Array.isArray(laneData && laneData.lane) ? laneData.lane : [];
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    for (const p of lane) {
      if (!Array.isArray(p) || !isNum(p[0]) || !isNum(p[1])) continue;
      minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]);
      minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]);
    }
    minX = Math.min(minX, numOr(pose.x, 0)); maxX = Math.max(maxX, numOr(pose.x, 0));
    minY = Math.min(minY, numOr(pose.y, 0)); maxY = Math.max(maxY, numOr(pose.y, 0));
    if (Number.isFinite(minX) && maxX > minX && maxY > minY) {
      const pad = 0.6;
      const w = (maxX - minX) + 2 * pad;
      const h = (maxY - minY) + 2 * pad;
      applyView((minX + maxX) / 2, (minY + maxY) / 2, Math.max(h, 1.0));
    } else {
      applyView(numOr(pose.x, 0), numOr(pose.y, 0), arena.viewH);
    }
  } else {
    applyView(numOr(pose.x, 0), numOr(pose.y, 0), arena.viewH);
  }

  renderer.render(scene, camera);
}

el.btnFit.addEventListener('click', () => {
  arena.fitMode = !arena.fitMode;
  el.btnFit.textContent = arena.fitMode ? 'follow the car' : 'fit whole track';
  el.btnFit.classList.toggle('on', arena.fitMode);
});

// ===========================================================================
// PANEL 2 — camera frame + detector overlay
// ===========================================================================

const OV = {
  det: '#4ade80',
  true: '#f0b429',
  band: '#38bdf8',
  roi: '#a78bfa',
  fit: '#4ade80',
  lost: '#ff6b6b',
  text: '#c9d4e3',
};

function resizeOverlay() {
  const w = Math.max(1, el.camWrap.clientWidth);
  const h = Math.max(1, el.camWrap.clientHeight);
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  el.overlay.width = Math.round(w * dpr);
  el.overlay.height = Math.round(h * dpr);
}
window.addEventListener('resize', resizeOverlay);

/** Draw the detector overlay in frame-pixel coordinates. */
function drawOverlay(state) {
  const cv = el.overlay;
  const ctx = cv.getContext('2d');
  if (!ctx) return;

  const fw = latest.frameW;
  const fh = latest.frameH;
  // Map frame pixels -> canvas pixels (x and y independently, so a slightly
  // different display aspect still lines up).
  ctx.setTransform(cv.width / fw, 0, 0, cv.height / fh, 0, 0);
  ctx.clearRect(0, 0, fw, fh);
  ctx.lineWidth = 1;

  const dbg = (state && state.debug) || {};
  const meas = state && state.measurement;
  const roiTop = isNum(dbg.roi_top) ? dbg.roi_top : null;

  // -- ROI top line --------------------------------------------------------
  if (roiTop !== null) {
    ctx.strokeStyle = OV.roi;
    ctx.globalAlpha = 0.55;
    ctx.setLineDash([6, 5]);
    ctx.beginPath();
    ctx.moveTo(0, roiTop);
    ctx.lineTo(fw, roiTop);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;
    ctx.fillStyle = OV.roi;
    ctx.font = '10px monospace';
    ctx.fillText(`roi_top ${roiTop}`, 4, Math.max(10, roiTop - 4));
  }

  // -- band centroids: [row, x, pixels], rows are ROI-relative --------------
  const bands = Array.isArray(dbg.band_points) ? dbg.band_points : [];
  for (const b of bands) {
    if (!Array.isArray(b) || !isNum(b[0]) || !isNum(b[1])) continue;
    const row = (roiTop !== null ? roiTop : 0) + b[0];
    const px = isNum(b[2]) ? b[2] : 0;
    const r = 2 + Math.min(4, Math.sqrt(Math.max(0, px)) / 14);
    ctx.fillStyle = OV.band;
    ctx.globalAlpha = 0.35 + 0.5 * Math.min(1, px / 400);
    ctx.beginPath();
    ctx.arc(b[1], row, r, 0, Math.PI * 2);
    ctx.fill();
    ctx.globalAlpha = 1;
  }

  // -- fitted quadratic ----------------------------------------------------
  // The fit is in ROI-RELATIVE rows: for a full-frame row r the fitted column
  // is x = a*(r - roi_top)^2 + b*(r - roi_top) + c.
  const fit = dbg.fit;
  if (fit && isNum(fit.a) && isNum(fit.b) && isNum(fit.c) && roiTop !== null) {
    ctx.strokeStyle = OV.fit;
    ctx.globalAlpha = 0.9;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    let started = false;
    for (let r = roiTop; r <= fh - 1; r += 2) {
      const d = r - roiTop;
      const x = fit.a * d * d + fit.b * d + fit.c;
      if (!started) { ctx.moveTo(x, r); started = true; }
      else ctx.lineTo(x, r);
    }
    ctx.stroke();
    ctx.globalAlpha = 1;
    ctx.lineWidth = 1;
  }

  // -- lookahead row + detected / true centre ------------------------------
  const lkRow = isNum(dbg.lookahead_row) ? dbg.lookahead_row : null;
  const lkX = isNum(dbg.lookahead_x) ? dbg.lookahead_x : null;

  if (lkRow !== null) {
    ctx.strokeStyle = OV.roi;
    ctx.globalAlpha = 0.8;
    ctx.beginPath();
    ctx.moveTo(0, lkRow);
    ctx.lineTo(fw, lkRow);
    ctx.stroke();
    ctx.globalAlpha = 1;

    // Vertical centre reference (the camera's optical axis).
    ctx.strokeStyle = '#3a4658';
    ctx.beginPath();
    ctx.moveTo(fw / 2, 0);
    ctx.lineTo(fw / 2, fh);
    ctx.stroke();

    if (lkX !== null) {
      ctx.fillStyle = OV.det;
      ctx.beginPath();
      ctx.arc(lkX, lkRow, 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = OV.text;
      ctx.font = '10px monospace';
      ctx.fillText(`det ${fmt(lkX, 1)}px`, Math.min(lkX + 8, fw - 70), lkRow - 6);
    }

    // True centre: w/2 + true_offset_m * px_per_m. px_per_m is not served by
    // the API, so it is derived once from the first good frame (see
    // derivePxPerM) and cached; 130 is the fallback (LaneConfig default).
    const pxPerM = latest.pxPerM !== null ? latest.pxPerM : 130;
    const trueOff = state && isNum(state.true_offset_m) ? state.true_offset_m : null;
    if (trueOff !== null) {
      const tx = fw / 2 + trueOff * pxPerM;
      ctx.strokeStyle = OV.true;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(tx, lkRow, 5, 0, Math.PI * 2);
      ctx.stroke();
      ctx.lineWidth = 1;
      ctx.fillStyle = OV.true;
      ctx.font = '10px monospace';
      ctx.fillText(`true ${fmt(tx, 1)}px`, Math.min(tx + 8, fw - 74), lkRow + 14);
    }
  }

  // -- lane lost -----------------------------------------------------------
  if (!meas) {
    ctx.fillStyle = 'rgba(8,11,17,0.55)';
    ctx.fillRect(0, 0, fw, 26);
    ctx.fillStyle = OV.lost;
    ctx.font = 'bold 13px monospace';
    const reason = isStr(dbg.reason) && dbg.reason ? dbg.reason : 'no measurement';
    ctx.fillText(`LANE LOST — ${reason}`, 6, 18);
  } else {
    ctx.fillStyle = OV.det;
    ctx.font = '10px monospace';
    ctx.fillText(
      `off ${fmtS(meas.offset_m, 3)} m  curv ${fmtS(meas.curvature_inv_m, 2)}  ` +
      `conf ${fmt(meas.confidence, 2)}  n ${fmtInt(meas.n_points)}`,
      6, 12
    );
  }
}

/**
 * px_per_m is not in the JSON contract. Derive it once from a frame where the
 * detector actually saw a real offset: px_per_m = (lookahead_x - w/2) / offset_m.
 * Frames with a near-zero offset are skipped (the division is meaningless
 * there), and 130 (the LaneConfig default) is used until a good frame arrives.
 */
function derivePxPerM(state) {
  if (latest.pxPerM !== null) return;
  const dbg = (state && state.debug) || {};
  const meas = state && state.measurement;
  if (!meas || !isNum(meas.offset_m) || Math.abs(meas.offset_m) < 0.01) return;
  if (!isNum(dbg.lookahead_x) || !isNum(dbg.w)) return;
  const v = (dbg.lookahead_x - dbg.w / 2) / meas.offset_m;
  if (Number.isFinite(v) && v > 20 && v < 2000) {
    latest.pxPerM = v;
    el.pxm.textContent = v.toFixed(1);
  }
}

function updateCameraPanel(state) {
  const dbg = (state && state.debug) || {};
  if (isNum(dbg.w) && dbg.w > 0) latest.frameW = dbg.w;
  if (isNum(dbg.h) && dbg.h > 0) latest.frameH = dbg.h;

  derivePxPerM(state);
  drawOverlay(state);

  el.lkRow.textContent = fmt(dbg.lookahead_row, 1);
  el.lkX.textContent = fmt(dbg.lookahead_x, 1);
  const pxPerM = latest.pxPerM !== null ? latest.pxPerM : 130;
  const trueOff = state && isNum(state.true_offset_m) ? state.true_offset_m : null;
  el.lkTrue.textContent = trueOff === null ? '--'
    : (latest.frameW / 2 + trueOff * pxPerM).toFixed(1);
  el.pxm.textContent = pxPerM.toFixed(1);
}

// ===========================================================================
// PANEL 3 — controls + debug numbers
// ===========================================================================

function setBusy(busy) {
  for (const b of [el.btnStart, el.btnEmergency, el.btnReset]) b.disabled = busy;
  el.selScenario.disabled = busy;
}

function reply(text, kind) {
  el.controlReply.textContent = text;
  el.controlReply.className = 'reply' + (kind ? ' ' + kind : '');
}

/** POST /control. Buttons stay disabled until the server answers. */
async function control(payload) {
  setBusy(true);
  try {
    const r = await fetch('/control', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    const body = await r.json().catch(() => ({}));
    if (!r.ok || !body.ok) {
      reply(`error: ${body.error || `HTTP ${r.status}`}`, 'err');
      return;
    }
    const extra = body.scenario ? ` (${body.scenario})` : '';
    reply(`ok: ${body.action}${extra}`, 'ok');
  } catch (err) {
    reply(`error: ${err.message || err}`, 'err');
  } finally {
    setBusy(false);
  }
}

el.btnStart.addEventListener('click', () => control({ action: 'start' }));
el.btnEmergency.addEventListener('click', () => control({ action: 'emergency' }));
el.btnReset.addEventListener('click', () => control({ action: 'reset' }));
el.selScenario.addEventListener('change', () => {
  const name = el.selScenario.value;
  if (name) control({ action: 'scenario', scenario: name });
});

// --- numbers panel ---------------------------------------------------------

// Each row: [label, value-getter, optional CSS class getter].
const NUM_ROWS = [
  ['offset_m', (s) => fmtS(s.measurement && s.measurement.offset_m, 3)],
  ['curvature 1/m', (s) => fmtS(s.measurement && s.measurement.curvature_inv_m, 3)],
  ['confidence', (s) => fmt(s.measurement && s.measurement.confidence, 3)],
  ['n_points', (s) => fmtInt(s.measurement && s.measurement.n_points)],
  ['true_offset_m', (s) => fmtS(s.true_offset_m, 3)],
  ['cmd speed m/s', (s) => fmt(s.command && s.command.speed, 3)],
  ['cmd curvature', (s) => fmtS(s.command && s.command.curvature, 4)],
  ['det sigma', (s) => fmt(s.debug && s.debug.sigma, 2)],
  ['det threshold', (s) => fmt(s.debug && s.debug.threshold, 2)],
  ['polarity', (s) => (s.debug && s.debug.polarity) || '--'],
  ['light_frac', (s) => pct(s.debug && s.debug.light_frac)],
  ['dark_frac', (s) => pct(s.debug && s.debug.dark_frac)],
  ['mask_pixels', (s) => fmtInt(s.debug && s.debug.mask_pixels)],
  ['bands w/ px', (s) => fmtInt(s.debug && s.debug.n_bands_with_pixels)],
  ['resid px', (s) => fmt(s.debug && s.debug.resid, 2)],
  ['detected', (s) => fmtInt(s.metrics && s.metrics.detected)],
  ['lost', (s) => fmtInt(s.metrics && s.metrics.lost)],
  ['departures', (s) => fmtInt(s.metrics && s.metrics.departures)],
  ['max |offset|', (s) => fmt(s.metrics && s.metrics.max_abs_offset, 3)],
  ['mean |offset|', (s) => fmt(s.metrics && s.metrics.mean_abs_offset, 3)],
  ['detection_rate', (s) => pct(s.metrics && s.metrics.detection_rate)],
  ['distance m', (s) => fmt(s.metrics && s.metrics.distance_m, 2)],
  ['time s', (s) => fmt(s.metrics && s.metrics.time_s, 2)],
  ['stop_frames', (s) => fmtInt(s.metrics && s.metrics.stop_frames)],
  ['sim step', (s) => fmtInt(s.step)],
  ['sim t', (s) => fmt(s.t, 2)],
  ['wall fps', (s) => fmt(s.fps, 1)],
];

const numCells = NUM_ROWS.map(([label, get]) => {
  const row = document.createElement('div');
  row.className = 'row';
  const k = document.createElement('span');
  k.className = 'k';
  k.textContent = label;
  const v = document.createElement('span');
  v.className = 'v';
  v.textContent = '--';
  row.append(k, v);
  el.nums.append(row);
  return { row, v, get };
});

// Two extra, prominent rows for the run outcome.
const outcomeRow = document.createElement('div');
outcomeRow.className = 'row wide';
outcomeRow.innerHTML = '<span class="k">run status</span><span class="v" id="v-outcome">--</span>';
el.nums.append(outcomeRow);
const outcomeCell = $('v-outcome');

function updateNumbers(state) {
  for (const c of numCells) c.v.textContent = c.get(state);

  const m = state.metrics || {};
  if (m.crashed) {
    outcomeCell.textContent = 'CRASHED — left the corridor';
    outcomeRow.className = 'row wide bad';
  } else if (m.finished) {
    outcomeCell.textContent = 'FINISHED';
    outcomeRow.className = 'row wide good';
  } else if (state.robot_state === 'stopped') {
    outcomeCell.textContent = 'STOPPED (emergency) — START resumes';
    outcomeRow.className = 'row wide hot';
  } else if (state.robot_state === 'wait_start') {
    outcomeCell.textContent = 'WAITING FOR START';
    outcomeRow.className = 'row wide hot';
  } else {
    outcomeCell.textContent = state.robot_state || '--';
    outcomeRow.className = 'row wide';
  }
}

function updateStatus(state) {
  const ready = !!(state && state.ready);
  el.chipReady.textContent = `ready: ${ready ? 'yes' : 'no'}`;
  el.chipReady.className = 'chip ' + (ready ? 'on' : 'off');

  el.chipState.textContent = `state: ${(state && state.robot_state) || '?'}`;
  el.chipState.className = 'chip ' + (
    (state && state.robot_state) === 'run' ? 'on' : 'warn'
  );
  el.chipCp.textContent = `checkpoints: ${fmtInt(state && state.checkpoints, '0')}`;
  el.chipScenario.textContent = `scenario: ${(state && state.scenario) || '?'}`;
  el.chipFps.textContent = `fps: ${fmt(state && state.fps, 1)}`;
}

// ===========================================================================
// poll loops — three independent chains, none blocks another
// ===========================================================================

pollLoop(100, async () => {                       // /state.json — 10 Hz
  const s = await getJSON('/state.json');
  latest.state = s;
  if (!s.ready) {
    // Server is up but the sim has not published a tick yet.
    el.arenaWaiting.textContent = 'waiting for the simulator…';
    el.arenaWaiting.classList.add('show');
    updateStatus(s);
    updateNumbers({ metrics: {}, debug: {}, robot_state: 'boot' });
    return;
  }
  el.arenaWaiting.classList.remove('show');
  updateStatus(s);
  updateNumbers(s);
  updateArena(s, latest.lane);
  updateCameraPanel(s);
});

pollLoop(100, async () => {                       // /frame.jpg — 10 Hz
  // Cache-buster only; the server parses `w=` out of the query string and
  // ignores anything else. No-store headers already come from the server.
  el.frameImg.src = `/frame.jpg?w=640&_=${Date.now()}`;
});

pollLoop(250, async () => {                       // /lane.json — 4 Hz
  const l = await getJSON('/lane.json');
  latest.lane = l;
  updateArena(latest.state, l);
});

// Scenario dropdown — loaded once, refreshed slowly in case of a restart.
pollLoop(5000, async () => {
  const d = await getJSON('/scenarios');
  const list = Array.isArray(d.scenarios) ? d.scenarios : [];
  const cur = el.selScenario.value;
  const sig = list.map((s) => s.name).join(',');
  if (el.selScenario.dataset.sig === sig) return;
  el.selScenario.dataset.sig = sig;
  el.selScenario.textContent = '';
  for (const s of list) {
    const o = document.createElement('option');
    o.value = s.name;
    o.textContent = s.description ? `${s.name} — ${s.description}` : s.name;
    el.selScenario.append(o);
  }
  if (cur) el.selScenario.value = cur;
});

// Frame load / error feedback.
el.frameImg.addEventListener('load', () => { el.camMsg.hidden = true; });
el.frameImg.addEventListener('error', () => {
  el.camMsg.hidden = false;
  el.camMsg.textContent = 'no frame yet — the simulator has not published a camera frame';
});

// ===========================================================================
// boot
// ===========================================================================

initArena();
resizeOverlay();
// First paint of the overlay even before any state arrives.
drawOverlay(null);
