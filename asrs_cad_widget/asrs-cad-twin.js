/**
 * =============================================================================
 * Proyek Perancangan IND-02 — ASRS 2.5D Photorealistic CAD Digital Twin Engine
 * =============================================================================
 * Self-contained, zero-dependency HTML5 Canvas 2D renderer + kinematic engine
 * designed to track REAL ESP32 / Firebase RTDB hardware motion in real time
 * (and also support offline preview cycles).
 *
 * Key Capabilities:
 *  - Dynamic 9-bay (A1–C3) stock rendering over an all-empty studio CAD backdrop
 *  - True isometric Column-by-Column x Row-by-Row Painter's Algorithm
 *    (Col 1 -> Wall 1 -> Col 2 -> Wall 2 -> Col 3 -> Right Frame) with
 *    zero wall-slicing, zero ceiling-chopping, and zero fork-piercing artifacts
 *  - Per-bay calibrated coordinate normalization (works cleanly even when
 *    individual slots have custom X/Z revolutions like A1=0.2, A2=1.0, z_offset=0.1)
 *  - Stage-aware live ESP32/Firebase telemetry tracking (`updateFromTelemetry()`)
 *    + immediate command dispatch tracking (`notifyCommand()`) so the 3D twin
 *    moves continuously and smoothly whenever the real ASRS moves
 *  - Interactive 3D bay hit-testing (`onBayClick(bayId)`)
 */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory();
  } else {
    root.AsrsCadTwin = factory();
  }
}(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  // ---------------------------------------------------------------------------
  // 1. CONSTANTS & 1200x896 STUDIO CAD CALIBRATION GEOMETRY
  // ---------------------------------------------------------------------------
  const NATIVE_W = 1200;
  const NATIVE_H = 896;

  // Visual Z clearance lift inside a cubby (~9.2px on screen so box never clips
  // into the solid shelf floor above it — ~5px visible headroom on all 9 bays!)
  const VISUAL_LIFT_REV = 0.135;

  // Crop viewports inside the 1200x896 studio render
  const CROP_MODES = {
    compact: { x: 142, y: 14, w: 868, h: 882 },
    wide:    { x: 40,  y: 12, w: 1120, h: 874 }
  };

  // All 16 pixel-locked PNG assets relative to `assetPath`
  const ASSET_MANIFEST = {
    backdrop:       'backdrop_empty.png',
    boxBay:         'sprite_box_bay.png',
    boxCarry:       'sprite_box_carry.png',
    carriage:       'sprite_carriage.png',
    leftMotor:      'sprite_left_motor.png',
    leftYellow:     'sprite_left_yellow.png',
    rightMotor:     'sprite_right_motor.png',
    beamTex:        'sprite_beam_tex.png',
    maskShelfA:     'mask_shelf_a.png',
    maskShelfB:     'mask_shelf_b.png',
    maskRoofC:      'mask_roof_c.png',
    maskWall1:      'mask_wall1.png',
    maskWall2:      'mask_wall2.png',
    maskTopBar:     'mask_top_bar.png',
    maskLeftRail:   'mask_left_rail.png',
    maskRightFrame: 'mask_right_frame.png'
  };

  // Native 1200x896 calibration per bay (A1..C3)
  // Canonical visual coordinates: Col 1/2/3 -> 2.0/5.0/8.0, Row A/B/C seated -> 1.10/3.60/6.10
  const BAY_INFO = {
    A1: { id: 'A1', row: 'A', col: 1, xRev: 2.0, zRev: 1.5, zSeatVis: 1.10, boxRest: { x: 363, y: 500 }, quad: [[338,444],[456,496],[456,664],[338,605]], labelPos: [396,472] },
    A2: { id: 'A2', row: 'A', col: 2, xRev: 5.0, zRev: 1.5, zSeatVis: 1.10, boxRest: { x: 486, y: 556 }, quad: [[462,498],[578,550],[578,724],[462,666]], labelPos: [520,528] },
    A3: { id: 'A3', row: 'A', col: 3, xRev: 8.0, zRev: 1.5, zSeatVis: 1.10, boxRest: { x: 609, y: 612 }, quad: [[584,552],[702,606],[702,786],[584,728]], labelPos: [642,584] },
    B1: { id: 'B1', row: 'B', col: 1, xRev: 2.0, zRev: 4.0, zSeatVis: 3.60, boxRest: { x: 363, y: 332 }, quad: [[338,278],[456,330],[456,494],[338,442]], labelPos: [396,306] },
    B2: { id: 'B2', row: 'B', col: 2, xRev: 5.0, zRev: 4.0, zSeatVis: 3.60, boxRest: { x: 486, y: 388 }, quad: [[462,332],[578,384],[578,548],[462,496]], labelPos: [520,360] },
    B3: { id: 'B3', row: 'B', col: 3, xRev: 8.0, zRev: 4.0, zSeatVis: 3.60, boxRest: { x: 609, y: 444 }, quad: [[584,386],[702,438],[702,604],[584,550]], labelPos: [642,416] },
    C1: { id: 'C1', row: 'C', col: 1, xRev: 2.0, zRev: 6.5, zSeatVis: 6.10, boxRest: { x: 363, y: 160 }, quad: [[338,112],[456,163],[456,328],[338,276]], labelPos: [396,142] },
    C2: { id: 'C2', row: 'C', col: 2, xRev: 5.0, zRev: 6.5, zSeatVis: 6.10, boxRest: { x: 486, y: 214 }, quad: [[462,165],[578,214],[578,380],[462,330]], labelPos: [520,192] },
    C3: { id: 'C3', row: 'C', col: 3, xRev: 8.0, zRev: 6.5, zSeatVis: 6.10, boxRest: { x: 609, y: 270 }, quad: [[584,216],[702,268],[702,436],[584,382]], labelPos: [642,244] }
  };

  // Column horizontal clip intervals (1200x896 space) for true per-column shelf occlusion
  const COL_CLIP_X = {
    1: [326, 463],
    2: [463, 585],
    3: [585, 718]
  };

  // Seated vertical perspective offset (boxRest.y - beamY) across (row, col)
  // Calibrated to exact carriage projection so boxCarry hands off to boxBay with 0.0px jump!
  const PERSP_DY_GRID = [
    [-96.3, -97.2, -95.7], // Row A seated (zVis = 1.10)
    [-94.3, -92.9, -89.2], // Row B seated (zVis = 3.60)
    [-96.3, -94.6, -88.8]  // Row C seated (zVis = 6.10)
  ];

  // Depth unit vector pointing into the rack cubbies (isometric Y-depth axis)
  const DEPTH_UX = 0.889;
  const DEPTH_UY = -0.458;

  // Retracted-to-seated fork travel along isometric depth axis (-22px, +11px when retracted)
  const FORK_TRAVEL_DX = 22;
  const FORK_TRAVEL_DY = -11;

  // ---------------------------------------------------------------------------
  // 2. MATH & ACCELSTEPPER KINEMATICS HELPERS
  // ---------------------------------------------------------------------------
  function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }
  function lerp(a, b, t) { return a + (b - a) * t; }
  function easeInOut(u) {
    u = clamp(u, 0, 1);
    return u < 0.5 ? 2 * u * u : 1 - Math.pow(-2 * u + 2, 2) / 2;
  }

  /**
   * AccelStepper symmetric trapezoidal motion profile for normalized u in [0, 1].
   * Returns { pos in [0,1], vel in [0,1] }.
   */
  function trapMotion(u, accelFrac = 0.22) {
    u = clamp(u, 0, 1);
    const a = clamp(accelFrac, 0.05, 0.49);
    const vMax = 1 / (1 - a);
    if (u < a) {
      return { pos: 0.5 * (vMax / a) * u * u, vel: u / a };
    }
    if (u <= 1 - a) {
      return { pos: 0.5 * vMax * a + vMax * (u - a), vel: 1.0 };
    }
    const d = 1 - u;
    return { pos: 1 - 0.5 * (vMax / a) * d * d, vel: d / a };
  }

  /**
   * Piecewise Z-axis horizontal crossbar endpoints in 1200x896 native CAD space.
   * Row A seated = 1.10, Row B seated = 3.60, Row C seated = 6.10 (+lift = 6.235).
   */
  function getBeamEndpoints(zVis) {
    const xL = 262;
    const xR = 695;
    const zc = clamp(zVis, 0, 6.32);
    let yL, yR;
    if (zc <= 1.10) {
      const t = clamp(zc / 1.10, 0, 1);
      yL = lerp(594, 546, t);
      yR = lerp(796, 746, t);
    } else {
      const u = clamp((zc - 1.10) / 5.0, 0, 1.06);
      yL = 546 - u * 336; // 67.2 px/rev -> Row C seated (6.10): 210
      yR = 746 - u * 352; // 70.4 px/rev -> Row C seated (6.10): 394
    }
    return { xL, yL, xR, yR };
  }

  /**
   * Piecewise X-axis carriage center along the slanted horizontal beam.
   * X = 0 -> 336 (cleanly inside left rail so carried box never clips left extrusion!),
   * Col 1 (2.0) -> 371, Col 2 (5.0) -> 494, Col 3 (8.0) -> 612 (5px clear of right blue Z-belt!).
   */
  function getCarriagePos(xVis, zVis) {
    const { xL, yL, xR, yR } = getBeamEndpoints(zVis);
    const xClamped = clamp(xVis, 0, 8.2);
    let cx;
    if (xClamped <= 2.0) {
      cx = 336 + xClamped * 17.5;
    } else if (xClamped <= 5.0) {
      cx = 371 + (xClamped - 2.0) * 41.0;
    } else {
      cx = 494 + (xClamped - 5.0) * 39.333;
    }
    const u = (cx - xL) / (xR - xL);
    const beamY = yL + u * (yR - yL);
    return { cx, beamY, u, slope: (yR - yL) / (xR - xL) };
  }

  /**
   * Horizontal carriage-to-boxRest offset (-8 at Col 1/2, smoothly -3 at Col 3 where cx=612 -> boxRest=609).
   */
  function getCarriedPerspDx(xVis) {
    if (xVis <= 5.0) return -8.0;
    return lerp(-8.0, -3.0, clamp((xVis - 5.0) / 3.0, 0, 1));
  }

  /**
   * Bilinearly interpolated carried-box vertical perspective offset across (xVis, zVis).
   */
  function getCarriedPerspDy(xVis, zVis) {
    const cPos = clamp((xVis - 2.0) / 3.0, 0, 2);
    const rPos = clamp((zVis - 1.10) / 2.5, 0, 2);
    const c0 = Math.floor(cPos), c1 = Math.min(2, c0 + 1), tc = cPos - c0;
    const r0 = Math.floor(rPos), r1 = Math.min(2, r0 + 1), tr = rPos - r0;
    const top = lerp(PERSP_DY_GRID[r0][c0], PERSP_DY_GRID[r0][c1], tc);
    const bot = lerp(PERSP_DY_GRID[r1][c0], PERSP_DY_GRID[r1][c1], tc);
    return lerp(top, bot, tr);
  }

  function pointInQuad(px, py, quad) {
    let inside = false;
    for (let i = 0, j = quad.length - 1; i < quad.length; j = i++) {
      const xi = quad[i][0], yi = quad[i][1];
      const xj = quad[j][0], yj = quad[j][1];
      const intersect = ((yi > py) !== (yj > py)) &&
        (px < (xj - xi) * (py - yi) / ((yj - yi) || 1e-6) + xi);
      if (intersect) inside = !inside;
    }
    return inside;
  }

  function roundRect(c, x, y, w, h, r) {
    c.beginPath();
    c.moveTo(x + r, y);
    c.arcTo(x + w, y, x + w, y + h, r);
    c.arcTo(x + w, y + h, x, y + h, r);
    c.arcTo(x, y + h, x, y, r);
    c.arcTo(x, y, x + w, y, r);
    c.closePath();
  }

  // ---------------------------------------------------------------------------
  // 3. MAIN DIGITAL TWIN ENGINE CLASS
  // ---------------------------------------------------------------------------
  class AsrsCadTwin {
    constructor(options = {}) {
      this.canvas = typeof options.canvas === 'string'
        ? document.querySelector(options.canvas)
        : options.canvas;
      if (!this.canvas) {
        throw new Error('AsrsCadTwin: valid `canvas` element or CSS selector is required.');
      }
      this.ctx = this.canvas.getContext('2d');

      this.options = {
        assetPath: options.assetPath !== undefined ? options.assetPath : './assets/',
        cropMode: options.cropMode === 'wide' ? 'wide' : 'compact',
        showTopChips: options.showTopChips !== false,
        showLegend: options.showLegend !== false,
        showBayLabels: !!options.showBayLabels,
        showMotionVectors: options.showMotionVectors !== false,
        showHomeMarker: options.showHomeMarker !== false,
        enableClickToSelect: options.enableClickToSelect !== false,
        autoStart: options.autoStart !== false,
        onBayClick: typeof options.onBayClick === 'function' ? options.onBayClick : null,
        onStateChange: typeof options.onStateChange === 'function' ? options.onStateChange : null,
        onReady: typeof options.onReady === 'function' ? options.onReady : null
      };

      if (this.options.assetPath && !this.options.assetPath.endsWith('/')) {
        this.options.assetPath += '/';
      }

      // Per-bay calibrated firmware coordinates (synced from WMS BOX_COORDINATES / Firebase)
      this.coordinates = {};
      for (const [id, info] of Object.entries(BAY_INFO)) {
        this.coordinates[id] = { x: info.xRev, z: info.zRev };
      }
      if (options.coordinates) this.setCoordinates(options.coordinates);

      // Dynamic 9-bay occupancy map
      this.occ = {
        A1: false, A2: false, A3: false,
        B1: false, B2: false, B3: false,
        C1: false, C2: false, C3: false
      };
      if (options.initialOccupancy) this.setOccupancy(options.initialOccupancy);

      // Kinematic & telemetry state
      // Note: xVis in [0..8.0] and zVis in [0..6.235] are the smooth visual coordinates;
      // xRev and zRev mirror the live firmware revolution readouts for HUD badges.
      this.state = {
        mode: 'live',          // 'live' | 'demo' | 'manual'
        xRev: 0.0,             // displayed firmware X rev
        zRev: 0.0,             // displayed firmware Z rev
        xVis: 0.0,             // rendered visual X [0..8.0]
        zVis: 0.0,             // rendered visual Z [0..6.235]
        goalXRev: 0.0,         // live firmware X goal
        goalZRev: 0.0,         // live firmware Z goal
        goalXVis: 0.0,         // live visual X goal
        goalZVis: 0.0,         // live visual Z goal
        forkExt: 0.0,          // 0 = retracted, 1 = fully extended into bay
        goalForkExt: 0.0,
        servoState: 'STOP',    // STOP | EXTEND | RETRACT
        servoExtRev: 1.5,
        servoRetRev: 1.5,
        servoMsRev: 1000,
        zOffsetRev: 0.40,
        rpm: 60.0,
        accel: 800.0,
        stepStage: 0,          // 0..6
        op: 'NONE',            // NONE | STORAGE | RETRIEVAL | HOMING | HOME | TEST | JOG | E-STOP | MANUAL
        targetBay: null,       // active operation bay ('A1'..'C3')
        selectedBay: 'A1',     // WMS selected bay highlight
        hoveredBay: null,
        running: false,
        locked: false,
        homingSwitchHit: false,
        carriedBox: false,
        tempShowShelfBox: null,
        tempHideShelfBox: null,
        xDir: 0,
        zDir: 0,
        forkDir: 0,
        jogVelX: 0,            // continuous manual jog velocity (-1, 0, +1)
        jogVelZ: 0,
        statusMsg: 'STANDBY — AT HOME (0,0)',
        online: false,
        lastTelemetryMs: 0,
        rtdbPulseUntil: 0,
        _rtdbActive: false
      };

      // Remember previous raw ESP32 packet to distinguish real encoder movement
      // from unchanged start-of-stage snapshots repeated by 1Hz polls!
      this._lastRaw = null;
      this._pendingCmdUntil = 0;

      // Active offline demo choreography descriptor
      this._demo = null;

      this._view = { width: 574, height: 548, scale: 0.62, offX: 0, offY: 0, dpr: 1 };
      this.assets = {};
      this.assetsReady = false;
      this.assetError = null;

      this._rafId = null;
      this._lastFrameTs = 0;
      this._elapsedSec = 0;
      this._resizeObserver = null;

      this._bindEvents();
      this.resize();
      this._loadAssets();
    }

    // -------------------------------------------------------------------------
    // ASSET LOADING & RESPONSIVE SIZING
    // -------------------------------------------------------------------------
    _loadAssets() {
      const entries = Object.entries(ASSET_MANIFEST);
      Promise.all(entries.map(([key, filename]) => new Promise((resolve, reject) => {
        const img = new Image();
        img.decoding = 'async';
        img.onload = () => { this.assets[key] = img; resolve(); };
        img.onerror = () => reject(new Error(`Failed to load ${this.options.assetPath}${filename}`));
        img.src = this.options.assetPath + filename;
      }))).then(() => {
        this.assetsReady = true;
        this.renderNow();
        if (this.options.onReady) this.options.onReady(this);
        if (this.options.autoStart) this.start();
      }).catch((err) => {
        this.assetError = err;
        console.error('[AsrsCadTwin]', err);
        this.renderNow();
      });
    }

    resize() {
      const rect = this.canvas.getBoundingClientRect();
      const cssW = Math.max(240, Math.round(rect.width || this.canvas.width || 574));
      const cssH = Math.max(220, Math.round(rect.height || this.canvas.height || 548));
      const dpr = Math.min(window.devicePixelRatio || 1, 2);

      const targetW = Math.round(cssW * dpr);
      const targetH = Math.round(cssH * dpr);
      if (this.canvas.width !== targetW || this.canvas.height !== targetH) {
        this.canvas.width = targetW;
        this.canvas.height = targetH;
      }

      const crop = CROP_MODES[this.options.cropMode] || CROP_MODES.compact;
      const scale = Math.min(cssW / crop.w, cssH / crop.h);
      const offX = (cssW - crop.w * scale) / 2 - crop.x * scale;
      const offY = (cssH - crop.h * scale) / 2 - crop.y * scale - 6;

      this._view = { width: cssW, height: cssH, scale, offX, offY, dpr };
      if (this.assetsReady) this.renderNow();
    }

    _bindEvents() {
      if (typeof ResizeObserver !== 'undefined') {
        this._resizeObserver = new ResizeObserver(() => this.resize());
        this._resizeObserver.observe(this.canvas);
      }
      this._onWindowResize = () => this.resize();
      window.addEventListener('resize', this._onWindowResize);

      if (this.options.enableClickToSelect) {
        this._onPointerMove = (ev) => {
          const bay = this._hitTestBay(ev);
          if (bay !== this.state.hoveredBay) {
            this.state.hoveredBay = bay;
            this.canvas.style.cursor = bay ? 'pointer' : 'default';
          }
        };
        this._onPointerLeave = () => {
          this.state.hoveredBay = null;
          this.canvas.style.cursor = 'default';
        };
        this._onClick = (ev) => {
          const bay = this._hitTestBay(ev);
          if (bay) {
            this.state.selectedBay = bay;
            if (this.options.onBayClick) this.options.onBayClick(bay);
          }
        };
        this.canvas.addEventListener('mousemove', this._onPointerMove);
        this.canvas.addEventListener('mouseleave', this._onPointerLeave);
        this.canvas.addEventListener('click', this._onClick);
      }
    }

    _hitTestBay(ev) {
      const rect = this.canvas.getBoundingClientRect();
      const sx = ev.clientX - rect.left;
      const sy = ev.clientY - rect.top;
      const cadX = (sx - this._view.offX) / this._view.scale;
      const cadY = (sy - this._view.offY) / this._view.scale;
      for (const id of Object.keys(BAY_INFO)) {
        if (pointInQuad(cadX, cadY, BAY_INFO[id].quad)) return id;
      }
      return null;
    }

    start() {
      if (this._rafId !== null) return;
      this._lastFrameTs = performance.now();
      const tick = (now) => {
        const dt = clamp((now - this._lastFrameTs) / 1000, 0.001, 0.08);
        this._lastFrameTs = now;
        this._elapsedSec += dt;
        this._step(dt, now);
        this.renderNow();
        this._rafId = requestAnimationFrame(tick);
      };
      this._rafId = requestAnimationFrame(tick);
    }

    stop() {
      if (this._rafId !== null) {
        cancelAnimationFrame(this._rafId);
        this._rafId = null;
      }
    }

    destroy() {
      this.stop();
      if (this._resizeObserver) this._resizeObserver.disconnect();
      window.removeEventListener('resize', this._onWindowResize);
      if (this._onPointerMove) {
        this.canvas.removeEventListener('mousemove', this._onPointerMove);
        this.canvas.removeEventListener('mouseleave', this._onPointerLeave);
        this.canvas.removeEventListener('click', this._onClick);
      }
    }

    // -------------------------------------------------------------------------
    // PUBLIC INTEGRATION API (REAL HARDWARE + WMS HOOKS)
    // -------------------------------------------------------------------------
    setOccupancy(occMap) {
      if (!occMap) return;
      for (const id of Object.keys(this.occ)) {
        if (occMap[id] !== undefined) this.occ[id] = !!occMap[id];
      }
    }

    setCoordinates(coordMap) {
      if (!coordMap) return;
      for (const id of Object.keys(this.coordinates)) {
        if (coordMap[id] && !isNaN(coordMap[id].x) && !isNaN(coordMap[id].z)) {
          this.coordinates[id] = {
            x: Number(coordMap[id].x),
            z: Number(coordMap[id].z)
          };
        }
      }
    }

    setSelectedBay(bayId) {
      if (bayId && BAY_INFO[bayId]) {
        this.state.selectedBay = bayId;
      }
    }

    setOption(key, value) {
      this.options[key] = value;
      if (key === 'cropMode') this.resize();
    }

    /**
     * Immediate command dispatch hook — call right when the WMS writes to
     * `/asrs/control/command` so the 3D twin starts responding with 0ms lag
     * while awaiting ESP32 Firebase stage confirmations.
     */
    notifyCommand(cmdStr) {
      if (!cmdStr) return;
      const cmd = String(cmdStr).trim().toUpperCase();
      const s = this.state;
      const now = performance.now();

      if (cmd === 'E_STOP') {
        this.triggerEStop();
        return;
      }
      if (cmd === 'UNLOCK') {
        this.unlock();
        return;
      }
      if (cmd === 'STOP' || cmd === 'SERVO_STOP' || cmd === 'SERVO:STOP') {
        s.running = false;
        s.jogVelX = 0; s.jogVelZ = 0;
        s.goalXRev = s.xRev; s.goalZRev = s.zRev;
        s.goalXVis = s.xVis; s.goalZVis = s.zVis;
        s.goalForkExt = s.forkExt;
        s.servoState = 'STOP';
        s.xDir = 0; s.zDir = 0; s.forkDir = 0;
        this._pendingCmdUntil = 0;
        return;
      }
      if (cmd === 'ZERO') {
        s.xRev = 0; s.zRev = 0; s.xVis = 0; s.zVis = 0;
        s.goalXRev = 0; s.goalZRev = 0; s.goalXVis = 0; s.goalZVis = 0;
        s.statusMsg = 'ZEROED POSITION (0.00, 0.00)';
        return;
      }
      if (s.locked) return;

      this._demo = null;
      s.mode = 'live';
      s.jogVelX = 0; s.jogVelZ = 0;

      // STORE:B2:5.00:4.00 or RETRIEVE:A2:1.00:1.50
      if (cmd.startsWith('STORE:') || cmd.startsWith('RETRIEVE:')) {
        const parts = cmd.split(':');
        const bayId = parts[1];
        if (BAY_INFO[bayId]) {
          const isStore = cmd.startsWith('STORE:');
          const c = this._getBayFirmwareCoord(bayId, parts[2], parts[3]);
          s.op = isStore ? 'STORAGE' : 'RETRIEVAL';
          s.targetBay = bayId;
          s.selectedBay = bayId;
          s.stepStage = 1;
          s.running = true;
          s.carriedBox = isStore;
          s.tempHideShelfBox = isStore ? bayId : null;
          s.tempShowShelfBox = null;
          s.goalXRev = c.x;
          s.goalXVis = BAY_INFO[bayId].xRev;
          s.goalZRev = s.zRev;
          s.goalZVis = s.zVis;
          s.goalForkExt = 0;
          s.statusMsg = `DISPATCH -> ${s.op} ${bayId}`;
          this._pendingCmdUntil = now + 2200;
        }
        return;
      }

      // TESTSLOT:B2:5.00:4.00
      if (cmd.startsWith('TESTSLOT:')) {
        const parts = cmd.split(':');
        const bayId = parts[1];
        if (BAY_INFO[bayId]) {
          const c = this._getBayFirmwareCoord(bayId, parts[2], parts[3]);
          s.op = 'TEST';
          s.targetBay = bayId;
          s.selectedBay = bayId;
          s.stepStage = 0;
          s.running = true;
          s.carriedBox = false;
          s.goalXRev = c.x;
          s.goalZRev = c.z;
          s.goalXVis = BAY_INFO[bayId].xRev;
          s.goalZVis = BAY_INFO[bayId].zSeatVis;
          s.goalForkExt = 0;
          s.statusMsg = `TEST MOVE -> ${bayId}`;
          this._pendingCmdUntil = now + 2200;
        }
        return;
      }

      // HOME / HOMING
      if (cmd === 'HOME' || cmd === 'HOMING') {
        s.op = cmd;
        s.stepStage = 0;
        s.running = true;
        s.goalXRev = 0; s.goalZRev = 0;
        s.goalXVis = 0; s.goalZVis = 0;
        s.goalForkExt = 0;
        s.statusMsg = cmd === 'HOMING' ? 'HOMING -> LS_X_RIGHT (0,0)' : 'RETURNING HOME (0,0)';
        this._pendingCmdUntil = now + 2200;
        return;
      }

      // Manual Servo Extend / Retract (SERVO:EXTEND_REV, SERVO:EXTEND, SERVO:RETRACT_REV, SERVO:RETRACT)
      if (cmd.startsWith('SERVO:EXTEND') || cmd.startsWith('SERVO_EXTEND') || cmd === 'EXTEND') {
        s.servoState = 'EXTEND';
        s.goalForkExt = 1.0;
        s.running = true;
        this._pendingCmdUntil = now + 2200;
        return;
      }
      if (cmd.startsWith('SERVO:RETRACT') || cmd.startsWith('SERVO_RETRACT') || cmd === 'RETRACT') {
        s.servoState = 'RETRACT';
        s.goalForkExt = 0.0;
        s.running = true;
        this._pendingCmdUntil = now + 2200;
        return;
      }

      // Manual revolution jog commands: M1:<rev>, M2:<rev>, or bare numeric <rev> (both axes)
      if (cmd.startsWith('M1:') || cmd.startsWith('MOVE1:')) {
        const delta = parseFloat(cmd.split(':')[1]);
        if (!isNaN(delta)) {
          s.op = 'JOG'; s.running = true;
          s.goalXRev = clamp(s.xRev + delta, 0, 8.2);
          s.goalXVis = this._rawXToVis(s.goalXRev, null);
          this._pendingCmdUntil = now + 2000;
        }
        return;
      }
      if (cmd.startsWith('M2:') || cmd.startsWith('MOVE2:')) {
        const delta = parseFloat(cmd.split(':')[1]);
        if (!isNaN(delta)) {
          s.op = 'JOG'; s.running = true;
          s.goalZRev = clamp(s.zRev + delta, 0, 7.0);
          s.goalZVis = this._rawZToVis(s.goalZRev, null, 'NONE', 0);
          this._pendingCmdUntil = now + 2000;
        }
        return;
      }
      if (/^[+-]?\d+(\.\d+)?$/.test(cmd)) {
        const delta = parseFloat(cmd);
        s.op = 'JOG'; s.running = true;
        s.goalXRev = clamp(s.xRev + delta, 0, 8.2);
        s.goalZRev = clamp(s.zRev + delta, 0, 7.0);
        s.goalXVis = this._rawXToVis(s.goalXRev, null);
        s.goalZVis = this._rawZToVis(s.goalZRev, null, 'NONE', 0);
        this._pendingCmdUntil = now + 2000;
        return;
      }

      // Continuous directional jog aliases
      if (cmd === 'M1_CW' || cmd === 'X_PLUS' || cmd === 'JOG_X_POS') {
        s.op = 'JOG'; s.running = true; s.jogVelX = 1; this._pendingCmdUntil = now + 1800;
      } else if (cmd === 'M1_CCW' || cmd === 'X_MINUS' || cmd === 'JOG_X_NEG') {
        s.op = 'JOG'; s.running = true; s.jogVelX = -1; this._pendingCmdUntil = now + 1800;
      } else if (cmd === 'M2_CW' || cmd === 'Z_PLUS' || cmd === 'JOG_Z_POS') {
        s.op = 'JOG'; s.running = true; s.jogVelZ = 1; this._pendingCmdUntil = now + 1800;
      } else if (cmd === 'M2_CCW' || cmd === 'Z_MINUS' || cmd === 'JOG_Z_NEG') {
        s.op = 'JOG'; s.running = true; s.jogVelZ = -1; this._pendingCmdUntil = now + 1800;
      }
    }

    _getBayFirmwareCoord(bayId, xOpt, zOpt) {
      const cfg = this.coordinates[bayId] || BAY_INFO[bayId];
      const px = parseFloat(xOpt);
      const pz = parseFloat(zOpt);
      return {
        x: (!isNaN(px) && px >= 0) ? px : cfg.x,
        z: (!isNaN(pz) && pz >= 0) ? pz : cfg.z
      };
    }

    /**
     * Authoritative real-time Firebase `/asrs/telemetry` handler (works with BOTH
     * Server-Sent Events EventSource streaming AND HTTP polling!).
     *
     * Key design: ESP32 pushes telemetry at each stage entry (so `m1_rev` during
     * Stage 1 is still the start-of-stage reading until Stage 2 starts!). This method
     * distinguishes fresh encoder updates from repeated same-stage snapshots and sets
     * the true stage destination goal so the 3D machine moves smoothly in real time
     * instead of snapping back to 0 every second!
     */
    updateFromTelemetry(data, occMap = null, coordMap = null) {
      if (occMap) this.setOccupancy(occMap);
      if (coordMap) this.setCoordinates(coordMap);
      if (!data) {
        this.state.online = false;
        return;
      }

      const now = performance.now();
      const s = this.state;
      this._demo = null;
      s.mode = 'live';
      s.online = true;
      s.lastTelemetryMs = now;
      s.rtdbPulseUntil = now + 380;

      const rawM1 = Number(data.m1_rev || 0);
      const rawM2 = Number(data.m2_rev || 0);
      const rawLocked = !!data.locked;
      const rawRunning = !!data.running;
      const rawStage = Number(data.step_stage || 0);
      const rawOp = String(data.op || 'NONE').toUpperCase();
      const rawBox = String(data.box || 'NONE').toUpperCase();
      const rawBay = BAY_INFO[rawBox] ? rawBox : null;
      const rawServo = String(data.servo_state || 'STOP').toUpperCase();
      const rawStatus = String(data.status || (rawLocked ? 'EMERGENCY STOP (LOCKED)' : 'ONLINE')).toUpperCase();

      // Update motor kinematic parameters from ESP32
      s.rpm = Number(data.rpm || s.rpm || 60);
      s.accel = Number(data.accel || s.accel || 800);
      s.servoExtRev = Number(data.servo_ext_rev || s.servoExtRev || 1.5);
      s.servoRetRev = Number(data.servo_ret_rev || s.servoRetRev || 1.5);
      s.servoMsRev = Number(data.servo_ms_rev || s.servoMsRev || 1000);
      if (data.z_offset !== undefined && !isNaN(parseFloat(data.z_offset))) {
        s.zOffsetRev = Math.max(0.02, Math.abs(parseFloat(data.z_offset)));
      }

      // If we just dispatched a command < 2s ago and Firebase hasn't echoed `running: true`
      // or a state change yet, don't let the stale previous idle packet cancel our motion!
      const prev = this._lastRaw;
      const packetChanged = !prev ||
        prev.stepStage !== rawStage ||
        prev.running !== rawRunning ||
        prev.locked !== rawLocked ||
        prev.servoState !== rawServo ||
        prev.status !== rawStatus ||
        Math.abs(prev.m1Rev - rawM1) > 0.005 ||
        Math.abs(prev.m2Rev - rawM2) > 0.005;

      if (!rawLocked && !rawRunning && rawStage === 0 && !packetChanged && now < this._pendingCmdUntil) {
        return;
      }
      if (rawRunning || rawStage > 0 || rawLocked || packetChanged) {
        this._pendingCmdUntil = 0;
      }

      const isFirstPacket = !prev;
      const stageChanged = !prev || prev.stepStage !== rawStage || prev.op !== rawOp || prev.targetBay !== rawBay;
      const rawM1Changed = !prev || Math.abs(prev.m1Rev - rawM1) > 0.005;
      const rawM2Changed = !prev || Math.abs(prev.m2Rev - rawM2) > 0.005;

      this._lastRaw = {
        m1Rev: rawM1,
        m2Rev: rawM2,
        stepStage: rawStage,
        op: rawOp,
        targetBay: rawBay,
        servoState: rawServo,
        running: rawRunning,
        locked: rawLocked,
        status: rawStatus
      };

      s.locked = rawLocked;
      s.running = rawRunning;
      s.stepStage = rawStage;
      s.op = rawOp;
      s.targetBay = rawBay;
      s.servoState = rawServo;
      s.statusMsg = rawStatus;

      // 1. E-STOP SAFETY LOCK: Halt immediately in place!
      if (s.locked) {
        s.running = false;
        s.jogVelX = 0; s.jogVelZ = 0;
        s.goalXRev = s.xRev; s.goalZRev = s.zRev;
        s.goalXVis = s.xVis; s.goalZVis = s.zVis;
        s.goalForkExt = s.forkExt;
        s.xDir = 0; s.zDir = 0; s.forkDir = 0;
        if (this.options.onStateChange) this.options.onStateChange(this.getSnapshot());
        return;
      }

      // 2. First packet at startup (idle): snap directly to hardware position
      if (isFirstPacket && !rawRunning && rawStage === 0) {
        s.xRev = rawM1; s.zRev = rawM2;
        s.goalXRev = rawM1; s.goalZRev = rawM2;
        s.xVis = this._rawXToVis(rawM1, rawBay);
        s.zVis = this._rawZToVis(rawM2, rawBay, rawOp, 0);
        s.goalXVis = s.xVis; s.goalZVis = s.zVis;
        s.forkExt = rawServo === 'EXTEND' ? 1.0 : 0.0;
        s.goalForkExt = s.forkExt;
        s.carriedBox = false;
        if (this.options.onStateChange) this.options.onStateChange(this.getSnapshot());
        return;
      }

      // 3. ACTIVE 6-STEP ASRS SEQUENCE (step_stage 1..6)
      //    Set true stage destination goal so motion glides continuously between ESP32 syncs!
      if (rawStage >= 1 && rawStage <= 6 && rawBay) {
        s.jogVelX = 0; s.jogVelZ = 0;
        const bayInfo = BAY_INFO[rawBay];
        const bayCal = this.coordinates[rawBay] || { x: bayInfo.xRev, z: bayInfo.zRev };
        const isStore = (rawOp === 'STORAGE');
        const zHoverVis = bayInfo.zSeatVis + VISUAL_LIFT_REV; // clean ~9.2px above shelf floor
        const zSeatVis = bayInfo.zSeatVis;                    // flush on shelf floor
        const zAfterCal = isStore ? Math.max(0, bayCal.z - s.zOffsetRev) : (bayCal.z + s.zOffsetRev);

        if (rawStage === 1) {
          // STEP 1: Move X -> targetBay
          if (stageChanged && rawM1Changed) {
            s.xRev = rawM1;
            s.xVis = this._rawXToVis(rawM1, rawBay);
          }
          s.goalXRev = bayCal.x;
          s.goalXVis = bayInfo.xRev;
          s.goalZRev = rawM2;
          s.goalZVis = this._rawZToVis(rawM2, rawBay, rawOp, rawStage);
          s.goalForkExt = 0.0;
          s.carriedBox = isStore;
          s.tempHideShelfBox = isStore ? rawBay : null;
          s.tempShowShelfBox = null;
        } else if (rawStage === 2) {
          // STEP 2: Move Z -> targetBay (X already arrived at targetBay!)
          s.goalXRev = bayCal.x;
          s.goalXVis = bayInfo.xRev;
          s.goalZRev = bayCal.z;
          s.goalZVis = isStore ? zHoverVis : zSeatVis;
          s.goalForkExt = 0.0;
          s.carriedBox = isStore;
          s.tempHideShelfBox = isStore ? rawBay : null;
          s.tempShowShelfBox = null;
        } else if (rawStage === 3) {
          // STEP 3: 360° Servo Extend into targetBay
          s.goalXRev = bayCal.x;
          s.goalXVis = bayInfo.xRev;
          s.goalZRev = bayCal.z;
          s.goalZVis = isStore ? zHoverVis : zSeatVis;
          s.goalForkExt = 1.0;
          s.carriedBox = isStore;
          s.tempHideShelfBox = isStore ? rawBay : null;
          s.tempShowShelfBox = null;
        } else if (rawStage === 4) {
          // STEP 4: Z Drop (-zOffset for STORE) or Z Lift (+zOffset for RETRIEVE)
          s.goalXRev = bayCal.x;
          s.goalXVis = bayInfo.xRev;
          s.goalZRev = zAfterCal;
          s.goalZVis = isStore ? zSeatVis : zHoverVis;
          s.goalForkExt = 1.0;
          s.forkDir = 0;
          s.carriedBox = true;
          s.tempHideShelfBox = rawBay;
          s.tempShowShelfBox = null;
        } else if (rawStage === 5) {
          // STEP 5: 360° Servo Retract out of targetBay
          s.goalXRev = bayCal.x;
          s.goalXVis = bayInfo.xRev;
          s.goalZRev = zAfterCal;
          s.goalZVis = isStore ? zSeatVis : zHoverVis;
          s.goalForkExt = 0.0;
          if (isStore) {
            s.carriedBox = false;
            s.tempShowShelfBox = rawBay;
            s.tempHideShelfBox = null;
          } else {
            s.carriedBox = true;
            s.tempHideShelfBox = rawBay;
            s.tempShowShelfBox = null;
          }
        } else if (rawStage === 6) {
          // STEP 6: Simultaneous Return Home (0, 0)
          s.goalXRev = 0.0;
          s.goalZRev = 0.0;
          s.goalXVis = 0.0;
          s.goalZVis = 0.0;
          s.goalForkExt = 0.0;
          if (isStore) {
            s.carriedBox = false;
            s.tempShowShelfBox = rawBay;
            s.tempHideShelfBox = null;
          } else {
            s.carriedBox = true;
            s.tempHideShelfBox = rawBay;
            s.tempShowShelfBox = null;
          }
        }
      } else {
        // 4. NON-ASRS OPERATION OR IDLE (step_stage === 0):
        //    Handles TESTSLOT, HOME, HOMING, Manual Jog, Manual Servo, and Cycle Complete!
        s.tempShowShelfBox = null;
        s.tempHideShelfBox = null;

        if (rawServo === 'EXTEND') {
          s.goalForkExt = 1.0;
        } else if (rawServo === 'RETRACT') {
          s.goalForkExt = 0.0;
        } else if (!rawRunning) {
          s.goalForkExt = 0.0;
          s.forkDir = 0;
        }

        if (rawRunning) {
          if (rawStatus.includes('HOME') || rawStatus.includes('HOMING') || rawStatus.includes('RETURN')) {
            s.jogVelX = 0; s.jogVelZ = 0;
            s.goalXRev = 0.0; s.goalZRev = 0.0;
            s.goalXVis = 0.0; s.goalZVis = 0.0;
            s.goalForkExt = 0.0;
          } else if (rawBay && (rawStatus.includes('TEST') || rawStatus.includes('MOVING') || rawStatus.includes('SLOT'))) {
            s.jogVelX = 0; s.jogVelZ = 0;
            const bayInfo = BAY_INFO[rawBay];
            const bayCal = this.coordinates[rawBay] || { x: bayInfo.xRev, z: bayInfo.zRev };
            s.goalXRev = bayCal.x;
            s.goalZRev = bayCal.z;
            s.goalXVis = bayInfo.xRev;
            s.goalZVis = bayInfo.zSeatVis;
          } else {
            if (rawM1Changed) {
              s.goalXRev = rawM1;
              s.goalXVis = this._rawXToVis(rawM1, rawBay);
            }
            if (rawM2Changed) {
              s.goalZRev = rawM2;
              s.goalZVis = this._rawZToVis(rawM2, rawBay, rawOp, 0);
            }
          }
        } else {
          // Idle / Completed — smoothly converge to authoritative ESP32 position
          s.jogVelX = 0; s.jogVelZ = 0;
          s.goalXRev = rawM1;
          s.goalZRev = rawM2;
          s.goalXVis = this._rawXToVis(rawM1, rawBay);
          s.goalZVis = this._rawZToVis(rawM2, rawBay, rawOp, 0);
          if (rawOp === 'RETRIEVAL' && rawStatus.includes('COMPLETE') && rawM1 < 0.08 && rawM2 < 0.08) {
            s.carriedBox = true; // retrieved pallet resting on fork at Home I/O
          } else if (rawM1 < 0.08 && rawM2 < 0.08) {
            s.carriedBox = false;
          }
        }
      }

      s.homingSwitchHit = rawStatus.includes('LS_X_RIGHT') ||
        (rawStatus.includes('HOMING COMPLETE') && rawM1 < 0.05 && rawM2 < 0.05);

      if (this.options.onStateChange) this.options.onStateChange(this.getSnapshot());
    }

    /**
     * Direct manual pose setter (for commissioning / manual jog tests).
     */
    setPose(xVis, zVis, forkExt = 0, carriedBox = false) {
      const s = this.state;
      s.mode = 'manual';
      this._demo = null;
      s.goalXVis = clamp(Number(xVis) || 0, 0, 8.0);
      s.goalZVis = clamp(Number(zVis) || 0, 0, 6.235);
      s.goalXRev = s.goalXVis;
      s.goalZRev = s.goalZVis;
      s.goalForkExt = clamp(Number(forkExt) || 0, 0, 1);
      s.carriedBox = !!carriedBox;
      s.op = 'MANUAL';
      s.stepStage = 0;
      s.statusMsg = `MANUAL POSE -> X:${s.goalXVis.toFixed(2)}r  Z:${s.goalZVis.toFixed(2)}r`;
    }

    // -------------------------------------------------------------------------
    // BUILT-IN OFFLINE PREVIEW SIMULATOR (OPTIONAL SANDBOX)
    // -------------------------------------------------------------------------
    runDemoCycle(opType = 'STORAGE', bayId = 'C2', opts = {}) {
      const bay = BAY_INFO[String(bayId).toUpperCase()];
      if (!bay) return false;
      const op = String(opType).toUpperCase() === 'RETRIEVAL' ? 'RETRIEVAL' : 'STORAGE';
      if (this.state.locked) {
        this.state.statusMsg = 'REJECTED: SYSTEM LOCKED — SEND UNLOCK FIRST';
        return false;
      }

      const rpm = Number(opts.rpm || this.state.rpm || 60);
      const duration = Number(opts.duration || (rpm >= 100 ? 7.2 : 9.5));
      this.state.mode = 'demo';
      this.state.selectedBay = bay.id;
      this.state.targetBay = bay.id;
      this.state.op = op;
      this.state.rpm = rpm;
      this.state.homingSwitchHit = false;
      this.state.jogVelX = 0; this.state.jogVelZ = 0;

      if (op === 'RETRIEVAL') this.occ[bay.id] = true;
      if (op === 'STORAGE') this.occ[bay.id] = false;

      const cal = this.coordinates[bay.id] || { x: bay.xRev, z: bay.zRev };
      this._demo = {
        kind: 'cycle',
        op,
        bayId: bay.id,
        txVis: bay.xRev,
        tzSeatVis: bay.zSeatVis,
        txRev: cal.x,
        tzRev: cal.z,
        t: 0,
        dur: duration,
        committed: false,
        onComplete: typeof opts.onComplete === 'function' ? opts.onComplete : null
      };
      return true;
    }

    runHomingDemo(opts = {}) {
      if (this.state.locked) return false;
      const startXVis = this.state.xVis > 0.3 ? this.state.xVis : 4.6;
      const startZVis = this.state.zVis > 0.3 ? this.state.zVis : 3.6;
      this.state.mode = 'demo';
      this.state.op = 'HOMING';
      this.state.targetBay = null;
      this.state.carriedBox = false;
      this.state.forkExt = 0;
      this._demo = {
        kind: 'homing',
        startXVis,
        startZVis,
        t: 0,
        dur: Number(opts.duration || 4.5),
        onComplete: typeof opts.onComplete === 'function' ? opts.onComplete : null
      };
      return true;
    }

    triggerEStop() {
      const s = this.state;
      s.locked = true;
      s.running = false;
      s.jogVelX = 0; s.jogVelZ = 0;
      s.xDir = 0; s.zDir = 0; s.forkDir = 0;
      s.servoState = 'STOP';
      s.op = 'E-STOP';
      s.statusMsg = 'EMERGENCY STOP (LOCKED) — MOTORS HALTED';
      this._demo = null;
      if (this.options.onStateChange) this.options.onStateChange(this.getSnapshot());
    }

    unlock() {
      const s = this.state;
      s.locked = false;
      s.op = 'IDLE';
      s.statusMsg = 'IDLE (UNLOCKED) — SAFETY LOCK RELEASED';
      if (this.options.onStateChange) this.options.onStateChange(this.getSnapshot());
    }

    getSnapshot() {
      return {
        xRev: this.state.xRev,
        zRev: this.state.zRev,
        xVis: this.state.xVis,
        zVis: this.state.zVis,
        forkExt: this.state.forkExt,
        servoState: this.state.servoState,
        stepStage: this.state.stepStage,
        op: this.state.op,
        targetBay: this.state.targetBay,
        selectedBay: this.state.selectedBay,
        running: this.state.running,
        locked: this.state.locked,
        statusMsg: this.state.statusMsg,
        occupiedCount: Object.values(this.occ).filter(Boolean).length
      };
    }

    // -------------------------------------------------------------------------
    // 4. PER-BAY CALIBRATED COORDINATE MAPPING (ZERO MISALIGNMENT!)
    // -------------------------------------------------------------------------
    _rawXToVis(rawX, bayId) {
      if (rawX <= 0.001) return 0.0;
      if (bayId && BAY_INFO[bayId]) {
        const calX = (this.coordinates[bayId] && this.coordinates[bayId].x > 0.01)
          ? this.coordinates[bayId].x
          : BAY_INFO[bayId].xRev;
        return clamp((rawX / calX) * BAY_INFO[bayId].xRev, 0, 8.1);
      }
      // Fallback when no specific bay is active: use clean reference scale (8.0 max)
      const maxX = Math.max(2.0, ...Object.values(this.coordinates).map(c => c.x || 0));
      return clamp((rawX / maxX) * 8.0, 0, 8.1);
    }

    _rawZToVis(rawZ, bayId, op, stepStage) {
      if (rawZ <= 0.001) return 0.0;
      const zOff = this.state.zOffsetRev || 0.40;
      if (bayId && BAY_INFO[bayId]) {
        const b = BAY_INFO[bayId];
        const calZ = (this.coordinates[bayId] && this.coordinates[bayId].z > 0.01)
          ? this.coordinates[bayId].z
          : b.zRev;
        const zHoverVis = b.zSeatVis + VISUAL_LIFT_REV;

        if (op === 'STORAGE') {
          if (stepStage === 4 || stepStage === 5) {
            const dropFrac = clamp((calZ - rawZ) / Math.max(0.02, zOff), 0, 1);
            return clamp(zHoverVis - dropFrac * VISUAL_LIFT_REV, 0, 6.25);
          }
          if (stepStage === 6) {
            const zStart = Math.max(0.05, calZ - zOff);
            return clamp((rawZ / zStart) * b.zSeatVis, 0, 6.25);
          }
          return clamp((rawZ / calZ) * zHoverVis, 0, 6.25);
        } else if (op === 'RETRIEVAL') {
          if (stepStage === 4 || stepStage === 5) {
            const liftFrac = clamp((rawZ - calZ) / Math.max(0.02, zOff), 0, 1);
            return clamp(b.zSeatVis + liftFrac * VISUAL_LIFT_REV, 0, 6.25);
          }
          if (stepStage === 6) {
            const zStart = Math.max(0.05, calZ + zOff);
            return clamp((rawZ / zStart) * zHoverVis, 0, 6.25);
          }
          return clamp((rawZ / calZ) * b.zSeatVis, 0, 6.25);
        }
        return clamp((rawZ / calZ) * b.zSeatVis, 0, 6.25);
      }
      const maxZ = Math.max(1.5, ...Object.values(this.coordinates).map(c => c.z || 0));
      return clamp((rawZ / maxZ) * 6.10, 0, 6.25);
    }

    // -------------------------------------------------------------------------
    // 5. 60 FPS PHYSICS STEPPER (LIVE REAL HARDWARE TRACKING OR DEMO)
    // -------------------------------------------------------------------------
    _step(dt, nowMs) {
      const s = this.state;
      s._rtdbActive = nowMs < s.rtdbPulseUntil;

      if (s.locked) {
        s.running = false;
        s.xDir = 0; s.zDir = 0; s.forkDir = 0;
        return;
      }

      if (s.mode === 'demo' && this._demo) {
        this._stepDemo(dt);
        return;
      }

      // Continuous manual jog (when operator holds/sends M1_CW / M2_CW etc.)
      const revPerSec = Math.max(0.6, (s.rpm || 60) / 60);
      if (s.jogVelX !== 0) {
        s.goalXVis = clamp(s.goalXVis + s.jogVelX * revPerSec * 1.4 * dt, 0, 8.0);
        s.goalXRev = clamp(s.goalXRev + s.jogVelX * revPerSec * dt, 0, 8.0);
      }
      if (s.jogVelZ !== 0) {
        s.goalZVis = clamp(s.goalZVis + s.jogVelZ * revPerSec * 1.4 * dt, 0, 6.10);
        s.goalZRev = clamp(s.goalZRev + s.jogVelZ * revPerSec * dt, 0, 6.50);
      }

      // Compute realistic visual travel speed matched to firmware RPM & bay calibration!
      // Example: if A1 is calibrated at 0.2 rev (0.2s real time) and 2.0 visual rev,
      // visual speed scales proportionally so arrival time matches the real hardware!
      let visSpeedX = revPerSec * 1.25;
      let visSpeedZ = revPerSec * 1.25;
      if (s.targetBay && BAY_INFO[s.targetBay]) {
        const b = BAY_INFO[s.targetBay];
        const cal = this.coordinates[s.targetBay] || { x: b.xRev, z: b.zRev };
        const scaleX = clamp(b.xRev / Math.max(0.15, cal.x), 0.7, 6.0);
        const scaleZ = clamp(b.zSeatVis / Math.max(0.15, cal.z), 0.7, 6.0);
        visSpeedX = revPerSec * scaleX * 1.15;
        visSpeedZ = revPerSec * scaleZ * 1.15;
      }
      // Stage 4 (Z lift/drop) is a short precision 0.135 visual rev move — smooth & visible
      if (s.stepStage === 4) {
        visSpeedZ = clamp(VISUAL_LIFT_REV / Math.max(0.28, (s.zOffsetRev || 0.4) / revPerSec), 0.25, 0.75);
      }

      // Smooth damped motion along visual X & Z (plus displayed firmware xRev / zRev)
      const dxVis = s.goalXVis - s.xVis;
      const dzVis = s.goalZVis - s.zVis;
      const maxStepX = visSpeedX * dt;
      const maxStepZ = visSpeedZ * dt;

      if (Math.abs(dxVis) > 0.004) {
        const damped = dxVis * (1 - Math.exp(-10.0 * dt));
        const step = Math.abs(damped) > maxStepX ? Math.sign(dxVis) * maxStepX : damped;
        const minStep = Math.sign(dxVis) * Math.min(Math.abs(dxVis), maxStepX * 0.45);
        s.xVis += Math.abs(step) < Math.abs(minStep) ? minStep : step;
        s.xDir = dxVis > 0 ? 1 : -1;
      } else {
        s.xVis = s.goalXVis;
        s.xDir = 0;
      }

      if (Math.abs(dzVis) > 0.004) {
        const damped = dzVis * (1 - Math.exp(-10.0 * dt));
        const step = Math.abs(damped) > maxStepZ ? Math.sign(dzVis) * maxStepZ : damped;
        const minStep = Math.sign(dzVis) * Math.min(Math.abs(dzVis), maxStepZ * 0.45);
        s.zVis += Math.abs(step) < Math.abs(minStep) ? minStep : step;
        s.zDir = dzVis > 0 ? 1 : -1;
      } else {
        s.zVis = s.goalZVis;
        s.zDir = 0;
      }

      // Smoothly interpolate displayed firmware rev counters alongside visual pose
      const dxRev = s.goalXRev - s.xRev;
      const dzRev = s.goalZRev - s.zRev;
      if (Math.abs(dxRev) > 0.002) {
        s.xRev += Math.sign(dxRev) * Math.min(Math.abs(dxRev), revPerSec * 1.3 * dt);
      } else {
        s.xRev = s.goalXRev;
      }
      if (Math.abs(dzRev) > 0.002) {
        s.zRev += Math.sign(dzRev) * Math.min(Math.abs(dzRev), revPerSec * 1.3 * dt);
      } else {
        s.zRev = s.goalZRev;
      }

      // Smooth 360° continuous servo fork stroke matched to real `servo_ext_rev * servo_ms_rev`
      const extDurSec = Math.max(0.35, (s.servoExtRev * s.servoMsRev) / 1000);
      const retDurSec = Math.max(0.35, (s.servoRetRev * s.servoMsRev) / 1000);
      const df = s.goalForkExt - s.forkExt;
      if (Math.abs(df) > 0.004) {
        const rate = df > 0 ? (dt / extDurSec) : (dt / retDurSec);
        s.forkExt += Math.sign(df) * Math.min(Math.abs(df), rate);
        s.forkDir = df > 0 ? 1 : -1;
      } else {
        s.forkExt = s.goalForkExt;
        s.forkDir = 0;
      }
    }

    _stepDemo(dt) {
      const d = this._demo;
      const s = this.state;
      d.t += dt;
      s._rtdbActive = (Math.floor(this._elapsedSec * 4) % 3 === 0);

      if (d.kind === 'homing') {
        const u = clamp(d.t / d.dur, 0, 1);
        s.xDir = 0; s.zDir = 0; s.forkDir = 0;
        if (u < 0.72) {
          const p = u / 0.72;
          s.xVis = d.startXVis * (1 - p);
          s.zVis = d.startZVis * (1 - p);
          s.xRev = s.xVis; s.zRev = s.zVis;
          s.running = true;
          s.xDir = s.xVis > 0.02 ? -1 : 0;
          s.zDir = s.zVis > 0.02 ? -1 : 0;
          s.homingSwitchHit = false;
          s.statusMsg = 'HOMING (CONTINUOUS SPIN -> LS_X_RIGHT)';
        } else if (u < 0.94) {
          s.xVis = 0; s.zVis = 0; s.xRev = 0; s.zRev = 0;
          s.running = false;
          s.homingSwitchHit = true;
          s.statusMsg = 'LS_X_RIGHT (GPIO 19) HIT -> ZEROED (0,0)';
        } else {
          s.xVis = 0; s.zVis = 0; s.xRev = 0; s.zRev = 0;
          s.running = false;
          s.homingSwitchHit = false;
          s.op = 'IDLE';
          s.statusMsg = 'HOMING COMPLETE (AT 0,0) — READY';
          const cb = d.onComplete;
          this._demo = null;
          if (cb) cb();
        }
        if (this.options.onStateChange) this.options.onStateChange(this.getSnapshot());
        return;
      }

      // --- 6-STEP ACCELSTEPPER ASRS DEMO CYCLE (USING CLEAN VISUAL CLEARANCE!) ---
      const u = clamp(d.t / d.dur, 0, 1);
      const isStore = (d.op === 'STORAGE');
      const txVis = d.txVis;
      const zSeatVis = d.tzSeatVis;
      const zHoverVis = zSeatVis + VISUAL_LIFT_REV;
      const zOff = s.zOffsetRev || 0.40;

      const t1 = 0.05; // Dispatch
      const t2 = 0.22; // Step 1: Move X
      const t3 = 0.40; // Step 2: Move Z
      const t4 = 0.54; // Step 3: Servo Extend + settle
      const t5 = 0.64; // Step 4: Z Drop / Lift (~9.2px clearance, zero ceiling clip!)
      const t6 = 0.77; // Step 5: Servo Retract + settle
      const t7 = 0.94; // Step 6: Simultaneous Home

      s.xDir = 0; s.zDir = 0; s.forkDir = 0;
      s.servoState = 'STOP';
      s.tempShowShelfBox = null;
      s.tempHideShelfBox = null;

      if (u < t1) {
        s.xVis = 0; s.zVis = 0; s.xRev = 0; s.zRev = 0; s.forkExt = 0;
        s.stepStage = 0; s.running = false;
        s.carriedBox = isStore;
        s.statusMsg = `DISPATCH -> ${isStore ? 'STORE' : 'RETRIEVE'}:${d.bayId}`;
      } else if (u < t2) {
        const m = trapMotion((u - t1) / (t2 - t1), 0.22);
        s.xVis = txVis * m.pos; s.zVis = 0;
        s.xRev = d.txRev * m.pos; s.zRev = 0; s.forkExt = 0;
        s.stepStage = 1; s.running = true; s.xDir = 1;
        s.carriedBox = isStore;
        s.statusMsg = `STEP 1/6: MOVING X -> ${d.bayId} (${d.txRev.toFixed(1)} rev)`;
      } else if (u < t3) {
        const m = trapMotion((u - t2) / (t3 - t2), 0.22);
        const zGoalVis = isStore ? zHoverVis : zSeatVis;
        s.xVis = txVis; s.zVis = zGoalVis * m.pos;
        s.xRev = d.txRev; s.zRev = d.tzRev * m.pos; s.forkExt = 0;
        s.stepStage = 2; s.running = true; s.zDir = 1;
        s.carriedBox = isStore;
        s.statusMsg = `STEP 2/6: MOVING Z -> ${d.bayId} (${d.tzRev.toFixed(1)} rev)`;
      } else if (u < t4) {
        const frac = (u - t3) / (t4 - t3);
        const settling = frac >= 0.86;
        s.xVis = txVis; s.zVis = isStore ? zHoverVis : zSeatVis;
        s.xRev = d.txRev; s.zRev = d.tzRev;
        s.forkExt = easeInOut(clamp(frac / 0.86, 0, 1));
        s.stepStage = 3; s.running = true;
        s.servoState = settling ? 'STOP' : 'EXTEND';
        s.forkDir = settling ? 0 : 1;
        s.carriedBox = isStore;
        s.statusMsg = settling ? 'STEP 3/6: SERVO SETTLE (150ms)' : 'STEP 3/6: SERVO EXTEND (2000us)';
      } else if (u < t5) {
        const m = trapMotion((u - t4) / (t5 - t4), 0.28);
        s.xVis = txVis; s.xRev = d.txRev; s.forkExt = 1.0;
        s.stepStage = 4; s.running = true;
        if (isStore) {
          s.zVis = zHoverVis - VISUAL_LIFT_REV * m.pos;
          s.zRev = d.tzRev - zOff * m.pos;
          s.zDir = -1;
          s.carriedBox = true;
          s.statusMsg = `STEP 4/6: Z DROP (-${zOff.toFixed(2)}r) -> ${d.bayId}`;
        } else {
          s.zVis = zSeatVis + VISUAL_LIFT_REV * m.pos;
          s.zRev = d.tzRev + zOff * m.pos;
          s.zDir = 1;
          s.carriedBox = true;
          s.tempHideShelfBox = d.bayId;
          s.statusMsg = `STEP 4/6: Z LIFT (+${zOff.toFixed(2)}r) <- ${d.bayId}`;
        }
      } else if (u < t6) {
        const frac = (u - t5) / (t6 - t5);
        const settling = frac >= 0.86;
        s.xVis = txVis; s.xRev = d.txRev;
        s.zVis = isStore ? zSeatVis : zHoverVis;
        s.zRev = isStore ? (d.tzRev - zOff) : (d.tzRev + zOff);
        s.forkExt = 1.0 - easeInOut(clamp(frac / 0.86, 0, 1));
        s.stepStage = 5; s.running = true;
        s.servoState = settling ? 'STOP' : 'RETRACT';
        s.forkDir = settling ? 0 : -1;
        if (isStore) {
          s.carriedBox = false;
          s.tempShowShelfBox = d.bayId;
        } else {
          s.carriedBox = true;
          s.tempHideShelfBox = d.bayId;
        }
        s.statusMsg = settling ? 'STEP 5/6: SERVO SETTLE (150ms)' : 'STEP 5/6: SERVO RETRACT (1000us)';
      } else if (u < t7) {
        const zStartVis = isStore ? zSeatVis : zHoverVis;
        const zStartRev = isStore ? Math.max(0, d.tzRev - zOff) : (d.tzRev + zOff);
        const maxDist = Math.max(txVis, zStartVis, 0.1);
        const frac = (u - t6) / (t7 - t6);
        const ux = clamp(frac / (txVis / maxDist), 0, 1);
        const uz = clamp(frac / (zStartVis / maxDist), 0, 1);
        const mx = trapMotion(ux, 0.22);
        const mz = trapMotion(uz, 0.22);
        s.xVis = txVis * (1 - mx.pos);
        s.zVis = zStartVis * (1 - mz.pos);
        s.xRev = d.txRev * (1 - mx.pos);
        s.zRev = zStartRev * (1 - mz.pos);
        s.forkExt = 0;
        s.stepStage = 6; s.running = true;
        s.xDir = ux < 1 ? -1 : 0;
        s.zDir = uz < 1 ? -1 : 0;
        if (isStore) { s.carriedBox = false; s.tempShowShelfBox = d.bayId; }
        else { s.carriedBox = true; s.tempHideShelfBox = d.bayId; }
        s.statusMsg = 'STEP 6/6: RETURNING HOME (0,0)';
      } else {
        s.xVis = 0; s.zVis = 0; s.xRev = 0; s.zRev = 0; s.forkExt = 0;
        s.stepStage = 0; s.running = false;
        if (!d.committed) {
          this.occ[d.bayId] = isStore;
          d.committed = true;
        }
        s.carriedBox = !isStore;
        s.statusMsg = `CYCLE COMPLETE — ${d.bayId} ${isStore ? 'STORED' : 'RETRIEVED'}`;
        const cb = d.onComplete;
        this._demo = null;
        if (cb) cb(d.bayId, d.op);
      }

      if (this.options.onStateChange) this.options.onStateChange(this.getSnapshot());
    }

    // -------------------------------------------------------------------------
    // 6. TRUE ISOMETRIC COL-BY-COL x ROW-BY-ROW PAINTER'S RENDERER
    //    (ZERO WALL SLICING, ZERO CEILING CHOPPING, ZERO FORK PIERCING!)
    // -------------------------------------------------------------------------
    renderNow() {
      const c = this.ctx;
      const { width, height, scale, offX, offY, dpr } = this._view;
      const s = this.state;
      const t = this._elapsedSec;

      c.save();
      c.setTransform(dpr, 0, 0, dpr, 0, 0);

      // Studio neutral backdrop fill (matched to left/right edge tones of backdrop_empty.png)
      const bgGrad = c.createLinearGradient(0, 0, width, height);
      bgGrad.addColorStop(0, '#DCDAD9');
      bgGrad.addColorStop(0.5, '#D7D6D5');
      bgGrad.addColorStop(1, '#CECDCC');
      c.fillStyle = bgGrad;
      c.fillRect(0, 0, width, height);

      if (!this.assetsReady) {
        c.fillStyle = '#334155';
        c.font = '600 13px "JetBrains Mono", "Liberation Mono", monospace';
        c.textAlign = 'center';
        c.textBaseline = 'middle';
        c.fillText(
          this.assetError ? 'ASSET LOAD ERROR (Check assetPath)' : 'LOADING IND-02 3D CAD ASSETS...',
          width / 2, height / 2
        );
        c.restore();
        return;
      }

      // Enter 1200x896 native studio CAD coordinate space
      c.save();
      c.translate(offX, offY);
      c.scale(scale, scale);

      // LAYER 1: All-Empty 9-Bay Photorealistic Studio Backdrop
      c.drawImage(this.assets.backdrop, 0, 0);

      // Seamless horizontal edge feather when displayed in wide containers (e.g. full-width 3D Twin tab)
      if (offX > 1 || (offX + NATIVE_W * scale) < width - 1) {
        const fL = c.createLinearGradient(0, 0, 110, 0);
        fL.addColorStop(0, '#DAD8D7');
        fL.addColorStop(1, 'rgba(218, 216, 215, 0)');
        c.fillStyle = fL;
        c.fillRect(-10, 0, 120, NATIVE_H);

        const fR = c.createLinearGradient(NATIVE_W - 110, 0, NATIVE_W, 0);
        fR.addColorStop(0, 'rgba(208, 207, 206, 0)');
        fR.addColorStop(1, '#D0CFCE');
        c.fillStyle = fR;
        c.fillRect(NATIVE_W - 110, 0, 120, NATIVE_H);
      }

      // LAYER 2: Interactive Bay Quad Highlights (Selected / Hovered / Active Operation)
      this._drawBayHighlights(c, t);

      // Compute current gantry, carriage & carried-box geometry
      const xVis = s.xVis;
      const zVis = s.zVis;
      const { xL, yL, xR, yR } = getBeamEndpoints(zVis);
      const { cx, beamY } = getCarriagePos(xVis, zVis);

      const perspDx = getCarriedPerspDx(xVis);
      const perspDy = getCarriedPerspDy(xVis, zVis);
      const carriedBoxX = (cx + perspDx) - (1 - s.forkExt) * FORK_TRAVEL_DX;
      const carriedBoxY = (beamY + perspDy) - (1 - s.forkExt) * FORK_TRAVEL_DY;

      // Determine exact cubby the fork is currently entering/inside (smooth from forkExt > 0.08)
      const activeInsideBay = (s.forkExt > 0.08 && s.targetBay && BAY_INFO[s.targetBay])
        ? s.targetBay
        : null;

      // LAYERS 3–9: TRUE BACK-TO-FRONT ISOMETRIC PAINTER'S ORDER
      // Col 1 (A1 -> ShelfA1 -> B1 -> ShelfB1 -> C1 -> RoofC1) -> Wall 1 ->
      // Col 2 (A2 -> ShelfA2 -> B2 -> ShelfB2 -> C2 -> RoofC2) -> Wall 2 ->
      // Col 3 (A3 -> ShelfA3 -> B3 -> ShelfB3 -> C3 -> RoofC3)
      for (const col of [1, 2, 3]) {
        for (const row of ['A', 'B', 'C']) {
          const bayId = `${row}${col}`;
          if (this._isBayVisuallyStocked(bayId)) {
            this._drawBayBox(c, bayId);
          }
          if (activeInsideBay === bayId) {
            this._drawForkAndCarriedBox(c, cx, beamY, s.forkExt, carriedBoxX, carriedBoxY);
          }
          this._drawShelfMaskForCol(c, row, col);
        }
        if (col === 1) c.drawImage(this.assets.maskWall1, 454, 154);
        if (col === 2) c.drawImage(this.assets.maskWall2, 576, 208);
      }

      // If fork is retracted / travelling outside the rack in the front corridor,
      // draw fork + carried box cleanly IN FRONT OF all 9 cubbies and both dividers!
      if (!activeInsideBay) {
        this._drawForkAndCarriedBox(c, cx, beamY, s.forkExt, carriedBoxX, carriedBoxY, true);
      }

      // LAYER 10: Fixed Outer Frame Occlusion (Top Crossbar + Right Vertical Frame / Pulley / Rail)
      c.drawImage(this.assets.maskTopBar, 270, 139);
      c.drawImage(this.assets.maskRightFrame, 658, 270);

      // LAYER 11: Animated Right Vertical Z-Belt & Dynamic Flexible Cable Drag Chain
      this._drawVerticalBeltMotion(c, zVis, s.zDir !== 0);
      this._drawDynamicDragChain(c, yR);

      // LAYER 12: Movable Horizontal Gantry Assembly (Left Pulley, Left Rail Mask, Beam+Belt, Motors)
      c.drawImage(this.assets.leftYellow, 293, yL - 47);
      c.drawImage(this.assets.maskLeftRail, 245, 145);
      this._drawHorizontalBeam(c, xL, yL, xR, yR, xVis);
      c.drawImage(this.assets.leftMotor, 200, yL - 53);
      c.drawImage(this.assets.rightMotor, 709, yR - 73);

      // LAYER 13: Movable Black 4-Bolt Carriage Plate
      c.drawImage(this.assets.carriage, cx - 50, beamY - 47);

      // LAYER 14: Bay Labels, Active Motion Vectors, Motor Spin Arcs, MCU LEDs, HOME (0,0) & Safety Overlays
      this._drawBayLabels(c);
      if (this.options.showMotionVectors) {
        this._drawMotionVectors(c, cx, beamY, xL, yL, xR, yR, t);
      }
      this._drawMcuLeds(c);
      if (this.options.showHomeMarker) {
        this._drawHomeMarker(c);
      }
      if (s.homingSwitchHit) {
        this._drawLimitSwitchSpark(c, t);
      }
      if (s.locked) {
        this._drawEStopOverlay(c, cx, beamY, t);
      }

      c.restore(); // exit 1200x896 CAD space

      if (this.options.showTopChips) this._drawTopOverlayChips(c, width);
      if (this.options.showLegend) this._drawBottomLegend(c, width, height);

      c.restore();
    }

    _isBayVisuallyStocked(id) {
      const s = this.state;
      if (s.tempHideShelfBox === id) return false;
      if (s.tempShowShelfBox === id) return true;
      return !!this.occ[id];
    }

    _drawBayBox(c, id) {
      const p = BAY_INFO[id].boxRest;
      // Soft ambient contact shadow tucked cleanly beneath wooden pallet feet
      c.save();
      c.fillStyle = 'rgba(0, 0, 0, 0.34)';
      c.shadowColor = 'rgba(0, 0, 0, 0.45)';
      c.shadowBlur = 8;
      c.beginPath();
      c.moveTo(p.x + 12, p.y + 116);
      c.lineTo(p.x + 84, p.y + 148);
      c.lineTo(p.x + 92, p.y + 136);
      c.lineTo(p.x + 24, p.y + 106);
      c.closePath();
      c.fill();
      c.restore();
      c.drawImage(this.assets.boxBay, p.x, p.y);
    }

    _drawShelfMaskForCol(c, row, col) {
      const [x0, x1] = COL_CLIP_X[col];
      c.save();
      c.beginPath();
      c.rect(x0, 0, x1 - x0, NATIVE_H);
      c.clip();
      if (row === 'A') c.drawImage(this.assets.maskShelfA, 330, 415);
      else if (row === 'B') c.drawImage(this.assets.maskShelfB, 330, 250);
      else if (row === 'C') c.drawImage(this.assets.maskRoofC, 330, 70);
      c.restore();
    }

    _drawForkAndCarriedBox(c, cx, beamY, forkExt, carriedBoxX, carriedBoxY, inCorridor = false) {
      this._drawForkAssembly(c, cx, beamY, forkExt, carriedBoxY);
      if (this.state.carriedBox) {
        c.save();
        c.fillStyle = 'rgba(0, 0, 0, 0.24)';
        c.beginPath();
        c.ellipse(carriedBoxX + 50, carriedBoxY + 132, 34, 11, 0.38, 0, Math.PI * 2);
        c.fill();
        if (inCorridor) {
          c.shadowColor = 'rgba(0, 0, 0, 0.48)';
          c.shadowBlur = 14;
          c.shadowOffsetX = 6;
          c.shadowOffsetY = 6;
        }
        c.drawImage(this.assets.boxCarry, carriedBoxX, carriedBoxY);
        c.restore();
      }
    }

    _drawBayHighlights(c, t) {
      const s = this.state;
      const pulse = 0.55 + 0.45 * Math.sin(t * 7.0);

      for (const [id, info] of Object.entries(BAY_INFO)) {
        const isActiveTarget = (s.targetBay === id && (s.running || s.stepStage > 0));
        const isSelected = (s.selectedBay === id);
        const isHovered = (s.hoveredBay === id);

        if (!isActiveTarget && !isSelected && !isHovered) continue;

        c.save();
        c.beginPath();
        info.quad.forEach((pt, idx) => idx === 0 ? c.moveTo(pt[0], pt[1]) : c.lineTo(pt[0], pt[1]));
        c.closePath();

        if (isActiveTarget) {
          c.fillStyle = `rgba(255, 188, 91, ${0.10 + 0.08 * pulse})`;
          c.fill();
          c.strokeStyle = `rgba(255, 204, 60, ${0.78 + 0.22 * pulse})`;
          c.lineWidth = 3.5;
          c.setLineDash([12, 7]);
          c.stroke();
          c.setLineDash([]);
        } else if (isSelected) {
          c.fillStyle = 'rgba(255, 188, 91, 0.08)';
          c.fill();
          c.strokeStyle = 'rgba(255, 188, 91, 0.80)';
          c.lineWidth = 2.6;
          c.stroke();
        } else if (isHovered) {
          c.fillStyle = 'rgba(166, 233, 255, 0.10)';
          c.fill();
          c.strokeStyle = 'rgba(166, 233, 255, 0.85)';
          c.lineWidth = 2.2;
          c.stroke();
        }
        c.restore();
      }
    }

    _drawBayLabels(c) {
      const s = this.state;
      for (const [id, info] of Object.entries(BAY_INFO)) {
        const isActiveTarget = (s.targetBay === id && (s.running || s.stepStage > 0));
        const isSelected = (s.selectedBay === id);
        const isHovered = (s.hoveredBay === id);
        if (!this.options.showBayLabels && !isHovered && !isActiveTarget) continue;

        const lx = info.labelPos[0];
        const ly = info.labelPos[1] + 8;
        c.save();
        roundRect(c, lx - 20, ly - 11, 40, 22, 6);
        c.fillStyle = isActiveTarget
          ? '#FFBC5B'
          : (isSelected ? 'rgba(15, 23, 42, 0.90)' : 'rgba(15, 23, 42, 0.78)');
        c.fill();
        c.strokeStyle = isActiveTarget
          ? '#0F172A'
          : (isSelected ? '#FFBC5B' : 'rgba(255, 255, 255, 0.28)');
        c.lineWidth = 1.6;
        c.stroke();
        c.fillStyle = isActiveTarget ? '#000000' : (this._isBayVisuallyStocked(id) ? '#A6E9FF' : '#E2E8F0');
        c.font = 'bold 13px "JetBrains Mono", "Liberation Mono", monospace';
        c.textAlign = 'center';
        c.textBaseline = 'middle';
        c.fillText(id, lx, ly + 0.5);
        c.restore();
      }
    }

    _drawForkAssembly(c, cx, beamY, forkExt, carriedBoxY) {
      // Compact physically-accurate telescopic stroke (max 54px so prongs stay 100% inside target bay!)
      const extLen = 14 + forkExt * 40;
      const bx = cx + 14;
      const by = beamY + 46;
      c.save();

      // Blue 360° continuous rotation servo housing tucked cleanly behind carriage plate
      c.fillStyle = '#1D4ED8';
      c.strokeStyle = '#0F172A';
      c.lineWidth = 1.5;
      c.beginPath();
      c.moveTo(bx - 10, by - 40); c.lineTo(bx + 14, by - 52);
      c.lineTo(bx + 14, by - 8);  c.lineTo(bx - 10, by + 4);
      c.closePath();
      c.fill(); c.stroke();

      c.fillStyle = '#3B82F6';
      c.beginPath();
      c.moveTo(bx - 10, by - 40); c.lineTo(bx + 14, by - 52);
      c.lineTo(bx + 6, by - 56);  c.lineTo(bx - 18, by - 44);
      c.closePath();
      c.fill();

      // Dual stainless-steel fork prongs aligned cleanly into wooden pallet fork-pockets
      const forkBaseY = (carriedBoxY !== undefined) ? (carriedBoxY + 128) : (by + 4);
      for (const p of [{ ox: -10, oy: -4 }, { ox: 12, oy: 6 }]) {
        const px0 = bx + p.ox;
        const py0 = forkBaseY + p.oy;
        const px1 = px0 + DEPTH_UX * extLen;
        const py1 = py0 + DEPTH_UY * extLen;

        c.strokeStyle = '#475569';
        c.lineWidth = 9;
        c.lineCap = 'butt';
        c.beginPath(); c.moveTo(px0, py0 + 2.5); c.lineTo(px1, py1 + 2.5); c.stroke();

        const grad = c.createLinearGradient(px0, py0, px1, py1);
        grad.addColorStop(0, '#CBD5E1');
        grad.addColorStop(0.5, '#F1F5F9');
        grad.addColorStop(1, '#94A3B8');
        c.strokeStyle = grad;
        c.lineWidth = 6.5;
        c.beginPath(); c.moveTo(px0, py0); c.lineTo(px1, py1); c.stroke();

        if (forkExt > 0.02 && forkExt < 0.98) {
          c.strokeStyle = 'rgba(45, 212, 191, 0.88)';
          c.lineWidth = 1.8;
          c.beginPath(); c.moveTo(px0, py0 - 2.5); c.lineTo(px1, py1 - 2.5); c.stroke();
        }
      }
      c.restore();
    }

    _drawHorizontalBeam(c, xL, yL, xR, yR, xVis) {
      c.save();
      const dx = xR - xL, dy = yR - yL;
      const len = Math.hypot(dx, dy);
      const ang = Math.atan2(dy, dx);
      c.translate(xL - 8, yL - 8 * (dy / dx));
      c.rotate(ang);

      const drawLen = len + 26;
      const beamH = 34;
      const tex = this.assets.beamTex;
      let xPos = 0;
      while (xPos < drawLen) {
        const segW = Math.min(tex.width, drawLen - xPos);
        c.drawImage(tex, 0, 0, segW, tex.height, xPos, -beamH / 2, segW, beamH);
        xPos += segW;
      }

      // Animated GT2 blue timing belt teeth synced with X-axis travel
      c.save();
      c.beginPath();
      c.rect(8, -7, drawLen - 16, 8);
      c.clip();
      const pitch = 10;
      const shift = ((xVis * 38) % pitch + pitch) % pitch;
      c.fillStyle = 'rgba(96, 165, 250, 0.36)';
      for (let bx = -pitch + shift; bx < drawLen; bx += pitch) {
        c.fillRect(bx, -6, 4, 6);
      }
      c.restore();
      c.restore();
    }

    _drawVerticalBeltMotion(c, zVis, runningZ) {
      c.save();
      c.beginPath();
      c.moveTo(672, 336); c.lineTo(680, 336);
      c.lineTo(685, 824); c.lineTo(677, 824);
      c.closePath();
      c.clip();
      const pitch = 11;
      const shift = ((zVis * 62) % pitch + pitch) % pitch;
      c.fillStyle = runningZ ? 'rgba(147, 197, 253, 0.54)' : 'rgba(147, 197, 253, 0.14)';
      for (let y = 332 - shift; y < 828; y += pitch) {
        const u = (y - 336) / (824 - 336);
        c.fillRect(672 + u * 5, y, 8, 4);
      }
      c.restore();
    }

    _drawDynamicDragChain(c, rightY) {
      c.save();
      const xTop = 742;
      const yTop = Math.min(735, rightY + 42);
      const xBot = 735;
      const yBot = 768;
      if (yBot - yTop < 18) { c.restore(); return; }

      c.strokeStyle = '#1E2022';
      c.lineWidth = 20;
      c.lineCap = 'round';
      c.beginPath();
      c.moveTo(xTop, yTop);
      c.quadraticCurveTo(xTop + 14, (yTop + yBot) / 2, xBot, yBot);
      c.stroke();

      const steps = Math.max(4, Math.floor((yBot - yTop) / 14));
      c.strokeStyle = '#3F444A';
      c.lineWidth = 2.5;
      for (let i = 1; i < steps; i++) {
        const u = i / steps;
        const px = xTop + (xBot - xTop) * u + Math.sin(u * Math.PI) * 7;
        const py = yTop + (yBot - yTop) * u;
        c.beginPath(); c.moveTo(px - 8, py - 2); c.lineTo(px + 8, py + 2); c.stroke();
      }
      c.restore();
    }

    _drawMotionVectors(c, cx, beamY, xL, yL, xR, yR, t) {
      const s = this.state;
      if (s.xDir !== 0) {
        const x1 = cx - s.xDir * 64, x2 = cx + s.xDir * 78;
        const slope = (yR - yL) / (xR - xL);
        const y1 = beamY - 94 + (x1 - cx) * slope;
        const y2 = beamY - 94 + (x2 - cx) * slope;
        const xLbl = s.op === 'HOMING'
          ? 'HOMING X'
          : (s.xDir > 0 ? `X -> ${s.targetBay || s.goalXRev.toFixed(1) + 'r'}` : 'X -> HOME');
        this._drawGlowArrow(c, x1, y1, x2, y2, '#3B82F6', '#60A5FA', t, xLbl, -44, 0);
        this._drawSpinArc(c, 232, yL - 8, 28, s.xDir, t, '#60A5FA');
      }
      if (s.zDir !== 0) {
        const zMidY = clamp(yR, 415, 685);
        const y1 = zMidY + s.zDir * 76, y2 = zMidY - s.zDir * 76;
        let zLbl = s.op === 'HOMING'
          ? 'HOMING Z'
          : (s.zDir > 0 ? `Z -> ${s.targetBay || s.goalZRev.toFixed(1) + 'r'}` : 'Z -> HOME');
        if (s.stepStage === 4) {
          zLbl = s.op === 'STORAGE'
            ? `Z DROP -${s.zOffsetRev.toFixed(2)}r`
            : `Z LIFT +${s.zOffsetRev.toFixed(2)}r`;
        }
        this._drawGlowArrow(c, 646, y1, 646, y2, '#F97316', '#FDBA74', t, zLbl, -14, -96);
        this._drawSpinArc(c, 756, 818, 26, -s.zDir, t, '#FB923C');
      }
      if (s.forkDir !== 0) {
        const ax = cx + 6, ay = beamY + 112;
        const x1 = ax - s.forkDir * DEPTH_UX * 52, y1 = ay - s.forkDir * DEPTH_UY * 52;
        const x2 = ax + s.forkDir * DEPTH_UX * 62, y2 = ay + s.forkDir * DEPTH_UY * 62;
        this._drawGlowArrow(
          c, x1, y1, x2, y2, '#14B8A6', '#5EEAD4', t,
          s.forkDir > 0 ? 'EXTEND 2000us' : 'RETRACT 1000us', 24, 18
        );
        this._drawSpinArc(c, 782, yR + 17, 25, s.forkDir, t, '#2DD4BF');
      }
    }

    _drawGlowArrow(c, x1, y1, x2, y2, color, glowColor, phase, labelText, labelOffY = -40, labelOffX = 0) {
      c.save();
      const dx = x2 - x1, dy = y2 - y1;
      const len = Math.hypot(dx, dy);
      if (len < 10) { c.restore(); return; }
      const ang = Math.atan2(dy, dx);
      c.translate(x1, y1);
      c.rotate(ang);

      c.shadowColor = glowColor;
      c.shadowBlur = 16;
      c.fillStyle = color;
      c.strokeStyle = 'rgba(255,255,255,0.92)';
      c.lineWidth = 2.2;

      const hw = 10, headL = 28, headW = 22;
      c.beginPath();
      c.moveTo(0, -hw);
      c.lineTo(len - headL, -hw);
      c.lineTo(len - headL, -headW);
      c.lineTo(len, 0);
      c.lineTo(len - headL, headW);
      c.lineTo(len - headL, hw);
      c.lineTo(0, hw);
      c.closePath();
      c.globalAlpha = 0.88; c.fill();
      c.globalAlpha = 1.0;  c.stroke();

      c.shadowBlur = 0;
      c.strokeStyle = 'rgba(255,255,255,0.94)';
      c.lineWidth = 3;
      const chevSpacing = 22;
      const offset = (phase * 110) % chevSpacing;
      for (let px = offset + 6; px < len - headL - 4; px += chevSpacing) {
        c.beginPath();
        c.moveTo(px - 6, -6);
        c.lineTo(px + 2, 0);
        c.lineTo(px - 6, 6);
        c.stroke();
      }
      c.restore();

      if (labelText) {
        const mx = (x1 + x2) / 2 + labelOffX;
        const my = (y1 + y2) / 2;
        c.save();
        c.font = 'bold 17px "JetBrains Mono", "Liberation Mono", monospace';
        const tw = c.measureText(labelText).width;
        const pw = tw + 18, ph = 28;
        const bx = mx - pw / 2, by = my + labelOffY;
        c.fillStyle = 'rgba(15, 23, 42, 0.92)';
        c.strokeStyle = color;
        c.lineWidth = 2.2;
        roundRect(c, bx, by, pw, ph, 7);
        c.fill(); c.stroke();
        c.fillStyle = '#FFFFFF';
        c.textAlign = 'center';
        c.textBaseline = 'middle';
        c.fillText(labelText, mx, by + ph / 2 + 1);
        c.restore();
      }
    }

    _drawSpinArc(c, cx, cy, r, dir, tSec, color) {
      c.save();
      c.translate(cx, cy);
      const start = (tSec * 7.2 * dir) % (Math.PI * 2);
      c.strokeStyle = color;
      c.shadowColor = color;
      c.shadowBlur = 10;
      c.lineWidth = 4.5;
      c.lineCap = 'round';
      c.beginPath();
      c.arc(0, 0, r, start, start + Math.PI * 1.15);
      c.stroke();

      const endAng = dir > 0 ? start + Math.PI * 1.15 : start;
      const ex = Math.cos(endAng) * r, ey = Math.sin(endAng) * r;
      const tang = endAng + (dir > 0 ? Math.PI / 2 : -Math.PI / 2);
      c.fillStyle = '#FFFFFF';
      c.beginPath();
      c.moveTo(ex + Math.cos(tang) * 7, ey + Math.sin(tang) * 7);
      c.lineTo(ex + Math.cos(tang + 2.4) * 8, ey + Math.sin(tang + 2.4) * 8);
      c.lineTo(ex + Math.cos(tang - 2.4) * 8, ey + Math.sin(tang - 2.4) * 8);
      c.closePath();
      c.fill();
      c.restore();
    }

    _drawMcuLeds(c) {
      const s = this.state;
      c.save();
      c.fillStyle = '#4ADE80'; c.shadowColor = '#4ADE80'; c.shadowBlur = 10;
      c.beginPath(); c.arc(888, 369, 4.2, 0, Math.PI * 2); c.fill();

      if (s.locked) {
        c.fillStyle = '#FF4D4F'; c.shadowColor = '#FF4D4F'; c.shadowBlur = 18;
        c.beginPath(); c.arc(866, 378, 6.2, 0, Math.PI * 2); c.fill();
      } else if (s.running) {
        c.fillStyle = '#AFFF00'; c.shadowColor = '#AFFF00'; c.shadowBlur = 16;
        c.beginPath(); c.arc(866, 378, 5.2, 0, Math.PI * 2); c.fill();
      }

      if (s._rtdbActive) {
        c.fillStyle = '#38BDF8'; c.shadowColor = '#38BDF8'; c.shadowBlur = 14;
        c.beginPath(); c.arc(908, 360, 4.5, 0, Math.PI * 2); c.fill();
      }
      c.restore();
    }

    _drawHomeMarker(c) {
      const s = this.state;
      const atHome = (s.xVis < 0.08 && s.zVis < 0.08);
      c.save();
      c.strokeStyle = atHome ? '#AFFF00' : 'rgba(15, 23, 42, 0.55)';
      c.lineWidth = 2.2;
      c.beginPath(); c.moveTo(220, 712); c.lineTo(265, 628); c.stroke();

      c.fillStyle = atHome ? '#AFFF00' : 'rgba(255,255,255,0.82)';
      c.strokeStyle = '#0F172A';
      c.lineWidth = 2;
      c.beginPath(); c.arc(265, 628, 6.5, 0, Math.PI * 2); c.fill(); c.stroke();

      c.fillStyle = 'rgba(15, 23, 42, 0.88)';
      roundRect(c, 160, 712, 124, 26, 6); c.fill();
      c.strokeStyle = atHome ? '#AFFF00' : 'rgba(255,255,255,0.22)';
      c.lineWidth = 1.6; c.stroke();

      c.fillStyle = atHome ? '#AFFF00' : '#E2E8F0';
      c.font = 'bold 14px "JetBrains Mono", "Liberation Mono", monospace';
      c.textAlign = 'center';
      c.textBaseline = 'middle';
      c.fillText('HOME (0,0)', 222, 726);
      c.restore();
    }

    _drawLimitSwitchSpark(c, t) {
      const pulse = 0.6 + 0.4 * Math.sin(t * 24);
      c.save();
      c.fillStyle = `rgba(175, 255, 0, ${0.45 * pulse})`;
      c.beginPath(); c.arc(265, 628, 28, 0, Math.PI * 2); c.fill();
      c.fillStyle = '#AFFF00';
      c.beginPath(); c.arc(265, 628, 10, 0, Math.PI * 2); c.fill();
      c.restore();
    }

    _drawEStopOverlay(c, cx, beamY, t) {
      const flash = 0.5 + 0.5 * Math.sin(t * 14);
      c.save();
      c.strokeStyle = `rgba(255, 77, 79, ${0.7 + 0.3 * flash})`;
      c.shadowColor = '#FF4D4F';
      c.shadowBlur = 20;
      c.lineWidth = 4;
      roundRect(c, cx - 64, beamY - 62, 128, 198, 10);
      c.stroke();

      c.shadowBlur = 0;
      const bx = cx - 108, by = Math.max(70, beamY - 122);
      c.fillStyle = 'rgba(185, 28, 28, 0.94)';
      c.strokeStyle = '#FCA5A5';
      c.lineWidth = 2.5;
      roundRect(c, bx, by, 216, 46, 8);
      c.fill(); c.stroke();

      c.fillStyle = '#FFFFFF';
      c.font = 'bold 18px "JetBrains Mono", "Liberation Mono", monospace';
      c.textAlign = 'center';
      c.textBaseline = 'middle';
      c.fillText('E-STOP LOCKED', cx, by + 16);
      c.font = '600 12px "JetBrains Mono", "Liberation Mono", monospace';
      c.fillStyle = '#FEE2E2';
      c.fillText('MOTORS HALTED · SAFE LOCK', cx, by + 35);
      c.restore();
    }

    _drawTopOverlayChips(c, w) {
      const s = this.state;
      c.save();
      const pillH = 24;
      c.font = '700 10px "JetBrains Mono", "Liberation Mono", monospace';

      const isBusy = s.running || s.stepStage > 0;
      const modeTag = s.locked
        ? 'SAFETY LOCK · E-STOP'
        : (s.stepStage > 0
          ? `STEP ${s.stepStage}/6 · ${s.op}`
          : (isBusy && s.op && s.op !== 'NONE' && s.op !== 'IDLE' ? `LIVE · ${s.op}` : 'LIVE DIGITAL TWIN · STANDBY'));
      const leftW = Math.min(w * 0.54, c.measureText(modeTag).width + 28);
      roundRect(c, 10, 10, leftW, pillH, 7);
      c.fillStyle = s.locked ? 'rgba(185, 28, 28, 0.92)' : 'rgba(15, 23, 42, 0.84)';
      c.fill();
      c.strokeStyle = s.locked ? 'rgba(252,165,165,0.6)' : 'rgba(255,255,255,0.20)';
      c.lineWidth = 1;
      c.stroke();

      const dotCol = s.locked ? '#FF4D4F' : (isBusy ? '#FFEA00' : '#AFFF00');
      c.fillStyle = dotCol;
      c.beginPath(); c.arc(21, 22, 3.6, 0, Math.PI * 2); c.fill();
      c.fillStyle = '#FFFFFF';
      c.textAlign = 'left';
      c.textBaseline = 'middle';
      c.fillText(modeTag, 30, 22.5);

      const bayId = (isBusy && s.targetBay) ? s.targetBay : (s.selectedBay || s.targetBay || 'A1');
      const bc = this.coordinates[bayId] || BAY_INFO[bayId];
      const rightTxt = isBusy
        ? `X:${s.xRev.toFixed(2)}r · Z:${s.zRev.toFixed(2)}r`
        : `TARGET: ${bayId} (X ${bc.x.toFixed(1)} · Z ${bc.z.toFixed(1)})`;
      const rightW = c.measureText(rightTxt).width + 18;
      roundRect(c, w - rightW - 10, 10, rightW, pillH, 7);
      c.fillStyle = 'rgba(15, 23, 42, 0.84)';
      c.fill();
      c.strokeStyle = 'rgba(255,255,255,0.20)';
      c.stroke();
      c.fillStyle = isBusy ? '#AFFF00' : '#FFFFFF';
      c.textAlign = 'center';
      c.fillText(rightTxt, w - rightW / 2 - 10, 22.5);
      c.restore();
    }

    _drawBottomLegend(c, w, h) {
      c.save();
      const barH = 24;
      const y = h - barH - 8;
      roundRect(c, 8, y, w - 16, barH, 7);
      c.fillStyle = 'rgba(15, 23, 42, 0.84)';
      c.fill();
      c.strokeStyle = 'rgba(255, 255, 255, 0.18)';
      c.lineWidth = 1;
      c.stroke();

      const compact = w < 440;
      const items = compact
        ? [['#3B82F6', 'X: M1'], ['#F97316', 'Z: M2'], ['#14B8A6', '360° Servo']]
        : [['#3B82F6', 'X-Axis M1 (GPIO 14/27)'], ['#F97316', 'Z-Axis M2 (GPIO 26/25)'], ['#14B8A6', '360° Servo Fork (GPIO 32)']];

      c.font = '600 9.5px "JetBrains Mono", "Liberation Mono", monospace';
      const segW = (w - 32) / 3;
      items.forEach(([col, txt], idx) => {
        const ix = 18 + idx * segW;
        const iy = y + barH / 2;
        c.fillStyle = col;
        roundRect(c, ix, iy - 3.5, 7, 7, 2);
        c.fill();
        c.fillStyle = '#E2E8F0';
        c.textAlign = 'left';
        c.textBaseline = 'middle';
        c.fillText(txt, ix + 12, iy + 0.5);
      });
      c.restore();
    }
  }

  AsrsCadTwin.BAY_INFO = BAY_INFO;
  AsrsCadTwin.ASSET_MANIFEST = ASSET_MANIFEST;
  AsrsCadTwin.getBeamEndpoints = getBeamEndpoints;
  AsrsCadTwin.getCarriagePos = getCarriagePos;
  AsrsCadTwin.trapMotion = trapMotion;

  return AsrsCadTwin;
}));
