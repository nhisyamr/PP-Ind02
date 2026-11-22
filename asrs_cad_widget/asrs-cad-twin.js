/**
 * =============================================================================
 * Proyek Perancangan IND-02 — ASRS 2.5D Photorealistic CAD Digital Twin Engine
 * =============================================================================
 * Self-contained, zero-dependency HTML5 Canvas 2D renderer + kinematic engine
 * designed to drop directly into `asrs_wms_dashboard_(5).html` (or any WMS).
 *
 * Key Capabilities:
 *  - Dynamic 9-bay (A1–C3) stock rendering over an all-empty studio CAD backdrop
 *  - Watertight 14-stage Painter's Algorithm with 3D shelf, divider & frame occlusion
 *  - Exact 1200x896 pixel-locked 2.5D kinematics matching ESP32 firmware:
 *      * Motor 1 (X-Axis, GPIO 14/27): 200 st/rev, belt-driven horizontal gantry
 *      * Motor 2 (Z-Axis, GPIO 26/25): 200 st/rev, vertical lift + dynamic drag chain
 *      * 360° Continuous Servo (GPIO 32): 50Hz 14-bit PWM (1000/1500/2000 us) fork
 *      * Z Lift / Drop Clearance Offset: ±0.40 rev (configurable via telemetry)
 *  - Smooth 60 FPS telemetry interpolation + optional dead-reckoning between
 *    1000 ms Firebase RTDB polls (`updateFromTelemetry()`)
 *  - Built-in AccelStepper trapezoidal demo simulator (`runDemoCycle()`,
 *    `runHomingDemo()`, `triggerEStop()`, `unlock()`)
 *  - Interactive 3D bay hit-testing (`onBayClick(bayId)` hooks to `onUserClickSlot`)
 *
 * Usage:
 *   const twin = new AsrsCadTwin({
 *     canvas: '#asrsCadCanvas',
 *     assetPath: './assets/',
 *     onBayClick: (bayId) => onUserClickSlot(bayId)
 *   });
 *   twin.updateFromTelemetry(telemetryJson, BOX_OCCUPANCY, BOX_COORDINATES);
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

  // Crop viewports inside the 1200x896 studio render
  const CROP_MODES = {
    compact: { x: 142, y: 14, w: 868, h: 882 }, // Matches the left CAD card in asrs_ind02_animation.gif
    wide:    { x: 40,  y: 12, w: 1120, h: 874 } // Full studio frame with generous side margins
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
  // Default firmware coordinates: Col 1/2/3 -> X = 2.0/5.0/8.0 rev; Row A/B/C -> Z = 1.5/4.0/6.5 rev
  const BAY_INFO = {
    A1: { id: 'A1', row: 'A', col: 1, xRev: 2.0, zRev: 1.5, boxRest: { x: 363, y: 500 }, quad: [[338,444],[456,496],[456,664],[338,605]], labelPos: [396,472] },
    A2: { id: 'A2', row: 'A', col: 2, xRev: 5.0, zRev: 1.5, boxRest: { x: 486, y: 556 }, quad: [[462,498],[578,550],[578,724],[462,666]], labelPos: [520,528] },
    A3: { id: 'A3', row: 'A', col: 3, xRev: 8.0, zRev: 1.5, boxRest: { x: 609, y: 612 }, quad: [[584,552],[702,606],[702,786],[584,728]], labelPos: [642,584] },
    B1: { id: 'B1', row: 'B', col: 1, xRev: 2.0, zRev: 4.0, boxRest: { x: 363, y: 332 }, quad: [[338,278],[456,330],[456,494],[338,442]], labelPos: [396,306] },
    B2: { id: 'B2', row: 'B', col: 2, xRev: 5.0, zRev: 4.0, boxRest: { x: 486, y: 388 }, quad: [[462,332],[578,384],[578,548],[462,496]], labelPos: [520,360] },
    B3: { id: 'B3', row: 'B', col: 3, xRev: 8.0, zRev: 4.0, boxRest: { x: 609, y: 444 }, quad: [[584,386],[702,438],[702,604],[584,550]], labelPos: [642,416] },
    C1: { id: 'C1', row: 'C', col: 1, xRev: 2.0, zRev: 6.5, boxRest: { x: 363, y: 160 }, quad: [[338,112],[456,163],[456,328],[338,276]], labelPos: [396,142] },
    C2: { id: 'C2', row: 'C', col: 2, xRev: 5.0, zRev: 6.5, boxRest: { x: 486, y: 214 }, quad: [[462,165],[578,214],[578,380],[462,330]], labelPos: [520,192] },
    C3: { id: 'C3', row: 'C', col: 3, xRev: 8.0, zRev: 6.5, boxRest: { x: 609, y: 270 }, quad: [[584,216],[702,268],[702,436],[584,382]], labelPos: [642,244] }
  };

  const ROW_BAYS = {
    A: ['A1', 'A2', 'A3'],
    B: ['B1', 'B2', 'B3'],
    C: ['C1', 'C2', 'C3']
  };

  // Seated vertical perspective offset (boxRest.y - beamY) across (row, col)
  // Guarantees 0.0px handoff jump between carried box and seated shelf box on ALL 9 bays!
  const PERSP_DY_GRID = [
    [-96.3, -97.2, -98.0], // Row A seated (zVis = 1.10)
    [-94.3, -92.9, -91.4], // Row B seated (zVis = 3.60)
    [-96.3, -94.6, -90.9]  // Row C seated (zVis = 6.10)
  ];

  // Depth unit vector pointing into the rack cubbies (isometric Y-depth axis)
  const DEPTH_UX = 0.889;
  const DEPTH_UY = -0.458;

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
   * Row A seated = 1.10, Row B seated = 3.60, Row C seated = 6.10 (+0.40 lift = 6.50).
   */
  function getBeamEndpoints(zVis) {
    const xL = 262;
    const xR = 695;
    let yL, yR;
    if (zVis <= 1.10) {
      const t = clamp(zVis / 1.10, 0, 1);
      yL = lerp(594, 546, t);
      yR = lerp(796, 746, t);
    } else {
      const u = clamp((zVis - 1.10) / 5.0, 0, 1.15);
      yL = 546 - u * 336; // 67.2 px/rev -> Row C seated (6.10): 210, lift (6.50): 183
      yR = 746 - u * 352; // 70.4 px/rev -> Row C seated (6.10): 394, lift (6.50): 366
    }
    return { xL, yL, xR, yR };
  }

  /**
   * Piecewise X-axis carriage center along the slanted horizontal beam.
   * X = 0 -> 320 (clear of left rail), Col 1 (2.0) -> 371, Col 2 (5.0) -> 494, Col 3 (8.0) -> 617.
   */
  function getCarriagePos(xVis, zVis) {
    const { xL, yL, xR, yR } = getBeamEndpoints(zVis);
    const xClamped = clamp(xVis, 0, 8.6);
    const cx = xClamped <= 2.0
      ? (320 + xClamped * 25.5)
      : (371 + (xClamped - 2.0) * 41.0);
    const u = (cx - xL) / (xR - xL);
    const beamY = yL + u * (yR - yL);
    return { cx, beamY, u, slope: (yR - yL) / (xR - xL) };
  }

  /**
   * Bilinearly interpolated carried-box vertical perspective offset across (xVis, zVis).
   */
  function getCarriedPerspDy(xVis, zVis) {
    const cPos = clamp((xVis - 2.0) / 3.0, 0, 2); // 0=Col1, 1=Col2, 2=Col3
    const rPos = clamp((zVis - 1.10) / 2.5, 0, 2); // 0=RowA, 1=RowB, 2=RowC
    const c0 = Math.floor(cPos), c1 = Math.min(2, c0 + 1), tc = cPos - c0;
    const r0 = Math.floor(rPos), r1 = Math.min(2, r0 + 1), tr = rPos - r0;
    const top = lerp(PERSP_DY_GRID[r0][c0], PERSP_DY_GRID[r0][c1], tc);
    const bot = lerp(PERSP_DY_GRID[r1][c0], PERSP_DY_GRID[r1][c1], tc);
    return lerp(top, bot, tr);
  }

  /**
   * Point-in-polygon ray casting for interactive 3D bay click/hover selection.
   */
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
        predictBetweenPolls: options.predictBetweenPolls !== false,
        enableClickToSelect: options.enableClickToSelect !== false,
        autoStart: options.autoStart !== false,
        onBayClick: typeof options.onBayClick === 'function' ? options.onBayClick : null,
        onStateChange: typeof options.onStateChange === 'function' ? options.onStateChange : null,
        onReady: typeof options.onReady === 'function' ? options.onReady : null
      };

      // Normalize trailing slash on assetPath
      if (this.options.assetPath && !this.options.assetPath.endsWith('/')) {
        this.options.assetPath += '/';
      }

      // Dynamic coordinate map (defaults match firmware BOX_X_REV / BOX_Z_REV)
      this.coordinates = {};
      for (const [id, info] of Object.entries(BAY_INFO)) {
        this.coordinates[id] = { x: info.xRev, z: info.zRev };
      }
      if (options.coordinates) this.setCoordinates(options.coordinates);

      // Dynamic 9-bay occupancy map (false = empty, true = stocked)
      this.occ = {
        A1: false, A2: false, A3: false,
        B1: false, B2: false, B3: false,
        C1: false, C2: false, C3: false
      };
      if (options.initialOccupancy) this.setOccupancy(options.initialOccupancy);

      // Rendered kinematic & telemetry state
      this.state = {
        mode: 'live',          // 'live' | 'demo' | 'manual'
        xRev: 0.0,             // smoothed rendered X (firmware rev)
        zRev: 0.0,             // smoothed rendered Z (firmware rev)
        zVisOverride: null,    // optional direct visual Z override
        forkExt: 0.0,          // 0 = retracted, 1 = extended into rack
        targetXRev: 0.0,       // target from latest telemetry
        targetZRev: 0.0,
        targetForkExt: 0.0,
        servoState: 'STOP',    // STOP | EXTEND | RETRACT
        servoExtRev: 1.5,
        servoRetRev: 1.5,
        servoMsRev: 1000,
        zOffsetRev: 0.40,
        rpm: 60.0,
        accel: 800.0,
        stepStage: 0,          // 0..6
        op: 'NONE',            // NONE | STORAGE | RETRIEVAL | HOMING | E-STOP | MANUAL
        targetBay: null,       // active operation bay ('A1'..'C3')
        selectedBay: 'A1',     // WMS selected bay highlight
        hoveredBay: null,      // pointer hover highlight
        running: false,
        locked: false,
        homingSwitchHit: false,
        carriedBox: false,
        tempShowShelfBox: null, // bay newly dropped during stage 5..6 before WMS commits occ
        tempHideShelfBox: null, // bay newly lifted during stage 4..6 before WMS commits occ
        xDir: 0,
        zDir: 0,
        forkDir: 0,
        statusMsg: 'STANDBY — AT HOME (0,0)',
        online: false,
        lastTelemetryMs: 0,
        rtdbPulseUntil: 0
      };

      // Active offline demo choreography descriptor (when mode === 'demo')
      this._demo = null;

      // Cached viewport transform for responsive mouse/touch hit-testing
      this._view = { width: 574, height: 548, scale: 0.62, offX: 0, offY: 0, dpr: 1 };

      // Asset loading state
      this.assets = {};
      this.assetsReady = false;
      this.assetError = null;

      // Animation loop handle
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
    // PUBLIC INTEGRATION API (FOR asrs_wms_dashboard_(5).html)
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
     * Direct hook for `fetchTelemetry()` in `asrs_wms_dashboard_(5).html`.
     * Feeds raw Firebase `/asrs/telemetry.json` payload + optional `BOX_OCCUPANCY` & `BOX_COORDINATES`.
     */
    updateFromTelemetry(data, occMap = null, coordMap = null) {
      if (occMap) this.setOccupancy(occMap);
      if (coordMap) this.setCoordinates(coordMap);
      if (!data) {
        this.state.online = false;
        return;
      }

      this.state.mode = 'live';
      this._demo = null;
      this.state.online = true;
      const now = performance.now();
      this.state.lastTelemetryMs = now;
      this.state.rtdbPulseUntil = now + 380;

      const s = this.state;
      const prevStage = s.stepStage;
      const prevOp = s.op;

      s.locked = !!data.locked;
      s.running = !!data.running;
      s.rpm = Number(data.rpm || s.rpm || 60);
      s.accel = Number(data.accel || s.accel || 800);
      s.servoExtRev = Number(data.servo_ext_rev || s.servoExtRev || 1.5);
      s.servoRetRev = Number(data.servo_ret_rev || s.servoRetRev || 1.5);
      s.servoMsRev = Number(data.servo_ms_rev || s.servoMsRev || 1000);
      if (data.z_offset !== undefined && !isNaN(parseFloat(data.z_offset))) {
        s.zOffsetRev = parseFloat(data.z_offset);
      }

      const rawBox = String(data.box || 'NONE').toUpperCase();
      s.targetBay = BAY_INFO[rawBox] ? rawBox : null;
      s.op = String(data.op || 'NONE').toUpperCase();
      s.stepStage = Number(data.step_stage || 0);
      s.servoState = String(data.servo_state || 'STOP').toUpperCase();
      s.statusMsg = String(data.status || (s.locked ? 'EMERGENCY STOP (LOCKED)' : 'ONLINE')).toUpperCase();
      s.zVisOverride = null;

      // Authoritative encoder positions from ESP32 AccelStepper
      if (data.m1_rev !== undefined && !isNaN(Number(data.m1_rev))) {
        s.targetXRev = Number(data.m1_rev);
      }
      if (data.m2_rev !== undefined && !isNaN(Number(data.m2_rev))) {
        s.targetZRev = Number(data.m2_rev);
      }

      // Snap immediately if this is the very first telemetry packet or if locked
      if (prevOp === 'NONE' && !s.running && s.stepStage === 0) {
        s.xRev = s.targetXRev;
        s.zRev = s.targetZRev;
      }

      // Infer servo fork target position from firmware stage + servo_state
      if (s.servoState === 'EXTEND') {
        s.targetForkExt = 1.0;
      } else if (s.servoState === 'RETRACT') {
        s.targetForkExt = 0.0;
      } else if (s.stepStage === 4) {
        s.targetForkExt = 1.0; // Stage 4 (Z lift/drop) always executes with fork fully extended
        s.forkDir = 0;
      } else if (s.stepStage === 0 || s.stepStage === 1 || s.stepStage === 2 || s.stepStage === 6) {
        s.targetForkExt = 0.0;
        if (s.servoState === 'STOP') s.forkDir = 0;
      }
      if (s.servoState === 'STOP' && Math.abs(s.targetForkExt - s.forkExt) < 0.02) {
        s.forkDir = 0;
      }

      // Infer physical carried box state & transient shelf handoff
      s.tempShowShelfBox = null;
      s.tempHideShelfBox = null;
      if (s.op === 'STORAGE' && s.targetBay) {
        if (s.stepStage >= 1 && s.stepStage <= 4) {
          s.carriedBox = true;
          s.tempHideShelfBox = s.targetBay;
        } else if (s.stepStage === 5 || s.stepStage === 6) {
          s.carriedBox = false;
          s.tempShowShelfBox = s.targetBay;
        } else if (prevStage >= 5 && s.stepStage === 0) {
          s.carriedBox = false;
        }
      } else if (s.op === 'RETRIEVAL' && s.targetBay) {
        if (s.stepStage >= 1 && s.stepStage <= 3) {
          s.carriedBox = false;
        } else if (s.stepStage >= 4 && s.stepStage <= 6) {
          s.carriedBox = true;
          s.tempHideShelfBox = s.targetBay;
        } else if (prevStage >= 5 && s.stepStage === 0) {
          s.carriedBox = true; // arrived at Home I/O port with retrieved box
        }
      } else if (!s.running && s.stepStage === 0 && !s.statusMsg.includes('RETRIEV')) {
        s.carriedBox = false;
      }

      // Detect limit switch homing hit from firmware status banner
      s.homingSwitchHit = s.statusMsg.includes('LS_X_RIGHT') ||
        (s.statusMsg.includes('HOMING COMPLETE') && s.xRev < 0.05 && s.zRev < 0.05);

      if (this.options.onStateChange) this.options.onStateChange(this.getSnapshot());
    }

    /**
     * Direct manual pose setter (great for Settings -> Manual Jog preview).
     */
    setPose(xRev, zRev, forkExt = 0, carriedBox = false) {
      this.state.mode = 'manual';
      this._demo = null;
      this.state.targetXRev = clamp(Number(xRev) || 0, 0, 8.6);
      this.state.targetZRev = clamp(Number(zRev) || 0, 0, 7.2);
      this.state.targetForkExt = clamp(Number(forkExt) || 0, 0, 1);
      this.state.carriedBox = !!carriedBox;
      this.state.zVisOverride = null;
      this.state.op = 'MANUAL';
      this.state.stepStage = 0;
      this.state.statusMsg = `MANUAL POSE -> X:${this.state.targetXRev.toFixed(2)}r  Z:${this.state.targetZRev.toFixed(2)}r`;
    }

    // -------------------------------------------------------------------------
    // BUILT-IN OFFLINE DEMO SIMULATOR (ALL 9 BAYS + HOMING + E-STOP)
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
      const duration = Number(opts.duration || (rpm >= 100 ? 7.5 : 10.5));
      this.state.mode = 'demo';
      this.state.selectedBay = bay.id;
      this.state.targetBay = bay.id;
      this.state.op = op;
      this.state.rpm = rpm;
      this.state.homingSwitchHit = false;
      this.state.zVisOverride = null;

      // Ensure realistic starting shelf occupancy for demo clarity
      if (op === 'RETRIEVAL') this.occ[bay.id] = true;
      if (op === 'STORAGE') this.occ[bay.id] = false;

      this._demo = {
        kind: 'cycle',
        op,
        bayId: bay.id,
        tx: this.coordinates[bay.id].x,
        tz: this.coordinates[bay.id].z,
        t: 0,
        dur: duration,
        committed: false,
        onComplete: typeof opts.onComplete === 'function' ? opts.onComplete : null
      };
      return true;
    }

    runHomingDemo(opts = {}) {
      if (this.state.locked) {
        this.state.statusMsg = 'REJECTED: SYSTEM LOCKED — SEND UNLOCK FIRST';
        return false;
      }
      const startX = this.state.xRev > 0.3 ? this.state.xRev : 4.6;
      const startZ = this.state.zRev > 0.3 ? this.state.zRev : 3.8;
      this.state.mode = 'demo';
      this.state.op = 'HOMING';
      this.state.targetBay = null;
      this.state.carriedBox = false;
      this.state.forkExt = 0;
      this._demo = {
        kind: 'homing',
        startX,
        startZ,
        t: 0,
        dur: Number(opts.duration || 4.8),
        onComplete: typeof opts.onComplete === 'function' ? opts.onComplete : null
      };
      return true;
    }

    triggerEStop() {
      this.state.locked = true;
      this.state.running = false;
      this.state.xDir = 0;
      this.state.zDir = 0;
      this.state.forkDir = 0;
      this.state.servoState = 'STOP';
      this.state.op = 'E-STOP';
      this.state.statusMsg = 'EMERGENCY STOP (LOCKED) — MOTORS HALTED';
      this._demo = null;
      if (this.options.onStateChange) this.options.onStateChange(this.getSnapshot());
    }

    unlock() {
      this.state.locked = false;
      this.state.op = 'IDLE';
      this.state.statusMsg = 'IDLE (UNLOCKED) — SAFETY LOCK RELEASED';
      if (this.options.onStateChange) this.options.onStateChange(this.getSnapshot());
    }

    getSnapshot() {
      return {
        xRev: this.state.xRev,
        zRev: this.state.zRev,
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
    // 4. FRAME PHYSICS STEPPER (60 FPS LIVE INTERPOLATION OR DEMO CHOREOGRAPHY)
    // -------------------------------------------------------------------------
    _step(dt, nowMs) {
      const s = this.state;
      if (s.locked) {
        s.running = false;
        s.xDir = 0; s.zDir = 0; s.forkDir = 0;
        return;
      }

      if (s.mode === 'demo' && this._demo) {
        this._stepDemo(dt);
        return;
      }

      // --- LIVE TELEMETRY & MANUAL POSE SMOOTHING ---
      const revPerSec = Math.max(0.6, (s.rpm || 60) / 60);
      const maxDelta = revPerSec * 1.45 * dt;

      // Optional dead-reckoning between 1000ms Firebase polls during active ASRS stages
      if (s.mode === 'live' && this.options.predictBetweenPolls && s.running && s.targetBay) {
        const bayC = this.coordinates[s.targetBay] || BAY_INFO[s.targetBay];
        if (s.stepStage === 1 && bayC) {
          s.targetXRev = this._advanceToward(s.targetXRev, bayC.x, revPerSec * dt);
        } else if (s.stepStage === 2 && bayC) {
          s.targetZRev = this._advanceToward(s.targetZRev, bayC.z, revPerSec * dt);
        } else if (s.stepStage === 4 && bayC) {
          const zGoal = s.op === 'STORAGE' ? (bayC.z - s.zOffsetRev) : (bayC.z + s.zOffsetRev);
          s.targetZRev = this._advanceToward(s.targetZRev, zGoal, revPerSec * 0.7 * dt);
        } else if (s.stepStage === 6) {
          s.targetXRev = this._advanceToward(s.targetXRev, 0, revPerSec * dt);
          s.targetZRev = this._advanceToward(s.targetZRev, 0, revPerSec * dt);
        }
      }

      // Smooth damped motion toward targets
      const dx = s.targetXRev - s.xRev;
      const dz = s.targetZRev - s.zRev;
      const smoothAlpha = 1 - Math.exp(-9.0 * dt);

      if (Math.abs(dx) > 0.002) {
        const stepX = clamp(dx * smoothAlpha, -maxDelta, maxDelta);
        s.xRev += Math.abs(stepX) > Math.abs(dx) ? dx : stepX;
        s.xDir = dx > 0 ? 1 : -1;
      } else {
        s.xRev = s.targetXRev;
        s.xDir = 0;
      }

      if (Math.abs(dz) > 0.002) {
        const stepZ = clamp(dz * smoothAlpha, -maxDelta, maxDelta);
        s.zRev += Math.abs(stepZ) > Math.abs(dz) ? dz : stepZ;
        s.zDir = dz > 0 ? 1 : -1;
      } else {
        s.zRev = s.targetZRev;
        s.zDir = 0;
      }

      // Smooth 360° continuous servo fork stroke
      const extDurSec = Math.max(0.35, (s.servoExtRev * s.servoMsRev) / 1000);
      const retDurSec = Math.max(0.35, (s.servoRetRev * s.servoMsRev) / 1000);
      const df = s.targetForkExt - s.forkExt;
      if (Math.abs(df) > 0.003) {
        const rate = df > 0 ? (dt / extDurSec) : (dt / retDurSec);
        s.forkExt += Math.sign(df) * Math.min(Math.abs(df), rate);
        s.forkDir = df > 0 ? 1 : -1;
      } else {
        s.forkExt = s.targetForkExt;
        s.forkDir = 0;
      }

      // Keep RTDB pulse alive briefly after each packet
      s._rtdbActive = nowMs < s.rtdbPulseUntil;
    }

    _advanceToward(cur, goal, step) {
      const diff = goal - cur;
      if (Math.abs(diff) <= step) return goal;
      return cur + Math.sign(diff) * step;
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
          s.xRev = d.startX * (1 - p);
          s.zRev = d.startZ * (1 - p);
          s.running = true;
          s.xDir = s.xRev > 0.02 ? -1 : 0;
          s.zDir = s.zRev > 0.02 ? -1 : 0;
          s.homingSwitchHit = false;
          s.statusMsg = 'HOMING (CONTINUOUS SPIN -> LS_X_RIGHT)';
        } else if (u < 0.94) {
          s.xRev = 0; s.zRev = 0;
          s.running = false;
          s.homingSwitchHit = true;
          s.statusMsg = 'LS_X_RIGHT (GPIO 19) HIT -> ZEROED (0,0)';
        } else {
          s.xRev = 0; s.zRev = 0;
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

      // --- 6-STEP ACCELSTEPPER ASRS DEMO CYCLE ---
      const u = clamp(d.t / d.dur, 0, 1);
      const isStore = (d.op === 'STORAGE');
      const tx = d.tx, tz = d.tz, zOff = s.zOffsetRev || 0.40;

      // Exact fractional stage windows summing to 1.0
      const t1 = 0.05; // Dispatch
      const t2 = 0.22; // Step 1: Move X
      const t3 = 0.40; // Step 2: Move Z
      const t4 = 0.54; // Step 3: Servo Extend + 150ms Settle
      const t5 = 0.64; // Step 4: Z Drop (-0.40) or Lift (+0.40)
      const t6 = 0.77; // Step 5: Servo Retract + 150ms Settle
      const t7 = 0.94; // Step 6: Simultaneous Home (0,0)

      s.xDir = 0; s.zDir = 0; s.forkDir = 0;
      s.servoState = 'STOP';
      s.tempShowShelfBox = null;
      s.tempHideShelfBox = null;

      if (u < t1) {
        s.xRev = 0; s.zRev = 0; s.forkExt = 0;
        s.stepStage = 0; s.running = false;
        s.carriedBox = isStore;
        s.statusMsg = `DISPATCH -> ${isStore ? 'STORE' : 'RETRIEVE'}:${d.bayId}`;
      } else if (u < t2) {
        const m = trapMotion((u - t1) / (t2 - t1), 0.22);
        s.xRev = tx * m.pos; s.zRev = 0; s.forkExt = 0;
        s.stepStage = 1; s.running = true; s.xDir = 1;
        s.carriedBox = isStore;
        s.statusMsg = `STEP 1/6: MOVING X -> ${d.bayId} (${tx.toFixed(1)} rev)`;
      } else if (u < t3) {
        const m = trapMotion((u - t2) / (t3 - t2), 0.22);
        s.xRev = tx; s.zRev = tz * m.pos; s.forkExt = 0;
        s.stepStage = 2; s.running = true; s.zDir = 1;
        s.carriedBox = isStore;
        s.statusMsg = `STEP 2/6: MOVING Z -> ${d.bayId} (${tz.toFixed(1)} rev)`;
      } else if (u < t4) {
        const frac = (u - t3) / (t4 - t3);
        const settling = frac >= 0.86;
        s.xRev = tx; s.zRev = tz;
        s.forkExt = easeInOut(clamp(frac / 0.86, 0, 1));
        s.stepStage = 3; s.running = true;
        s.servoState = settling ? 'STOP' : 'EXTEND';
        s.forkDir = settling ? 0 : 1;
        s.carriedBox = isStore;
        s.statusMsg = settling ? 'STEP 3/6: SERVO SETTLE (150ms)' : 'STEP 3/6: SERVO EXTEND (2000us)';
      } else if (u < t5) {
        const m = trapMotion((u - t4) / (t5 - t4), 0.28);
        s.xRev = tx; s.forkExt = 1.0;
        s.stepStage = 4; s.running = true;
        if (isStore) {
          s.zRev = tz - zOff * m.pos;
          s.zDir = -1;
          s.carriedBox = true;
          s.statusMsg = `STEP 4/6: Z DROP (-${zOff.toFixed(2)}r) -> ${d.bayId}`;
        } else {
          s.zRev = tz + zOff * m.pos;
          s.zDir = 1;
          s.carriedBox = true;
          s.tempHideShelfBox = d.bayId;
          s.statusMsg = `STEP 4/6: Z LIFT (+${zOff.toFixed(2)}r) <- ${d.bayId}`;
        }
      } else if (u < t6) {
        const frac = (u - t5) / (t6 - t5);
        const settling = frac >= 0.86;
        s.xRev = tx;
        s.zRev = isStore ? (tz - zOff) : (tz + zOff);
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
        const zStart = isStore ? (tz - zOff) : (tz + zOff);
        const maxDist = Math.max(tx, zStart, 0.1);
        const frac = (u - t6) / (t7 - t6);
        const ux = clamp(frac / (tx / maxDist), 0, 1);
        const uz = clamp(frac / (zStart / maxDist), 0, 1);
        const mx = trapMotion(ux, 0.22);
        const mz = trapMotion(uz, 0.22);
        s.xRev = tx * (1 - mx.pos);
        s.zRev = zStart * (1 - mz.pos);
        s.forkExt = 0;
        s.stepStage = 6; s.running = true;
        s.xDir = ux < 1 ? -1 : 0;
        s.zDir = uz < 1 ? -1 : 0;
        if (isStore) { s.carriedBox = false; s.tempShowShelfBox = d.bayId; }
        else { s.carriedBox = true; s.tempHideShelfBox = d.bayId; }
        s.statusMsg = 'STEP 6/6: RETURNING HOME (0,0)';
      } else {
        s.xRev = 0; s.zRev = 0; s.forkExt = 0;
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
    // 5. COORDINATE NORMALIZATION (SUPPORTS CUSTOM WMS BOX_COORDINATES!)
    // -------------------------------------------------------------------------
    _normalizeX(rawX) {
      // Compute current average X per column from configured coordinates
      const c1 = (this.coordinates.A1.x + this.coordinates.B1.x + this.coordinates.C1.x) / 3 || 2.0;
      const c2 = (this.coordinates.A2.x + this.coordinates.B2.x + this.coordinates.C2.x) / 3 || 5.0;
      const c3 = (this.coordinates.A3.x + this.coordinates.B3.x + this.coordinates.C3.x) / 3 || 8.0;
      if (rawX <= c1) return (rawX / (c1 || 2.0)) * 2.0;
      if (rawX <= c2) return 2.0 + ((rawX - c1) / Math.max(0.1, c2 - c1)) * 3.0;
      return 5.0 + ((rawX - c2) / Math.max(0.1, c3 - c2)) * 3.0;
    }

    _mapVisualZ(rawZ) {
      const s = this.state;
      if (s.zVisOverride !== null) return s.zVisOverride;

      // Normalize configured row Z heights onto canonical visual rows (1.5, 4.0, 6.5)
      const rA = (this.coordinates.A1.z + this.coordinates.A2.z + this.coordinates.A3.z) / 3 || 1.5;
      const rB = (this.coordinates.B1.z + this.coordinates.B2.z + this.coordinates.B3.z) / 3 || 4.0;
      const rC = (this.coordinates.C1.z + this.coordinates.C2.z + this.coordinates.C3.z) / 3 || 6.5;

      let zNorm;
      if (rawZ <= rA) zNorm = (rawZ / (rA || 1.5)) * 1.5;
      else if (rawZ <= rB) zNorm = 1.5 + ((rawZ - rA) / Math.max(0.1, rB - rA)) * 2.5;
      else zNorm = 4.0 + ((rawZ - rB) / Math.max(0.1, rC - rB)) * 2.5;

      const b = s.targetBay ? BAY_INFO[s.targetBay] : null;
      const zOff = s.zOffsetRev || 0.40;
      if (!b) return clamp(zNorm * (6.10 / 6.50), 0, 6.55);

      const baseZ = b.zRev;          // 1.50 | 4.00 | 6.50
      const seatedZ = baseZ - 0.40;  // 1.10 | 3.60 | 6.10
      if (s.op === 'STORAGE') {
        // Approaches hovering +0.40 above shelf, then drops -0.40 onto shelf floor
        const delta = zNorm - baseZ;
        return clamp(baseZ + delta * (0.40 / Math.max(0.05, zOff)), 0, 6.60);
      } else if (s.op === 'RETRIEVAL') {
        // Approaches at seated height (fork enters under pallet), then lifts +0.40 off shelf
        if (zNorm <= baseZ) return clamp(zNorm * (seatedZ / baseZ), 0, 6.50);
        return clamp(seatedZ + (zNorm - baseZ) * (0.40 / Math.max(0.05, zOff)), 0, 6.60);
      }
      return clamp(zNorm * (6.10 / 6.50), 0, 6.55);
    }

    // -------------------------------------------------------------------------
    // 6. 14-LAYER PAINTER'S ALGORITHM 2.5D RENDERER
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

      // LAYER 2: Interactive Bay Highlights (Selected / Hovered / Active Operation)
      this._drawBayHighlights(c, t);

      // Compute current gantry, carriage & carried-box geometry
      const xVis = this._normalizeX(s.xRev);
      const zVis = this._mapVisualZ(s.zRev);
      const { xL, yL, xR, yR } = getBeamEndpoints(zVis);
      const { cx, beamY } = getCarriagePos(xVis, zVis);

      const perspDy = getCarriedPerspDy(xVis, zVis);
      const forkDx = 30;
      const forkDy = -15;
      const carriedBoxX = (cx - 8) - (1 - s.forkExt) * forkDx;
      const carriedBoxY = (beamY + perspDy) - (1 - s.forkExt) * forkDy;

      // Determine whether the carried box is currently slotted INSIDE a specific row
      const insideRow = (s.forkExt > 0.35 && s.targetBay && BAY_INFO[s.targetBay])
        ? BAY_INFO[s.targetBay].row
        : null;

      // LAYERS 3–8: Bottom-to-Top Row Painter's Order with Interleaved 3D Shelf Occlusion
      // Row A boxes -> (optional inside-Row-A fork+box) -> Shelf A ceiling lip
      this._drawShelfRow(c, 'A');
      if (insideRow === 'A') this._drawForkAndCarriedBox(c, cx, beamY, s.forkExt, carriedBoxX, carriedBoxY);
      c.drawImage(this.assets.maskShelfA, 330, 415);

      // Row B boxes -> (optional inside-Row-B fork+box) -> Shelf B ceiling lip
      this._drawShelfRow(c, 'B');
      if (insideRow === 'B') this._drawForkAndCarriedBox(c, cx, beamY, s.forkExt, carriedBoxX, carriedBoxY);
      c.drawImage(this.assets.maskShelfB, 330, 250);

      // Row C boxes -> (optional inside-Row-C fork+box) -> Top cabinet roof lip
      this._drawShelfRow(c, 'C');
      if (insideRow === 'C') this._drawForkAndCarriedBox(c, cx, beamY, s.forkExt, carriedBoxX, carriedBoxY);
      c.drawImage(this.assets.maskRoofC, 330, 70);

      // LAYER 9: Full-Height Vertical Column Divider Masks (Wall 1 & Wall 2)
      c.drawImage(this.assets.maskWall1, 454, 154);
      c.drawImage(this.assets.maskWall2, 576, 208);

      // If fork is retracted / travelling outside the rack, draw fork + carried box IN FRONT of all dividers & shelves
      if (!insideRow) {
        this._drawForkAndCarriedBox(c, cx, beamY, s.forkExt, carriedBoxX, carriedBoxY);
      }

      // LAYER 10: Fixed Outer Frame Occlusion (Top Crossbar + Right Frame / Pulley / Rail)
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

      // Optional crisp HTML-style HUD overlay chips rendered right on canvas
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

    _drawShelfRow(c, rowKey) {
      const ids = ROW_BAYS[rowKey];
      for (const id of ids) {
        if (!this._isBayVisuallyStocked(id)) continue;
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
    }

    _drawForkAndCarriedBox(c, cx, beamY, forkExt, carriedBoxX, carriedBoxY) {
      this._drawForkAssembly(c, cx, beamY, forkExt);
      if (this.state.carriedBox) {
        c.save();
        c.fillStyle = 'rgba(0, 0, 0, 0.25)';
        c.beginPath();
        c.ellipse(carriedBoxX + 52, carriedBoxY + 130, 36, 12, 0.38, 0, Math.PI * 2);
        c.fill();
        c.restore();
        c.drawImage(this.assets.boxCarry, carriedBoxX, carriedBoxY);
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

    _drawForkAssembly(c, cx, beamY, forkExt) {
      const extLen = 36 + forkExt * 68;
      const bx = cx + 22;
      const by = beamY + 56;
      c.save();

      // Blue 360° continuous rotation servo housing behind carriage
      c.fillStyle = '#1D4ED8';
      c.strokeStyle = '#0F172A';
      c.lineWidth = 1.5;
      c.beginPath();
      c.moveTo(bx - 12, by - 44); c.lineTo(bx + 18, by - 58);
      c.lineTo(bx + 18, by - 8);  c.lineTo(bx - 12, by + 6);
      c.closePath();
      c.fill(); c.stroke();

      c.fillStyle = '#3B82F6';
      c.beginPath();
      c.moveTo(bx - 12, by - 44); c.lineTo(bx + 18, by - 58);
      c.lineTo(bx + 8, by - 63);  c.lineTo(bx - 22, by - 49);
      c.closePath();
      c.fill();

      // Dual telescopic stainless-steel fork prongs
      for (const p of [{ ox: -16, oy: -4 }, { ox: 12, oy: 9 }]) {
        const px0 = bx + p.ox, py0 = by + p.oy;
        const px1 = px0 + DEPTH_UX * extLen, py1 = py0 + DEPTH_UY * extLen;
        c.strokeStyle = '#475569';
        c.lineWidth = 11;
        c.lineCap = 'butt';
        c.beginPath(); c.moveTo(px0, py0 + 3); c.lineTo(px1, py1 + 3); c.stroke();

        const grad = c.createLinearGradient(px0, py0, px1, py1);
        grad.addColorStop(0, '#CBD5E1');
        grad.addColorStop(0.5, '#F1F5F9');
        grad.addColorStop(1, '#94A3B8');
        c.strokeStyle = grad;
        c.lineWidth = 8;
        c.beginPath(); c.moveTo(px0, py0); c.lineTo(px1, py1); c.stroke();

        if (forkExt > 0.02 && forkExt < 0.98) {
          c.strokeStyle = 'rgba(45, 212, 191, 0.88)';
          c.lineWidth = 2;
          c.beginPath(); c.moveTo(px0, py0 - 3); c.lineTo(px1, py1 - 3); c.stroke();
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

      // Animated GT2 blue timing belt teeth synced with X-axis revolutions
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
          : (s.xDir > 0 ? `X -> ${s.targetBay || s.targetXRev.toFixed(1) + 'r'}` : 'X -> HOME');
        this._drawGlowArrow(c, x1, y1, x2, y2, '#3B82F6', '#60A5FA', t, xLbl, -44, 0);
        this._drawSpinArc(c, 232, yL - 8, 28, s.xDir, t, '#60A5FA');
      }
      if (s.zDir !== 0) {
        const zMidY = clamp(yR, 415, 685);
        const y1 = zMidY + s.zDir * 76, y2 = zMidY - s.zDir * 76;
        let zLbl = s.op === 'HOMING'
          ? 'HOMING Z'
          : (s.zDir > 0 ? `Z -> ${s.targetBay || s.targetZRev.toFixed(1) + 'r'}` : 'Z -> HOME');
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

      // Animated traveling chevrons along arrow shaft
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
      // Power LED (green)
      c.fillStyle = '#4ADE80'; c.shadowColor = '#4ADE80'; c.shadowBlur = 10;
      c.beginPath(); c.arc(888, 369, 4.2, 0, Math.PI * 2); c.fill();

      // IND1/IND2 (GPIO 15/2): Red when E-Stop locked, Lime when running
      if (s.locked) {
        c.fillStyle = '#FF4D4F'; c.shadowColor = '#FF4D4F'; c.shadowBlur = 18;
        c.beginPath(); c.arc(866, 378, 6.2, 0, Math.PI * 2); c.fill();
      } else if (s.running) {
        c.fillStyle = '#AFFF00'; c.shadowColor = '#AFFF00'; c.shadowBlur = 16;
        c.beginPath(); c.arc(866, 378, 5.2, 0, Math.PI * 2); c.fill();
      }

      // Firebase RTDB Sync LED (cyan pulse)
      if (s._rtdbActive) {
        c.fillStyle = '#38BDF8'; c.shadowColor = '#38BDF8'; c.shadowBlur = 14;
        c.beginPath(); c.arc(908, 360, 4.5, 0, Math.PI * 2); c.fill();
      }
      c.restore();
    }

    _drawHomeMarker(c) {
      const s = this.state;
      const atHome = (s.xRev < 0.05 && s.zRev < 0.05);
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

      // Left status chip
      const isBusy = s.running || s.stepStage > 0;
      const modeTag = s.locked
        ? 'SAFETY LOCK · E-STOP'
        : (s.stepStage > 0
          ? `STEP ${s.stepStage}/6 · ${s.op}`
          : (isBusy && s.op && s.op !== 'NONE' && s.op !== 'IDLE' ? `DIGITAL TWIN · ${s.op}` : 'LIVE DIGITAL TWIN · STANDBY'));
      const leftW = Math.min(w * 0.54, c.measureText(modeTag).width + 28);
      roundRect(c, 10, 10, leftW, pillH, 7);
      c.fillStyle = s.locked ? 'rgba(185, 28, 28, 0.92)' : 'rgba(15, 23, 42, 0.84)';
      c.fill();
      c.strokeStyle = s.locked ? 'rgba(252,165,165,0.6)' : 'rgba(255,255,255,0.20)';
      c.lineWidth = 1;
      c.stroke();

      const dotCol = s.locked ? '#FF4D4F' : (s.running ? '#FFEA00' : '#AFFF00');
      c.fillStyle = dotCol;
      c.beginPath(); c.arc(21, 22, 3.6, 0, Math.PI * 2); c.fill();
      c.fillStyle = '#FFFFFF';
      c.textAlign = 'left';
      c.textBaseline = 'middle';
      c.fillText(modeTag, 30, 22.5);

      // Right target / live coordinates chip
      const bayId = (isBusy && s.targetBay) ? s.targetBay : (s.selectedBay || s.targetBay || 'A1');
      const bc = this.coordinates[bayId] || BAY_INFO[bayId];
      const rightTxt = s.running
        ? `X:${s.xRev.toFixed(2)}r · Z:${s.zRev.toFixed(2)}r`
        : `TARGET: ${bayId} (X ${bc.x.toFixed(1)} · Z ${bc.z.toFixed(1)})`;
      const rightW = c.measureText(rightTxt).width + 18;
      roundRect(c, w - rightW - 10, 10, rightW, pillH, 7);
      c.fillStyle = 'rgba(15, 23, 42, 0.84)';
      c.fill();
      c.strokeStyle = 'rgba(255,255,255,0.20)';
      c.stroke();
      c.fillStyle = s.running ? '#AFFF00' : '#FFFFFF';
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

  // Expose static constants & helpers for custom WMS integrations
  AsrsCadTwin.BAY_INFO = BAY_INFO;
  AsrsCadTwin.ASSET_MANIFEST = ASSET_MANIFEST;
  AsrsCadTwin.getBeamEndpoints = getBeamEndpoints;
  AsrsCadTwin.getCarriagePos = getCarriagePos;
  AsrsCadTwin.trapMotion = trapMotion;

  return AsrsCadTwin;
}));
