// dashboard/telemetry.js — WebSocket client, UI controller, and SVG charting

// --- Lightweight SVG Real-time Chart Engine ---
class SVGChart {
    constructor(svgId, channels, maxPoints = 120) {
        this.svg = document.getElementById(svgId);
        this.channels = channels; // Array of { key, color, label }
        this.maxPoints = maxPoints;
        this.data = []; // Array of arrays: [t, val1, val2, ...]
        this.width = 400;
        this.height = 120;
        this.init();
    }

    init() {
        if (!this.svg) return;
        this.svg.setAttribute('viewBox', `0 0 ${this.width} ${this.height}`);
        this.svg.innerHTML = '';

        // Zero-line
        const zeroLine = document.createElementNS("http://www.w3.org/2000/svg", "line");
        zeroLine.setAttribute('x1', '35'); zeroLine.setAttribute('y1', '55');
        zeroLine.setAttribute('x2', '390'); zeroLine.setAttribute('y2', '55');
        zeroLine.setAttribute('stroke', 'rgba(255,255,255,0.08)');
        zeroLine.setAttribute('stroke-width', '1');
        this.svg.appendChild(zeroLine);

        // Axis Y
        const axisY = document.createElementNS("http://www.w3.org/2000/svg", "line");
        axisY.setAttribute('x1', '35'); axisY.setAttribute('y1', '10');
        axisY.setAttribute('x2', '35'); axisY.setAttribute('y2', '100');
        axisY.setAttribute('stroke', '#333'); axisY.setAttribute('stroke-width', '1');
        this.svg.appendChild(axisY);

        // Axis X
        const axisX = document.createElementNS("http://www.w3.org/2000/svg", "line");
        axisX.setAttribute('x1', '35'); axisX.setAttribute('y1', '100');
        axisX.setAttribute('x2', '390'); axisX.setAttribute('y2', '100');
        axisX.setAttribute('stroke', '#333'); axisX.setAttribute('stroke-width', '1');
        this.svg.appendChild(axisX);

        // Labels group
        this.labelsGroup = document.createElementNS("http://www.w3.org/2000/svg", "g");
        this.svg.appendChild(this.labelsGroup);

        // Channel paths
        this.paths = {};
        this.channels.forEach(ch => {
            const path = document.createElementNS("http://www.w3.org/2000/svg", "path");
            path.setAttribute('fill', 'none');
            path.setAttribute('stroke', ch.color);
            path.setAttribute('stroke-width', '1.5');
            path.setAttribute('stroke-linejoin', 'round');
            this.svg.appendChild(path);
            this.paths[ch.key] = path;
        });

        // Legend
        const legendG = document.createElementNS("http://www.w3.org/2000/svg", "g");
        this.channels.forEach((ch, idx) => {
            const lx = 40 + idx * 55;
            const rect = document.createElementNS("http://www.w3.org/2000/svg", "rect");
            rect.setAttribute('x', lx); rect.setAttribute('y', '5');
            rect.setAttribute('width', '10'); rect.setAttribute('height', '3');
            rect.setAttribute('fill', ch.color);
            legendG.appendChild(rect);
            const text = document.createElementNS("http://www.w3.org/2000/svg", "text");
            text.setAttribute('x', lx + 13); text.setAttribute('y', '8');
            text.setAttribute('fill', '#aaa'); text.setAttribute('font-size', '6px');
            text.setAttribute('font-family', 'JetBrains Mono, monospace');
            text.textContent = ch.label;
            legendG.appendChild(text);
        });
        this.svg.appendChild(legendG);
    }

    push(timestamp, values) {
        const entry = [timestamp];
        this.channels.forEach(ch => entry.push(values[ch.key] !== undefined ? values[ch.key] : 0.0));
        this.data.push(entry);
        if (this.data.length > this.maxPoints) this.data.shift();
        this.render();
    }

    reset() { this.data = []; this.render(); }

    render() {
        if (this.data.length === 0) {
            this.channels.forEach(ch => this.paths[ch.key] && this.paths[ch.key].setAttribute('d', ''));
            return;
        }

        let valMin = Infinity, valMax = -Infinity;
        this.data.forEach(entry => {
            for (let i = 1; i < entry.length; i++) {
                if (entry[i] < valMin) valMin = entry[i];
                if (entry[i] > valMax) valMax = entry[i];
            }
        });

        const span = valMax - valMin;
        if (span < 1e-4) { valMin -= 1.0; valMax += 1.0; }
        else { valMin -= span * 0.12; valMax += span * 0.12; }

        const plotWidth = 350, plotHeight = 88, startX = 35, startY = 100;
        const dStrings = {};
        this.channels.forEach(ch => dStrings[ch.key] = '');

        const stepX = plotWidth / Math.max(this.maxPoints - 1, 1);

        for (let idx = 0; idx < this.data.length; idx++) {
            const x = startX + idx * stepX;
            const entry = this.data[idx];
            this.channels.forEach((ch, chIdx) => {
                const val = entry[chIdx + 1];
                const normY = (val - valMin) / (valMax - valMin);
                const y = startY - normY * plotHeight;
                dStrings[ch.key] += idx === 0 ? `M ${x.toFixed(1)} ${y.toFixed(1)}` : ` L ${x.toFixed(1)} ${y.toFixed(1)}`;
            });
        }

        this.channels.forEach(ch => this.paths[ch.key] && this.paths[ch.key].setAttribute('d', dStrings[ch.key]));

        // Y-axis labels
        this.labelsGroup.innerHTML = '';
        [valMin, (valMin + valMax) / 2, valMax].forEach(tick => {
            const normY = (tick - valMin) / (valMax - valMin);
            const y = startY - normY * plotHeight;
            const text = document.createElementNS("http://www.w3.org/2000/svg", "text");
            text.setAttribute('x', '30'); text.setAttribute('y', (y + 3).toString());
            text.setAttribute('fill', '#555'); text.setAttribute('font-size', '7px');
            text.setAttribute('font-family', 'JetBrains Mono, monospace');
            text.setAttribute('text-anchor', 'end');
            text.textContent = tick.toFixed(2);
            this.labelsGroup.appendChild(text);
        });
    }
}


// --- Dashboard Application Logic ---
const DashboardApp = (() => {
    let ws;
    const wsUrl = "ws://localhost:8765";
    let isConnected = false;
    let reconnectTimeout = null;
    let frameCount = 0;
    let lastMsgTime = null;

    // Charts
    let chartAccel, chartLengths, chartMotion;

    // Mode
    let currentMode = "synthetic";

    // Sliders state
    let manualPose = { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 };

    // Neutral leg length in mm (from kinematics.js)
    const NEUTRAL_LEN_MM = StewartKinematics.neutralLengths();

    function init() {
        // Initialize 3D visualization using the EXISTING dswanith Stewart renderer
        const threeContainer = document.getElementById('threejs-container');
        StewartVisualizer.init(threeContainer);

        // Initialize 2D canvas track
        const trackCanvas = document.getElementById('vehicle-track-canvas');
        VehicleTrackViz.init(trackCanvas);

        // Initialize SVG charts
        chartAccel = new SVGChart('svg-chart-accel', [
            { key: 'ax', color: '#ff4444', label: 'AX (m/s²)' },
            { key: 'ay', color: '#00e5ff', label: 'AY (m/s²)' },
            { key: 'az', color: '#ffff44', label: 'AZ (m/s²)' }
        ]);

        chartLengths = new SVGChart('svg-chart-lengths', [
            { key: 'L1', color: '#ff4444', label: 'L1' },
            { key: 'L2', color: '#44aaff', label: 'L2' },
            { key: 'L3', color: '#44ff88', label: 'L3' },
            { key: 'L4', color: '#ffaa22', label: 'L4' },
            { key: 'L5', color: '#dd44ff', label: 'L5' },
            { key: 'L6', color: '#ffff44', label: 'L6' }
        ]);

        chartMotion = new SVGChart('svg-chart-currents', [
            { key: 'roll_deg',  color: '#ff4444', label: 'Roll(°)' },
            { key: 'pitch_deg', color: '#44aaff', label: 'Pitch(°)' },
            { key: 'heave_mm',  color: '#ffff44', label: 'Heave(mm)' }
        ]);

        setupEventHandlers();

        // Show "neutral" platform immediately
        _renderNeutral();

        // Connect to Python WebSocket server
        connectWS();
    }

    function _renderNeutral() {
        const pose = StewartKinematics.neutralPose();
        StewartVisualizer.updatePose(pose);
    }

    function connectWS() {
        updateConnStatus("CONNECTING…", "status-disconnected");
        try {
            ws = new WebSocket(wsUrl);
        } catch(e) {
            updateConnStatus("FAILED", "status-disconnected");
            scheduleReconnect();
            return;
        }

        ws.onopen = () => {
            isConnected = true;
            frameCount = 0;
            updateConnStatus("CONNECTED", "status-connected");
            clearTimeout(reconnectTimeout);
            reconnectTimeout = null;
            // Tell server which mode we're in
            sendCmd("set_mode", { mode: currentMode });
        };

        ws.onclose = () => {
            isConnected = false;
            updateConnStatus("DISCONNECTED", "status-disconnected");
            scheduleReconnect();
        };

        ws.onerror = () => { try { ws.close(); } catch(e) {} };

        ws.onmessage = (event) => {
            try {
                const msg = JSON.parse(event.data);
                handleWSMessage(msg);
            } catch (e) {
                console.error("WS decode error:", e);
            }
        };
    }

    function scheduleReconnect() {
        if (reconnectTimeout) return;
        reconnectTimeout = setTimeout(() => {
            reconnectTimeout = null;
            connectWS();
        }, 2000);
    }

    function updateConnStatus(txt, cls) {
        const el = document.getElementById('conn-status');
        if (el) { el.textContent = txt; el.className = `status-val ${cls}`; }
    }

    function sendCmd(cmd, payload = {}) {
        if (!isConnected || !ws || ws.readyState !== WebSocket.OPEN) return;
        ws.send(JSON.stringify({ command: cmd, ...payload }));
    }

    function handleWSMessage(msg) {
        frameCount++;
        lastMsgTime = msg.timestamp_s;

        // ── 1. Scenario / metadata ──────────────────────────────────────────
        const scnEl = document.getElementById('select-scenario');
        if (scnEl && scnEl.value !== msg.scenario) scnEl.value = msg.scenario;

        // Phase badge
        const phaseEl = document.getElementById('phase-label');
        if (phaseEl) phaseEl.textContent = (msg.phase || '').replace(/_/g, ' ').toUpperCase();

        // Frame counter & timestamp
        const fcEl = document.getElementById('frame-counter');
        if (fcEl) fcEl.textContent = `#${msg.frame_index}  T=${msg.timestamp_s.toFixed(2)}s`;

        // ── 2. Status flags ─────────────────────────────────────────────────
        const estopEl = document.getElementById('estop-status');
        if (estopEl) {
            if (msg.safety.estop) {
                estopEl.textContent = "ESTOP";
                estopEl.className = "status-val status-violation";
            } else {
                estopEl.textContent = msg.safety.status;
                estopEl.className = msg.safety.status === "NORMAL"
                    ? "status-val status-ok"
                    : "status-val status-violation";
            }
        }

        if (msg.mode !== currentMode) setModeUI(msg.mode);

        // ── 3. Vehicle telemetry metrics ────────────────────────────────────
        _setText('val-speed',   msg.vehicle.speed_kmh.toFixed(1));
        _setText('val-ay',      (msg.vehicle.accel_y_ms2 / 9.81).toFixed(3));
        _setText('val-az',      msg.vehicle.accel_z_ms2.toFixed(3));
        _setText('val-yawrate', (msg.vehicle.ang_vel_z_rads * 180 / Math.PI).toFixed(1));

        // ── 4. Platform pose overlay ────────────────────────────────────────
        _setText('pose-tx',    msg.motion_cueing.x_mm.toFixed(1));
        _setText('pose-ty',    msg.motion_cueing.y_mm.toFixed(1));
        _setText('pose-tz',    msg.motion_cueing.z_mm.toFixed(1));
        _setText('pose-roll',  msg.motion_cueing.roll_deg.toFixed(2));
        _setText('pose-pitch', msg.motion_cueing.pitch_deg.toFixed(2));
        _setText('pose-yaw',   msg.motion_cueing.yaw_deg.toFixed(2));

        // ── 5. 2D Canvas vehicle track ──────────────────────────────────────
        VehicleTrackViz.draw(msg);

        // ── 6. 3D Stewart Platform (THE MAIN VISUALIZATION) ─────────────────
        // Reconstruct absolute pose for the top-platform mesh orientation.
        // The actuator positions come directly from Python IK (joints.top_mm).
        const poseObj = {
            x:     msg.motion_cueing.x_mm,
            y:     msg.motion_cueing.y_mm,
            z:     StewartKinematics.NEUTRAL_HEIGHT + msg.motion_cueing.z_mm,
            roll:  msg.motion_cueing.roll_deg,
            pitch: msg.motion_cueing.pitch_deg,
            yaw:   msg.motion_cueing.yaw_deg
        };
        // Drive the EXISTING dswanith Three.js renderer with Python-computed joint positions
        StewartVisualizer.updatePlatformDirect(
            msg.joints.base_mm,
            msg.joints.top_mm,
            poseObj
        );

        // ── 7. Actuator progress bars ───────────────────────────────────────
        const minLen = 1500, maxLen = 2300;
        for (let i = 1; i <= 6; i++) {
            const mmKey = `L${i}_mm`;
            const lenMM = msg.stewart[mmKey];
            const neutralMM = NEUTRAL_LEN_MM[i - 1];
            const deltaMM = lenMM - neutralMM;

            _setText(`lbl-len-${i}`, `${lenMM.toFixed(0)} mm`);

            const pct = ((lenMM - minLen) / (maxLen - minLen)) * 100;
            const bar = document.getElementById(`bar-len-${i}`);
            if (bar) {
                bar.style.width = `${Math.min(Math.max(pct, 0), 100)}%`;
                if (Math.abs(deltaMM) > 50)      bar.className = "progress-fill danger";
                else if (Math.abs(deltaMM) > 20) bar.className = "progress-fill warn";
                else                              bar.className = "progress-fill";
            }

            // Delta label
            const dEl = document.getElementById(`lbl-delta-${i}`);
            if (dEl) {
                const sign = deltaMM >= 0 ? '+' : '';
                dEl.textContent = `${sign}${deltaMM.toFixed(1)}`;
                dEl.style.color = deltaMM > 0 ? '#39ff14' : deltaMM < 0 ? '#ff4444' : '#888';
            }
        }

        // ── 8. Safety warnings ──────────────────────────────────────────────
        const warnBox = document.getElementById('safety-warnings-list');
        if (warnBox) {
            warnBox.innerHTML = '';
            if (msg.safety.warnings.length === 0 && !msg.safety.estop) {
                warnBox.innerHTML = `<div class="warning-placeholder">✓ All systems normal.</div>`;
            } else {
                if (msg.safety.estop) {
                    const d = document.createElement('div');
                    d.className = "warning-item estop";
                    d.textContent = "🚨 ESTOP ACTIVE";
                    warnBox.appendChild(d);
                }
                msg.safety.warnings.forEach(w => {
                    const d = document.createElement('div');
                    d.className = "warning-item";
                    d.textContent = `⚠ ${w}`;
                    warnBox.appendChild(d);
                });
            }
        }

        // ── 9. Charts ───────────────────────────────────────────────────────
        const t = msg.timestamp_s;

        chartAccel.push(t, {
            ax: msg.vehicle.accel_x_ms2,
            ay: msg.vehicle.accel_y_ms2,
            az: msg.vehicle.accel_z_ms2
        });

        chartLengths.push(t, {
            L1: msg.stewart.L1_mm - NEUTRAL_LEN_MM[0],
            L2: msg.stewart.L2_mm - NEUTRAL_LEN_MM[1],
            L3: msg.stewart.L3_mm - NEUTRAL_LEN_MM[2],
            L4: msg.stewart.L4_mm - NEUTRAL_LEN_MM[3],
            L5: msg.stewart.L5_mm - NEUTRAL_LEN_MM[4],
            L6: msg.stewart.L6_mm - NEUTRAL_LEN_MM[5],
        });

        chartMotion.push(t, {
            roll_deg:  msg.motion_cueing.roll_deg,
            pitch_deg: msg.motion_cueing.pitch_deg,
            heave_mm:  msg.motion_cueing.z_mm
        });
    }

    function _setText(id, val) {
        const el = document.getElementById(id);
        if (el) el.textContent = val;
    }

    function setupEventHandlers() {
        // Mode buttons
        const btnSynth  = document.getElementById('btn-mode-synthetic');
        const btnManual = document.getElementById('btn-mode-manual');

        if (btnSynth) btnSynth.onclick = () => {
            currentMode = "synthetic";
            setModeUI("synthetic");
            sendCmd("set_mode", { mode: "synthetic" });
        };
        if (btnManual) btnManual.onclick = () => {
            currentMode = "manual";
            setModeUI("manual");
            sendCmd("set_mode", { mode: "manual" });
        };

        // Scenario selector
        const selEl = document.getElementById('select-scenario');
        if (selEl) selEl.onchange = (e) => {
            sendCmd("select_scenario", { scenario: e.target.value });
        };

        // START / PAUSE / RESET
        const btnStart = document.getElementById('btn-start');
        const btnPause = document.getElementById('btn-pause');
        const btnReset = document.getElementById('btn-reset');

        if (btnStart) btnStart.onclick = () => {
            sendCmd("start");
            btnStart.classList.add('btn-active');
            if (btnPause) btnPause.classList.remove('btn-active');
        };
        if (btnPause) btnPause.onclick = () => {
            sendCmd("pause");
            btnPause.classList.add('btn-active');
            if (btnStart) btnStart.classList.remove('btn-active');
        };
        if (btnReset) btnReset.onclick = () => {
            sendCmd("reset");
            if (btnStart) btnStart.classList.remove('btn-active');
            if (btnPause) btnPause.classList.remove('btn-active');
            chartAccel.reset();
            chartLengths.reset();
            chartMotion.reset();
            frameCount = 0;
            _renderNeutral();
        };

        // Manual pose sliders
        ['x','y','z','roll','pitch','yaw'].forEach(key => {
            const el  = document.getElementById(`slider-${key}`);
            const valEl = document.getElementById(`slider-val-${key}`);
            if (!el) return;
            el.oninput = (e) => {
                const val = parseFloat(e.target.value);
                if (valEl) valEl.textContent = val;
                manualPose[key] = val;
                if (currentMode === "manual") sendManualPoseToPython();
            };
        });
    }

    function sendManualPoseToPython() {
        const d2r = Math.PI / 180;
        sendCmd("set_manual_pose", {
            pose: [
                manualPose.x / 1000,
                manualPose.y / 1000,
                manualPose.z / 1000,
                manualPose.roll  * d2r,
                manualPose.pitch * d2r,
                manualPose.yaw   * d2r,
            ]
        });
    }

    function setModeUI(mode) {
        currentMode = mode;
        const btnSynth  = document.getElementById('btn-mode-synthetic');
        const btnManual = document.getElementById('btn-mode-manual');
        const rightTag  = document.getElementById('panel-right-tag');
        const sliderSec = document.getElementById('manual-sliders-section');
        const barSec    = document.getElementById('actuator-bars-section');

        if (mode === "synthetic") {
            if (btnSynth)  btnSynth.className  = "active";
            if (btnManual) btnManual.className  = "";
            if (rightTag)  rightTag.textContent = "ACTUATORS";
            if (sliderSec) sliderSec.className  = "hidden";
            if (barSec)    barSec.className     = "";
        } else {
            if (btnManual) btnManual.className  = "active";
            if (btnSynth)  btnSynth.className   = "";
            if (rightTag)  rightTag.textContent = "POSE SLIDERS";
            if (sliderSec) sliderSec.className  = "";
            if (barSec)    barSec.className     = "hidden";
            sendManualPoseToPython();
        }
    }

    return { init };
})();

// Initialize once DOM is ready
window.onload = () => DashboardApp.init();
