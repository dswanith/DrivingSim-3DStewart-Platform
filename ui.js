// ============================================================
// ui.js — UI Controls for Stewart Platform Simulator
// ============================================================

const StewartUI = (() => {
    let onPoseChange = null;

    const POSE_LIMITS = {
        x: { min: -500, max: 500, step: 5, default: 0, unit: 'mm' },
        y: { min: -500, max: 500, step: 5, default: 0, unit: 'mm' },
        z: { min: 1170, max: 2170, step: 10, default: StewartKinematics.neutralPose().z, unit: 'mm' },
        roll: { min: -30, max: 30, step: 0.5, default: 0, unit: '°' },
        pitch: { min: -30, max: 30, step: 0.5, default: 0, unit: '°' },
        yaw: { min: -30, max: 30, step: 0.5, default: 0, unit: '°' },
    };

    const PRESETS = (() => {
        const nz = StewartKinematics.neutralPose().z;
        return {
            'Neutral': { x: 0, y: 0, z: nz, roll: 0, pitch: 0, yaw: 0 },
            'Heave +': { x: 0, y: 0, z: nz + 200, roll: 0, pitch: 0, yaw: 0 },
            'Heave -': { x: 0, y: 0, z: nz - 200, roll: 0, pitch: 0, yaw: 0 },
            'Roll +': { x: 0, y: 0, z: nz, roll: 5, pitch: 0, yaw: 0 },
            'Roll -': { x: 0, y: 0, z: nz, roll: -5, pitch: 0, yaw: 0 },
            'Pitch +': { x: 0, y: 0, z: nz, roll: 0, pitch: 5, yaw: 0 },
            'Pitch -': { x: 0, y: 0, z: nz, roll: 0, pitch: -5, yaw: 0 },
            'Yaw +': { x: 0, y: 0, z: nz, roll: 0, pitch: 0, yaw: 5 },
            'Yaw -': { x: 0, y: 0, z: nz, roll: 0, pitch: 0, yaw: -5 },
        };
    })();

    let poseSliders = {};
    let poseDisplays = {};
    let deltaRows = [];        // [{bar, deltaVal, absVal}]
    let minLengthInput, maxLengthInput;
    let warningContainer;

    // ---- Init ----
    function init(callbacks) {
        onPoseChange = callbacks.onPoseChange;
        buildPoseControls();
        buildPresetButtons();
        buildReadouts();
        buildDeltaTable();
        buildEquationsPanel();
        buildLimitControls();
    }

    // ---- Pose Sliders ----
    function buildPoseControls() {
        const panel = document.getElementById('pose-controls');
        const labels = {
            x: 'X Lateral',
            y: 'Y Longitudinal',
            z: 'Z Height',
            roll: 'Roll (φ)',
            pitch: 'Pitch (θ)',
            yaw: 'Yaw (ψ)',
        };

        Object.keys(POSE_LIMITS).forEach(key => {
            const lim = POSE_LIMITS[key];
            const row = createSliderRow(labels[key], key, lim, () => {
                if (onPoseChange) onPoseChange(getCurrentPose());
            });
            panel.appendChild(row);
            poseSliders[key] = row.querySelector('input[type="range"]');
        });
    }

    function createSliderRow(label, id, limits, onChange) {
        const row = document.createElement('div');
        row.className = 'slider-row';

        const labelEl = document.createElement('label');
        labelEl.textContent = label;
        labelEl.htmlFor = `slider-${id}`;

        const sliderWrap = document.createElement('div');
        sliderWrap.className = 'slider-wrap';

        const slider = document.createElement('input');
        slider.type = 'range';
        slider.id = `slider-${id}`;
        slider.min = limits.min;
        slider.max = limits.max;
        slider.step = limits.step;
        slider.value = limits.default;

        const valDisplay = document.createElement('span');
        valDisplay.className = 'slider-value';
        valDisplay.textContent = `${parseFloat(limits.default).toFixed(1)} ${limits.unit}`;

        slider.addEventListener('input', () => {
            valDisplay.textContent = `${parseFloat(slider.value).toFixed(1)} ${limits.unit}`;
            if (onChange) onChange(parseFloat(slider.value));
        });

        sliderWrap.appendChild(slider);
        sliderWrap.appendChild(valDisplay);
        row.appendChild(labelEl);
        row.appendChild(sliderWrap);
        return row;
    }

    // ---- Presets ----
    function buildPresetButtons() {
        const container = document.getElementById('presets-section');
        const grid = document.createElement('div');
        grid.className = 'preset-grid';

        Object.keys(PRESETS).forEach(name => {
            const btn = document.createElement('button');
            btn.className = 'preset-btn';
            btn.textContent = name;
            btn.addEventListener('click', () => {
                const preset = PRESETS[name];
                setPose(preset);
                if (onPoseChange) onPoseChange(preset);
            });
            grid.appendChild(btn);
        });
        container.appendChild(grid);

        // Reset to Neutral button
        const resetBtn = document.createElement('button');
        resetBtn.className = 'preset-btn reset-btn';
        resetBtn.style.marginTop = '8px';
        resetBtn.style.gridColumn = '1 / -1';
        resetBtn.textContent = '⟳  Reset to Neutral';
        resetBtn.addEventListener('click', () => {
            const neutral = StewartKinematics.neutralPose();
            setPose(neutral);
            if (onPoseChange) onPoseChange(neutral);
        });
        container.appendChild(resetBtn);
    }

    // ---- Pose / Solver Readouts ----
    function buildReadouts() {
        const container = document.getElementById('readouts-section');

        const poseSection = document.createElement('div');
        poseSection.className = 'readout-group';
        poseSection.innerHTML = '<h4>Current Pose</h4>';
        const poseGrid = document.createElement('div');
        poseGrid.className = 'readout-grid';

        ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'].forEach(name => {
            const item = document.createElement('div');
            item.className = 'readout-item';
            const label = document.createElement('span');
            label.className = 'readout-label';
            label.textContent = name;
            const value = document.createElement('span');
            value.className = 'readout-value';
            value.id = `readout-${name.toLowerCase()}`;
            value.textContent = '0.0';
            item.appendChild(label);
            item.appendChild(value);
            poseGrid.appendChild(item);
            poseDisplays[name.toLowerCase()] = value;
        });

        // Solver status
        const statusItem = document.createElement('div');
        statusItem.className = 'readout-item';
        statusItem.innerHTML = `<span class="readout-label">Solver</span>`;
        const statusValue = document.createElement('span');
        statusValue.className = 'readout-value';
        statusValue.id = 'readout-solver';
        statusValue.textContent = '✓ IK (exact)';
        statusItem.appendChild(statusValue);
        poseGrid.appendChild(statusItem);
        poseDisplays['solver'] = statusValue;

        poseSection.appendChild(poseGrid);
        container.appendChild(poseSection);

        // Warning area
        warningContainer = document.createElement('div');
        warningContainer.id = 'warnings';
        warningContainer.className = 'warning-area';
        container.appendChild(warningContainer);
    }

    // ---- Actuator Delta Table ----
    function buildDeltaTable() {
        const container = document.getElementById('delta-table');
        const ACTUATOR_COLORS = ['#ff4444', '#44aaff', '#44ff88', '#ffaa22', '#dd44ff', '#ffff44'];
        const neutralL = StewartKinematics.neutralLengths();

        const table = document.createElement('table');
        table.className = 'delta-table';
        table.innerHTML = `
            <thead>
                <tr>
                    <th>Act.</th>
                    <th>L (mm)</th>
                    <th>ΔL (mm)</th>
                    <th>Extension</th>
                </tr>
            </thead>
        `;
        const tbody = document.createElement('tbody');

        for (let i = 0; i < 6; i++) {
            const tr = document.createElement('tr');
            const color = ACTUATOR_COLORS[i];

            const tdName = document.createElement('td');
            tdName.innerHTML = `<span class="act-badge" style="color:${color}">L${i + 1}</span>`;

            const tdAbs = document.createElement('td');
            const absVal = document.createElement('span');
            absVal.className = 'delta-abs';
            absVal.textContent = neutralL[i].toFixed(1);
            tdAbs.appendChild(absVal);

            const tdDelta = document.createElement('td');
            const deltaVal = document.createElement('span');
            deltaVal.className = 'delta-val';
            deltaVal.textContent = '0.00';
            tdDelta.appendChild(deltaVal);

            const tdBar = document.createElement('td');
            const barWrap = document.createElement('div');
            barWrap.className = 'delta-bar-wrap';
            const barFill = document.createElement('div');
            barFill.className = 'delta-bar-fill';
            barWrap.appendChild(barFill);
            tdBar.appendChild(barWrap);

            tr.appendChild(tdName);
            tr.appendChild(tdAbs);
            tr.appendChild(tdDelta);
            tr.appendChild(tdBar);
            tbody.appendChild(tr);

            deltaRows.push({ absVal, deltaVal, barFill });
        }

        table.appendChild(tbody);
        container.appendChild(table);
    }

    // ---- Equations Panel ----
    function buildEquationsPanel() {
        const container = document.getElementById('equations-content');
        container.innerHTML = `
<div class="eq-card">
    <div class="eq-title">Forward Transform</div>
    <div class="eq-body">
        <div class="eq-line"><span class="eq-sym">p<sub>i</sub></span> = <span class="eq-sym">T</span> + <span class="eq-sym">R</span> · <span class="eq-sym">P<sub>i</sub></span></div>
        <div class="eq-where">where</div>
        <div class="eq-line"><span class="eq-sym">T</span> = [x, y, z]<sup>T</sup> &nbsp;(translation)</div>
        <div class="eq-line"><span class="eq-sym">R</span> = R<sub>z</sub>(ψ) · R<sub>y</sub>(θ) · R<sub>x</sub>(φ)</div>
        <div class="eq-line eq-indent"><span class="eq-sym">φ</span> = Roll &nbsp; <span class="eq-sym">θ</span> = Pitch &nbsp; <span class="eq-sym">ψ</span> = Yaw</div>
    </div>
</div>

<div class="eq-card">
    <div class="eq-title">Rotation Matrix (ZYX Euler)</div>
    <div class="eq-body">
        <div class="eq-line"><span class="eq-sym">R</span> =</div>
        <div class="eq-matrix">
            <div>c<sub>ψ</sub>c<sub>θ</sub></div><div>c<sub>ψ</sub>s<sub>θ</sub>s<sub>φ</sub>−s<sub>ψ</sub>c<sub>φ</sub></div><div>c<sub>ψ</sub>s<sub>θ</sub>c<sub>φ</sub>+s<sub>ψ</sub>s<sub>φ</sub></div>
            <div>s<sub>ψ</sub>c<sub>θ</sub></div><div>s<sub>ψ</sub>s<sub>θ</sub>s<sub>φ</sub>+c<sub>ψ</sub>c<sub>φ</sub></div><div>s<sub>ψ</sub>s<sub>θ</sub>c<sub>φ</sub>−c<sub>ψ</sub>s<sub>φ</sub></div>
            <div>−s<sub>θ</sub></div><div>c<sub>θ</sub>s<sub>φ</sub></div><div>c<sub>θ</sub>c<sub>φ</sub></div>
        </div>
    </div>
</div>

<div class="eq-card">
    <div class="eq-title">Inverse Kinematics (Actuator Length)</div>
    <div class="eq-body">
        <div class="eq-line"><span class="eq-sym">L<sub>i</sub></span> = ‖ <span class="eq-sym">p<sub>i</sub></span> − <span class="eq-sym">B<sub>i</sub></span> ‖</div>
        <div class="eq-where">where <span class="eq-sym">B<sub>i</sub></span> = base attachment point</div>
    </div>
</div>

<div class="eq-card">
    <div class="eq-title">Displacement from Neutral</div>
    <div class="eq-body">
        <div class="eq-line"><span class="eq-sym">ΔL<sub>i</sub></span> = <span class="eq-sym">L<sub>i</sub></span> − <span class="eq-sym">L<sub>i</sub><sup>0</sup></span></div>
        <div class="eq-where">L<sub>i</sub><sup>0</sup> = neutral length (Z = ${StewartKinematics.NEUTRAL_HEIGHT.toFixed(3)} mm)</div>
    </div>
</div>
        `;
    }

    // ---- Limit Controls ----
    function buildLimitControls() {
        const container = document.getElementById('limits-section');
        const row = document.createElement('div');
        row.className = 'limit-row';

        const minGroup = document.createElement('div');
        minGroup.className = 'limit-group';
        minGroup.innerHTML = '<label>Min Length (mm)</label>';
        minLengthInput = document.createElement('input');
        minLengthInput.type = 'number';
        minLengthInput.value = '1200';
        minLengthInput.className = 'limit-input';
        minLengthInput.addEventListener('change', () => {
            if (onPoseChange) onPoseChange(getCurrentPose());
        });
        minGroup.appendChild(minLengthInput);

        const maxGroup = document.createElement('div');
        maxGroup.className = 'limit-group';
        maxGroup.innerHTML = '<label>Max Length (mm)</label>';
        maxLengthInput = document.createElement('input');
        maxLengthInput.type = 'number';
        maxLengthInput.value = '2200';
        maxLengthInput.className = 'limit-input';
        maxLengthInput.addEventListener('change', () => {
            if (onPoseChange) onPoseChange(getCurrentPose());
        });
        maxGroup.appendChild(maxLengthInput);

        row.appendChild(minGroup);
        row.appendChild(maxGroup);
        container.appendChild(row);
    }

    // ---- Public Getters / Setters ----
    function getCurrentPose() {
        return {
            x: parseFloat(poseSliders.x.value),
            y: parseFloat(poseSliders.y.value),
            z: parseFloat(poseSliders.z.value),
            roll: parseFloat(poseSliders.roll.value),
            pitch: parseFloat(poseSliders.pitch.value),
            yaw: parseFloat(poseSliders.yaw.value),
        };
    }

    function setPose(pose) {
        Object.keys(pose).forEach(key => {
            if (poseSliders[key]) {
                poseSliders[key].value = pose[key];
                const lim = POSE_LIMITS[key];
                const display = poseSliders[key].parentElement.querySelector('.slider-value');
                if (display) display.textContent = `${parseFloat(pose[key]).toFixed(1)} ${lim.unit}`;
            }
        });
    }

    // ---- Update Readouts ----
    function updateReadouts(pose, lengths) {
        const units = { x: ' mm', y: ' mm', z: ' mm', roll: '°', pitch: '°', yaw: '°' };
        ['x', 'y', 'z', 'roll', 'pitch', 'yaw'].forEach(key => {
            if (poseDisplays[key]) {
                poseDisplays[key].textContent = (pose[key] !== undefined ? pose[key].toFixed(2) : '—') + (units[key] || '');
            }
        });

        // Actuator limit warnings
        const minLen = parseFloat(minLengthInput.value) || 0;
        const maxLen = parseFloat(maxLengthInput.value) || Infinity;
        const warnings = [];

        // Delta table
        const neutralL = StewartKinematics.neutralLengths();
        const BAR_MAX = 300; // mm scale for bars

        lengths.forEach((l, i) => {
            const delta = l - neutralL[i];
            const absEl = deltaRows[i].absVal;
            const deltaEl = deltaRows[i].deltaVal;
            const barEl = deltaRows[i].barFill;

            absEl.textContent = l.toFixed(1);

            const sign = delta >= 0 ? '+' : '';
            deltaEl.textContent = `${sign}${delta.toFixed(2)}`;
            deltaEl.classList.remove('delta-pos', 'delta-neg', 'delta-zero');
            if (Math.abs(delta) < 0.05) {
                deltaEl.classList.add('delta-zero');
            } else if (delta > 0) {
                deltaEl.classList.add('delta-pos');
            } else {
                deltaEl.classList.add('delta-neg');
            }

            // Bar: centre at 50%, width proportional to |delta|, direction by sign
            const pct = Math.min(Math.abs(delta) / BAR_MAX, 1) * 50;
            if (delta >= 0) {
                barEl.style.left = '50%';
                barEl.style.right = 'auto';
                barEl.style.width = `${pct}%`;
                barEl.style.background = 'var(--accent-green)';
            } else {
                barEl.style.right = '50%';
                barEl.style.left = 'auto';
                barEl.style.width = `${pct}%`;
                barEl.style.background = 'var(--accent-red)';
            }

            // Limit warnings
            if (l < minLen) {
                warnings.push(`⚠ L${i + 1} (${l.toFixed(1)} mm) below min ${minLen} mm`);
            } else if (l > maxLen) {
                warnings.push(`⚠ L${i + 1} (${l.toFixed(1)} mm) exceeds max ${maxLen} mm`);
            }
        });

        warningContainer.innerHTML = warnings.length > 0
            ? warnings.map(w => `<div class="warning-msg">${w}</div>`).join('')
            : '<div class="all-ok">✓ All actuators within limits</div>';
    }

    function getActuatorLimits() {
        return {
            min: parseFloat(minLengthInput.value) || 0,
            max: parseFloat(maxLengthInput.value) || Infinity,
        };
    }

    return {
        init,
        getCurrentPose,
        setPose,
        updateReadouts,
        getActuatorLimits,
        POSE_LIMITS,
    };
})();
