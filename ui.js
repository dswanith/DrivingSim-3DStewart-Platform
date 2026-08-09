// ============================================================
// ui.js — UI Controls for Stewart Platform Simulator
// ============================================================

const StewartUI = (() => {
    let onPoseChange = null;
    let onActuatorChange = null;
    let onModeChange = null;
    let controlMode = 'pose'; // 'pose' or 'actuator'

    const POSE_LIMITS = {
        x: { min: -500, max: 500, step: 5, default: 0, unit: 'mm' },
        y: { min: -500, max: 500, step: 5, default: 0, unit: 'mm' },
        z: { min: 1170, max: 2170, step: 10, default: 1670, unit: 'mm' },
        roll: { min: -30, max: 30, step: 0.5, default: 0, unit: '°' },
        pitch: { min: -30, max: 30, step: 0.5, default: 0, unit: '°' },
        yaw: { min: -30, max: 30, step: 0.5, default: 0, unit: '°' },
    };

    const PRESETS = {
        'Neutral': { x: 0, y: 0, z: 1670, roll: 0, pitch: 0, yaw: 0 },
        'Heave +': { x: 0, y: 0, z: 1870, roll: 0, pitch: 0, yaw: 0 },
        'Heave -': { x: 0, y: 0, z: 1470, roll: 0, pitch: 0, yaw: 0 },
        'Roll +': { x: 0, y: 0, z: 1670, roll: 15, pitch: 0, yaw: 0 },
        'Roll -': { x: 0, y: 0, z: 1670, roll: -15, pitch: 0, yaw: 0 },
        'Pitch +': { x: 0, y: 0, z: 1670, roll: 0, pitch: 15, yaw: 0 },
        'Pitch -': { x: 0, y: 0, z: 1670, roll: 0, pitch: -15, yaw: 0 },
        'Yaw +': { x: 0, y: 0, z: 1670, roll: 0, pitch: 0, yaw: 15 },
        'Yaw -': { x: 0, y: 0, z: 1670, roll: 0, pitch: 0, yaw: -15 },
    };

    let poseSliders = {};
    let actuatorSliders = {};
    let lengthDisplays = {};
    let poseDisplays = {};
    let minLengthInput, maxLengthInput;
    let warningContainer;
    let poseControlPanel, actuatorControlPanel;
    let modeToggleBtn;

    function init(callbacks) {
        onPoseChange = callbacks.onPoseChange;
        onActuatorChange = callbacks.onActuatorChange;
        onModeChange = callbacks.onModeChange;

        buildModeToggle();
        buildPoseControls();
        buildActuatorControls();
        buildPresetButtons();
        buildReadouts();
        buildLimitControls();

        // Start in pose mode
        setMode('pose');
    }

    function buildModeToggle() {
        const section = document.getElementById('mode-toggle-section');
        const btnGroup = document.createElement('div');
        btnGroup.className = 'mode-toggle-group';

        const poseBtn = document.createElement('button');
        poseBtn.textContent = 'Pose Control';
        poseBtn.className = 'mode-btn active';
        poseBtn.id = 'mode-pose-btn';
        poseBtn.addEventListener('click', () => setMode('pose'));

        const actBtn = document.createElement('button');
        actBtn.textContent = 'Actuator Control';
        actBtn.className = 'mode-btn';
        actBtn.id = 'mode-act-btn';
        actBtn.addEventListener('click', () => setMode('actuator'));

        btnGroup.appendChild(poseBtn);
        btnGroup.appendChild(actBtn);
        section.appendChild(btnGroup);
    }

    function setMode(mode) {
        controlMode = mode;
        document.getElementById('mode-pose-btn').classList.toggle('active', mode === 'pose');
        document.getElementById('mode-act-btn').classList.toggle('active', mode === 'actuator');

        if (poseControlPanel) poseControlPanel.style.display = mode === 'pose' ? 'block' : 'none';
        if (actuatorControlPanel) actuatorControlPanel.style.display = mode === 'actuator' ? 'block' : 'none';

        if (onModeChange) onModeChange(mode);
    }

    function buildPoseControls() {
        poseControlPanel = document.getElementById('pose-controls');
        const labels = { x: 'X Displacement', y: 'Y Displacement', z: 'Z Height', roll: 'Roll', pitch: 'Pitch', yaw: 'Yaw' };

        Object.keys(POSE_LIMITS).forEach(key => {
            const lim = POSE_LIMITS[key];
            const row = createSliderRow(labels[key], key, lim, (val) => {
                if (controlMode === 'pose' && onPoseChange) {
                    onPoseChange(getCurrentPose());
                }
            });
            poseControlPanel.appendChild(row);
            poseSliders[key] = row.querySelector('input[type="range"]');
        });
    }

    function buildActuatorControls() {
        actuatorControlPanel = document.getElementById('actuator-controls');

        for (let i = 0; i < 6; i++) {
            const key = `L${i + 1}`;
            const lim = { min: 800, max: 2500, step: 10, default: 1670, unit: 'mm' };
            const row = createSliderRow(key, `act_${i}`, lim, (val) => {
                if (controlMode === 'actuator' && onActuatorChange) {
                    onActuatorChange(getCurrentActuatorLengths());
                }
            });
            actuatorControlPanel.appendChild(row);
            actuatorSliders[i] = row.querySelector('input[type="range"]');
        }
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

    function buildPresetButtons() {
        const container = document.getElementById('presets-section');
        const grid = document.createElement('div');
        grid.className = 'preset-grid';

        Object.keys(PRESETS).forEach(name => {
            const btn = document.createElement('button');
            btn.className = 'preset-btn';
            btn.textContent = name;
            btn.addEventListener('click', () => {
                setMode('pose');
                const preset = PRESETS[name];
                setPose(preset);
                if (onPoseChange) onPoseChange(preset);
            });
            grid.appendChild(btn);
        });
        container.appendChild(grid);
    }

    function buildReadouts() {
        const container = document.getElementById('readouts-section');

        // Pose readout
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
        poseSection.appendChild(poseGrid);
        container.appendChild(poseSection);

        // Actuator lengths readout
        const actSection = document.createElement('div');
        actSection.className = 'readout-group';
        actSection.innerHTML = '<h4>Actuator Lengths</h4>';
        const actGrid = document.createElement('div');
        actGrid.className = 'readout-grid';
        for (let i = 0; i < 6; i++) {
            const item = document.createElement('div');
            item.className = 'readout-item';
            const label = document.createElement('span');
            label.className = 'readout-label';
            label.textContent = `L${i + 1}`;
            const value = document.createElement('span');
            value.className = 'readout-value';
            value.id = `readout-l${i + 1}`;
            value.textContent = '0.0';
            item.appendChild(label);
            item.appendChild(value);
            actGrid.appendChild(item);
            lengthDisplays[i] = value;
        }
        actSection.appendChild(actGrid);
        container.appendChild(actSection);

        // Warning area
        warningContainer = document.createElement('div');
        warningContainer.id = 'warnings';
        warningContainer.className = 'warning-area';
        container.appendChild(warningContainer);
    }

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

    function getCurrentActuatorLengths() {
        const lengths = [];
        for (let i = 0; i < 6; i++) {
            lengths.push(parseFloat(actuatorSliders[i].value));
        }
        return lengths;
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

    function setActuatorLengths(lengths) {
        lengths.forEach((l, i) => {
            if (actuatorSliders[i]) {
                actuatorSliders[i].value = l.toFixed(1);
                const display = actuatorSliders[i].parentElement.querySelector('.slider-value');
                if (display) display.textContent = `${l.toFixed(1)} mm`;
            }
        });
    }

    function updateReadouts(pose, lengths) {
        const units = { x: ' mm', y: ' mm', z: ' mm', roll: '°', pitch: '°', yaw: '°' };
        Object.keys(poseDisplays).forEach(key => {
            poseDisplays[key].textContent = (pose[key] !== undefined ? pose[key].toFixed(2) : '—') + (units[key] || '');
        });

        const minLen = parseFloat(minLengthInput.value) || 0;
        const maxLen = parseFloat(maxLengthInput.value) || Infinity;
        let warnings = [];

        lengths.forEach((l, i) => {
            lengthDisplays[i].textContent = l.toFixed(2) + ' mm';
            lengthDisplays[i].classList.remove('warning', 'danger');

            if (l < minLen) {
                lengthDisplays[i].classList.add('danger');
                warnings.push(`⚠ L${i + 1} (${l.toFixed(1)} mm) below minimum ${minLen} mm`);
            } else if (l > maxLen) {
                lengthDisplays[i].classList.add('danger');
                warnings.push(`⚠ L${i + 1} (${l.toFixed(1)} mm) exceeds maximum ${maxLen} mm`);
            } else if (l < minLen + 50 || l > maxLen - 50) {
                lengthDisplays[i].classList.add('warning');
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
        getCurrentActuatorLengths,
        setPose,
        setActuatorLengths,
        updateReadouts,
        setMode,
        getActuatorLimits,
        POSE_LIMITS,
    };
})();
