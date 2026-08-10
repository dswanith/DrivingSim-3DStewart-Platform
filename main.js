// ============================================================
// main.js — Wires kinematics, renderer, and UI together
// ============================================================

(function () {
    'use strict';

    let currentPose = StewartKinematics.neutralPose();

    // Update simulation based on pose (IK is direct/exact — always converged)
    function updateSimulation(pose) {
        currentPose = pose;
        const result = StewartKinematics.inverseKinematics(pose);
        StewartRenderer.updatePlatform(result.basePoints, result.topPoints, pose);
        StewartUI.setActuatorLengths(result.lengths);
        window.__solverInfo = { converged: true, iterations: 1, residual: 0 };
        StewartUI.updateReadouts(pose, result.lengths);
    }

    function onActuatorChange(targetLengths) {
        // Use robust solver
        const solved = StewartKinematics.solveFromLengths(targetLengths, currentPose);
        const solvedPose = solved.pose;
        currentPose = solvedPose;
        StewartUI.setPose(solvedPose);
        // expose solver info
        window.__solverInfo = { converged: solved.converged, iterations: solved.iterations, residual: solved.residual };
        const result = StewartKinematics.inverseKinematics(solvedPose);
        StewartRenderer.updatePlatform(result.basePoints, result.topPoints, solvedPose);
        StewartUI.updateReadouts(solvedPose, result.lengths);
    }

    function onModeChange(mode) {
        // When switching to actuator mode, sync actuator sliders with current lengths
        if (mode === 'actuator') {
            const result = StewartKinematics.inverseKinematics(currentPose);
            StewartUI.setActuatorLengths(result.lengths);
        }
    }

    function animate() {
        requestAnimationFrame(animate);
        StewartRenderer.render();
    }

    function start() {
        // Initialize renderer
        const viewport = document.getElementById('viewport');
        StewartRenderer.init(viewport);

        // Initialize UI
        StewartUI.init({
            onPoseChange: updateSimulation,
            onActuatorChange: onActuatorChange,
            onModeChange: onModeChange,
        });

        // Set initial pose
        updateSimulation(currentPose);

        // Start render loop
        animate();

        console.log('Stewart Platform Simulator — Initialized');
        console.log('Neutral actuator lengths:', StewartKinematics.inverseKinematics(currentPose).lengths.map(l => l.toFixed(2)));
    }

    // Wait for DOM
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', start);
    } else {
        start();
    }
})();
