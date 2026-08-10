// ============================================================
// main.js — Wires kinematics, renderer, and UI together
// ============================================================

(function () {
    'use strict';

    let currentPose = StewartKinematics.neutralPose();

    // IK is direct/closed-form — always exact, no iteration needed
    function updateSimulation(pose) {
        currentPose = pose;
        const result = StewartKinematics.inverseKinematics(pose);
        StewartRenderer.updatePlatform(result.basePoints, result.topPoints, pose);
        StewartUI.updateReadouts(pose, result.lengths);
    }

    function animate() {
        requestAnimationFrame(animate);
        StewartRenderer.render();
    }

    function start() {
        const viewport = document.getElementById('viewport');
        StewartRenderer.init(viewport);

        StewartUI.init({ onPoseChange: updateSimulation });

        // Render initial neutral pose
        updateSimulation(currentPose);

        animate();

        console.log('Stewart Platform Simulator — Initialized');
        console.log('Neutral lengths (mm):', StewartKinematics.neutralLengths().map(l => l.toFixed(2)));
    }

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', start);
    } else {
        start();
    }
})();
