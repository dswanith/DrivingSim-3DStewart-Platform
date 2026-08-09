// ============================================================
// kinematics.js — Stewart Platform Inverse & Forward Kinematics
// All units: millimetres and degrees (converted to radians internally)
// ============================================================

const StewartKinematics = (() => {
    // ---- Constants ----
    const DEG2RAD = Math.PI / 180;
    const RAD2DEG = 180 / Math.PI;

    // Base attachment points (mm) — fixed on the ground plane (Z = 0)
    const BASE_POINTS = [
        [1086.667, 291.171, 0],  // B1 (15deg)
        [-291.171, 1086.667, 0],  // B2 (105deg)
        [-795.495, 795.495, 0],  // B3 (135deg)
        [-795.495, -795.495, 0],  // B4 (225deg)
        [-291.171, -1086.667, 0],  // B5 (255deg)
        [1086.667, -291.171, 0],  // B6 (345deg)
    ];

    // Platform attachment points (mm) — in the platform's local frame
    const PLATFORM_POINTS = [
        [434.151, 303.996, 0],  // P1 (35deg)
        [46.193, 527.983, 0],  // P2 (85deg)
        [-480.343, 223.988, 0],  // P3 (155deg)
        [-480.343, -223.988, 0],  // P4 (205deg)
        [46.193, -527.983, 0],  // P5 (275deg)
        [434.151, -303.996, 0],  // P6 (325deg)
    ];

    // Neutral platform height
    const NEUTRAL_HEIGHT = 1672.425;

    // ---- Rotation Matrix (ZYX intrinsic Euler angles) ----
    // Equivalent to Rz(yaw) * Ry(pitch) * Rx(roll)
    function rotationMatrix(rollDeg, pitchDeg, yawDeg) {
        const r = rollDeg * DEG2RAD;
        const p = pitchDeg * DEG2RAD;
        const y = yawDeg * DEG2RAD;

        const cr = Math.cos(r), sr = Math.sin(r);
        const cp = Math.cos(p), sp = Math.sin(p);
        const cy = Math.cos(y), sy = Math.sin(y);

        // Rz * Ry * Rx
        return [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ];
    }

    // Multiply 3x3 matrix by 3-vector
    function matVec(R, v) {
        return [
            R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
            R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
            R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
        ];
    }

    // Vector subtraction
    function vecSub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
    function vecAdd(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
    function vecLen(v) { return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

    // ---- Forward Transform ----
    // Given a pose {x, y, z, roll, pitch, yaw} (mm and degrees),
    // returns the 6 transformed top-platform points in world space.
    function forwardTransform(pose) {
        const T = [pose.x, pose.y, pose.z];
        const R = rotationMatrix(pose.roll, pose.pitch, pose.yaw);

        return PLATFORM_POINTS.map(p => vecAdd(T, matVec(R, p)));
    }

    // ---- Inverse Kinematics ----
    // Given a pose, returns { lengths: [L1..L6], topPoints: [...], basePoints: [...] }
    function inverseKinematics(pose) {
        const topPoints = forwardTransform(pose);
        const lengths = topPoints.map((tp, i) => vecLen(vecSub(tp, BASE_POINTS[i])));
        return {
            lengths,
            topPoints,
            basePoints: BASE_POINTS,
        };
    }

    // ---- Neutral pose ----
    function neutralPose() {
        return { x: 0, y: 0, z: NEUTRAL_HEIGHT, roll: 0, pitch: 0, yaw: 0 };
    }

    // ---- Newton-Raphson solver for actuator-length control ----
    // Given 6 target actuator lengths, find the platform pose.
    // Uses numerical Jacobian + damped least-squares.
    function solveFromLengths(targetLengths, initialPose, maxIter = 50, tol = 0.01) {
        let pose = { ...initialPose };
        const keys = ['x', 'y', 'z', 'roll', 'pitch', 'yaw'];
        const delta = [0.1, 0.1, 0.1, 0.001, 0.001, 0.001]; // perturbation sizes

        for (let iter = 0; iter < maxIter; iter++) {
            const current = inverseKinematics(pose);
            const error = targetLengths.map((tl, i) => tl - current.lengths[i]);

            // Check convergence
            const maxErr = Math.max(...error.map(Math.abs));
            if (maxErr < tol) break;

            // Numerical Jacobian (6x6)
            const J = [];
            for (let i = 0; i < 6; i++) {
                J[i] = [];
                for (let j = 0; j < 6; j++) {
                    const perturbedPose = { ...pose };
                    perturbedPose[keys[j]] += delta[j];
                    const perturbedLengths = inverseKinematics(perturbedPose).lengths;
                    J[i][j] = (perturbedLengths[i] - current.lengths[i]) / delta[j];
                }
            }

            // Solve J * dp = error using damped pseudo-inverse
            const dp = solveLinear6x6(J, error);
            if (!dp) break;

            // Apply update with clamping
            keys.forEach((k, i) => {
                let step = dp[i];
                // Clamp step to avoid wild jumps
                if (k === 'x' || k === 'y' || k === 'z') {
                    step = Math.max(-50, Math.min(50, step));
                } else {
                    step = Math.max(-2, Math.min(2, step));
                }
                pose[k] += step;
            });
        }

        return pose;
    }

    // Simple 6x6 linear solve using Gaussian elimination with partial pivoting
    function solveLinear6x6(A, b) {
        const n = 6;
        // Augmented matrix
        const M = A.map((row, i) => [...row, b[i]]);

        for (let col = 0; col < n; col++) {
            // Partial pivoting
            let maxVal = Math.abs(M[col][col]);
            let maxRow = col;
            for (let row = col + 1; row < n; row++) {
                if (Math.abs(M[row][col]) > maxVal) {
                    maxVal = Math.abs(M[row][col]);
                    maxRow = row;
                }
            }
            if (maxVal < 1e-12) return null; // Singular
            [M[col], M[maxRow]] = [M[maxRow], M[col]];

            // Eliminate below
            for (let row = col + 1; row < n; row++) {
                const factor = M[row][col] / M[col][col];
                for (let j = col; j <= n; j++) {
                    M[row][j] -= factor * M[col][j];
                }
            }
        }

        // Back substitution
        const x = new Array(n);
        for (let i = n - 1; i >= 0; i--) {
            x[i] = M[i][n];
            for (let j = i + 1; j < n; j++) {
                x[i] -= M[i][j] * x[j];
            }
            x[i] /= M[i][i];
        }
        return x;
    }

    // ---- Public API ----
    return {
        BASE_POINTS,
        PLATFORM_POINTS,
        NEUTRAL_HEIGHT,
        DEG2RAD,
        RAD2DEG,
        rotationMatrix,
        forwardTransform,
        inverseKinematics,
        neutralPose,
        solveFromLengths,
    };
})();
