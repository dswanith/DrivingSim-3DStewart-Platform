// ============================================================
// kinematics.js — Stewart Platform Inverse & Forward Kinematics
// All units: millimetres and degrees (converted to radians internally)
//
// Geometry: confirmed CAD coordinates
//   Base:     inner r=1101.4mm, outer r=1148.6mm, pairs at 90°/210°/330°
//   Platform: inner r=516.4mm,  outer r=543.6mm,  pairs at 30°/150°/270°
//   Height:   1672.425mm neutral
//
// CROSS-PAIRING (standard Stewart platform topology):
//   Each base pair fans out to its TWO nearest platform pairs.
//   Base pairs at 90°/210°/330°, platform pairs at 30°/150°/270°
//     North (90°)  → NW (150°) + NE (30°)
//     SW   (210°)  → S  (270°) + NW (150°)
//     SE   (330°)  → NE (30°)  + S  (270°)
//
//   Index i: leg i runs from BASE_POINTS[i] to PLATFORM_POINTS[i]
//   The PLATFORM_POINTS array is ordered to encode this cross-coupling.
// ============================================================

const StewartKinematics = (() => {
    const DEG2RAD = Math.PI / 180;
    const RAD2DEG = 180 / Math.PI;

    // Base attachment points (mm) — fixed, Z=0
    // Three pairs at 90°/210°/330°, inner r=1101.4mm, outer r=1148.6mm
    const BASE_POINTS = [
        [   0.000, 1101.400, 0],  // B1 — North, inner
        [   0.000, 1148.600, 0],  // B2 — North, outer
        [-953.840, -550.700, 0],  // B3 — SW, inner
        [-994.717, -574.300, 0],  // B4 — SW, outer
        [ 953.840, -550.700, 0],  // B5 — SE, inner
        [ 994.717, -574.300, 0],  // B6 — SE, outer
    ];

    // Platform attachment points (mm) — local frame, Z=0
    // Three pairs at 30°/150°/270°, inner r=516.4mm, outer r=543.6mm
    //
    // ORDERED FOR CROSS-PAIRING: index i connects to BASE_POINTS[i]
    //   Each base pair spreads to two different platform pairs (V-shape)
    const PLATFORM_POINTS = [
        [-447.216,  258.200, 0],  // T1 — NW inner  ← B1 (North inner)
        [ 470.771,  271.800, 0],  // T2 — NE outer  ← B2 (North outer)
        [   0.000, -516.400, 0],  // T3 — S  inner  ← B3 (SW inner)
        [-470.771,  271.800, 0],  // T4 — NW outer  ← B4 (SW outer)
        [ 447.216,  258.200, 0],  // T5 — NE inner  ← B5 (SE inner)
        [   0.000, -543.600, 0],  // T6 — S  outer  ← B6 (SE outer)
    ];

    const NEUTRAL_HEIGHT = 1672.425;

    // ---- Rotation Matrix (ZYX intrinsic: Rz*Ry*Rx) ----
    function rotationMatrix(rollDeg, pitchDeg, yawDeg) {
        const r = rollDeg  * DEG2RAD;
        const p = pitchDeg * DEG2RAD;
        const y = yawDeg   * DEG2RAD;
        const cr = Math.cos(r), sr = Math.sin(r);
        const cp = Math.cos(p), sp = Math.sin(p);
        const cy = Math.cos(y), sy = Math.sin(y);
        return [
            [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
            [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
            [-sp,    cp*sr,             cp*cr            ],
        ];
    }

    function matVec(R, v) {
        return [
            R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
            R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
            R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2],
        ];
    }

    function vecSub(a, b) { return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]; }
    function vecAdd(a, b) { return [a[0]+b[0], a[1]+b[1], a[2]+b[2]]; }
    function vecLen(v)    { return Math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]); }

    // ---- Forward Transform ----
    function forwardTransform(pose) {
        const T = [pose.x, pose.y, pose.z];
        const R = rotationMatrix(pose.roll, pose.pitch, pose.yaw);
        return PLATFORM_POINTS.map(p => vecAdd(T, matVec(R, p)));
    }

    // ---- Inverse Kinematics ----
    function inverseKinematics(pose) {
        const topPoints = forwardTransform(pose);
        const lengths   = topPoints.map((tp, i) => vecLen(vecSub(tp, BASE_POINTS[i])));
        return { lengths, topPoints, basePoints: BASE_POINTS };
    }

    // ---- Neutral pose ----
    function neutralPose() {
        return { x: 0, y: 0, z: NEUTRAL_HEIGHT, roll: 0, pitch: 0, yaw: 0 };
    }

    let _neutralLengths = null;
    function neutralLengths() {
        if (!_neutralLengths)
            _neutralLengths = inverseKinematics(neutralPose()).lengths;
        return _neutralLengths;
    }

    // ---- Numerical Jacobian (6×6) ----
    function getJacobian(pose) {
        const J = [];
        const current = inverseKinematics(pose);
        const keys  = ['x','y','z','roll','pitch','yaw'];
        const delta = 1e-4;
        for (let i = 0; i < 6; i++) {
            J[i] = [];
            for (let j = 0; j < 6; j++) {
                const p2 = { ...pose };
                p2[keys[j]] += delta;
                J[i][j] = (inverseKinematics(p2).lengths[i] - current.lengths[i]) / delta;
            }
        }
        return J;
    }

    function determinant6x6(m) {
        let det = 1;
        const A = m.map(r => [...r]);
        for (let i = 0; i < 6; i++) {
            let piv = i;
            for (let j = i+1; j < 6; j++)
                if (Math.abs(A[j][i]) > Math.abs(A[piv][i])) piv = j;
            [A[i], A[piv]] = [A[piv], A[i]];
            if (piv !== i) det *= -1;
            if (Math.abs(A[i][i]) < 1e-12) return 0;
            det *= A[i][i];
            for (let j = i+1; j < 6; j++) {
                const f = A[j][i] / A[i][i];
                for (let k = i+1; k < 6; k++) A[j][k] -= f * A[i][k];
            }
        }
        return det;
    }

    function conditionMetric(pose) {
        return Math.abs(determinant6x6(getJacobian(pose)));
    }

    // ---- Forward kinematics (Levenberg-Marquardt) ----
    function solveFromLengths(targetLengths, initialPose, maxIter=100, tol=1e-4) {
        let pose = { ...initialPose };
        const keys = ['x','y','z','roll','pitch','yaw'];
        let converged = false, iterations = 0;
        for (let iter = 0; iter < maxIter; iter++) {
            iterations++;
            const current = inverseKinematics(pose);
            const error   = targetLengths.map((tl, i) => tl - current.lengths[i]);
            const mse     = error.reduce((a,b) => a+b*b, 0) / 6;
            if (mse < tol) { converged = true; break; }
            const J   = getJacobian(pose);
            const JT  = J[0].map((_, i) => J.map(r => r[i]));
            const JTJ = multiply6x6(JT, J);
            for (let i = 0; i < 6; i++) JTJ[i][i] += 0.0001;
            const JTe = JT.map(r => r.reduce((s,v,i) => s + v*error[i], 0));
            const dp  = solveLinear6x6(JTJ, JTe);
            if (!dp) break;
            keys.forEach((k,i) => pose[k] += dp[i]);
        }
        return { pose, converged, iterations };
    }

    function multiply6x6(A, B) {
        const C = Array.from({length:6}, () => new Array(6).fill(0));
        for (let i=0;i<6;i++) for (let j=0;j<6;j++) for (let k=0;k<6;k++)
            C[i][j] += A[i][k]*B[k][j];
        return C;
    }

    function solveLinear6x6(A, b) {
        const n = 6;
        const M = A.map((row,i) => [...row, b[i]]);
        for (let col=0; col<n; col++) {
            let maxRow = col;
            for (let row=col+1; row<n; row++)
                if (Math.abs(M[row][col]) > Math.abs(M[maxRow][col])) maxRow = row;
            [M[col], M[maxRow]] = [M[maxRow], M[col]];
            if (Math.abs(M[col][col]) < 1e-15) return null;
            for (let row=col+1; row<n; row++) {
                const f = M[row][col] / M[col][col];
                for (let j=col; j<=n; j++) M[row][j] -= f*M[col][j];
            }
        }
        const x = new Array(n);
        for (let i=n-1; i>=0; i--) {
            x[i] = M[i][n];
            for (let j=i+1; j<n; j++) x[i] -= M[i][j]*x[j];
            x[i] /= M[i][i];
        }
        return x;
    }

    return {
        BASE_POINTS, PLATFORM_POINTS, NEUTRAL_HEIGHT,
        DEG2RAD, RAD2DEG,
        rotationMatrix, forwardTransform,
        inverseKinematics, neutralPose, neutralLengths,
        solveFromLengths, conditionMetric,
    };
})();
