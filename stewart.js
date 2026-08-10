// dashboard/stewart.js — Stewart Platform 3D Visualizer
//
// COORDINATE SYSTEM CONVENTION:
//   Kinematics (CAD) space:   X = left/right,  Y = front/back,  Z = up (Z-up)
//   Three.js world space:     X = left/right,  Y = up,          Z = toward viewer
//
//   Mapping from kinematics → Three.js:
//     THREE.x =  KIN.x
//     THREE.y =  KIN.z  (kinematics Z becomes Three.js Y "up")
//     THREE.z = -KIN.y  (kinematics Y becomes Three.js -Z)
//
//   This mapping is applied consistently everywhere.
//   Units: millimetres throughout (kinematics.js also uses mm).

const StewartVisualizer = (() => {
    let scene, camera, renderer, controls;
    let baseMesh, topMesh;
    let actuators = [];   // { cylinder, jointBase, jointTop, labelEl }
    let container;
    let labelContainer;

    // Actuator colors per leg (L1–L6)
    const ACTUATOR_COLORS = [
        0xff4444, 0x44aaff, 0x44ff88, 0xffaa22, 0xdd44ff, 0xffff44
    ];

    // Actuator visual radii (mm, matching scene scale)
    const BARREL_RADIUS = 28;
    const PISTON_RADIUS = 18;
    const JOINT_RADIUS  = 35;

    // ── Coordinate conversion: kinematics Z-up → Three.js Y-up ──────────────
    // Input: [x, y, z] in kinematics/CAD millimetres (Z-up)
    // Output: THREE.Vector3 in Three.js world space (Y-up)
    function kinToThree(x, y, z) {
        return new THREE.Vector3(x, z, -y);
    }

    // ── Build/init ───────────────────────────────────────────────────────────

    function init(containerEl) {
        container = containerEl;

        // Scene
        scene = new THREE.Scene();
        scene.background = new THREE.Color(0x06060e);
        scene.fog = new THREE.FogExp2(0x06060e, 0.00008);

        // Camera — isometric-ish starting view looking at platform center
        const NH = StewartKinematics.NEUTRAL_HEIGHT; // ~1672 mm
        camera = new THREE.PerspectiveCamera(
            40,
            container.clientWidth / container.clientHeight,
            10,
            30000
        );
        // Place camera so we see the full ~3m base width + ~1.8m height
        camera.position.set(3500, 2800, 3500);
        camera.lookAt(0, NH * 0.5, 0);

        // Renderer
        renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
        renderer.setSize(container.clientWidth, container.clientHeight);
        renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        renderer.shadowMap.enabled = true;
        renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        renderer.toneMapping = THREE.ACESFilmicToneMapping;
        renderer.toneMappingExposure = 1.1;
        container.appendChild(renderer.domElement);

        // Label overlay
        labelContainer = document.createElement('div');
        Object.assign(labelContainer.style, {
            position: 'absolute', top: '0', left: '0',
            pointerEvents: 'none', overflow: 'hidden',
            width: '100%', height: '100%'
        });
        container.appendChild(labelContainer);

        // Orbit controls — target platform mid-height
        controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.target.set(0, NH * 0.5, 0);
        controls.enableDamping = true;
        controls.dampingFactor = 0.05;
        controls.minDistance = 800;
        controls.maxDistance = 15000;
        controls.update();

        // Scene content
        setupLights();
        setupGround();
        setupAxes();
        buildBasePlatform();
        buildTopPlatform();
        buildActuators();

        // Render neutral pose immediately so platform appears on load
        updatePose(StewartKinematics.neutralPose());

        animate();
        window.addEventListener('resize', onResize);
    }

    function setupLights() {
        scene.add(new THREE.AmbientLight(0x334455, 1.2));

        const hemi = new THREE.HemisphereLight(0xaaccff, 0x221133, 0.5);
        scene.add(hemi);

        const dir1 = new THREE.DirectionalLight(0xffffff, 1.4);
        dir1.position.set(3000, 4000, 2000);
        dir1.castShadow = true;
        dir1.shadow.mapSize.set(2048, 2048);
        dir1.shadow.camera.near = 100;
        dir1.shadow.camera.far  = 12000;
        dir1.shadow.camera.left   = -2500;
        dir1.shadow.camera.right  =  2500;
        dir1.shadow.camera.top    =  2500;
        dir1.shadow.camera.bottom = -2500;
        scene.add(dir1);

        const dir2 = new THREE.DirectionalLight(0x00e5ff, 0.4);
        dir2.position.set(-2000, 3000, -1500);
        scene.add(dir2);
    }

    function setupGround() {
        // Grid centred under the base platform
        const grid = new THREE.GridHelper(8000, 60, 0x1a1a3a, 0x111128);
        grid.position.y = -5;
        scene.add(grid);

        const groundGeo = new THREE.CircleGeometry(4000, 64);
        const groundMat = new THREE.MeshStandardMaterial({
            color: 0x050510, roughness: 0.9, metalness: 0.1
        });
        const ground = new THREE.Mesh(groundGeo, groundMat);
        ground.rotation.x = -Math.PI / 2;
        ground.position.y = -6;
        ground.receiveShadow = true;
        scene.add(ground);
    }

    function setupAxes() {
        // World-space axes helper (small, near origin)
        const axes = new THREE.AxesHelper(400);
        axes.position.y = 0;
        scene.add(axes);
    }

    // ── Base platform ────────────────────────────────────────────────────────
    // Build a hexagonal plate whose vertices pass through the 6 base joint
    // positions. In kinematics space: XY plane (Z=0). In Three.js: XZ plane.
    function buildBasePlatform() {
        const pts = StewartKinematics.BASE_POINTS;

        // Build the hexagonal shape in kinematics XY coordinates
        const shape = new THREE.Shape();
        // Hull order that makes a sensible hexagon for this geometry:
        // B1(0,1101), B2(0,1148) are north-pair → start top, go clockwise
        // We build a convex hull by angle around centroid.
        const sorted = [...pts].map((p, i) => ({ x: p[0], y: p[1], i }));
        sorted.sort((a, b) => Math.atan2(a.y, a.x) - Math.atan2(b.y, b.x));

        shape.moveTo(sorted[0].x, sorted[0].y);
        for (let k = 1; k < sorted.length; k++) {
            shape.lineTo(sorted[k].x, sorted[k].y);
        }
        shape.closePath();

        // Extrude downward (negative Z in kinematics = negative Y in Three.js)
        const extrudeSettings = {
            depth: 60,
            bevelEnabled: true,
            bevelThickness: 10,
            bevelSize: 8,
            bevelSegments: 4
        };
        const geo = new THREE.ExtrudeGeometry(shape, extrudeSettings);

        const mat = new THREE.MeshStandardMaterial({
            color: 0xa8b2d1, roughness: 0.2, metalness: 0.8, emissive: 0x1a2238
        });
        baseMesh = new THREE.Mesh(geo, mat);

        // The ExtrudeGeometry shape is in XY. We need it in the XZ plane (Y-up world).
        // Rotate shape so it lies flat on the ground: X→X, Y→-Z, extrude up → +Y
        baseMesh.rotation.x = -Math.PI / 2;  // rotate shape from XY into XZ
        baseMesh.position.set(0, 0, 0);
        baseMesh.receiveShadow = true;
        scene.add(baseMesh);

        // Base joint markers (colored rings at each attachment point)
        pts.forEach((p, i) => {
            const ringGeo = new THREE.RingGeometry(20, 38, 16);
            const ringMat = new THREE.MeshBasicMaterial({
                color: ACTUATOR_COLORS[i], side: THREE.DoubleSide
            });
            const ring = new THREE.Mesh(ringGeo, ringMat);
            // Base joints at Z=0 in kinematics → y=0 in Three.js world, lying flat
            const pos = kinToThree(p[0], p[1], p[2]); // p[2]=0
            ring.position.set(pos.x, 2, pos.z);
            ring.rotation.x = -Math.PI / 2;
            scene.add(ring);
        });
    }

    // ── Top platform ─────────────────────────────────────────────────────────
    // Build a hexagonal plate through the 6 platform attachment points.
    // In platform-local frame (Z=0), same XY shape approach as base.
    function buildTopPlatform() {
        const pts = StewartKinematics.PLATFORM_POINTS;

        // Convex-hull sort by angle
        const sorted = [...pts].map((p, i) => ({ x: p[0], y: p[1], i }));
        sorted.sort((a, b) => Math.atan2(a.y, a.x) - Math.atan2(b.y, b.x));

        const shape = new THREE.Shape();
        shape.moveTo(sorted[0].x, sorted[0].y);
        for (let k = 1; k < sorted.length; k++) {
            shape.lineTo(sorted[k].x, sorted[k].y);
        }
        shape.closePath();

        const extrudeSettings = {
            depth: 45,
            bevelEnabled: true,
            bevelThickness: 8,
            bevelSize: 6,
            bevelSegments: 4
        };
        const geo = new THREE.ExtrudeGeometry(shape, extrudeSettings);
        const mat = new THREE.MeshStandardMaterial({
            color: 0xf1f5f9, roughness: 0.15, metalness: 0.85, emissive: 0x242936
        });
        topMesh = new THREE.Mesh(geo, mat);
        topMesh.castShadow = true;
        // Rotation applied same as base so the extruded shape lies flat
        topMesh.rotation.x = -Math.PI / 2;
        scene.add(topMesh);
    }

    // ── Actuators ────────────────────────────────────────────────────────────
    function buildActuators() {
        actuators = [];
        for (let i = 0; i < 6; i++) {
            const color = ACTUATOR_COLORS[i];

            // Outer barrel (lower ~55% of leg length)
            const barrelGeo = new THREE.CylinderGeometry(BARREL_RADIUS, BARREL_RADIUS, 1, 14);
            const barrelMat = new THREE.MeshStandardMaterial({
                color: 0x64748b, roughness: 0.25, metalness: 0.75
            });
            const barrel = new THREE.Mesh(barrelGeo, barrelMat);
            barrel.castShadow = true;
            scene.add(barrel);

            // Inner piston (upper ~55%, overlaps barrel midpoint)
            const pistonGeo = new THREE.CylinderGeometry(PISTON_RADIUS, PISTON_RADIUS, 1, 14);
            const pistonMat = new THREE.MeshStandardMaterial({
                color: 0xf8fafc, roughness: 0.06, metalness: 0.96
            });
            const piston = new THREE.Mesh(pistonGeo, pistonMat);
            piston.castShadow = true;
            scene.add(piston);

            // Base spherical joint
            const bjGeo = new THREE.SphereGeometry(JOINT_RADIUS, 16, 16);
            const bjMat = new THREE.MeshStandardMaterial({
                color, roughness: 0.2, metalness: 0.7,
                emissive: new THREE.Color(color).multiplyScalar(0.25)
            });
            const jointBase = new THREE.Mesh(bjGeo, bjMat);
            scene.add(jointBase);

            // Top spherical joint
            const tjGeo = new THREE.SphereGeometry(JOINT_RADIUS * 0.85, 16, 16);
            const tjMat = new THREE.MeshStandardMaterial({
                color, roughness: 0.2, metalness: 0.7,
                emissive: new THREE.Color(color).multiplyScalar(0.25)
            });
            const jointTop = new THREE.Mesh(tjGeo, tjMat);
            scene.add(jointTop);

            // Leg label
            const labelEl = document.createElement('div');
            labelEl.className = 'actuator-label';
            labelEl.textContent = `L${i + 1}`;
            labelEl.style.borderColor = '#' + new THREE.Color(color).getHexString();
            labelContainer.appendChild(labelEl);

            actuators.push({ barrel, piston, jointBase, jointTop, labelEl });
        }
    }

    // ── updateActuator ───────────────────────────────────────────────────────
    // Called every frame with kinematics-space coordinates (mm, Z-up).
    // Converts endpoints to Three.js space and positions/scales/rotates the
    // visual actuator components so they span exactly from basePoint to topPoint.
    function updateActuator(index, basePoint, topPoint) {
        const act = actuators[index];

        // Convert kinematics Z-up → Three.js Y-up
        const bPos = kinToThree(basePoint[0], basePoint[1], basePoint[2]);
        const tPos = kinToThree(topPoint[0],  topPoint[1],  topPoint[2]);

        // Direction and total length
        const dir = new THREE.Vector3().subVectors(tPos, bPos);
        const len = dir.length();
        if (len < 1e-6) return;  // degenerate guard

        // Quaternion that rotates Y-axis to point along dir (cylinder default axis = Y)
        const upAxis = new THREE.Vector3(0, 1, 0);
        const quat   = new THREE.Quaternion().setFromUnitVectors(upAxis, dir.clone().normalize());

        // Barrel covers bottom 55% of leg
        const barrelLen    = len * 0.55;
        const barrelCenter = new THREE.Vector3().lerpVectors(bPos, tPos, 0.275);
        act.barrel.position.copy(barrelCenter);
        act.barrel.quaternion.copy(quat);
        act.barrel.scale.set(1, barrelLen, 1);

        // Piston covers top 55% (overlap creates telescoping appearance)
        const pistonLen    = len * 0.55;
        const pistonCenter = new THREE.Vector3().lerpVectors(bPos, tPos, 0.725);
        act.piston.position.copy(pistonCenter);
        act.piston.quaternion.copy(quat);
        act.piston.scale.set(1, pistonLen, 1);

        // Joint spheres at exact endpoints
        act.jointBase.position.copy(bPos);
        act.jointTop.position.copy(tPos);

        // Label at midpoint
        const mid = new THREE.Vector3().lerpVectors(bPos, tPos, 0.5);
        updateLabel(act.labelEl, mid);
    }

    function updateLabel(labelEl, worldPos) {
        const vec = worldPos.clone().project(camera);
        const hw = container.clientWidth  / 2;
        const hh = container.clientHeight / 2;
        const x  = vec.x * hw + hw;
        const y  = -vec.y * hh + hh;
        if (vec.z > 1) {
            labelEl.style.display = 'none';
        } else {
            labelEl.style.display = 'block';
            labelEl.style.left = x + 'px';
            labelEl.style.top  = y + 'px';
        }
    }

    // ── updatePose ───────────────────────────────────────────────────────────
    // Primary animation entry-point.
    // pose = { x, y, z (mm), roll, pitch, yaw (degrees) }
    //
    // This function:
    //  1. Computes IK to get all 6 top attachment world positions
    //  2. Positions the top platform mesh at the correct location + orientation
    //  3. Updates each actuator visual from its exact endpoints
    //
    // SINGLE SOURCE OF TRUTH: StewartKinematics (kinematics.js) provides all
    // coordinates. The Three.js visualizer only does the Y-up conversion.
    function updatePose(pose) {
        // Run IK to get world-space top attachment points
        const ik = StewartKinematics.inverseKinematics(pose);
        //   ik.basePoints[i] = [x,y,z] in kinematics space (fixed)
        //   ik.topPoints[i]  = [x,y,z] in kinematics space (transformed)

        // ── Position and orient the top platform mesh ─────────────────────
        // The ExtrudeGeometry was built in kinematics XY (local frame, Z=0).
        // The mesh has rotation.x = -PI/2 baked in to make it lie flat.
        // Now we need to add the platform pose on top of that.
        //
        // Three.js position: convert platform center (x, y, z) to Y-up
        const threePos = kinToThree(pose.x, pose.y, pose.z);
        topMesh.position.copy(threePos);

        // Orientation: convert ZYX kinematics rotation to Three.js Y-up.
        // Strategy: apply the kinematics rotation R to the "lie flat" base rotation.
        //
        // The "lie flat" transform maps the shape's XY plane into the XZ world plane:
        //   flatQuat rotates the mesh so it sits horizontally (rotation.x = -PI/2)
        //
        // Then we apply the kinematic rotation in Three.js space.
        // The kinematic R matrix is expressed in Z-up. We need it in Y-up.
        // The basis change from Z-up to Y-up is: x'=x, y'=z, z'=-y
        // So R_three = C * R_kin * C^(-1)  where C = [[1,0,0],[0,0,-1],[0,1,0]]
        //
        // Rather than computing this explicitly, we set the quaternion from the
        // remapped rotation matrix columns directly:
        const R = StewartKinematics.rotationMatrix(pose.roll, pose.pitch, pose.yaw);
        // R is a 3x3 array in kinematics Z-up space.
        // In Y-up Three.js: x'=x, y'=z, z'=-y
        // Remapped R for Three.js:
        //   R_three[row][col] where row/col use (x→x, y→z, z→-y) mapping
        //
        // Column-major THREE.Matrix4:
        //   elements = [m00,m10,m20,0, m01,m11,m21,0, m02,m12,m22,0, 0,0,0,1]
        //
        // With basis swap (kinematics → Three.js): 
        //   Three-x ← kin-x (col 0 of R)
        //   Three-y ← kin-z (col 2 of R, but treated as "up" output)  
        //   Three-z ← -kin-y
        //
        // Correct remapping matrix (verified with neutral pose = identity):
        const m4 = new THREE.Matrix4().set(
            // THREE row-major: set(m00, m01, m02, m03,  m10, ...)
            //                     (col0, col1, col2, col3 of each row)
             R[0][0],  R[0][2], -R[0][1],  0,   // Three.js row 0: x-basis
             R[2][0],  R[2][2], -R[2][1],  0,   // Three.js row 1: y-basis (up)
            -R[1][0], -R[1][2],  R[1][1],  0,   // Three.js row 2: z-basis
             0,        0,        0,         1
        );
        const kinQuat = new THREE.Quaternion().setFromRotationMatrix(m4);

        // The flat bake: shape was built in XY, rotated -PI/2 around X to lie flat.
        // The kinematic rotation then rotates the platform about its own axes.
        // Apply: first lay flat, then rotate per pose.
        const flatQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(-Math.PI / 2, 0, 0));
        topMesh.quaternion.copy(kinQuat).multiply(flatQuat);

        // ── Update each actuator from its exact endpoints ─────────────────
        for (let i = 0; i < 6; i++) {
            updateActuator(i, ik.basePoints[i], ik.topPoints[i]);
        }
    }

    // ── updatePlatformDirect ─────────────────────────────────────────────────
    // Called from telemetry.js when data arrives via WebSocket.
    // basePoints and topPoints are in kinematics mm (Z-up) as arrays of [x,y,z].
    // pose is { x,y,z (mm), roll,pitch,yaw (degrees) }.
    function updatePlatformDirect(basePoints, topPoints, pose) {
        // Validate: sanity-check that z values are reasonable
        // Python sends metres → telemetry.js converts to mm before calling here
        if (typeof pose.z === 'number' && Math.abs(pose.z) < 10) {
            console.warn('updatePlatformDirect: pose.z looks like metres, expected mm', pose.z);
        }

        // Use the pose to drive the mesh (ensures rigid-body constraint)
        // then use the pre-computed topPoints from server for actuator endpoints.
        updatePose(pose);

        // Override actuator endpoints with server-computed values (authoritative)
        for (let i = 0; i < 6; i++) {
            updateActuator(i, basePoints[i], topPoints[i]);
        }
    }

    function onResize() {
        if (!container) return;
        camera.aspect = container.clientWidth / container.clientHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(container.clientWidth, container.clientHeight);
    }

    function animate() {
        requestAnimationFrame(animate);
        controls.update();
        renderer.render(scene, camera);

        // Keep labels tracking their 3D positions
        for (let i = 0; i < actuators.length; i++) {
            const act = actuators[i];
            const mid = new THREE.Vector3()
                .addVectors(act.jointBase.position, act.jointTop.position)
                .multiplyScalar(0.5);
            updateLabel(act.labelEl, mid);
        }
    }

    return { init, updatePose, updatePlatformDirect };
})();
