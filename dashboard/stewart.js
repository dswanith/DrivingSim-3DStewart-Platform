// dashboard/stewart.js — Stewart Platform 3D Visualizer wrapper

const StewartVisualizer = (() => {
    let scene, camera, renderer, controls;
    let baseMesh, topMesh;
    let actuators = [];    // { barrel, piston, topJoint, bottomJoint, collar, labelEl }
    let container;
    let labelContainer;

    const ACTUATOR_COLORS = [
        0xff4444, 0x44aaff, 0x44ff88, 0xffaa22, 0xdd44ff, 0xffff44
    ];
    const BARREL_RADIUS = 18;
    const PISTON_RADIUS = 12;
    const JOINT_RADIUS = 22;

    function init(containerEl) {
        container = containerEl;

        // Scene
        scene = new THREE.Scene();
        scene.background = new THREE.Color(0x06060e);
        scene.fog = new THREE.FogExp2(0x06060e, 0.00015);

        // Camera
        camera = new THREE.PerspectiveCamera(45, container.clientWidth / container.clientHeight, 1, 20000);
        camera.position.set(2200, 1800, 2200);
        camera.lookAt(0, 800, 0);

        // Renderer
        renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
        renderer.setSize(container.clientWidth, container.clientHeight);
        renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        renderer.shadowMap.enabled = true;
        renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        renderer.toneMapping = THREE.ACESFilmicToneMapping;
        renderer.toneMappingExposure = 1.0;
        container.appendChild(renderer.domElement);

        // Label container
        labelContainer = document.createElement('div');
        labelContainer.style.position = 'absolute';
        labelContainer.style.top = '0';
        labelContainer.style.left = '0';
        labelContainer.style.pointerEvents = 'none';
        labelContainer.style.overflow = 'hidden';
        labelContainer.style.width = '100%';
        labelContainer.style.height = '100%';
        container.appendChild(labelContainer);

        // Controls
        controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.target.set(0, 800, 0);
        controls.enableDamping = true;
        controls.dampingFactor = 0.05;
        controls.minDistance = 500;
        controls.maxDistance = 8000;
        controls.update();

        // Lights
        setupLights();

        // Ground grid
        setupGround();

        // Axes helper
        const axesHelper = new THREE.AxesHelper(600);
        scene.add(axesHelper);

        // Build base & top platforms + actuators
        buildBasePlatform();
        buildTopPlatform();
        buildActuators();

        // Start render loop
        animate();

        // Resize handler
        window.addEventListener('resize', onResize);
    }

    function setupLights() {
        const ambient = new THREE.AmbientLight(0x112233, 0.8);
        scene.add(ambient);

        const hemi = new THREE.HemisphereLight(0x00e5ff, 0x221133, 0.4);
        scene.add(hemi);

        const dir1 = new THREE.DirectionalLight(0xffffff, 1.2);
        dir1.position.set(2000, 3000, 1500);
        dir1.castShadow = true;
        dir1.shadow.mapSize.width = 1024;
        dir1.shadow.mapSize.height = 1024;
        dir1.shadow.camera.near = 100;
        dir1.shadow.camera.far = 8000;
        dir1.shadow.camera.left = -1500;
        dir1.shadow.camera.right = 1500;
        dir1.shadow.camera.top = 1500;
        dir1.shadow.camera.bottom = -1500;
        scene.add(dir1);

        const dir2 = new THREE.DirectionalLight(0x00e5ff, 0.3);
        dir2.position.set(-1500, 2000, -1000);
        scene.add(dir2);
    }

    function setupGround() {
        const grid = new THREE.GridHelper(6000, 40, 0x222244, 0x111122);
        grid.position.y = -2;
        scene.add(grid);

        const groundGeo = new THREE.CircleGeometry(3000, 64);
        const groundMat = new THREE.MeshStandardMaterial({
            color: 0x050510,
            roughness: 0.95,
            metalness: 0.1,
        });
        const ground = new THREE.Mesh(groundGeo, groundMat);
        ground.rotation.x = -Math.PI / 2;
        ground.position.y = -3;
        ground.receiveShadow = true;
        scene.add(ground);
    }

    function buildBasePlatform() {
        const pts = StewartKinematics.BASE_POINTS;
        const shape = new THREE.Shape();
        shape.moveTo(pts[0][0], pts[0][1]);
        for (let i = 1; i < pts.length; i++) {
            shape.lineTo(pts[i][0], pts[i][1]);
        }
        shape.closePath();

        const extrudeSettings = { depth: 20, bevelEnabled: true, bevelThickness: 5, bevelSize: 5, bevelSegments: 3 };
        const geo = new THREE.ExtrudeGeometry(shape, extrudeSettings);
        const mat = new THREE.MeshStandardMaterial({
            color: 0xa8b2d1,
            roughness: 0.2,
            metalness: 0.8,
            emissive: 0x1a2238,
        });
        baseMesh = new THREE.Mesh(geo, mat);
        baseMesh.rotation.x = Math.PI / 2;
        baseMesh.position.y = 0;
        baseMesh.receiveShadow = true;
        scene.add(baseMesh);

        // Attachment points
        pts.forEach((p, i) => {
            const ringGeo = new THREE.RingGeometry(15, 25, 16);
            const ringMat = new THREE.MeshBasicMaterial({ color: ACTUATOR_COLORS[i], side: THREE.DoubleSide });
            const ring = new THREE.Mesh(ringGeo, ringMat);
            ring.position.set(p[0], 2, p[1]);
            ring.rotation.x = -Math.PI / 2;
            scene.add(ring);
        });
    }

    function buildTopPlatform() {
        const pts = StewartKinematics.PLATFORM_POINTS;
        const shape = new THREE.Shape();
        shape.moveTo(pts[0][0], pts[0][1]);
        for (let i = 1; i < pts.length; i++) {
            shape.lineTo(pts[i][0], pts[i][1]);
        }
        shape.closePath();

        const extrudeSettings = { depth: 15, bevelEnabled: true, bevelThickness: 4, bevelSize: 4, bevelSegments: 3 };
        const geo = new THREE.ExtrudeGeometry(shape, extrudeSettings);
        const mat = new THREE.MeshStandardMaterial({
            color: 0xf1f5f9,
            roughness: 0.15,
            metalness: 0.85,
            emissive: 0x242936,
        });
        topMesh = new THREE.Mesh(geo, mat);
        topMesh.castShadow = true;
        scene.add(topMesh);
    }

    function buildActuators() {
        for (let i = 0; i < 6; i++) {
            const color = ACTUATOR_COLORS[i];

            // Barrel (outer cylinder)
            const barrelGeo = new THREE.CylinderGeometry(BARREL_RADIUS, BARREL_RADIUS, 1, 12);
            const barrelMat = new THREE.MeshStandardMaterial({
                color: 0x64748b,
                roughness: 0.25,
                metalness: 0.75,
            });
            const barrel = new THREE.Mesh(barrelGeo, barrelMat);
            barrel.castShadow = true;
            scene.add(barrel);

            // Piston (inner rod)
            const pistonGeo = new THREE.CylinderGeometry(PISTON_RADIUS, PISTON_RADIUS, 1, 12);
            const pistonMat = new THREE.MeshStandardMaterial({
                color: 0xf8fafc,
                roughness: 0.08,
                metalness: 0.95,
            });
            const piston = new THREE.Mesh(pistonGeo, pistonMat);
            piston.castShadow = true;
            scene.add(piston);

            // Bottom Joint
            const bjGeo = new THREE.SphereGeometry(JOINT_RADIUS, 16, 16);
            const bjMat = new THREE.MeshStandardMaterial({
                color: color,
                roughness: 0.2,
                metalness: 0.7,
                emissive: new THREE.Color(color).multiplyScalar(0.2),
            });
            const bottomJoint = new THREE.Mesh(bjGeo, bjMat);
            scene.add(bottomJoint);

            // Top Joint
            const tjGeo = new THREE.SphereGeometry(JOINT_RADIUS * 0.85, 16, 16);
            const tjMat = new THREE.MeshStandardMaterial({
                color: color,
                roughness: 0.2,
                metalness: 0.7,
                emissive: new THREE.Color(color).multiplyScalar(0.2),
            });
            const topJoint = new THREE.Mesh(tjGeo, tjMat);
            scene.add(topJoint);

            // Collar
            const collarGeo = new THREE.TorusGeometry(BARREL_RADIUS + 3, 3, 8, 16);
            const collarMat = new THREE.MeshStandardMaterial({ color: color });
            const collar = new THREE.Mesh(collarGeo, collarMat);
            scene.add(collar);

            // Label
            const labelEl = document.createElement('div');
            labelEl.className = 'actuator-label';
            labelEl.textContent = `L${i + 1}`;
            labelEl.style.borderColor = '#' + new THREE.Color(color).getHexString();
            labelContainer.appendChild(labelEl);

            actuators.push({ barrel, piston, bottomJoint, topJoint, collar, labelEl });
        }
    }

    function updateActuator(index, basePoint, topPoint) {
        const act = actuators[index];
        const bPos = new THREE.Vector3(basePoint[0], basePoint[2], basePoint[1]);
        const tPos = new THREE.Vector3(topPoint[0], topPoint[2], topPoint[1]);

        const midPoint = new THREE.Vector3().addVectors(bPos, tPos).multiplyScalar(0.5);
        const direction = new THREE.Vector3().subVectors(tPos, bPos);
        const totalLength = direction.length();

        const barrelLength = totalLength * 0.55;
        const pistonLength = totalLength * 0.55;

        const barrelCenter = new THREE.Vector3().lerpVectors(bPos, tPos, 0.275);
        const pistonCenter = new THREE.Vector3().lerpVectors(bPos, tPos, 0.725);

        const up = new THREE.Vector3(0, 1, 0);
        const quat = new THREE.Quaternion().setFromUnitVectors(up, direction.clone().normalize());

        act.barrel.position.copy(barrelCenter);
        act.barrel.quaternion.copy(quat);
        act.barrel.scale.set(1, barrelLength, 1);

        act.piston.position.copy(pistonCenter);
        act.piston.quaternion.copy(quat);
        act.piston.scale.set(1, pistonLength, 1);

        const collarPos = new THREE.Vector3().lerpVectors(bPos, tPos, 0.5);
        act.collar.position.copy(collarPos);
        act.collar.quaternion.copy(quat);
        act.collar.rotateX(Math.PI / 2);

        act.bottomJoint.position.copy(bPos);
        act.topJoint.position.copy(tPos);

        updateLabel(act.labelEl, midPoint);
    }

    function updateLabel(labelEl, worldPos) {
        const vec = worldPos.clone().project(camera);
        const widthHalf = container.clientWidth / 2;
        const heightHalf = container.clientHeight / 2;

        const x = (vec.x * widthHalf) + widthHalf;
        const y = -(vec.y * heightHalf) + heightHalf;

        if (vec.z > 1) {
            labelEl.style.display = 'none';
        } else {
            labelEl.style.display = 'block';
            labelEl.style.left = x + 'px';
            labelEl.style.top = y + 'px';
        }
    }

    function updatePose(pose) {
        // Run Inverse Kinematics locally for ThreeJS coordinates
        const ikResult = StewartKinematics.inverseKinematics(pose);
        
        // Update top plate
        topMesh.position.set(pose.x, pose.z, pose.y);
        topMesh.rotation.set(0, 0, 0);
        topMesh.quaternion.set(0, 0, 0, 1);

        const layFlat = new THREE.Quaternion().setFromEuler(new THREE.Euler(Math.PI / 2, 0, 0));
        const R = StewartKinematics.rotationMatrix(pose.roll, pose.pitch, pose.yaw);
        const platformQuat = new THREE.Quaternion().setFromRotationMatrix(
            new THREE.Matrix4().set(
                R[0][0], R[0][2], R[0][1], 0,
                R[2][0], R[2][2], R[2][1], 0,
                R[1][0], R[1][2], R[1][1], 0,
                0, 0, 0, 1
            )
        );
        topMesh.quaternion.copy(platformQuat.multiply(layFlat));
        topMesh.position.set(pose.x, pose.z, pose.y);

        // Update actuators
        for (let i = 0; i < 6; i++) {
            updateActuator(i, ikResult.basePoints[i], ikResult.topPoints[i]);
        }
    }

    function updatePlatformDirect(basePoints, topPoints, pose) {
        // Position top mesh directly from values received from WebSocket (already computed by Python IK)
        topMesh.position.set(pose.x, pose.z, pose.y);
        topMesh.rotation.set(0, 0, 0);
        topMesh.quaternion.set(0, 0, 0, 1);

        const layFlat = new THREE.Quaternion().setFromEuler(new THREE.Euler(Math.PI / 2, 0, 0));
        const R = StewartKinematics.rotationMatrix(pose.roll, pose.pitch, pose.yaw);
        const platformQuat = new THREE.Quaternion().setFromRotationMatrix(
            new THREE.Matrix4().set(
                R[0][0], R[0][2], R[0][1], 0,
                R[2][0], R[2][2], R[2][1], 0,
                R[1][0], R[1][2], R[1][1], 0,
                0, 0, 0, 1
            )
        );
        topMesh.quaternion.copy(platformQuat.multiply(layFlat));
        topMesh.position.set(pose.x, pose.z, pose.y);

        // Position actuators using joints from WebSocket (Python is the single source of truth!)
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
        
        // Refresh labels on camera drag
        for (let i = 0; i < actuators.length; i++) {
            const act = actuators[i];
            const bPos = act.bottomJoint.position;
            const tPos = act.topJoint.position;
            const mid = new THREE.Vector3().addVectors(bPos, tPos).multiplyScalar(0.5);
            updateLabel(act.labelEl, mid);
        }
    }

    return {
        init,
        updatePose,
        updatePlatformDirect,
    };
})();
