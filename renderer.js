// ============================================================
// renderer.js — Three.js 3D Visualization for Stewart Platform
// ============================================================

const StewartRenderer = (() => {
    let scene, camera, renderer, controls;
    let baseMesh, topMesh;
    let actuators = [];    // { barrel, piston, topJoint, bottomJoint, label }
    let axesHelper;
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
        scene.background = new THREE.Color(0x0a0a1a);
        scene.fog = new THREE.FogExp2(0x0a0a1a, 0.00015);

        // Camera
        camera = new THREE.PerspectiveCamera(50, container.clientWidth / container.clientHeight, 1, 20000);
        camera.position.set(2500, 2000, 2500);
        camera.lookAt(0, 800, 0);

        // Renderer
        renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
        renderer.setSize(container.clientWidth, container.clientHeight);
        renderer.setPixelRatio(window.devicePixelRatio);
        renderer.shadowMap.enabled = true;
        renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        renderer.toneMapping = THREE.ACESFilmicToneMapping;
        renderer.toneMappingExposure = 1.2;
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
        controls.dampingFactor = 0.08;
        controls.minDistance = 500;
        controls.maxDistance = 8000;
        controls.update();

        // Lights
        setupLights();

        // Ground
        setupGround();

        // Axes
        axesHelper = new THREE.AxesHelper(600);
        scene.add(axesHelper);

        // Build platforms & actuators
        buildBasePlatform();
        buildTopPlatform();
        buildActuators();

        // Resize handler
        window.addEventListener('resize', onResize);
    }

    function setupLights() {
        const ambient = new THREE.AmbientLight(0x334466, 0.6);
        scene.add(ambient);

        const hemi = new THREE.HemisphereLight(0x88aaff, 0x444422, 0.5);
        scene.add(hemi);

        const dir1 = new THREE.DirectionalLight(0xffffff, 1.0);
        dir1.position.set(2000, 3000, 1500);
        dir1.castShadow = true;
        dir1.shadow.mapSize.width = 2048;
        dir1.shadow.mapSize.height = 2048;
        dir1.shadow.camera.near = 100;
        dir1.shadow.camera.far = 8000;
        dir1.shadow.camera.left = -2000;
        dir1.shadow.camera.right = 2000;
        dir1.shadow.camera.top = 2000;
        dir1.shadow.camera.bottom = -2000;
        scene.add(dir1);

        const dir2 = new THREE.DirectionalLight(0x6688cc, 0.4);
        dir2.position.set(-1500, 2000, -1000);
        scene.add(dir2);

        const point = new THREE.PointLight(0x4488ff, 0.4, 5000);
        point.position.set(0, 1500, 0);
        scene.add(point);
    }

    function setupGround() {
        // Grid
        const grid = new THREE.GridHelper(6000, 30, 0x222244, 0x111133);
        grid.position.y = -2;
        scene.add(grid);

        // Ground plane
        const groundGeo = new THREE.CircleGeometry(3000, 64);
        const groundMat = new THREE.MeshStandardMaterial({
            color: 0x0d0d22,
            roughness: 0.9,
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
            color: 0x1a2a4a,
            roughness: 0.3,
            metalness: 0.7,
            emissive: 0x0a1525,
        });
        baseMesh = new THREE.Mesh(geo, mat);
        baseMesh.rotation.x = -Math.PI / 2;
        baseMesh.position.y = -10;
        baseMesh.castShadow = true;
        baseMesh.receiveShadow = true;
        scene.add(baseMesh);

        // Attachment point markers on base
        pts.forEach((p, i) => {
            const markerGeo = new THREE.RingGeometry(15, 25, 16);
            const markerMat = new THREE.MeshBasicMaterial({ color: ACTUATOR_COLORS[i], side: THREE.DoubleSide });
            const marker = new THREE.Mesh(markerGeo, markerMat);
            marker.position.set(p[0], 2, p[1]);
            marker.rotation.x = -Math.PI / 2;
            scene.add(marker);
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
            color: 0x2244aa,
            roughness: 0.2,
            metalness: 0.8,
            emissive: 0x112255,
        });
        topMesh = new THREE.Mesh(geo, mat);
        topMesh.castShadow = true;
        scene.add(topMesh);
    }

    function buildActuators() {
        for (let i = 0; i < 6; i++) {
            const color = ACTUATOR_COLORS[i];

            // Barrel (lower part of actuator)
            const barrelGeo = new THREE.CylinderGeometry(BARREL_RADIUS, BARREL_RADIUS, 1, 12);
            const barrelMat = new THREE.MeshStandardMaterial({
                color: 0x333344,
                roughness: 0.3,
                metalness: 0.8,
            });
            const barrel = new THREE.Mesh(barrelGeo, barrelMat);
            barrel.castShadow = true;
            scene.add(barrel);

            // Piston (upper part, slides inside barrel)
            const pistonGeo = new THREE.CylinderGeometry(PISTON_RADIUS, PISTON_RADIUS, 1, 12);
            const pistonMat = new THREE.MeshStandardMaterial({
                color: 0x8899bb,
                roughness: 0.15,
                metalness: 0.9,
                emissive: new THREE.Color(color).multiplyScalar(0.08),
            });
            const piston = new THREE.Mesh(pistonGeo, pistonMat);
            piston.castShadow = true;
            scene.add(piston);

            // Bottom joint (universal/spherical at base)
            const bjGeo = new THREE.SphereGeometry(JOINT_RADIUS, 16, 16);
            const bjMat = new THREE.MeshStandardMaterial({
                color: color,
                roughness: 0.25,
                metalness: 0.7,
                emissive: new THREE.Color(color).multiplyScalar(0.15),
            });
            const bottomJoint = new THREE.Mesh(bjGeo, bjMat);
            bottomJoint.castShadow = true;
            scene.add(bottomJoint);

            // Top joint
            const tjGeo = new THREE.SphereGeometry(JOINT_RADIUS * 0.85, 16, 16);
            const tjMat = new THREE.MeshStandardMaterial({
                color: color,
                roughness: 0.25,
                metalness: 0.7,
                emissive: new THREE.Color(color).multiplyScalar(0.15),
            });
            const topJoint = new THREE.Mesh(tjGeo, tjMat);
            topJoint.castShadow = true;
            scene.add(topJoint);

            // Collar ring at barrel-piston interface
            const collarGeo = new THREE.TorusGeometry(BARREL_RADIUS + 3, 4, 8, 16);
            const collarMat = new THREE.MeshStandardMaterial({
                color: color,
                roughness: 0.3,
                metalness: 0.6,
            });
            const collar = new THREE.Mesh(collarGeo, collarMat);
            scene.add(collar);

            // Label
            const labelEl = document.createElement('div');
            labelEl.className = 'actuator-label';
            labelEl.textContent = `L${i + 1}`;
            labelEl.style.color = '#' + new THREE.Color(color).getHexString();
            labelContainer.appendChild(labelEl);

            actuators.push({ barrel, piston, bottomJoint, topJoint, collar, labelEl });
        }
    }

    function updateActuator(index, basePoint, topPoint) {
        const act = actuators[index];
        const bx = basePoint[0], by = basePoint[1], bz = basePoint[2];
        const tx = topPoint[0], ty = topPoint[1], tz = topPoint[2];

        // In Three.js: x = platform x, y = platform z (height), z = platform y
        const bPos = new THREE.Vector3(bx, bz, by);
        const tPos = new THREE.Vector3(tx, tz, ty);

        const midPoint = new THREE.Vector3().addVectors(bPos, tPos).multiplyScalar(0.5);
        const direction = new THREE.Vector3().subVectors(tPos, bPos);
        const totalLength = direction.length();

        // Barrel occupies the lower 55%, piston the upper 55% (they overlap 10%)
        const barrelLength = totalLength * 0.55;
        const pistonLength = totalLength * 0.55;

        const barrelCenter = new THREE.Vector3().lerpVectors(bPos, tPos, 0.275);
        const pistonCenter = new THREE.Vector3().lerpVectors(bPos, tPos, 0.725);

        // Orientation
        const up = new THREE.Vector3(0, 1, 0);
        const quat = new THREE.Quaternion().setFromUnitVectors(up, direction.clone().normalize());

        // Barrel
        act.barrel.position.copy(barrelCenter);
        act.barrel.quaternion.copy(quat);
        act.barrel.scale.set(1, barrelLength, 1);

        // Piston
        act.piston.position.copy(pistonCenter);
        act.piston.quaternion.copy(quat);
        act.piston.scale.set(1, pistonLength, 1);

        // Collar at the barrel-piston interface
        const collarPos = new THREE.Vector3().lerpVectors(bPos, tPos, 0.5);
        act.collar.position.copy(collarPos);
        act.collar.quaternion.copy(quat);
        // Rotate torus to be perpendicular
        act.collar.rotateX(Math.PI / 2);

        // Joints
        act.bottomJoint.position.copy(bPos);
        act.topJoint.position.copy(tPos);

        // Label positioning (project to screen)
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

    function updatePlatform(basePoints, topPoints, pose) {
        // Update top platform mesh position/rotation
        topMesh.position.set(pose.x, pose.z, pose.y);
        topMesh.rotation.set(0, 0, 0);

        // Apply rotation in Three.js coordinate space
        const euler = new THREE.Euler(
            pose.roll * StewartKinematics.DEG2RAD,
            pose.yaw * StewartKinematics.DEG2RAD,
            -pose.pitch * StewartKinematics.DEG2RAD,
            'YZX'
        );
        topMesh.setRotationFromEuler(euler);

        // The extrude geometry is in XY plane, rotated to be horizontal
        // We need to apply the platform rotation properly
        // Reset and rebuild
        topMesh.rotation.set(0, 0, 0);
        topMesh.quaternion.set(0, 0, 0, 1);

        // Build rotation: first lay flat (rotate -90 about x), then apply platform orientation
        const layFlat = new THREE.Quaternion().setFromEuler(new THREE.Euler(-Math.PI / 2, 0, 0));
        const R = StewartKinematics.rotationMatrix(pose.roll, pose.pitch, pose.yaw);
        const platformQuat = new THREE.Quaternion().setFromRotationMatrix(
            new THREE.Matrix4().set(
                R[0][0], R[0][1], R[0][2], 0,
                R[2][0], R[2][1], R[2][2], 0,
                R[1][0], R[1][1], R[1][2], 0,
                0, 0, 0, 1
            )
        );
        topMesh.quaternion.copy(platformQuat.multiply(layFlat));
        topMesh.position.set(pose.x, pose.z, pose.y);

        // Update every actuator
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

    function render() {
        controls.update();
        renderer.render(scene, camera);
    }

    return {
        init,
        updatePlatform,
        render,
    };
})();
