const VehicleTrackViz = (() => {
    let canvas, ctx;
    let width, height;

    // Draw parameters
    const scale = 1.0;
    let time = 0.0;

    function init(canvasEl) {
        canvas = canvasEl;
        ctx = canvas.getContext('2d');
        resize();
        window.addEventListener('resize', resize);
    }

    function resize() {
        if (!canvas) return;
        const rect = canvas.parentNode.getBoundingClientRect();
        canvas.width = rect.width;
        canvas.height = rect.height;
        width = canvas.width;
        height = canvas.height;
    }

    function draw(data) {
        if (!ctx) return;
        ctx.clearRect(0, 0, width, height);

        // Extract values
        const speed = data.vehicle.speed_kmh;
        const ax = data.vehicle.accel_x_ms2;
        const ay = data.vehicle.accel_y_ms2;
        const az = data.vehicle.accel_z_ms2;
        const roll = data.vehicle.roll_rad;
        const pitch = data.vehicle.pitch_rad;
        const yaw = data.vehicle.yaw_rad;
        const yawrate = data.vehicle.ang_vel_z_rads;
        
        const scenario = data.scenario;
        const phase = data.phase;
        const t = data.timestamp_s;

        // Draw background grid
        drawGrid();

        // Save state
        ctx.save();

        if (scenario === 'turn') {
            drawTurnScenario(speed, ay, yaw, yawrate, t);
        } else if (scenario === 'lane_change') {
            drawLaneChangeScenario(speed, ay, yaw, t);
        } else if (scenario === 'pothole') {
            drawPotholeScenario(speed, az, pitch, t);
        } else if (scenario === 'speed_hump') {
            drawSpeedHumpScenario(speed, az, pitch, t);
        } else {
            // Default: Straight
            drawStraightScenario(speed, az, t);
        }

        ctx.restore();
    }

    function drawGrid() {
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.03)';
        ctx.lineWidth = 1;
        const step = 20;

        for (let x = 0; x < width; x += step) {
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, height);
            ctx.stroke();
        }
        for (let y = 0; y < height; y += step) {
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(width, y);
            ctx.stroke();
        }
    }

    function drawStraightScenario(speed, az, t) {
        // Draw straight road
        const roadY = height / 2;
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
        ctx.lineWidth = 40;
        ctx.beginPath();
        ctx.moveTo(0, roadY);
        ctx.lineTo(width, roadY);
        ctx.stroke();

        // Dash lane line
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 2;
        ctx.setLineDash([15, 15]);
        ctx.beginPath();
        ctx.moveTo(0, roadY);
        ctx.lineTo(width, roadY);
        ctx.stroke();
        ctx.setLineDash([]);

        // Animated lines
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
        ctx.lineWidth = 80;
        ctx.beginPath();
        ctx.moveTo(0, roadY - 40);
        ctx.lineTo(width, roadY - 40);
        ctx.moveTo(0, roadY + 40);
        ctx.lineTo(width, roadY + 40);
        ctx.stroke();

        // Draw car
        const carX = width / 2;
        const carY = roadY;
        
        // Draw slight vibration based on az
        const vibration = Math.sin(t * 50) * Math.min(Math.abs(az) * 3, 5);
        drawCarSprite(carX, carY + vibration, 0, 0);
    }

    function drawPotholeScenario(speed, az, pitch, t) {
        const roadY = height / 2;
        const impactTime = 4.0;
        const carX = width / 2;

        // Draw road
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
        ctx.lineWidth = 40;
        ctx.beginPath();
        ctx.moveTo(0, roadY);
        ctx.lineTo(width, roadY);
        ctx.stroke();

        // Draw pothole location
        // The pothole approaches from the right, hits at t=4.0
        // Time offset
        const xOffset = (impactTime - t) * 80; // Speed scaling
        const potholeX = carX + xOffset;

        if (potholeX > 0 && potholeX < width) {
            ctx.fillStyle = '#ff0055';
            ctx.shadowBlur = 15;
            ctx.shadowColor = '#ff0055';
            ctx.beginPath();
            ctx.ellipse(potholeX, roadY, 15, 8, 0, 0, 2 * Math.PI);
            ctx.fill();
            ctx.shadowBlur = 0; // Reset
            
            // Text indicator
            ctx.fillStyle = '#ff0055';
            ctx.font = '500 9px Inter';
            ctx.fillText("POTHOLE", potholeX - 25, roadY - 15);
        }

        // Draw impact waves when hitting
        if (Math.abs(az) > 8.0) {
            ctx.strokeStyle = 'rgba(255, 0, 85, 0.5)';
            ctx.lineWidth = 3;
            ctx.beginPath();
            ctx.arc(carX, roadY, 30 + Math.sin(t * 10) * 10, 0, 2 * Math.PI);
            ctx.stroke();
        }

        // Car pitching motion is visualised as vertical shift
        const vertShift = -pitch * 300; // roll/pitch conversion to px
        drawCarSprite(carX, roadY + vertShift, 0, 0);
    }

    function drawSpeedHumpScenario(speed, az, pitch, t) {
        const roadY = height / 2;
        const humpTime = 4.0;
        const carX = width / 2;

        // Road
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
        ctx.lineWidth = 40;
        ctx.beginPath();
        ctx.moveTo(0, roadY);
        ctx.lineTo(width, roadY);
        ctx.stroke();

        // Draw speed hump as a vertical bump outline
        const xOffset = (humpTime - t) * 80;
        const humpX = carX + xOffset;

        if (humpX > -50 && humpX < width + 50) {
            ctx.fillStyle = '#ffb703';
            ctx.shadowBlur = 10;
            ctx.shadowColor = '#ffb703';
            ctx.beginPath();
            ctx.arc(humpX, roadY, 12, 0, Math.PI, true);
            ctx.fill();
            ctx.shadowBlur = 0;
            
            ctx.fillStyle = '#ffb703';
            ctx.font = '500 9px Inter';
            ctx.fillText("SPEED BUMP", humpX - 30, roadY - 18);
        }

        const vertShift = -pitch * 150;
        drawCarSprite(carX, roadY + vertShift, 0, 0);
    }

    function drawTurnScenario(speed, ay, yaw, yawrate, t) {
        const cx = width / 2;
        const cy = height / 2;

        // Left turn: track curves counterclockwise
        // Draw turn arc
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
        ctx.lineWidth = 30;
        ctx.beginPath();
        ctx.arc(cx - 80, cy - 80, 100, 0, 2 * Math.PI);
        ctx.stroke();

        // Draw yaw angle heading vector
        const carX = cx;
        const carY = cy;

        // Steering angle approx
        const steer = yawrate * 2.0;

        drawCarSprite(carX, carY, yaw, steer);
        drawLateralVector(carX, carY, ay, yaw);
    }

    function drawLaneChangeScenario(speed, ay, yaw, t) {
        const roadY = height / 2;
        const cx = width / 2;

        // Draw lane lines showing S curve
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
        ctx.lineWidth = 50;
        
        ctx.beginPath();
        // Draw sinusoidal path of the road itself for visual effect
        ctx.moveTo(0, roadY);
        for (let x = 0; x < width; x += 10) {
            const laneShift = Math.sin((x / width) * 2 * Math.PI) * 15;
            ctx.lineTo(x, roadY + laneShift);
        }
        ctx.stroke();

        // Car position shifts laterally based on yaw integration
        // The telemetry contains the vehicle's position path or yaw
        // We will drift the car's Y coordinate based on yaw
        const lateralShift = Math.sin(t * 1.5) * 20;

        // Determine steer from derivative approximation
        const steer = Math.cos(t * 1.5) * 0.25;

        drawCarSprite(cx, roadY + lateralShift, yaw, steer);
        drawLateralVector(cx, roadY + lateralShift, ay, yaw);
    }

    function drawCarSprite(x, y, heading, steerAngle = 0) {
        ctx.save();
        ctx.translate(x, y);
        ctx.rotate(heading);

        // Body Shadow
        ctx.shadowColor = 'rgba(0, 229, 255, 0.35)';
        ctx.shadowBlur = 10;

        // Car Body (Top down blue/cyan box)
        ctx.fillStyle = '#0066cc';
        ctx.strokeStyle = '#00e5ff';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.roundRect(-24, -12, 48, 24, 4);
        ctx.fill();
        ctx.stroke();

        ctx.shadowBlur = 0; // Reset shadow

        // Windshield
        ctx.fillStyle = 'rgba(255, 255, 255, 0.2)';
        ctx.beginPath();
        ctx.roundRect(4, -8, 8, 16, 2);
        ctx.fill();

        // Wheels
        ctx.fillStyle = '#333';
        
        // Rear wheels (fixed)
        ctx.fillRect(-16, -15, 8, 4);
        ctx.fillRect(-16, 11, 8, 4);

        // Front wheels (steerable)
        ctx.save();
        ctx.translate(14, -13);
        ctx.rotate(steerAngle);
        ctx.fillRect(-4, -2, 8, 4);
        ctx.restore();

        ctx.save();
        ctx.translate(14, 13);
        ctx.rotate(steerAngle);
        ctx.fillRect(-4, -2, 8, 4);
        ctx.restore();

        // Headlights
        ctx.fillStyle = '#ffff66';
        ctx.fillRect(23, -9, 2, 3);
        ctx.fillRect(23, 6, 2, 3);

        ctx.restore();
    }

    function drawLateralVector(x, y, ay, heading) {
        if (Math.abs(ay) < 0.1) return;

        ctx.save();
        ctx.translate(x, y);
        ctx.rotate(heading);

        // Lateral is perpendicular to heading: positive Y in vehicle frame is left
        // Draw lateral acceleration vector (arrow pointing left/right)
        const arrowLength = ay * 12; // Scale factor

        ctx.strokeStyle = '#ffb703';
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(0, -arrowLength); // Negative Y is left in standard 2D canvas, wait
        // In vehicle frame, ay is left (+Y). Let's draw it in correct direction:
        // Left is upwards on canvas relative to heading
        ctx.stroke();

        // Arrow head
        ctx.fillStyle = '#ffb703';
        ctx.beginPath();
        ctx.moveTo(-5, -arrowLength + (ay > 0 ? 5 : -5));
        ctx.lineTo(0, -arrowLength);
        ctx.lineTo(5, -arrowLength + (ay > 0 ? 5 : -5));
        ctx.fill();

        ctx.fillStyle = '#fff';
        ctx.font = '8px var(--font-mono)';
        ctx.fillText(`${ay > 0 ? '←' : '→'} ${Math.abs(ay / 9.81).toFixed(2)} G`, 8, -arrowLength / 2);

        ctx.restore();
    }

    return {
        init,
        draw,
    };
})();
