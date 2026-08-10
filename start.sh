#!/usr/bin/env bash
# ============================================================
# start.sh — Launch DrivingSim 6-DOF Stewart Platform
# ============================================================
#
# Usage:
#   ./start.sh                     # Run pothole scenario (default)
#   ./start.sh speed_hump          # Run speed hump scenario
#   ./start.sh turn                # Run turn scenario
#   ./start.sh straight            # Run straight road scenario
#   ./start.sh pothole             # Run pothole scenario
#
# This script:
#   1. Starts the Python WebSocket sim server (port 8765)
#   2. Starts a local HTTP server (port 8080) for the dashboard
#   3. Opens the browser dashboard
#
# Requirements: Python 3.8+, websockets, numpy, pyyaml
# Install: pip3 install websockets numpy pyyaml scipy
# ============================================================

SCENARIO="${1:-speed_hump}"
WS_PORT=8765
HTTP_PORT=8080

# Go to project root
cd "$(dirname "$0")"

echo "============================================================"
echo " DrivingSim 6-DOF Stewart Platform - Integration Demo"
echo "============================================================"
echo " Scenario  : $SCENARIO"
echo " WS Server : ws://localhost:$WS_PORT"
echo " Dashboard : http://localhost:$HTTP_PORT/dashboard/"
echo "============================================================"

# Kill any existing servers on these ports
lsof -ti:$WS_PORT | xargs kill -9 2>/dev/null
lsof -ti:$HTTP_PORT | xargs kill -9 2>/dev/null
sleep 0.5

# Start Python simulation server in background
echo "[1/3] Starting Python WebSocket server..."
python3 run_sim.py --scenario "$SCENARIO" &
SIM_PID=$!
echo "  PID: $SIM_PID"

# Wait for server to be ready
sleep 2

# Start HTTP server for the dashboard
echo "[2/3] Starting HTTP server for dashboard..."
python3 -m http.server $HTTP_PORT --directory . &
HTTP_PID=$!
echo "  PID: $HTTP_PID"

sleep 1

# Open browser
echo "[3/3] Opening dashboard in browser..."
open "http://localhost:$HTTP_PORT/dashboard/index.html"

echo ""
echo "============================================================"
echo " RUNNING. Press Ctrl+C to stop all servers."
echo "============================================================"
echo ""
echo " CONTROLS:"
echo "   1. Open the browser (auto-opened above)"
echo "   2. Select scenario from dropdown (already set to: $SCENARIO)"
echo "   3. Click START to begin the simulation"
echo "   4. Watch the Stewart platform move!"
echo ""

# Cleanup on exit
trap "echo ''; echo 'Stopping...'; kill $SIM_PID $HTTP_PID 2>/dev/null; exit 0" INT TERM

# Wait for either process to die
wait $SIM_PID $HTTP_PID
