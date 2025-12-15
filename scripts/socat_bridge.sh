#!/bin/bash
# Socat bridge script with auto-reconnect
# Usage: ./socat_bridge.sh

LIDAR_PORT="/tmp/lidar"
LIDAR_TCP="127.0.0.1:8889"

MOTOR_PORT="/tmp/motor"
MOTOR_TCP="127.0.0.1:8890"

# Kill any existing socat processes for these ports
cleanup() {
    echo "Cleaning up..."
    pkill -f "socat.*$LIDAR_PORT" 2>/dev/null
    pkill -f "socat.*$MOTOR_PORT" 2>/dev/null
    rm -f "$LIDAR_PORT" "$MOTOR_PORT"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Function to run socat with auto-restart
run_socat() {
    local name=$1
    local port=$2
    local tcp=$3

    while true; do
        echo "[$name] Connecting to $tcp..."
        socat pty,raw,echo=0,link=$port TCP:$tcp 2>&1
        echo "[$name] Disconnected, retrying in 2 seconds..."
        rm -f "$port"
        sleep 2
    done
}

# Clean up old links
rm -f "$LIDAR_PORT" "$MOTOR_PORT"

echo "Starting socat bridges..."
echo "  LIDAR: $LIDAR_TCP -> $LIDAR_PORT"
echo "  MOTOR: $MOTOR_TCP -> $MOTOR_PORT"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Start bridges in background
run_socat "LIDAR" "$LIDAR_PORT" "$LIDAR_TCP" &
LIDAR_PID=$!

run_socat "MOTOR" "$MOTOR_PORT" "$MOTOR_TCP" &
MOTOR_PID=$!

# Wait for both
wait $LIDAR_PID $MOTOR_PID
