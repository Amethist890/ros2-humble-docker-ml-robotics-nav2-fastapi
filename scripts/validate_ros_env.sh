#!/bin/bash
# ROS 2 Environment Validator
# Run this inside the container to verify setup

set -e

echo "=== ROS 2 Humble Environment Validator ==="
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

pass() { echo -e "${GREEN}PASS${NC}: $1"; }
fail() { echo -e "${RED}FAIL${NC}: $1"; }
warn() { echo -e "${YELLOW}WARN${NC}: $1"; }

# Test 1: ROS installation
echo "[1/7] Checking ROS installation..."
if [ -d "/opt/ros/humble" ]; then
    pass "/opt/ros/humble exists"
else
    fail "/opt/ros/humble not found"
    exit 1
fi

# Test 2: ros2 binary
echo "[2/7] Checking ros2 binary..."
if [ -f "/opt/ros/humble/bin/ros2" ]; then
    pass "ros2 binary found"
else
    fail "ros2 binary missing"
    exit 1
fi

# Test 3: Source and verify
echo "[3/7] Checking ros2 command..."
source /opt/ros/humble/setup.bash
if ros2 --help >/dev/null 2>&1; then
    pass "ros2 command works"
else
    fail "ros2 command failed"
    exit 1
fi

# Test 4: DISPLAY variable
echo "[4/7] Checking DISPLAY variable..."
if [ -n "$DISPLAY" ]; then
    pass "DISPLAY=$DISPLAY"
else
    warn "DISPLAY not set (GUI may not work)"
fi

# Test 5: X11 connection
echo "[5/7] Checking X11 GUI support..."
if command -v xeyes >/dev/null 2>&1; then
    # Try to run xeyes briefly
    timeout 2 xeyes >/dev/null 2>&1 &
    sleep 1
    if pgrep xeyes >/dev/null; then
        pkill xeyes 2>/dev/null || true
        pass "X11 GUI working"
    else
        warn "X11 may not be connected (check VcXsrv)"
    fi
else
    apt-get update >/dev/null 2>&1 && apt-get install -y x11-apps >/dev/null 2>&1
    pass "x11-apps installed"
fi

# Test 6: GPU support
echo "[6/7] Checking GPU support..."
if command -v nvidia-smi >/dev/null 2>&1; then
    if nvidia-smi >/dev/null 2>&1; then
        pass "NVIDIA GPU available"
        nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null || true
    else
        warn "nvidia-smi failed (GPU may not be accessible)"
    fi
else
    warn "nvidia-smi not found (GPU not available)"
fi

# Test 7: Turtlesim
echo "[7/7] Checking turtlesim..."
if ros2 pkg list 2>/dev/null | grep -q turtlesim; then
    pass "turtlesim package available"
else
    warn "turtlesim not installed"
fi

echo ""
echo "=== VALIDATION COMPLETE ==="
echo ""
echo "Quick commands to test:"
echo "  ros2 run turtlesim turtlesim_node"
echo "  ros2 run turtlesim turtle_teleop_key"
echo "  ros2 topic list"
echo ""

