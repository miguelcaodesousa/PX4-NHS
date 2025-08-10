#!/bin/bash

# =========================
# ACTIVATE PYTHON VENV
# =========================
cd ~ || exit
source px4-venv/bin/activate

# =========================
# CONFIGURATION
# =========================
# ADS-B target coordinates
LATITUDE=53.407972
LONGITUDE=-1.456556
ALTITUDE=120
VELOCITY=25

# Drone home position
export PX4_HOME_LAT=53.407972
export PX4_HOME_LON=-1.456556
export PX4_HOME_ALT=120.0

# PX4 model
MODEL="gz_rc_cessna"

# Paths
PX4_DIR="$HOME/PX4-Stable"
ADSB_SCRIPT="$PX4_DIR/inject_adsb.py"

# =========================
# ENVIRONMENT EXPORTS
# =========================
export GZ_IP=127.0.0.1
export PX4_SIM_SPEED_FACTOR=1

# Detect Windows host IP
HOST_IP=$(awk '/nameserver/ {print $2; exit}' /etc/resolv.conf)
export HOST_IP
echo "🧠 Detected Windows Host IP: $HOST_IP"

# =========================
# CLEAN UP OLD PX4 PROCESSES
# =========================
echo "🧹 Killing any existing PX4 processes..."
pkill -f px4 2>/dev/null

# =========================
# MOVE TO PX4 DIRECTORY
# =========================
cd "$PX4_DIR" || { echo "❌ PX4 directory not found!"; exit 1; }

# =========================
# LAUNCH PX4 SITL (foreground)
# =========================
echo "🧱 Launching PX4 SITL for $MODEL..."
(
    # Wait until PX4 is likely initialized before ADS-B injection
    sleep 15
    echo "🛩️ Starting ADS-B injection..."
    python3 "$ADSB_SCRIPT" --lat $LATITUDE --lon $LONGITUDE --alt $ALTITUDE --vel $VELOCITY &
) &
make px4_sitl_default "$MODEL"
