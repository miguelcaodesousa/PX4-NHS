#!/usr/bin/env bash

export PX4_HOME_LAT=53.407972
export PX4_HOME_LON=-1.456556
export PX4_HOME_ALT=120.0

# Prints mavlink start exectuable based on sessions IP

WINDOWS_IP=$(awk '/nameserver/ {print $2; exit}' /etc/resolv.conf)
export WINDOWS_IP
WSL_IP=$(hostname -I | awk '{print $1}')
export WSL_IP

echo "🧠 Copy and paste the following into the PX4 shell to initiate the mavlink connections:

# QGroundControl connection
mavlink start -u 14540 -o 14550 -t  $WINDOWS_IP -m onboard -x

# ADSB fake signal .py injector for simulation
mavlink start -u 14600 -o 14601 -t $WSL_IP -m custom -x"

# ADSB detect & avoidance logic .py connection 
mavlink start -u 14620 -o 14621 -t $WSL_IP -m onboard -x

# Set mavlink connections to only connect to 1 port
param set MAV_0_FORWARD 0
param set MAV_1_FORWARD 0
param set MAV_2_FORWARD 0

cd ~/PX4-Stable   # or wherever PX4 is installed
make px4_sitl gz_rc_cessna


