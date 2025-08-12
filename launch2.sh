#!/usr/bin/env bash

export PX4_HOME_LAT=53.407972
export PX4_HOME_LON=-1.456556
export PX4_HOME_ALT=120.0

cd ~/PX4-Stable   # or wherever PX4 is installed
make px4_sitl gz_rc_cessna
