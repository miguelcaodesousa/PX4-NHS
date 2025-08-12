#!/usr/bin/env python3
"""
adsbdetect.py — Listen for ADSB_VEHICLE from PX4 and trigger RTL when any target
(Optionally filtered by ICAO) enters a user-chosen distance from ownship.

How to use (example):
  # make sure PX4 has a MAVLink lane for this (e.g., -u 14620 -o 14621 -t <your_ip>)
  # then run:
  PX4_IP=172.17.24.175 PX4_PORT=14620 LOCAL_PORT=14621 \
  ICAO_HEX=0x70001 \
  python3 adsbdetect.py

You’ll be prompted:
  Enter detection threshold in meters (default 1000):
"""

import os
import sys
import time
import math
import socket
import signal
from pymavlink.dialects.v20 import common as mavlink2

# -------------------------
# Config via environment (override as needed)
# -------------------------
PX4_IP      = os.getenv("PX4_IP", "172.17.24.175")
PX4_PORT    = int(os.getenv("PX4_PORT", "14620"))     # PX4 listen (-u) for this lane
LOCAL_PORT  = int(os.getenv("LOCAL_PORT", "14621"))   # our bound source (-o on PX4)
SYSID       = int(os.getenv("SYSID", "246"))          # distinct from injector (245)
COMPID      = int(os.getenv("COMPID", str(mavlink2.MAV_COMP_ID_ONBOARD_COMPUTER)))
ICAO_HEX    = os.getenv("ICAO_HEX", "").strip()       # e.g., "0x70001" or decimal; empty = ANY

def _parse_icao(s: str):
    if not s:
        return None
    try:
        return int(s, 0)  # supports 0x... or decimal
    except Exception:
        return None

ICAO_FILTER = _parse_icao(ICAO_HEX)

# -------------------------
# Prompt for threshold interactively
# -------------------------
try:
    _inp = input("Enter detection threshold in meters (default 1000): ").strip()
    THRESHOLD_M = float(_inp) if _inp else 1000.0
except Exception:
    THRESHOLD_M = 1000.0

# -------------------------
# Utilities
# -------------------------
def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlmb/2)**2
    return 2 * R * math.asin(math.sqrt(a))

# -------------------------
# Link setup (UDP bound + MAVLink v2)
# -------------------------
def open_link():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", LOCAL_PORT))
    target = (PX4_IP, PX4_PORT)

    mav = mavlink2.MAVLink(sock)
    mav.srcSystem = SYSID
    mav.srcComponent = COMPID

    print("\n=== ADS-B Detector (RTL) ===")
    print(f"Partner: UDP {LOCAL_PORT} → {PX4_IP}:{PX4_PORT}")
    print(f"Sys/Comp: {SYSID}/{COMPID}  Threshold: {THRESHOLD_M:.0f} m"
          + (f"  ICAO filter: {ICAO_FILTER:#06x}" if ICAO_FILTER is not None else "  ICAO filter: ANY"))

    # Clean exit
    def _cleanup(*_):
        try:
            sock.close()
        finally:
            sys.exit(0)

    signal.signal(signal.SIGINT, _cleanup)
    signal.signal(signal.SIGTERM, _cleanup)

    return sock, mav, target

# -------------------------
# Heartbeat @ 1 Hz (MAVLink v2)
# -------------------------
def send_heartbeat(mav, sock, target):
    msg = mavlink2.MAVLink_heartbeat_message(
        mavlink2.MAV_TYPE_GCS,          # type
        mavlink2.MAV_AUTOPILOT_INVALID, # autopilot
        0,                              # base_mode
        0,                              # custom_mode
        mavlink2.MAV_STATE_ACTIVE,      # system_status
        2                               # mavlink_version (v2)
    )
    sock.sendto(msg.pack(mav), target)

# -------------------------
# Command RTL
# -------------------------
def send_rtl(mav, sock, target):
    cmd = mavlink2.MAVLink_command_long_message(
        target_system=1,                         # PX4 sysid (SITL default 1)
        target_component=1,                      # autopilot component
        command=mavlink2.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        confirmation=0,
        param1=0, param2=0, param3=0, param4=0,
        param5=0, param6=0, param7=0
    )
    sock.sendto(cmd.pack(mav), target)
    print("⚠️  RTL command sent (MAV_CMD_NAV_RETURN_TO_LAUNCH).")

# -------------------------
# Main loop
# -------------------------
def main():
    sock, mav, target = open_link()
    sock.setblocking(False)

    last_hb = 0.0
    triggered = False
    first_fix_printed = False

    own_lat = None
    own_lon = None

    print("Listening for GLOBAL_POSITION_INT (ownship) and ADSB_VEHICLE (traffic)...")

    while True:
        now = time.monotonic()

        # Heartbeat once per second so PX4 considers us alive
        if now - last_hb >= 1.0:
            send_heartbeat(mav, sock, target)
            last_hb = now

        # Try to receive one UDP datagram (may decode to multiple MAVLink msgs)
        try:
            data, _ = sock.recvfrom(2048)
        except BlockingIOError:
            time.sleep(0.01)
            continue

        try:
            msgs = mav.parse_buffer(data)
        except Exception:
            continue
        if not msgs:
            continue

        for m in msgs:
            mtype = m.get_type()  # robust type check

            if mtype == 'GLOBAL_POSITION_INT':
                own_lat = m.lat / 1e7
                own_lon = m.lon / 1e7
                if not first_fix_printed:
                    print(f"[OWN] lat={own_lat:.6f} lon={own_lon:.6f}")
                    first_fix_printed = True

            elif mtype == 'ADSB_VEHICLE' and not triggered:
                if ICAO_FILTER is not None and m.ICAO_address != ICAO_FILTER:
                    continue
                if own_lat is None or own_lon is None:
                    continue  # wait for ownship position

                tgt_lat = m.lat / 1e7
                tgt_lon = m.lon / 1e7
                d = haversine_m(own_lat, own_lon, tgt_lat, tgt_lon)
                print(f"[ADSB] ICAO {m.ICAO_address:#06x}  range {d:.1f} m")

                if d <= THRESHOLD_M:
                    print(f"✅ Threshold crossed: {d:.1f} m ≤ {THRESHOLD_M:.0f} m")
                    send_rtl(mav, sock, target)
                    triggered = True
                    # Optional: exit after trigger
                    # sys.exit(0)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
