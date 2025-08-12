#!/usr/bin/env python3
"""
ADS-B HOLD v1 (dynamic WSL IP + MAVLink port)
- Simulates an inbound helicopter from the NW toward the Sheffield helipad
- When it gets close (range/altitude thresholds), command PX4 to HOLD (AUTO.LOITER)
- Auto-detects WSL IP and uses avoidance link port (14600 by default)
- Tries udpin first (PX4 sending to us), then falls back to udpout
- No resume logic yet (manual resume via QGC or pxh)

Run:
  source ~/px4-venv/bin/activate
  # optional overrides:
  #   export WSL_IP=$(hostname -I | awk '{print $1}')
  #   export PX4_MAV_PORT=14600
  python3 adsb_hold_v1.py
"""

import math, time, os, subprocess

# --------------------
# Site / scenario
# --------------------
PAD_LAT = 53.408917      # 53°24'32.1"N
PAD_LON = -1.456639      # 1°27'23.9"W
PAD_ALT_FT = 400

# Ownship (fixed-wing stand-in). Only used for relative geometry/clock code.
UAV_LAT = 53.407972
UAV_LON = -1.456556
UAV_ALT_FT = 400
UAV_HEADING_DEG = 0

# Dummy helicopter approach (NW → pad)
START_RANGE_NM = 3.0
HELO_GS_KT = 90.0
HELO_START_AGL_FT = 1000
HELO_FINAL_AGL_FT = 150

# Trigger (keep this conservative)
TRIGGER_RANGE_NM = 1.5
TRIGGER_VERT_FT  = 700
TICK_HZ = 1.0
SIM_SECONDS = 240

# --------------------
# PX4 / MAVLink config (dynamic)
# --------------------
def _detect_wsl_ip():
    ip_env = os.getenv("WSL_IP")
    if ip_env:
        return ip_env.strip()
    try:
        return subprocess.check_output(
            "hostname -I | awk '{print $1}'", shell=True
        ).decode().strip()
    except Exception:
        return "127.0.0.1"

WSL_IP = _detect_wsl_ip()
PX4_MAV_PORT = int(os.getenv("PX4_MAV_PORT", "14600"))  # matches: mavlink start -u 14600 ...
PX4_SYSID = 1
PX4_COMPID = 1

# Prefer listening for PX4 sending to us; fallback to sending to PX4
PX4_LINK_UDPIN  = f'udpin:0.0.0.0:{PX4_MAV_PORT}'
PX4_LINK_UDPOUT = f'udpout:{WSL_IP}:{PX4_MAV_PORT}'

NM_TO_M = 1852.0
FT_TO_M = 0.3048

# --------------------
# Geo helpers
# --------------------
def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dlmb/2)**2
    return 2*R*math.atan2(math.sqrt(a), math.sqrt(1-a))

def bearing_deg(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    y = math.sin(dlon) * math.cos(p2)
    x = math.cos(p1)*math.sin(p2) - math.sin(p1)*math.cos(p2)*math.cos(dlon)
    return (math.degrees(math.atan2(y, x)) + 360) % 360

def project_point(lat, lon, crs_deg, dist_m):
    R = 6371000.0
    cr = math.radians(crs_deg)
    dR = dist_m / R
    lat1 = math.radians(lat); lon1 = math.radians(lon)
    lat2 = math.asin(math.sin(lat1)*math.cos(dR) + math.cos(lat1)*math.sin(dR)*math.cos(cr))
    lon2 = lon1 + math.atan2(math.sin(cr)*math.sin(dR)*math.cos(lat1),
                              math.cos(dR)-math.sin(lat1)*math.sin(lat2))
    return math.degrees(lat2), ((math.degrees(lon2)+540)%360)-180

def clock_code(bearing, own_heading):
    rel = (bearing - own_heading + 360) % 360
    return f"{int(((rel + 15) % 360) / 30) + 1} o’clock"

# --------------------
# Helo sim state
# --------------------
class Helo:
    def __init__(self):
        # place start NW of pad (course-to-pad ≈ 135; start bearing-from-pad ≈ 315)
        start_bearing_from_pad = 315
        self.lat, self.lon = project_point(PAD_LAT, PAD_LON,
                                           start_bearing_from_pad,
                                           START_RANGE_NM * NM_TO_M)
        self.alt_ft = PAD_ALT_FT + HELO_START_AGL_FT
        self.tgt_lat, self.tgt_lon = PAD_LAT, PAD_LON
        self.tgt_alt_ft = PAD_ALT_FT + HELO_FINAL_AGL_FT
        self.gs_mps = HELO_GS_KT * 0.514444

    def step(self, dt):
        dist_m = haversine_m(self.lat, self.lon, self.tgt_lat, self.tgt_lon)
        if dist_m < 5.0:
            self.lat, self.lon, self.alt_ft = self.tgt_lat, self.tgt_lon, self.tgt_alt_ft
            return
        crs = bearing_deg(self.lat, self.lon, self.tgt_lat, self.tgt_lon)
        step_m = self.gs_mps * dt
        frac = min(step_m / max(dist_m, 1.0), 1.0)
        self.alt_ft += (self.tgt_alt_ft - self.alt_ft) * frac
        self.lat, self.lon = project_point(self.lat, self.lon, crs, step_m)

# --------------------
# PX4 actions
# --------------------
def px4_hold():
    try:
        from pymavlink import mavutil

        def _connect():
            # 1) Try to LISTEN for PX4 packets if PX4 is sending to our WSL IP:PORT
            try:
                m = mavutil.mavlink_connection(PX4_LINK_UDPIN, source_system=245)
                m.wait_heartbeat(timeout=3)
                return m, "udpin"
            except Exception:
                pass
            # 2) Fallback: actively send to PX4 at our WSL IP:PORT
            m = mavutil.mavlink_connection(PX4_LINK_UDPOUT, source_system=245)
            m.wait_heartbeat(timeout=5)
            return m, "udpout"

        m, mode = _connect()
        # Switch to LOITER (Hold)
        m.mav.command_long_send(
            PX4_SYSID, PX4_COMPID,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
            0, 0,0,0,0, 0,0,0
        )
        print(f"➡️  Sent PX4 HOLD (LOITER_UNLIM) via {mode} ({WSL_IP}:{PX4_MAV_PORT}).")
    except Exception as e:
        print(f"⚠️  Could not reach PX4 to send HOLD (tried {PX4_LINK_UDPIN} then {PX4_LINK_UDPOUT}): {e}")

# --------------------
# Main
# --------------------
def main():
    helo = Helo()
    print("🟢 ADS-B HOLD v1 — triggers LOITER when dummy traffic gets close.")
    print(f"   WSL_IP={WSL_IP}  PX4_MAV_PORT={PX4_MAV_PORT}")
    print(f"   Will try {PX4_LINK_UDPIN} then {PX4_LINK_UDPOUT}\n")

    hold_sent = False
    t0 = time.time()
    while time.time() - t0 < SIM_SECONDS:
        helo.step(1.0 / TICK_HZ)

        rng_m = haversine_m(UAV_LAT, UAV_LON, helo.lat, helo.lon)
        rng_nm = rng_m / NM_TO_M
        brg = bearing_deg(UAV_LAT, UAV_LON, helo.lat, helo.lon)
        clk = clock_code(brg, UAV_HEADING_DEG)
        vsep_ft = abs(helo.alt_ft - UAV_ALT_FT)

        print(f"Intruder {rng_nm:.2f} NM, {clk}, Δalt {int(vsep_ft)} ft")

        if not hold_sent and (rng_nm <= TRIGGER_RANGE_NM or vsep_ft <= TRIGGER_VERT_FT):
            print("⚠️  Trigger reached → commanding HOLD.")
            px4_hold()
            hold_sent = True

        time.sleep(1.0 / TICK_HZ)

    print("\n✅ v1 done (no auto-resume).")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n⏹️  Stopped.")
