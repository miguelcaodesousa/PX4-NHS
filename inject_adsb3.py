#!/usr/bin/env python3
# ADS-B injector with env-config, clear startup print, stable cadence.
from pymavlink import mavutil
import os, time, math, subprocess

# -------------------------
# ENV / CONFIG
# -------------------------
def detect_wsl_ip():
    out = subprocess.getoutput("hostname -I | awk '{print $1}'").strip()
    return out or "127.0.0.1"

def detect_windows_ip():
    try:
        with open("/etc/resolv.conf","r",encoding="utf-8") as f:
            for line in f:
                if "nameserver" in line:
                    return line.split()[1].strip()
    except Exception:
        pass
    return "127.0.0.1"

LINK_MODE   = os.getenv("LINK_MODE", "PX4").upper()  # "PX4" or "QGC"
WSL_IP      = os.getenv("WSL_IP", detect_wsl_ip())
TARGET_IP   = os.getenv("TARGET_IP", detect_windows_ip())  # QGC mode target
PX4_LISTEN_PORT = int(os.getenv("PX4_LISTEN_PORT", "14600"))  # PX4 -u (listen) port
QGC_PORT    = int(os.getenv("QGC_PORT", "14550"))
SOURCE_SYS  = int(os.getenv("SOURCE_SYS", "245"))
SOURCE_COMP = int(os.getenv("SOURCE_COMP", "196"))  # obstacle avoidance

# Motion / identity
START_LAT   = float(os.getenv("START_LAT", "53.4105"))
START_LON   = float(os.getenv("START_LON", "-1.4458"))
ALT_MSL_M   = float(os.getenv("ALT_MSL_M", "120.0"))
GS_MPS      = float(os.getenv("GS_MPS", "30.0"))
HDG_DEG     = float(os.getenv("HDG_DEG", "135.0"))
TURN_RATE_DPS = float(os.getenv("TURN_RATE_DPS", "0.0"))

RATE_HZ     = float(os.getenv("RATE_HZ", "1.0"))
DT          = 1.0 / max(0.5, RATE_HZ)

ICAO        = int(os.getenv("ICAO", "70001"))
CALLSIGN9   = (os.getenv("CALLSIGN", "UAV70001").encode("ascii","ignore") + b"\x00"*9)[:9]
CALLSIGN_PRINT = CALLSIGN9.rstrip(b"\x00").decode(errors="ignore")
EMITTER     = int(os.getenv("EMITTER", "3"))   # light UAV
SQUAWK      = int(os.getenv("SQUAWK", "1000"))
ALT_TYPE    = int(os.getenv("ALT_TYPE", "1"))  # 1 = GNSS

# -------------------------
# Helpers
# -------------------------
def project_point(lat_deg, lon_deg, crs_deg, dist_m):
    R = 6371000.0
    lat1 = math.radians(lat_deg); lon1 = math.radians(lon_deg)
    crs = math.radians(crs_deg); dR = dist_m / R
    lat2 = math.asin(math.sin(lat1)*math.cos(dR) + math.cos(lat1)*math.sin(dR)*math.cos(crs))
    lon2 = lon1 + math.atan2(math.sin(crs)*math.sin(dR)*math.cos(lat1),
                              math.cos(dR) - math.sin(lat1)*math.sin(lat2))
    return math.degrees(lat2), ((math.degrees(lon2)+540)%360)-180

def build_flags(mv):
    f = 0
    f |= getattr(mv, "ADSB_FLAGS_VALID_COORDS",   1<<0)
    f |= getattr(mv, "ADSB_FLAGS_VALID_ALTITUDE", 1<<1)
    f |= getattr(mv, "ADSB_FLAGS_VALID_HEADING",  1<<2)
    f |= getattr(mv, "ADSB_FLAGS_VALID_VELOCITY", 1<<3)
    f |= getattr(mv, "ADSB_FLAGS_VALID_CALLSIGN", 1<<4)
    f |= getattr(mv, "ADSB_FLAGS_VALID_SQUAWK",   1<<5)
    f |= getattr(mv, "ADSB_FLAGS_SIMULATED",      1<<6)
    return f

def open_link():
    if LINK_MODE == "QGC":
        link = f"udpout:{TARGET_IP}:{QGC_PORT}"
        wait_hb = False  # QGC usually won't heartbeat back on this path
    else:
        link = f"udpout:{WSL_IP}:{PX4_LISTEN_PORT}"  # send to PX4 -u (listen) port
        wait_hb = True   # PX4 will heartbeat
    m = mavutil.mavlink_connection(link, source_system=SOURCE_SYS, source_component=SOURCE_COMP)
    print(f"→ ADS-B to {link} | mode={LINK_MODE} | sys={SOURCE_SYS} comp={SOURCE_COMP}")
    print(f"   ICAO={ICAO} CALLSIGN={CALLSIGN_PRINT} RATE={RATE_HZ}Hz")
    if wait_hb:
        try:
            m.wait_heartbeat(timeout=5)
            print("✅ Heartbeat received from PX4")
        except Exception:
            print("⚠️  No heartbeat (continuing anyway)")
    return m

# -------------------------
# Main
# -------------------------
def main():
    mv = mavutil.mavlink
    flags = build_flags(mv)
    master = open_link()

    lat, lon = START_LAT, START_LON
    alt_m = ALT_MSL_M
    hdg = HDG_DEG
    alt_mm = int(round(alt_m * 1000.0))
    next_t = time.monotonic()

    try:
        while True:
            now = time.monotonic()
            if now < next_t:
                time.sleep(max(0.0, next_t - now))
            next_t += DT

            hdg = (hdg + TURN_RATE_DPS * DT) % 360.0
            step = GS_MPS * DT
            lat, lon = project_point(lat, lon, hdg, step)

            lat_e7 = int(round(lat * 1e7))
            lon_e7 = int(round(lon * 1e7))
            hdg_cd = int(round(hdg * 100.0))
            v_cms  = int(round(GS_MPS * 100.0))
            tslc   = 0  # reset on each send

            master.mav.adsb_vehicle_send(
                ICAO, lat_e7, lon_e7, ALT_TYPE, alt_mm,
                hdg_cd, v_cms, 0,
                CALLSIGN9, EMITTER, tslc, flags, SQUAWK
            )
    except KeyboardInterrupt:
        print("\n⏹️  Stopped.")

if __name__ == "__main__":
    main()
