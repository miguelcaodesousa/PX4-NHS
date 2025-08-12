#!/usr/bin/env python3
# ADS-B injector with env-config, bound source port, tslc handling, steady cadence.
import os, time, math, subprocess, socket, signal
from pymavlink.dialects.v20 import common as mavlink2

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

LINK_MODE       = os.getenv("LINK_MODE", "PX4").upper()  # "PX4" or "QGC"
WSL_IP          = os.getenv("WSL_IP", detect_wsl_ip())
TARGET_IP       = os.getenv("TARGET_IP", detect_windows_ip())  # QGC mode target
PX4_LISTEN_PORT = int(os.getenv("PX4_LISTEN_PORT", "14600"))   # PX4 -u (listen) port
LOCAL_PORT      = int(os.getenv("LOCAL_PORT", "14601"))        # our source port
QGC_PORT        = int(os.getenv("QGC_PORT", "14550"))
SOURCE_SYS      = int(os.getenv("SOURCE_SYS", "245"))
SOURCE_COMP     = int(os.getenv("SOURCE_COMP", "196"))  # obstacle avoidance comp

# Motion / identity
START_LAT       = float(os.getenv("START_LAT", "53.415168"))
START_LON       = float(os.getenv("START_LON", "-1.464263"))
ALT_MSL_M       = float(os.getenv("ALT_MSL_M", "120.0"))
GS_MPS          = float(os.getenv("GS_MPS", "50.0"))
HDG_DEG         = float(os.getenv("HDG_DEG", "90.0"))
TURN_RATE_DPS   = float(os.getenv("TURN_RATE_DPS", "0.0"))
VS_MPS          = float(os.getenv("VS_MPS", "0.0"))

RATE_HZ         = float(os.getenv("RATE_HZ", "1.0"))
DT              = 1.0 / max(0.5, RATE_HZ)

ICAO            = int(os.getenv("ICAO", "70001"))
CALLSIGN9       = (os.getenv("CALLSIGN", "G-YORX AIR AMBULANCE").encode("ascii","ignore") + b"\x00"*9)[:9]
CALLSIGN_PRINT  = CALLSIGN9.rstrip(b"\x00").decode(errors="ignore")
EMITTER         = int(os.getenv("EMITTER", str(mavlink2.ADSB_EMITTER_TYPE_UAV)))
SQUAWK          = int(os.getenv("SQUAWK", "1000"))
ALT_TYPE        = int(os.getenv("ALT_TYPE", str(mavlink2.ADSB_ALTITUDE_TYPE_PRESSURE_QNH)))  # 0=QNH, 1=GNSS

# -------------------------
# Helpers
# -------------------------
def project_point(lat_deg, lon_deg, crs_deg, dist_m):
    R = 6378137.0
    lat1 = math.radians(lat_deg); lon1 = math.radians(lon_deg)
    crs = math.radians(crs_deg); dR = dist_m / R
    lat2 = math.asin(math.sin(lat1)*math.cos(dR) + math.cos(lat1)*math.sin(dR)*math.cos(crs))
    lon2 = lon1 + math.atan2(math.sin(crs)*math.sin(dR)*math.cos(lat1),
                              math.cos(dR) - math.sin(lat1)*math.sin(lat2))
    return math.degrees(lat2), ((math.degrees(lon2)+540)%360)-180

def build_flags():
    f  = mavlink2.ADSB_FLAGS_VALID_COORDS
    f |= mavlink2.ADSB_FLAGS_VALID_ALTITUDE
    f |= mavlink2.ADSB_FLAGS_VALID_HEADING
    f |= mavlink2.ADSB_FLAGS_VALID_VELOCITY
    f |= mavlink2.ADSB_FLAGS_VALID_CALLSIGN
    f |= mavlink2.ADSB_FLAGS_VALID_SQUAWK
    f |= mavlink2.ADSB_FLAGS_SIMULATED
    return f

# -------------------------
# Link (bound UDP + MAVLink v2 encoder)
# -------------------------
def open_link():
    if LINK_MODE == "QGC":
        target = (TARGET_IP, QGC_PORT)
        src_port = LOCAL_PORT  # still bind for stability
    else:
        target = (WSL_IP, PX4_LISTEN_PORT)
        src_port = LOCAL_PORT

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", src_port))
    mav = mavlink2.MAVLink(sock)
    mav.srcSystem = SOURCE_SYS
    mav.srcComponent = SOURCE_COMP

    print(f"→ ADS-B to UDP {src_port} → {target[0]}:{target[1]} | mode={LINK_MODE} | sys={SOURCE_SYS} comp={SOURCE_COMP}")
    print(f"   ICAO={ICAO} CALLSIGN={CALLSIGN_PRINT} RATE={RATE_HZ:.2f}Hz ALT_TYPE={ALT_TYPE} EMITTER={EMITTER}")

    return sock, mav, target

# -------------------------
# Main
# -------------------------
def main():
    flags = build_flags()
    sock, mav, target = open_link()

    lat, lon = START_LAT, START_LON
    alt_m    = ALT_MSL_M
    hdg      = HDG_DEG

    # cadence control
    next_t   = time.monotonic()
    tslc     = 1  # seconds since last comm; keep 1..15 for “fresh” look

    # clean exit
    def _cleanup(*_):
        try: sock.close()
        finally: os._exit(0)
    signal.signal(signal.SIGINT, _cleanup)
    signal.signal(signal.SIGTERM, _cleanup)

    while True:
        now = time.monotonic()
        if now < next_t:
            time.sleep(max(0.0, next_t - now))
            continue
        # slip correction: don’t drift
        missed = max(1, round((now - next_t) / DT) + 1)
        next_t += missed * DT

        # motion
        hdg = (hdg + TURN_RATE_DPS * DT) % 360.0
        step = GS_MPS * DT
        lat, lon = project_point(lat, lon, hdg, step)
        alt_m += VS_MPS * DT

        # fields
        lat_e7 = int(round(lat * 1e7))
        lon_e7 = int(round(lon * 1e7))
        alt_mm = int(round(alt_m * 1000.0))
        hdg_cd = int(round(hdg * 100.0)) % 36000
        v_cms  = int(round(GS_MPS * 100.0))
        vv_cms = int(round(VS_MPS * 100.0))

        # keep tslc in a comfy 1..15 s window
        tslc = 1 if tslc >= 15 else (tslc + 1)

        pkt = mavlink2.MAVLink_adsb_vehicle_message(
            ICAO, lat_e7, lon_e7, ALT_TYPE, alt_mm,
            hdg_cd, v_cms, vv_cms,
            CALLSIGN9, EMITTER, tslc, flags, SQUAWK
        )
        sock.sendto(pkt.pack(mav), target)

        # lightweight live print every ~5s
        if int(now) % 5 == 0:
            # avoid spamming every loop inside the same second
            pass

if __name__ == "__main__":
    main()
