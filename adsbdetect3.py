#!/usr/bin/env python3
import sys, time, math, signal, socket
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

# =========================
# Config
# =========================
DEFAULT_ZONE1 = (53.408361, -1.453722)  # Emergency Zone 1 (decimal degrees)
DEFAULT_ZONE2 = (53.409083, -1.457889)  # Emergency Zone 2 (decimal degrees)
DEFAULT_INTRUDER_SPEED_MS = 25.0        # If ADS-B message lacks speed
DEFAULT_DRONE_SPEED_MS = 10.0           # If GLOBAL_POSITION_INT lacks velocity
DEFAULT_THRESHOLD_M = 600.0
# -------------------------
# Utilities
# -------------------------
def prompt_threshold(default_val):
    t = input(f"\nSet trigger threshold in meters [default {default_val}]: ").strip()
    if t == "":
        return default_val
    try:
        return float(t)
    except ValueError:
        print("Invalid input, using default.")
        return default_val

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat/2)**2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(dlon/2)**2)
    return 2 * R * math.asin(math.sqrt(a))

def meters_to_latlon_offsets(dnorth, deast, ref_lat_deg):
    # Small-angle approximation, good for short hops
    dlat = dnorth / 111320.0
    dlon = deast / (111320.0 * math.cos(math.radians(ref_lat_deg)))
    return dlat, dlon

def predict_position(lat, lon, speed_ms, heading_deg, dt_s):
    """
    Simple constant-velocity projection along heading (0=N, 90=E).
    """
    if speed_ms <= 1e-3 or dt_s <= 0.0 or heading_deg is None:
        return lat, lon
    hdg_rad = math.radians(heading_deg % 360.0)
    dnorth = speed_ms * dt_s * math.cos(hdg_rad)
    deast  = speed_ms * dt_s * math.sin(hdg_rad)
    dlat, dlon = meters_to_latlon_offsets(dnorth, deast, lat)
    return lat + dlat, lon + dlon

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

# -------------------------
# Link setup (same pattern you had)
# -------------------------
def open_link():
    # Replace these with your existing values/wiring
    target_sys = 1
    target_comp = 1

    # Your UDP socket + MAVLink context (adapt to your ports/targets)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Example local bind (adjust if needed)
    sock.bind(("0.0.0.0", 14620))
    # Target is your mavlink -t endpoint
    target = ("127.0.0.1", 14621)

    mav = mavlink2.MAVLink(None)
    mav.srcSystem = 255
    mav.srcComponent = 190

    def _cleanup(*_):
        try:
            sock.close()
        finally:
            sys.exit(0)

    signal.signal(signal.SIGINT, _cleanup)
    signal.signal(signal.SIGTERM, _cleanup)

    return sock, mav, target, target_sys, target_comp

# -------------------------
# MAVLink helpers
# -------------------------
def send_heartbeat(mav, sock, target):
    msg = mavlink2.MAVLink_heartbeat_message(
        mavlink2.MAV_TYPE_GCS,
        mavlink2.MAV_AUTOPILOT_INVALID,
        0, 0,
        mavlink2.MAV_STATE_ACTIVE,
        2
    )
    sock.sendto(msg.pack(mav), target)

def send_rtl(mav, sock, target, target_sys, target_comp):
    cmd = mavlink2.MAVLink_command_long_message(
        target_system=target_sys,
        target_component=target_comp,
        command=mavlink2.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        confirmation=0,
        param1=0, param2=0, param3=0, param4=0,
        param5=0, param6=0, param7=0
    )
    sock.sendto(cmd.pack(mav), target)
    print("⚠️  RTL command sent.")

def send_loiter_unlim(mav, sock, target, target_sys, target_comp):
    # Loiter at current position indefinitely (PX4 will hold)
    cmd = mavlink2.MAVLink_command_long_message(
        target_system=target_sys,
        target_component=target_comp,
        command=mavlink2.MAV_CMD_NAV_LOITER_UNLIM,
        confirmation=0,
        param1=0, param2=0, param3=0, param4=math.nan,
        param5=math.nan, param6=math.nan, param7=math.nan
    )
    sock.sendto(cmd.pack(mav), target)
    print("⛰️  HOLD/Loiter command sent (NAV_LOITER_UNLIM).")

def send_reposition(mav, sock, target, target_sys, target_comp, lat, lon, alt_m):
    """
    PX4: DO_REPOSITION sets a loiter waypoint immediately.
    param1 ground speed (m/s) 0=unchanged
    param2 mask: 0 pos+alt, 1 pos only, etc. We'll use 0.
    param5 lat, param6 lon, param7 alt (AMSL)
    """
    cmd = mavlink2.MAVLink_command_long_message(
        target_system=target_sys,
        target_component=target_comp,
        command=mavlink2.MAV_CMD_DO_REPOSITION,
        confirmation=0,
        param1=0,       # keep current
        param2=0,       # mask
        param3=0,       # radius unused
        param4=float('nan'),  # yaw unchanged
        param5=lat, param6=lon, param7=alt_m
    )
    sock.sendto(cmd.pack(mav), target)
    print(f"➡️  Reposition to ({lat:.6f}, {lon:.6f}, {alt_m:.1f}m)")

def send_land_at(mav, sock, target, target_sys, target_comp, lat, lon, alt_m):
    """
    Command a global land. PX4 accepts lat/lon/alt via NAV_LAND.
    """
    cmd = mavlink2.MAVLink_command_long_message(
        target_system=target_sys,
        target_component=target_comp,
        command=mavlink2.MAV_CMD_NAV_LAND,
        confirmation=0,
        param1=0, param2=0, param3=0, param4=float('nan'),
        param5=lat, param6=lon, param7=alt_m
    )
    sock.sendto(cmd.pack(mav), target)
    print(f"🛬  LAND command sent at ({lat:.6f}, {lon:.6f}) alt {alt_m:.1f}m")

# -------------------------
# Interactive prompts
# -------------------------
def prompt_action_mode():
    print("\nSelect avoidance action:")
    print("  1 - HOLD (Loiter)")
    print("  2 - RTL (Return to Launch)")
    print("  3 - LAND at emergency zone")
    choice = input("Choice [1/2/3] (default 1): ").strip()
    if choice not in ("1", "2", "3", ""):
        print("Invalid choice, defaulting to 1 (HOLD).")
        return 1
    return int(choice) if choice else 1

def prompt_zone(name, default_lat, default_lon):
    print(f"\nEnter {name} coordinates (decimal degrees). Press Enter to accept defaults.")
    lat_s = input(f"  {name} latitude  [default {default_lat:.6f}]: ").strip()
    lon_s = input(f"  {name} longitude [default {default_lon:.6f}]: ").strip()
    if lat_s == "" and lon_s == "":
        return (default_lat, default_lon)
    try:
        lat = float(lat_s) if lat_s else default_lat
        lon = float(lon_s) if lon_s else default_lon
        return (lat, lon)
    except ValueError:
        print("  Invalid input. Using defaults.")
        return (default_lat, default_lon)

# -------------------------
# Decision logic
# -------------------------
def choose_best_zone(own_lat, own_lon, own_speed_ms,
                     tgt_lat, tgt_lon, tgt_speed_ms, tgt_heading_deg,
                     zone1, zone2):
    # Time to reach each zone for our drone
    dist1 = haversine_m(own_lat, own_lon, zone1[0], zone1[1])
    dist2 = haversine_m(own_lat, own_lon, zone2[0], zone2[1])
    spd   = own_speed_ms if own_speed_ms > 0.1 else DEFAULT_DRONE_SPEED_MS
    t1 = dist1 / spd
    t2 = dist2 / spd

    # Predict intruder position at those times
    p1_lat, p1_lon = predict_position(tgt_lat, tgt_lon, tgt_speed_ms, tgt_heading_deg, t1)
    p2_lat, p2_lon = predict_position(tgt_lat, tgt_lon, tgt_speed_ms, tgt_heading_deg, t2)

    # Separation at arrival
    sep1 = haversine_m(zone1[0], zone1[1], p1_lat, p1_lon)
    sep2 = haversine_m(zone2[0], zone2[1], p2_lat, p2_lon)

    print(f"\n📐 Zone evaluation:")
    print(f"  Zone1 dist={dist1:.1f} m, ETA={t1:.1f} s, predicted sep={sep1:.1f} m")
    print(f"  Zone2 dist={dist2:.1f} m, ETA={t2:.1f} s, predicted sep={sep2:.1f} m")

    if sep2 > sep1:
        print("✅ Selecting Zone 2 for maximum separation.")
        return zone2, sep2
    else:
        print("✅ Selecting Zone 1 for maximum separation.")
        return zone1, sep1

# -------------------------
# Main loop
# -------------------------
def main():
    # ---- Prompts ----
    trigger_threshold = prompt_threshold(DEFAULT_THRESHOLD_M)

    action_mode = prompt_action_mode()
    zone1 = prompt_zone("Emergency Zone 1", *DEFAULT_ZONE1)
    zone2 = prompt_zone("Emergency Zone 2", *DEFAULT_ZONE2)

    sock, mav, target, target_sys, target_comp = open_link()
    sock.setblocking(False)

    last_hb = 0.0
    triggered = False
    first_fix_printed = False

    own_lat = None
    own_lon = None
    own_alt_m = None
    own_speed_ms = None

    tgt_lat = None
    tgt_lon = None
    tgt_speed_ms = None
    tgt_heading_deg = None

    print("\nListening for GLOBAL_POSITION_INT (ownship) and ADSB_VEHICLE (traffic)...")
    print(f"Action mode: {action_mode} (1=HOLD, 2=RTL, 3=LAND)")
    print(f"Zone1: {zone1[0]:.6f}, {zone1[1]:.6f}")
    print(f"Zone2: {zone2[0]:.6f}, {zone2[1]:.6f}")

    while True:
        now = time.monotonic()
        if now - last_hb >= 1.0:
            send_heartbeat(mav, sock, target)
            last_hb = now

        # Read UDP if available
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
            mtype = m.get_type()

            if mtype == 'GLOBAL_POSITION_INT':
                own_lat = m.lat / 1e7
                own_lon = m.lon / 1e7
                own_alt_m = m.alt / 1000.0  # mm -> m
                # Optional velocities in cm/s
                try:
                    vx = getattr(m, 'vx', 0) / 100.0
                    vy = getattr(m, 'vy', 0) / 100.0
                    own_speed_ms = math.hypot(vx, vy)
                except Exception:
                    own_speed_ms = None

                if not first_fix_printed:
                    print(f"[OWN] lat={own_lat:.6f} lon={own_lon:.6f} alt={own_alt_m:.1f}m")
                    first_fix_printed = True

            elif mtype == 'ADSB_VEHICLE' and not triggered:
                # Optional ICAO filter could go here if needed
                tgt_lat = m.lat / 1e7
                tgt_lon = m.lon / 1e7

                # ADS-B heading: centi-degrees, horizontal velocity: cm/s (if provided)
                try:
                    tgt_heading_deg = (m.heading / 100.0) if getattr(m, 'heading', 0) > 0 else None
                except Exception:
                    tgt_heading_deg = None
                try:
                    hv = getattr(m, 'hor_velocity', 0) / 100.0
                    tgt_speed_ms = hv if hv > 0 else DEFAULT_INTRUDER_SPEED_MS
                except Exception:
                    tgt_speed_ms = DEFAULT_INTRUDER_SPEED_MS

                d = haversine_m(own_lat, own_lon, tgt_lat, tgt_lon) if (own_lat and own_lon) else float('inf')
                print(f"[ADSB] ICAO {m.ICAO_address:#06x} range {d:.1f} m  "
                      f"tgt_spd={tgt_speed_ms:.1f} m/s  hdg={tgt_heading_deg if tgt_heading_deg is not None else 'N/A'}")

                if own_lat is None or own_lon is None:
                    continue  # wait for ownship fix

                if d <= trigger_threshold:
                    print(f"✅ Threshold crossed: {d:.1f} m ≤ {trigger_threshold} m")
                    triggered = True


                    mav.mav.statustext_send(6, b"*** EVASIVE MANOEUVRE TRIGGERED ***")

                    if action_mode == 1:
                        send_loiter_unlim(mav, sock, target, target_sys, target_comp)

                    elif action_mode == 2:
                        send_rtl(mav, sock, target, target_sys, target_comp)

                    elif action_mode == 3:
                        # Estimate best zone and land
                        if own_speed_ms is None or own_speed_ms <= 0.1:
                            own_speed_ms = DEFAULT_DRONE_SPEED_MS
                        if tgt_speed_ms is None or tgt_speed_ms <= 0.1:
                            tgt_speed_ms = DEFAULT_INTRUDER_SPEED_MS

                        best_zone, sep = choose_best_zone(
                            own_lat, own_lon, own_speed_ms,
                            tgt_lat, tgt_lon, tgt_speed_ms, tgt_heading_deg,
                            zone1, zone2
                        )
                        # Use current altitude if known, else 50m AGL guess
                        landing_alt = own_alt_m if own_alt_m is not None else 50.0

                        # First reposition, then land (helps PX4 get there cleanly)
                        send_reposition(mav, sock, target, target_sys, target_comp,
                                        best_zone[0], best_zone[1], landing_alt)
                        # Give it a nudge to actually land at that spot
                        send_land_at(mav, sock, target, target_sys, target_comp,
                                     best_zone[0], best_zone[1], landing_alt)

                    # Optional: exit after action in test harness
                    # sys.exit(0)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
