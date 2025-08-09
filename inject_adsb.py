from pymavlink import mavutil

# Connect to PX4 (which is sending to this port)
master = mavutil.mavlink_connection('udpin:0.0.0.0:14601', source_system=2)

print("Waiting for heartbeat...")
master.wait_heartbeat()
print("✅ Connected to PX4")

# Send ADS-B message with correct argument order
master.mav.adsb_vehicle_send(
    70001,                          # ICAO_address
    int(53.4105 * 1e7),             # lat
    int(-1.4458 * 1e7),             # lon
    0,                              # altitude_type (GNSS)
    120000,                         # alt (in mm)
    9000,                           # heading (centidegrees)
    2500,                           # hor_velocity (cm/s)
    0,                              # ver_velocity (cm/s)
    b'UAV70001\x00',                # callsign (9-byte null-padded ASCII)
    3,                              # emitter_type (light UAV)
    1,                              # tslc (time since last contact, s)
    1,                              # flags (position valid)
    1000                            # squawk
)

print("✅ ADS-B message sent.")
