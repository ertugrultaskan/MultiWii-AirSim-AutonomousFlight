# -*- coding: utf-8 -*-
"""
CommunicationScript.py  -  AirSim <-> MultiWii (Arduino Nano) bridge

What it does:
  1) Asks you for the target points (in metres, relative to the takeoff point).
  2) Reads IMU, barometer and GPS data from AirSim.
  3) Sends this data and the targets to MultiWii on the Arduino Nano over MSP.
  4) Receives the 4 motor values computed by MultiWii and applies them to the
     drone in AirSim.
  5) The drone takes off, visits the targets in order, waits WAIT_TIME_S at
     each one, then returns to the takeoff point and lands.
  6) While flying, it prints the live position (North, East, Altitude), the
     distance to the current target and the wait countdown.

Running (cmd):
  py -3.9 CommunicationScript.py              -> finds the COM port by itself
  py -3.9 CommunicationScript.py --port COM5  -> you tell it the COM port

Emergency:
  Ctrl+C once   -> the drone lands where it is
  Ctrl+C twice  -> motors stop immediately (the drone falls!)
"""

import argparse
import math
import signal
import struct
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: 'pyserial' is not installed.  Run this command:  py -3.9 -m pip install pyserial")
    sys.exit(1)

try:
    import airsim
except ImportError:
    print("ERROR: 'airsim' is not installed. See the Python installation step in the guide.")
    sys.exit(1)


# ============================================================================
#  SETTINGS  (you can change the numbers here)
# ============================================================================
BAUD_RATE          = 250000   # must match SERIAL0_COM_SPEED in config.h
WAIT_TIME_S        = 10       # how many seconds to wait at each target (integer)
TAKEOFF_ALTITUDE   = 3.0      # metres: climb to this height before handing over to MultiWii
MAX_TARGETS        = 20       # the Nano's EEPROM holds 38 steps, 20 fits comfortably
MIN_ALTITUDE       = 2.0      # metres
MAX_ALTITUDE       = 50.0     # metres (MultiWii allows at most 100 m)
MAX_DISTANCE       = 400.0    # metres between two points (MultiWii rejects more than 500 m)
GPS_INTERVAL_S     = 0.1      # GPS is sent 10 times per second (like a real GPS)
REPLY_TIMEOUT      = 0.5      # seconds: alarm if MultiWii does not answer within this time

# PID values adapted for HIL: {index: (P, I, D)}
# 0=ROLL 1=PITCH 2=YAW 3=ALT 4=POS 5=POSR 6=NAVR 7=LEVEL 8=MAG
PID_VALUES = {
    0: (4, 4, 0),       # ROLL : much softer than default (33,30,23) to tolerate the delay
    1: (4, 4, 0),       # PITCH: same
    2: (40, 20, 0),     # YAW  : nose rotation (default 68,45,0)
    7: (18, 0, 100),    # LEVEL: angle control that keeps the drone level (default 90,10,100)
}

# ============================================================================
#  MSP COMMAND NUMBERS (same as MultiWii Protocol.cpp)
# ============================================================================
MSP_IDENT       = 100
MSP_PID         = 112
MSP_SET_PID     = 202
MSP_BOXIDS      = 119
MSP_WP          = 118
MSP_SET_RAW_GPS = 201
MSP_SET_BOX     = 203
MSP_RESET_CONF  = 208
MSP_SET_WP      = 209
MSP_HIL_AIRSIM  = 230         # the new command we added

# Mission step types (MultiWii.h)
MISSION_HOLD_TIME = 3
MISSION_RTH       = 4
MISSION_FLAG_END  = 0xA5

# Navigation states (types.h -> enum navstate)
NAV_STATE_TEXT = {
    0: "Navigation off", 1: "Return home starting", 2: "Returning home",
    3: "Holding position", 4: "Waiting at target", 5: "Flying to target",
    6: "Preparing next step", 7: "Jump", 8: "Landing starting",
    9: "Landing", 10: "Landed", 11: "Settling before landing", 12: "Descent starting",
}
NAV_ERROR_TEXT = {
    1: "Target too far (500 m limit)", 2: "GPS degraded", 3: "Target in EEPROM is corrupt",
    6: "Invalid jump", 7: "Invalid mission step", 9: "GPS signal lost",
}
NAV_LANDED = 10

# RC channels (MultiWii order: ROLL, PITCH, YAW, THROTTLE, AUX1..AUX4)
LOW, MID, HIGH = 1000, 1500, 2000

EARTH_RADIUS = 6378137.0      # metres (MultiWii uses the same value)


# ============================================================================
#  MSP: talking to MultiWii over the serial port
# ============================================================================
class MSP:
    """MSP v1 packet:  '$' 'M' '<' size command data... checksum"""

    def __init__(self, port):
        self.ser = serial.Serial(port, BAUD_RATE, timeout=0.05)

    def send(self, command, data=b""):
        checksum = len(data) ^ command
        for b in data:
            checksum ^= b
        self.ser.write(b"$M<" + bytes([len(data), command]) + data + bytes([checksum]))

    def receive(self, command, timeout=REPLY_TIMEOUT):
        """Returns the reply to the expected command. None on error."""
        deadline = time.time() + timeout
        state, data, size, cmd, checksum, direction = 0, b"", 0, 0, 0, b""
        while time.time() < deadline:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            for c in chunk:
                if state == 0:
                    state = 1 if c == ord("$") else 0
                elif state == 1:
                    state = 2 if c == ord("M") else 0
                elif state == 2:
                    if c in (ord(">"), ord("!")):
                        direction, state = bytes([c]), 3
                    else:
                        state = 0
                elif state == 3:
                    size, checksum, data, state = c, c, b"", 4
                elif state == 4:
                    cmd, checksum, state = c, checksum ^ c, 5 if size else 6
                elif state == 5:
                    data += bytes([c])
                    checksum ^= c
                    if len(data) == size:
                        state = 6
                elif state == 6:
                    state = 0
                    if c != checksum or cmd != command:
                        continue            # corrupt or a different reply: skip
                    if direction == b"!":
                        return None         # MultiWii does not know this command
                    return data
        return None

    def request(self, command, data=b"", timeout=REPLY_TIMEOUT):
        self.ser.reset_input_buffer()
        self.send(command, data)
        return self.receive(command, timeout)

    def close(self):
        self.ser.close()


# ============================================================================
#  Helper functions
# ============================================================================
def clamp(value, low, high):
    return max(low, min(high, value))


def int16(x):
    return int(clamp(round(x), -32768, 32767))


def ask_number(question, low=None, high=None, integer=False):
    """Asks the user for a number. A comma is also accepted (2,5 = 2.5)."""
    while True:
        text = input(question).strip().replace(",", ".")
        try:
            value = int(text) if integer else float(text)
        except ValueError:
            print("   Please type a number.")
            continue
        if low is not None and value < low:
            print("   Must be at least %s." % low)
            continue
        if high is not None and value > high:
            print("   Must be at most %s." % high)
            continue
        return value


def ask_targets():
    print()
    print("=" * 62)
    print(" TARGET POINTS")
    print(" The takeoff point is (0, 0). Values are in METRES.")
    print(" North: + north, - south   |   East: + east, - west")
    print("=" * 62)
    count = ask_number("How many target points? (1-%d): " % MAX_TARGETS, 1, MAX_TARGETS, True)
    targets = []
    prev_n, prev_e = 0.0, 0.0
    for i in range(count):
        print("\n--- Target %d ---" % (i + 1))
        while True:
            n = ask_number("  North (m): ")
            e = ask_number("  East  (m): ")
            distance = math.hypot(n - prev_n, e - prev_e)
            if distance > MAX_DISTANCE:
                print("   Distance to the previous point is %.0f m. It can be at most %.0f m." % (distance, MAX_DISTANCE))
                continue
            break
        alt = ask_number("  Altitude (m, %.0f-%.0f): " % (MIN_ALTITUDE, MAX_ALTITUDE),
                         MIN_ALTITUDE, MAX_ALTITUDE)
        targets.append((n, e, alt))
        prev_n, prev_e = n, e
    if math.hypot(prev_n, prev_e) > MAX_DISTANCE:
        print("WARNING: The last point is %.0f m from home; the return may take long." % math.hypot(prev_n, prev_e))
    print("\nSummary:")
    for i, (n, e, alt) in enumerate(targets):
        print("  %d) North %.1f m, East %.1f m, Altitude %.1f m, wait %d s"
              % (i + 1, n, e, alt, WAIT_TIME_S))
    print("  %d) Return to the takeoff point and land" % (len(targets) + 1))
    return targets


def metres_to_gps(home_lat, home_lon, north, east):
    lat = home_lat + math.degrees(north / EARTH_RADIUS)
    lon = home_lon + math.degrees(east / (EARTH_RADIUS * math.cos(math.radians(home_lat))))
    return lat, lon


def find_port(requested):
    if requested:
        return requested
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("ERROR: No COM port found. Is the Nano plugged in? Is the CH340 driver installed?")
        sys.exit(1)
    candidates = [p for p in ports if any(s in (p.description or "").upper()
                                          for s in ("CH340", "USB-SERIAL", "ARDUINO", "USB SERIAL"))]
    if len(candidates) == 1:
        print("Nano found: %s (%s)" % (candidates[0].device, candidates[0].description))
        return candidates[0].device
    print("Ports found:")
    for i, p in enumerate(ports):
        print("  %d) %s  %s" % (i + 1, p.device, p.description))
    choice = ask_number("Which one is the Nano? Type the number: ", 1, len(ports), True)
    return ports[choice - 1].device


# ============================================================================
#  Converting AirSim data to MultiWii units
# ============================================================================
ACC_1G = 512.0                       # MultiWii: 1 G = 512
GYRO_SCALE = 4.1                     # MultiWii: 1 degree/s = 4.1
MAG_WORLD = (220.0, 0.0, 420.0)      # North, East, Down: virtual magnetic field with no declination


def rotate_world_to_body(q, v):
    """Rotates a world (NED) vector into the body (FRD) frame. q = body->world."""
    w, x, y, z = q.w_val, q.x_val, q.y_val, q.z_val
    # Rotation matrix R (body->world); body = R^T * world
    r00 = 1 - 2 * (y * y + z * z); r01 = 2 * (x * y - w * z);     r02 = 2 * (x * z + w * y)
    r10 = 2 * (x * y + w * z);     r11 = 1 - 2 * (x * x + z * z); r12 = 2 * (y * z - w * x)
    r20 = 2 * (x * z - w * y);     r21 = 2 * (y * z + w * x);     r22 = 1 - 2 * (x * x + y * y)
    vx, vy, vz = v
    return (r00 * vx + r10 * vy + r20 * vz,
            r01 * vx + r11 * vy + r21 * vz,
            r02 * vx + r12 * vy + r22 * vz)


def hil_packet(imu, pressure_pa, rc):
    """AirSim (forward-right-down axes) -> MultiWii axes.
    In MultiWii: ROLL+ = roll right, PITCH+ = nose down, YAW+ = turn right."""
    a = imu.linear_acceleration       # m/s^2, accelerometer (gravity included)
    w = imu.angular_velocity          # rad/s
    k = ACC_1G / 9.80665
    acc = (-a.y_val * k, -a.x_val * k, -a.z_val * k)
    d = math.degrees(1.0) * GYRO_SCALE
    gyro = (w.x_val * d, -w.y_val * d, w.z_val * d)
    m = rotate_world_to_body(imu.orientation, MAG_WORLD)
    mag = (m[1], m[0], m[2])
    values = [int16(v) for v in acc + gyro + mag]
    return struct.pack("<9hi8h", *values, int(pressure_pa), *[int(c) for c in rc])


# ============================================================================
#  MAIN PROGRAM
# ============================================================================
def main():
    ap = argparse.ArgumentParser(description="AirSim <-> MultiWii autonomous flight bridge")
    ap.add_argument("--port", help="The Nano's COM port, e.g. COM5")
    ap.add_argument("--ip", default="127.0.0.1", help="IP address of the AirSim computer")
    args = ap.parse_args()

    targets = ask_targets()
    input("\nIs AirSim open and the drone on the ground? Press Enter to start...")

    # ---------------------------------------------------------------- AirSim
    print("\n[1/6] Connecting to AirSim...")
    try:
        client = airsim.MultirotorClient(ip=args.ip)
        client.confirmConnection()
    except Exception as e:
        print("ERROR: Could not connect to AirSim. Is AirSim (Africa_001.exe) open?\n  ", e)
        sys.exit(1)
    client.reset()
    time.sleep(1.0)
    client.enableApiControl(True)
    client.armDisarm(True)

    print("[2/6] Waiting for AirSim GPS to be ready...")
    deadline = time.time() + 20
    while True:
        gps = client.getGpsData()
        if gps.gnss.fix_type >= 3:
            break
        if time.time() > deadline:
            print("ERROR: AirSim GPS was not ready within 20 s.")
            sys.exit(1)
        time.sleep(0.2)
    home_lat = gps.gnss.geo_point.latitude
    home_lon = gps.gnss.geo_point.longitude
    print("      Takeoff point: latitude %.7f, longitude %.7f" % (home_lat, home_lon))

    # -------------------------------------------------------------- MultiWii
    port = find_port(args.port)
    print("[3/6] Connecting to MultiWii (%s)..." % port)
    try:
        msp = MSP(port)
    except serial.SerialException as e:
        print("ERROR: Could not open %s. The Arduino IDE Serial Monitor may be open, close it.\n  %s" % (port, e))
        sys.exit(1)
    time.sleep(2.5)                   # the Nano restarts when the port opens, wait
    reply = None
    for _ in range(10):
        reply = msp.request(MSP_IDENT)
        if reply:
            break
    if not reply:
        print("ERROR: MultiWii is not answering. Is the port correct? Was the code uploaded to the Nano?")
        sys.exit(1)
    print("      MultiWii version: %.2f, type: %d" % (reply[0] / 100.0, reply[1]))
    idle_rc = [MID, MID, MID, LOW, LOW, LOW, LOW, LOW]
    test = msp.request(MSP_HIL_AIRSIM, hil_packet(client.getImuData(), client.getBarometerData().pressure, idle_rc))
    if test is None or len(test) != 19:
        print("ERROR: MultiWii has no HIL code or an old one. Check the MultiWii steps in the guide.")
        sys.exit(1)

    # Reset settings to defaults (so old PID/box settings do not interfere)
    if msp.request(MSP_RESET_CONF, timeout=2.0) is None:
        print("ERROR: Could not reset the settings.")
        sys.exit(1)

    # Set the PID values for HIL
    pid = msp.request(MSP_PID)
    if not pid:
        print("ERROR: Could not read the PID values.")
        sys.exit(1)
    pid = bytearray(pid)
    for idx, (p_, i_, d_) in PID_VALUES.items():
        pid[idx * 3:idx * 3 + 3] = bytes([p_, i_, d_])
    if msp.request(MSP_SET_PID, bytes(pid)) is None:
        print("ERROR: Could not send the PID values.")
        sys.exit(1)

    # Box (mode) settings: which AUX channel turns on which mode
    ids = msp.request(MSP_BOXIDS)
    if not ids:
        print("ERROR: Could not get the mode list.")
        sys.exit(1)
    def bit(aux, level):              # aux 1-4, level 0=low 1=mid 2=high
        return 1 << ((aux - 1) * 3 + level)
    always_on = bit(1, 0) | bit(1, 1) | bit(1, 2)
    mode_setup = {
        0: bit(2, 2),                 # ARM      : AUX2 high
        1: always_on,                 # ANGLE    : always on (drone always tries to stay level)
        5: always_on,                 # MAG      : always on (keeps the nose direction)
        3: bit(3, 2),                 # BARO     : AUX3 high
        20: bit(3, 2),                # MISSION  : AUX3 high
        21: bit(4, 2),                # LAND     : AUX4 high
    }
    boxes = [mode_setup.get(i, 0) for i in ids]
    if msp.request(MSP_SET_BOX, struct.pack("<%dH" % len(boxes), *boxes)) is None:
        print("ERROR: Could not send the mode settings.")
        sys.exit(1)
    if 20 not in ids:
        print("ERROR: MultiWii has no MISSION mode. Is USE_MSP_WP enabled in config.h?")
        sys.exit(1)

    # ----------------------------------------------------------- Upload mission
    print("[4/6] Uploading targets to MultiWii...")
    steps = []
    for i, (n, e, alt) in enumerate(targets):
        lat, lon = metres_to_gps(home_lat, home_lon, n, e)
        steps.append((i + 1, MISSION_HOLD_TIME, int(round(lat * 1e7)), int(round(lon * 1e7)),
                      int(alt * 100), WAIT_TIME_S, 0))
    last_altitude = int(targets[-1][2] * 100)         # return home at the last target's altitude
    steps.append((len(targets) + 1, MISSION_RTH, 0, 0, last_altitude, 1, MISSION_FLAG_END))  # 1 = land at home
    for (no, action, lat, lon, alt_cm, p1, flag) in steps:
        packet = struct.pack("<BBiiIhhhB", no, action, lat, lon, alt_cm, p1, 0, 0, flag)
        if msp.request(MSP_SET_WP, packet, timeout=2.0) is None:
            print("ERROR: Could not upload step %d." % no)
            sys.exit(1)
        back = msp.request(MSP_WP, bytes([no]), timeout=1.0)
        if not back or struct.unpack("<BBii", back[:10])[2:] != (lat, lon):
            print("ERROR: Could not verify step %d." % no)
            sys.exit(1)
    print("      %d steps uploaded and verified." % len(steps))

    # ------------------------------------------------------------ Main loop
    rc = [MID, MID, MID, LOW, LOW, LOW, LOW, LOW]   # R P Y T AUX1-4
    phase = "PREPARE"
    phase_time = time.time()
    last_gps = 0.0
    last_reply = time.time()
    last_print = 0.0
    loop_count, loop_hz = 0, 0.0
    ctrl_c = 0
    x0 = y0 = z0 = None
    north, east = 0.0, 0.0
    wait_start = None
    alt_f, climb_rate, loop_no, pressure = 0.0, 0.0, 0, 101325.0
    prev_t = time.time()
    throttle, airborne, hover_throttle, integral, steady_since = 1100.0, False, 1500.0, 0.0, None
    prev_nav = (-1, -1)
    mission_started = False

    ctrl_c_counter = [0]
    def on_ctrl_c(sig, frame):                   # on Ctrl+C just increase the counter
        ctrl_c_counter[0] += 1
    signal.signal(signal.SIGINT, on_ctrl_c)

    print("[5/6] Sensors warming up, MultiWii calibrating (a few seconds)...")
    while True:
        try:
            # --- Ctrl+C check
            if ctrl_c_counter[0] > ctrl_c:
                ctrl_c = ctrl_c_counter[0]
                if ctrl_c == 1:
                    print("\n\nCtrl+C: the drone is landing where it is. (Press Ctrl+C again to stop immediately)")
                    rc[7] = HIGH                         # AUX4 high = LAND
                    phase = "LANDING"
                else:
                    print("\n\nCtrl+C: stopping the motors immediately!")
                    break

            now = time.time()
            dt = max(now - prev_t, 1e-3)
            prev_t = now

            # --- Read the sensors from AirSim
            imu = client.getImuData()
            if loop_no % 3 == 0:                           # barometer: every 3rd loop is enough
                pressure = client.getBarometerData().pressure
            if phase == "TAKEOFF" or now - last_print > 0.5:  # true position: for takeoff and display
                kin = client.simGetGroundTruthKinematics()
                if z0 is None:
                    x0, y0, z0 = kin.position.x_val, kin.position.y_val, kin.position.z_val
                north = kin.position.x_val - x0            # metres, north +
                east = kin.position.y_val - y0             # metres, east +
                alt_f = -(kin.position.z_val - z0)         # metres, up +
                climb_rate = -kin.linear_velocity.z_val    # m/s, up +
            loop_no += 1

            # --- Exchange with MultiWii (sensors + RC -> motors + status)
            reply = msp.request(MSP_HIL_AIRSIM, hil_packet(imu, pressure, rc), timeout=0.1)
            if reply is None or len(reply) != 19:
                if reply is not None:
                    print("ERROR: The code on MultiWii does not match this script (bad HIL reply).")
                    break
                if now - last_reply > REPLY_TIMEOUT:
                    print("\nERROR: MultiWii is not answering! AirSim is landing with its own pilot...")
                    client.hoverAsync().join()
                    client.landAsync().join()
                    break
                continue
            last_reply = now
            m = struct.unpack("<4h5BiH", reply)
            motors, flags, gps_mode, nav, nav_error, step, mw_alt = m[0:4], m[4], m[5], m[6], m[7], m[8], m[9]
            armed = bool(flags & 1)

            # --- Apply the motor values to AirSim (1000-2000 -> 0.0-1.0)
            p = [clamp((x - 1000) / 1000.0, 0.0, 1.0) for x in motors]
            # MultiWii QUADX order: 0=rear right, 1=front right, 2=rear left, 3=front left
            # AirSim order        : front right, rear left, front left, rear right
            client.moveByMotorPWMsAsync(p[1], p[2], p[3], p[0], 0.5)

            # --- GPS (10 Hz)
            if now - last_gps >= GPS_INTERVAL_S:
                last_gps = now
                g = client.getGpsData().gnss
                speed = math.hypot(g.velocity.x_val, g.velocity.y_val)
                gps_packet = struct.pack("<BBiihH", 1 if g.fix_type >= 3 else 0, 10,
                                         int(round(g.geo_point.latitude * 1e7)),
                                         int(round(g.geo_point.longitude * 1e7)),
                                         int(g.geo_point.altitude), int(speed * 100))
                msp.request(MSP_SET_RAW_GPS, gps_packet, timeout=0.1)

            # --- Phases
            if phase == "PREPARE":
                ready = (not (flags & 32)) and (flags & 16) and (flags & 64)
                if ready and now - phase_time > 5.0:
                    print("[6/6] Motors starting (ARM)...")
                    rc[5] = HIGH                         # AUX2 high = ARM
                    phase, phase_time = "ARM", now
                elif now - phase_time > 30:
                    print("ERROR: MultiWii was not ready within 30 s (flags=%d)." % flags)
                    break

            elif phase == "ARM":
                if armed:
                    print("      Takeoff: climbing to %.1f m..." % min(TAKEOFF_ALTITUDE, targets[0][2]))
                    phase, phase_time = "TAKEOFF", now
                elif now - phase_time > 5:
                    print("ERROR: MultiWii did not ARM. Is the drone level on the ground?")
                    break

            elif phase == "TAKEOFF":
                target_alt = min(TAKEOFF_ALTITUDE, targets[0][2])
                if not airborne:
                    throttle += 150 * dt                 # raise the throttle slowly
                    if climb_rate > 0.1:                 # left the ground: this throttle ~ hover throttle
                        airborne, hover_throttle = True, throttle - 40
                else:
                    error = target_alt - alt_f
                    target_rate = clamp(0.8 * error, -0.5, 1.0)    # climb at most 1 m/s
                    rate_error = target_rate - climb_rate
                    integral = clamp(integral + 60 * rate_error * dt, -200, 200)
                    throttle = hover_throttle + 120 * rate_error + integral
                    if abs(error) < 0.3 and abs(climb_rate) < 0.3:
                        steady_since = steady_since or now
                        if now - steady_since > 1.5:
                            print("\n      Mission started! MultiWii has taken control.")
                            rc[6] = HIGH                 # AUX3 high = BARO + MISSION
                            phase, phase_time = "MISSION", now
                    else:
                        steady_since = None
                throttle = clamp(throttle, 1100, 1850)
                rc[3] = int(throttle)
                if now - phase_time > 30:
                    print("ERROR: Takeoff not completed within 30 s, landing.")
                    rc[7] = HIGH
                    phase = "LANDING"

            elif phase in ("MISSION", "LANDING"):
                if gps_mode != 0:
                    mission_started = True
                if nav == NAV_LANDED:
                    rc[3] = LOW                          # on the ground: cut throttle -> motors stop
                if nav_error in NAV_ERROR_TEXT and phase == "MISSION":
                    print("\nWARNING: %s -> landing where it is." % NAV_ERROR_TEXT[nav_error])
                    rc[7] = HIGH
                    phase = "LANDING"
                if (nav, step) != prev_nav:
                    prev_nav = (nav, step)
                    wait_start = now if nav == 4 else None
                    text = NAV_STATE_TEXT.get(nav, str(nav))
                    if nav in (4, 5) and 1 <= step <= len(targets):
                        text += " (target %d/%d)" % (step, len(targets))
                    if nav == 4:
                        text += " -> ARRIVED, waiting %d s" % WAIT_TIME_S
                    print("\n      >> %s" % text)
                if not armed and (mission_started or phase == "LANDING"):
                    if ctrl_c or phase == "LANDING":
                        print("\nThe drone has landed and the motors have stopped.")
                    else:
                        print("\nThe drone landed at home and the motors have stopped. Mission complete!")
                    break

            loop_count += 1
            if now - last_print > 0.5:
                loop_hz = loop_count / (now - last_print) if last_print else 0.0
                last_print, loop_count = now, 0
                extra = ""                               # distance to target / wait countdown
                if phase == "MISSION" and nav in (4, 5) and 1 <= step <= len(targets):
                    tn, te = targets[step - 1][0], targets[step - 1][1]
                    extra = "target %d: %5.1f m" % (step, math.hypot(tn - north, te - east))
                    if nav == 4 and wait_start is not None:
                        remaining = max(0, WAIT_TIME_S - (now - wait_start))
                        extra += "  wait %2.0f s" % remaining
                elif phase == "MISSION" and nav in (1, 2):
                    extra = "home: %5.1f m" % math.hypot(north, east)
                sys.stdout.write("\r [%-8s] N=%6.1f  E=%6.1f  Alt=%5.1f m (MW %5.1f)  %-30s %3.0f Hz "
                                 % (phase, north, east, alt_f, mw_alt / 100.0, extra, loop_hz))
                sys.stdout.flush()

        except Exception as error_message:               # AirSim closed, cable unplugged, etc.
            print("\n\nERROR: %s" % error_message)
            break

    # ------------------------------------------------------------- Shutdown
    rc = [MID, MID, MID, LOW, LOW, LOW, LOW, LOW]
    for _ in range(5):                                   # disarm MultiWii
        try:
            msp.request(MSP_HIL_AIRSIM, hil_packet(client.getImuData(), client.getBarometerData().pressure, rc),
                        timeout=0.1)
        except Exception:
            pass
    try:
        client.moveByMotorPWMsAsync(0, 0, 0, 0, 1.0)
        client.armDisarm(False)
        client.enableApiControl(False)
    except Exception:
        pass
    msp.close()
    print("Program finished.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCancelled.")