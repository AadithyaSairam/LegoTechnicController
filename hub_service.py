"""
hub_service.py - Backend service for the LEGO Technic Porsche GT4 e-Performance (42176)

Handles everything NOT related to the UI:
  - BLE connection + pairing to the Technic Move Hub
  - Steering calibration
  - Combined drive/steering/lights command (per DanieleBenedettelli/TechnicMoveHub)
  - Telemetry (battery, firmware/hardware version) via raw LWP3 notifications
  - Onboard sensors (temperature, accelerometer, gyro, tilt) via raw LWP3 notifications
  - Xbox/PlayStation controller reading (pygame)
  - A fixed-rate control loop on a background thread
  - A watchdog that stops the motors if that loop stalls or dies
  - Per-tick CSV telemetry logging

Exposes a single shared `state` dict (thread-safe via get_state/set_state)
that any UI can poll and display. No tkinter/UI code lives in this file.

Sensor values in `state` are raw numbers (or None when unknown), not
display strings - formatting is the UI's job, and analysis code needs the
numbers. See telemetry_log.py for the on-disk schema.
"""

import threading
import time

import pygame
from technicmovehub import technicmovehub  # class name is lowercase

import lwp3
import telemetry_log

# ---------------------------------------------------------------------------
# CONFIG
# ---------------------------------------------------------------------------
DEADZONE = 0.15
MAX_VALUE = 100
LED_COLOR_ON_CONNECT = "blue"

CONTROL_HZ = 20.0
CONTROL_PERIOD = 1.0 / CONTROL_HZ

# If the control loop has not produced a command within this long, the
# watchdog assumes it is wedged or dead and stops the motors itself.
# Must comfortably exceed CONTROL_PERIOD plus worst-case BLE write latency.
WATCHDOG_TIMEOUT = 0.30
WATCHDOG_PERIOD = 0.05

CONTROLLER_RETRY_PERIOD = 2.0
LOG_ENABLED = True

COMBINED_PORT = 0x36  # virtual port: drive + steering + lights in one packet

# Onboard sensor ports confirmed on the Technic Move Hub (88019), per
# github.com/orgs/pybricks/discussions/1733. Exact payload scaling below
# follows standard LWP3 conventions used on other LEGO hubs (Technic Hub,
# SPIKE) - NOT independently verified against this specific hub, so treat
# displayed units as approximate. Run `python port_scan.py` to have the hub
# report its own modes, units and value formats and settle this.
SENSOR_PORT_TEMPERATURE = 0x37
SENSOR_PORT_ACCELEROMETER = 0x38
SENSOR_PORT_GYRO = 0x39
SENSOR_PORT_TILT = 0x3A

# Lights byte values (from DanieleBenedettelli/TechnicMoveHub protocol docs)
LIGHTS_ON = 0x00              # front + back on
LIGHTS_ON_BRAKING = 0x01      # front + back on, braking (brighter rear)
LIGHTS_OFF = 0x04             # front + back off
LIGHTS_OFF_BRAKING = 0x05     # front off, back on braking

# ---------------------------------------------------------------------------
# SHARED STATE (thread-safe) - the UI reads this, the service writes to it
# ---------------------------------------------------------------------------
_state = {
    "connected": False,
    "battery_pct": None,
    "fw_version": "--",
    "hw_version": "--",
    "drive_value": 0,
    "steer_value": 0,
    "headlights_on": True,
    "braking": False,
    "failsafe": False,
    "status": "Starting...",
    "axes_debug": "",
    "temperature_c": None,
    "accel": None,          # (x, y, z) or None
    "gyro": None,
    "tilt": None,
    "loop_hz": 0.0,
    "loop_jitter_ms": 0.0,
    "loop_overruns": 0,
    "send_ms": 0.0,
    "log_path": "",
    "log_rows": 0,
}
_state_lock = threading.Lock()

# Serialises BLE writes between the control loop and the watchdog thread.
_send_lock = threading.Lock()

# Set once the hub is connected and calibrated, i.e. once it is capable of
# moving and therefore needs watching. Cleared on any send failure.
_hub_alive = threading.Event()

# Timestamp (perf_counter) of the last command issued by the CONTROL LOOP.
# Deliberately not updated by the watchdog's own stop commands, so a dead
# loop keeps looking dead.
_last_control_ts = 0.0

_stop_event = threading.Event()
_stopped_event = threading.Event()


def set_state(**kwargs):
    with _state_lock:
        _state.update(kwargs)


def get_state():
    with _state_lock:
        return dict(_state)


def request_stop(timeout=3.0):
    """Ask the service to shut down and wait for it to stop the motors.

    Called from the UI thread when the window closes. Without this the
    daemon thread is killed mid-drive and the hub happily keeps executing
    the last command it received.
    """
    _stop_event.set()
    _stopped_event.wait(timeout)


# ---------------------------------------------------------------------------
# COMBINED DRIVE + STEERING + LIGHTS COMMAND
# ---------------------------------------------------------------------------
def send_drive_command(hub, speed, steering_angle, lights_byte):
    """Sends the combined drive/steer/lights packet documented at
    github.com/DanieleBenedettelli/TechnicMoveHub
    Format: 0x0d,0x00,0x81,0x36,0x11,0x51,0x00,0x03,0x00,speed,steering,lights,0x00
    """
    speed_b = max(-100, min(100, int(speed))) & 0xFF
    steer_b = max(-100, min(100, int(steering_angle))) & 0xFF
    hub.send_raw_command(
        0x0d, 0x00, 0x81, COMBINED_PORT, 0x11, 0x51, 0x00, 0x03, 0x00,
        speed_b, steer_b, lights_byte, 0x00
    )


def safe_send(hub, speed, steering_angle, lights_byte):
    """send_drive_command with connection checking and full exception
    containment. Returns True if the packet went out.

    The technicmovehub library calls sys.exit(1) from inside its BLE
    thread when the hub is not connected, which surfaces here as
    SystemExit - a BaseException that `except Exception` would miss. A
    dropped BLE link must not be able to tear down the control loop, so
    this catches BaseException (re-raising only KeyboardInterrupt).
    """
    if hub is None:
        return False
    client = getattr(hub, "_client", None)
    if client is None or not client.is_connected:
        _hub_alive.clear()
        set_state(connected=False, status="Hub disconnected.")
        return False

    with _send_lock:
        started = time.perf_counter()
        try:
            send_drive_command(hub, speed, steering_angle, lights_byte)
        except KeyboardInterrupt:
            raise
        except BaseException as e:
            _hub_alive.clear()
            set_state(connected=False, status="Send failed: {!r}".format(e))
            return False
        set_state(send_ms=round((time.perf_counter() - started) * 1000, 1))
        return True


def calibrate_steering(hub):
    """Sends the confirmed two-step steering-calibration sequence from
    github.com/DanieleBenedettelli/TechnicMoveHub. The hub will NOT
    respond to drive commands until BOTH of these have been sent
    after connecting.
    """
    hub.send_raw_command(
        0x0d, 0x00, 0x81, COMBINED_PORT, 0x11, 0x51, 0x00, 0x03, 0x00,
        0x00, 0x00, 0x10, 0x00
    )
    time.sleep(0.3)
    hub.send_raw_command(
        0x0d, 0x00, 0x81, COMBINED_PORT, 0x11, 0x51, 0x00, 0x03, 0x00,
        0x00, 0x00, 0x08, 0x00
    )
    time.sleep(0.3)


def compute_lights_byte(headlights_on, braking):
    if braking:
        return LIGHTS_ON_BRAKING if headlights_on else LIGHTS_OFF_BRAKING
    return LIGHTS_ON if headlights_on else LIGHTS_OFF


# ---------------------------------------------------------------------------
# TELEMETRY (raw LWP3 notifications via bleak, through hub's internal client)
# ---------------------------------------------------------------------------
def parse_hub_property_notification(data):
    if len(data) < 5 or data[2] != lwp3.MSG_HUB_PROPERTIES:
        return
    property_id = data[3]
    payload = data[5:]

    if property_id == lwp3.PROP_BATTERY_VOLTAGE and len(payload) >= 1:
        set_state(battery_pct=payload[0])
    elif property_id == lwp3.PROP_FW_VERSION and len(payload) >= 4:
        set_state(fw_version=lwp3.decode_version(payload))
    elif property_id == lwp3.PROP_HW_VERSION and len(payload) >= 4:
        set_state(hw_version=lwp3.decode_version(payload))


def parse_sensor_notification(data):
    """Parses LWP3 'Port Value Single' (0x45) messages for the onboard
    temperature/accelerometer/gyro/tilt sensors. Payload layouts follow
    standard LWP3 conventions (int16 little-endian per axis) - run
    port_scan.py to confirm the real value formats for this hub.
    """
    if len(data) < 4 or data[2] != lwp3.MSG_PORT_VALUE_SINGLE:
        return
    port_id = data[3]
    payload = data[4:]

    if port_id == SENSOR_PORT_TEMPERATURE and len(payload) >= 2:
        set_state(temperature_c=lwp3.int16_le(payload, 0) / 10.0)
    elif len(payload) >= 6:
        xyz = tuple(lwp3.int16_le(payload, i) for i in (0, 2, 4))
        if port_id == SENSOR_PORT_ACCELEROMETER:
            set_state(accel=xyz)
        elif port_id == SENSOR_PORT_GYRO:
            set_state(gyro=xyz)
        elif port_id == SENSOR_PORT_TILT:
            set_state(tilt=xyz)


def setup_telemetry(hub):
    client = hub._client

    async def _notification_handler(_sender, data):
        data = bytearray(data)
        parse_hub_property_notification(data)
        parse_sensor_notification(data)

    async def _setup():
        await client.start_notify(lwp3.CHAR_UUID, _notification_handler)
        for prop in (lwp3.PROP_FW_VERSION, lwp3.PROP_HW_VERSION):
            await client.write_gatt_char(
                lwp3.CHAR_UUID, lwp3.hub_property(prop, lwp3.PROP_OP_REQUEST_UPDATE))
        await client.write_gatt_char(
            lwp3.CHAR_UUID,
            lwp3.hub_property(lwp3.PROP_BATTERY_VOLTAGE, lwp3.PROP_OP_ENABLE_UPDATES))
        # Subscribe to onboard sensors (temperature, accelerometer, gyro, tilt)
        for port in (SENSOR_PORT_TEMPERATURE, SENSOR_PORT_ACCELEROMETER,
                     SENSOR_PORT_GYRO, SENSOR_PORT_TILT):
            await client.write_gatt_char(
                lwp3.CHAR_UUID, lwp3.port_input_format_setup(port))

    hub._run_async_in_thread(_setup())


# ---------------------------------------------------------------------------
# CONTROLLER
# ---------------------------------------------------------------------------
def init_controller():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No controller detected. Plug in / pair your Xbox or PS controller first.")
    joy = pygame.joystick.Joystick(0)
    joy.init()
    return joy


def apply_deadzone(value, deadzone=DEADZONE):
    return 0.0 if abs(value) < deadzone else value


def read_triggers(joy):
    """Handles both common Windows/XInput trigger mappings:
    - separate axes 4 (LT) and 5 (RT), or
    - a single combined axis 2 (-1 = LT full, +1 = RT full)
    """
    num_axes = joy.get_numaxes()
    if num_axes >= 6:
        rt = (joy.get_axis(5) + 1) / 2
        lt = (joy.get_axis(4) + 1) / 2
    elif num_axes >= 3:
        combined = joy.get_axis(2)
        rt = max(combined, 0)
        lt = max(-combined, 0)
    else:
        rt = 0.0
        lt = 0.0
    return rt, lt


def read_controller(joy):
    """One snapshot of the controller, or None if it has gone away."""
    if joy is None or pygame.joystick.get_count() == 0:
        return None
    try:
        pygame.event.pump()
        throttle_raw, brake_raw = read_triggers(joy)
        return {
            "steer_raw": apply_deadzone(joy.get_axis(0)),
            "throttle_raw": throttle_raw,
            "brake_raw": brake_raw,
            "stop_pressed": bool(joy.get_button(1)),   # B / Circle
            "lights_button": bool(joy.get_button(3)),  # Y / Triangle
            "axes": [round(joy.get_axis(i), 2) for i in range(joy.get_numaxes())],
        }
    except pygame.error:
        return None


# ---------------------------------------------------------------------------
# WATCHDOG
# ---------------------------------------------------------------------------
def _watchdog_loop(hub):
    """Independent safety net: if the control loop stops producing
    commands while the hub is still connected and able to move, stop the
    motors. Runs until the service shuts down.
    """
    while not _stop_event.wait(WATCHDOG_PERIOD):
        if not _hub_alive.is_set():
            continue
        age = time.perf_counter() - _last_control_ts
        if age <= WATCHDOG_TIMEOUT:
            continue
        set_state(failsafe=True,
                  status="FAILSAFE: no command for {:.0f} ms - motors stopped.".format(age * 1000))
        safe_send(hub, 0, 0, LIGHTS_OFF_BRAKING)


# ---------------------------------------------------------------------------
# MAIN SERVICE LOOP (runs on a background thread)
# ---------------------------------------------------------------------------
def run():
    """Connects to the hub, calibrates, sets up telemetry, and runs the
    controller -> hub command loop at a fixed CONTROL_HZ until
    request_stop() is called. Intended to be run in its own thread:

        threading.Thread(target=hub_service.run, daemon=True).start()
    """
    global _last_control_ts

    _stop_event.clear()
    _stopped_event.clear()
    _hub_alive.clear()

    hub = None
    logger = None
    watchdog = None

    try:
        set_state(status="Connecting to Technic Move Hub...")
        hub = technicmovehub()
        if hub.connect(timeout=60.0) is None:
            set_state(status="Could not find/connect to hub. Check it's blinking and app is closed.")
            return

        set_state(status="Pairing (required by Windows for secured writes)...")
        try:
            hub._run_async_in_thread(hub._client.pair())
        except Exception as e:
            set_state(status="Pairing warning: {}".format(e))

        hub.led(LED_COLOR_ON_CONNECT)
        set_state(status="Calibrating steering...")
        calibrate_steering(hub)
        setup_telemetry(hub)

        if LOG_ENABLED:
            logger = telemetry_log.TelemetryLogger()
            set_state(log_path=logger.path or "(logging disabled: {})".format(
                getattr(logger, "error", "unknown error")))

        try:
            joy = init_controller()
            set_state(status="Connected. Driving enabled.")
        except Exception as e:
            joy = None
            set_state(status="Controller error: {} (retrying)".format(e))

        set_state(connected=True)

        # Arm the watchdog only now: the car can move from this point on.
        _last_control_ts = time.perf_counter()
        _hub_alive.set()
        watchdog = threading.Thread(target=_watchdog_loop, args=(hub,), daemon=True)
        watchdog.start()

        _control_loop(hub, joy, logger)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        set_state(status="Service error: {!r}".format(e))
    finally:
        _hub_alive.clear()
        _stop_event.set()
        if watchdog is not None:
            watchdog.join(timeout=1.0)
        if hub is not None:
            try:
                safe_send(hub, 0, 0, LIGHTS_OFF)
                hub.led("off")
                hub.disconnect()
            except KeyboardInterrupt:
                raise
            except BaseException:
                pass
        if logger is not None:
            logger.close()
        try:
            pygame.quit()
        except Exception:
            pass
        set_state(status="Disconnected.", connected=False, drive_value=0, steer_value=0)
        _stopped_event.set()


def _control_loop(hub, joy, logger):
    """Fixed-rate loop. Timing is deadline-based rather than
    `sleep(period)` so the period does not drift by however long the BLE
    write and sensor parsing happened to take, and so dt is measured
    rather than assumed - every derivative or integral term added later
    depends on that being true.
    """
    global _last_control_ts

    headlights_on = True
    lights_button_prev = False
    next_controller_retry = 0.0

    t_start = time.perf_counter()
    last_tick = t_start
    next_deadline = t_start
    loop_hz_ema = CONTROL_HZ
    jitter_ema = 0.0
    overruns = 0
    tick = 0
    last_debug_push = 0.0

    while not _stop_event.is_set():
        now = time.perf_counter()
        dt = now - last_tick
        last_tick = now
        # The first tick has no previous tick to measure against, so its
        # dt is ~0 and 1/dt is meaningless. Feeding that to the EMA
        # poisons the reported rate for hundreds of iterations.
        dt_valid = tick > 0 and dt > 0
        tick += 1

        # --- controller (hot-plug tolerant) ---
        reading = read_controller(joy)
        if reading is None:
            if joy is not None:
                joy = None
                set_state(status="Controller lost - motors stopped, retrying...")
            if now >= next_controller_retry:
                next_controller_retry = now + CONTROLLER_RETRY_PERIOD
                try:
                    pygame.joystick.quit()
                    pygame.joystick.init()
                    joy = init_controller()
                    set_state(status="Controller reconnected. Driving enabled.")
                except Exception:
                    joy = None

        # --- mix commands ---
        if reading is None:
            drive_value = steer_value = 0
            braking = False
            throttle_raw = brake_raw = steer_raw = 0.0
        else:
            throttle_raw = reading["throttle_raw"]
            brake_raw = reading["brake_raw"]
            steer_raw = reading["steer_raw"]
            drive_value = int((throttle_raw - brake_raw) * MAX_VALUE)
            steer_value = int(steer_raw * MAX_VALUE)
            braking = brake_raw > 0.5

            if reading["lights_button"] and not lights_button_prev:
                headlights_on = not headlights_on
            lights_button_prev = reading["lights_button"]

            if reading["stop_pressed"]:
                drive_value = 0
                steer_value = 0

            if now - last_debug_push >= 0.5:
                last_debug_push = now
                set_state(axes_debug=str(reading["axes"]))

        lights_byte = compute_lights_byte(headlights_on, braking)
        sent = safe_send(hub, drive_value, steer_value, lights_byte)
        if not sent:
            # Hub is gone; stop driving the loop's clock forward and let
            # the outer handler tear things down.
            set_state(drive_value=0, steer_value=0)
            break
        _last_control_ts = time.perf_counter()

        # --- loop health ---
        if dt_valid:
            loop_hz_ema = 0.9 * loop_hz_ema + 0.1 * (1.0 / dt)
            jitter_ema = 0.9 * jitter_ema + 0.1 * abs(dt - CONTROL_PERIOD) * 1000.0

        was_failsafe = get_state()["failsafe"]
        set_state(drive_value=drive_value, steer_value=steer_value,
                  headlights_on=headlights_on, braking=braking,
                  loop_hz=round(loop_hz_ema, 1),
                  loop_jitter_ms=round(jitter_ema, 1),
                  loop_overruns=overruns,
                  failsafe=False)
        if was_failsafe:
            set_state(status="Recovered from failsafe. Driving enabled.")

        # --- log ---
        if logger is not None and logger.enabled:
            snapshot = get_state()
            row = {
                "t_wall": round(time.time(), 3),
                "t_mono": round(now - t_start, 4),
                "dt": round(dt, 5) if dt_valid else "",
                "loop_hz": round(1.0 / dt, 2) if dt_valid else "",
                "send_ms": snapshot["send_ms"],
                "drive_cmd": drive_value,
                "steer_cmd": steer_value,
                "throttle_raw": round(throttle_raw, 4),
                "brake_raw": round(brake_raw, 4),
                "steer_raw": round(steer_raw, 4),
                "headlights": int(headlights_on),
                "braking": int(braking),
                "failsafe": int(was_failsafe),
                "battery_pct": snapshot["battery_pct"],
                "temp_c": snapshot["temperature_c"],
            }
            telemetry_log.flatten_xyz("accel", snapshot["accel"], row)
            telemetry_log.flatten_xyz("gyro", snapshot["gyro"], row)
            telemetry_log.flatten_xyz("tilt", snapshot["tilt"], row)
            logger.log(row)
            set_state(log_rows=logger.total_rows)

        # --- hold the rate ---
        next_deadline += CONTROL_PERIOD
        slack = next_deadline - time.perf_counter()
        if slack > 0:
            _stop_event.wait(slack)
        else:
            # Overran the budget (usually a slow BLE write). Resync the
            # deadline instead of trying to catch up with a burst.
            overruns += 1
            next_deadline = time.perf_counter()
