"""
test_tier0.py - Hardware-free checks for the protocol, logging and safety
code added in the "tier 0" pass.

Everything here runs without a hub or a controller. Anything that needs
real hardware (actual BLE behaviour, real sensor scaling) is out of scope
by definition - that is what port_scan.py is for.

    python test_tier0.py
"""

import csv
import os
import shutil
import sys
import tempfile
import types

import lwp3
import telemetry_log

_failures = []


def check(name, actual, expected):
    if actual == expected:
        print("  ok   {}".format(name))
    else:
        print("  FAIL {}\n         expected: {!r}\n         actual:   {!r}".format(
            name, expected, actual))
        _failures.append(name)


def check_true(name, value):
    check(name, bool(value), True)


def hexs(data):
    return bytes(data).hex(" ")


# ---------------------------------------------------------------------------
print("\nLWP3 message builders")
# ---------------------------------------------------------------------------
check("hub_property(battery, enable)",
      hexs(lwp3.hub_property(lwp3.PROP_BATTERY_VOLTAGE, lwp3.PROP_OP_ENABLE_UPDATES)),
      "05 00 01 06 02")

check("port_input_format_setup(0x37)",
      hexs(lwp3.port_input_format_setup(0x37)),
      "0a 00 41 37 00 01 00 00 00 01")

check("port_input_format_setup(mode 2, delta 5, no notify)",
      hexs(lwp3.port_input_format_setup(0x32, mode=2, delta_interval=5, notify=False)),
      "0a 00 41 32 02 05 00 00 00 00")

check("port_info_request(0x00)",
      hexs(lwp3.port_info_request(0x00)), "05 00 21 00 01")

check("port_mode_info_request(0x32, 2, VALUE_FORMAT)",
      hexs(lwp3.port_mode_info_request(0x32, 2, lwp3.MODE_INFO_VALUE_FORMAT)),
      "06 00 22 32 02 80")

# Declared length byte must match the real message length, or the hub
# silently ignores the packet.
for name, msg in (("hub_property", lwp3.hub_property(0x06, 0x02)),
                  ("input_format_setup", lwp3.port_input_format_setup(0x37)),
                  ("port_info_request", lwp3.port_info_request(0x12)),
                  ("mode_info_request", lwp3.port_mode_info_request(0x12, 0, 0))):
    check("{} length byte".format(name), msg[0], len(msg))

# ---------------------------------------------------------------------------
print("\nVersion decoding")
# ---------------------------------------------------------------------------
# LWP3 versions are a packed int32 (nibble major, nibble minor, BCD bugfix,
# BCD build), NOT four plain bytes.
check("decode_version 1.7.09.1915", lwp3.decode_version(bytes([0x15, 0x19, 0x09, 0x17])),
      "1.7.09.1915")
check("decode_version 1.0.00.0000", lwp3.decode_version(bytes([0x00, 0x00, 0x00, 0x10])),
      "1.0.00.0000")
check("decode_version short payload", lwp3.decode_version(b"\x01\x02"), "--")

# ---------------------------------------------------------------------------
print("\nPrimitive decoders")
# ---------------------------------------------------------------------------
check("int16_le positive", lwp3.int16_le(bytes([0x10, 0x27]), 0), 10000)
check("int16_le negative", lwp3.int16_le(bytes([0xF0, 0xD8]), 0), -10000)
check("int32_le negative", lwp3.int32_le(bytes([0xFF, 0xFF, 0xFF, 0xFF]), 0), -1)
check("float_le", round(lwp3.float_le(bytes([0x00, 0x00, 0xC8, 0x42]), 0), 3), 100.0)

check("decode_mode_values int16 x3",
      lwp3.decode_mode_values(bytes([0x01, 0x00, 0xFF, 0xFF, 0x64, 0x00]), 0x01, 3),
      [1, -1, 100])
check("decode_mode_values int8 signed",
      lwp3.decode_mode_values(bytes([0x9C, 0x64]), 0x00, 2), [-100, 100])
check("decode_mode_values truncated payload",
      lwp3.decode_mode_values(bytes([0x01, 0x00]), 0x01, 3), [1])

# ---------------------------------------------------------------------------
print("\nMessage parsers")
# ---------------------------------------------------------------------------
attached = lwp3.parse_hub_attached_io(
    bytearray([0x0F, 0x00, 0x04, 0x32, 0x01, 0x4B, 0x00,
               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
check("attached io port", attached["port"], 0x32)
check("attached io type", lwp3.io_type_name(attached["io_type"]),
      "TechnicLargeAngularMotorGrey")
check("unknown io type renders hex", lwp3.io_type_name(0x0999), "Unknown(0x0999)")

detached = lwp3.parse_hub_attached_io(bytearray([0x05, 0x00, 0x04, 0x32, 0x00]))
check("detached io has no type", "io_type" in detached, False)

info = lwp3.parse_port_information(
    bytearray([0x0B, 0x00, 0x43, 0x32, 0x01, 0x0F, 0x05, 0x1F, 0x00, 0x01, 0x00]))
check("port info mode_count", info["mode_count"], 5)
check("port info input_modes", info["input_modes"], 0x001F)
check("port info output_modes", info["output_modes"], 0x0001)
check("port info caps", (info["input"], info["output"], info["combinable"],
                         info["synchronizable"]), (True, True, True, True))

name_msg = bytearray([0x11, 0x00, 0x44, 0x32, 0x02, 0x00]) + b"POS\x00\x00\x00\x00\x00\x00\x00\x00"
check("mode name", lwp3.parse_port_mode_information(name_msg)["name"], "POS")

fmt_msg = bytearray([0x0A, 0x00, 0x44, 0x32, 0x02, 0x80, 0x01, 0x02, 0x04, 0x00])
fmt = lwp3.parse_port_mode_information(fmt_msg)
check("mode value_count", fmt["value_count"], 1)
check("mode data type", fmt["data_type_name"], "int32")

raw_msg = bytearray([0x0E, 0x00, 0x44, 0x32, 0x02, 0x01,
                     0x00, 0x00, 0xC8, 0xC2, 0x00, 0x00, 0xC8, 0x42])
rng = lwp3.parse_port_mode_information(raw_msg)
check("mode raw range", (rng["min"], rng["max"]), (-100.0, 100.0))

check("wrong message type returns None",
      lwp3.parse_port_information(bytearray([0x05, 0x00, 0x45, 0x00, 0x00])), None)
check("truncated message returns None",
      lwp3.parse_port_mode_information(bytearray([0x03, 0x00, 0x44])), None)
check("generic error", lwp3.parse_generic_error(bytearray([0x05, 0x00, 0x05, 0x21, 0x05])),
      {"command": 0x21, "code": 0x05})

# ---------------------------------------------------------------------------
print("\nTelemetry logger")
# ---------------------------------------------------------------------------
tmpdir = tempfile.mkdtemp()
try:
    logger = telemetry_log.TelemetryLogger(directory=tmpdir, prefix="t", flush_interval=0)
    check_true("logger enabled", logger.enabled)

    row = {"t_mono": 0.05, "drive_cmd": 42, "steer_cmd": -13, "ignored_key": "x"}
    telemetry_log.flatten_xyz("accel", (1, -2, 3), row)
    telemetry_log.flatten_xyz("gyro", None, row)
    logger.log(row)
    logger.log({"t_mono": 0.10, "drive_cmd": 0, "steer_cmd": 0})
    check("logger counts rows", logger.total_rows, 2)
    logger.close()

    with open(logger.path, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    check("csv row count", len(rows), 2)
    check("csv header order", list(rows[0].keys()), list(telemetry_log.FIELDS))
    check("csv drive_cmd", rows[0]["drive_cmd"], "42")
    check("csv accel flattened", (rows[0]["accel_x"], rows[0]["accel_y"], rows[0]["accel_z"]),
          ("1", "-2", "3"))
    check("csv missing gyro is blank", rows[0]["gyro_x"], "")
    check("csv drops unknown keys", "ignored_key" in rows[0], False)

    # A logger that cannot open its file must degrade to a no-op rather
    # than raising into the control loop.
    blocker = os.path.join(tmpdir, "blocker")
    with open(blocker, "w", encoding="utf-8") as fh:
        fh.write("this is a file, so it cannot also be a directory")
    broken = telemetry_log.TelemetryLogger(
        directory=os.path.join(blocker, "logs"), prefix="t")
    check("broken logger disabled", broken.enabled, False)
    broken.log({"t_mono": 1.0})
    broken.close()
    print("  ok   broken logger swallows writes")
finally:
    shutil.rmtree(tmpdir, ignore_errors=True)

# ---------------------------------------------------------------------------
print("\nCommand encoding and send safety")
# ---------------------------------------------------------------------------
# pygame is imported by hub_service; skip this section rather than fail if
# it is not installed.
try:
    import hub_service
except ImportError as e:
    print("  SKIP hub_service ({})".format(e))
    hub_service = None

if hub_service is not None:
    class FakeHub:
        def __init__(self, connected=True):
            self._client = types.SimpleNamespace(is_connected=connected)
            self.sent = []

        def send_raw_command(self, *args):
            self.sent.append(args)
            return True

    hub = FakeHub()
    hub_service.send_drive_command(hub, 50, -50, hub_service.LIGHTS_ON)
    packet = hub.sent[-1]
    check("drive packet length", len(packet), 13)
    check("drive packet header", packet[:4], (0x0D, 0x00, 0x81, 0x36))
    check("speed byte", packet[9], 50)
    check("steer byte (two's complement)", packet[10], 0xCE)   # -50
    check("lights byte", packet[11], hub_service.LIGHTS_ON)

    # Out-of-range values must clamp, not wrap into a nonsense byte.
    hub_service.send_drive_command(hub, 250, -250, hub_service.LIGHTS_OFF)
    packet = hub.sent[-1]
    check("speed clamps to +100", packet[9], 100)
    check("steer clamps to -100", packet[10], 0x9C)

    check("lights on, not braking",
          hub_service.compute_lights_byte(True, False), hub_service.LIGHTS_ON)
    check("lights on, braking",
          hub_service.compute_lights_byte(True, True), hub_service.LIGHTS_ON_BRAKING)
    check("lights off, braking",
          hub_service.compute_lights_byte(False, True), hub_service.LIGHTS_OFF_BRAKING)

    check("deadzone suppresses drift", hub_service.apply_deadzone(0.1), 0.0)
    check("deadzone passes real input", hub_service.apply_deadzone(0.9), 0.9)

    # safe_send must contain every failure mode. The library calls
    # sys.exit(1) from its BLE thread on disconnect, which is a
    # BaseException - if that escaped, a dropout would kill the control
    # loop and leave the car driving.
    class ExitingHub(FakeHub):
        def send_raw_command(self, *args):
            raise SystemExit(1)

    class ThrowingHub(FakeHub):
        def send_raw_command(self, *args):
            raise RuntimeError("ble write failed")

    hub_service._hub_alive.set()
    check("safe_send survives SystemExit",
          hub_service.safe_send(ExitingHub(), 0, 0, 0), False)
    check("SystemExit disarms watchdog", hub_service._hub_alive.is_set(), False)

    hub_service._hub_alive.set()
    check("safe_send survives exceptions",
          hub_service.safe_send(ThrowingHub(), 0, 0, 0), False)

    check("safe_send rejects None hub", hub_service.safe_send(None, 0, 0, 0), False)
    check("safe_send rejects disconnected hub",
          hub_service.safe_send(FakeHub(connected=False), 0, 0, 0), False)

    ok_hub = FakeHub()
    check("safe_send succeeds when connected",
          hub_service.safe_send(ok_hub, 10, 20, 0), True)
    check("safe_send actually sent", len(ok_hub.sent), 1)

    check("watchdog timeout exceeds loop period",
          hub_service.WATCHDOG_TIMEOUT > hub_service.CONTROL_PERIOD, True)

# ---------------------------------------------------------------------------
print()
if _failures:
    print("{} FAILED: {}".format(len(_failures), ", ".join(_failures)))
    sys.exit(1)
print("all checks passed")
