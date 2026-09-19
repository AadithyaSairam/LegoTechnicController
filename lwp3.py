"""
lwp3.py - LEGO Wireless Protocol v3 message builders and parsers.

Shared by hub_service.py (runtime telemetry) and port_scan.py (the port /
mode discovery tool) so the wire format is defined in exactly one place.

References:
  https://lego.github.io/lego-ble-wireless-protocol-docs/
  https://github.com/DanieleBenedettelli/TechnicMoveHub  (Move Hub specifics)

Every LWP3 message is: length, hub_id (0x00), message_type, payload...
Only single-byte lengths are handled here; that covers every message the
Technic Move Hub sends or accepts.
"""

import struct

CHAR_UUID = "00001624-1212-EFDE-1623-785FEABCD123"

# ---------------------------------------------------------------------------
# MESSAGE TYPES (byte 2)
# ---------------------------------------------------------------------------
MSG_HUB_PROPERTIES = 0x01
MSG_HUB_ACTIONS = 0x02
MSG_HUB_ATTACHED_IO = 0x04
MSG_GENERIC_ERROR = 0x05
MSG_PORT_INFO_REQUEST = 0x21
MSG_PORT_MODE_INFO_REQUEST = 0x22
MSG_PORT_INPUT_FORMAT_SETUP = 0x41
MSG_PORT_INFORMATION = 0x43
MSG_PORT_MODE_INFORMATION = 0x44
MSG_PORT_VALUE_SINGLE = 0x45
MSG_PORT_INPUT_FORMAT_SINGLE = 0x47
MSG_PORT_OUTPUT_COMMAND = 0x81
MSG_PORT_OUTPUT_FEEDBACK = 0x82

MSG_TYPE_NAMES = {
    MSG_HUB_PROPERTIES: "HubProperties",
    MSG_HUB_ACTIONS: "HubActions",
    MSG_HUB_ATTACHED_IO: "HubAttachedIO",
    MSG_GENERIC_ERROR: "GenericError",
    MSG_PORT_INFORMATION: "PortInformation",
    MSG_PORT_MODE_INFORMATION: "PortModeInformation",
    MSG_PORT_VALUE_SINGLE: "PortValueSingle",
    MSG_PORT_INPUT_FORMAT_SINGLE: "PortInputFormatSingle",
    MSG_PORT_OUTPUT_FEEDBACK: "PortOutputFeedback",
}

# ---------------------------------------------------------------------------
# HUB PROPERTIES
# ---------------------------------------------------------------------------
PROP_FW_VERSION = 0x03
PROP_HW_VERSION = 0x04
PROP_BATTERY_VOLTAGE = 0x06

PROP_OP_ENABLE_UPDATES = 0x02
PROP_OP_DISABLE_UPDATES = 0x03
PROP_OP_REQUEST_UPDATE = 0x05
PROP_OP_UPDATE = 0x06

# ---------------------------------------------------------------------------
# PORT INFORMATION / MODE INFORMATION request sub-types
# ---------------------------------------------------------------------------
PORT_INFO_VALUE = 0x00
PORT_INFO_MODE = 0x01
PORT_INFO_COMBOS = 0x02

MODE_INFO_NAME = 0x00
MODE_INFO_RAW = 0x01
MODE_INFO_PCT = 0x02
MODE_INFO_SI = 0x03
MODE_INFO_SYMBOL = 0x04
MODE_INFO_MAPPING = 0x05
MODE_INFO_VALUE_FORMAT = 0x80

DATA_TYPE_NAMES = {0x00: "int8", 0x01: "int16", 0x02: "int32", 0x03: "float"}
DATA_TYPE_SIZES = {0x00: 1, 0x01: 2, 0x02: 4, 0x03: 4}

# Device type IDs reported by HubAttachedIO. Unknown IDs render as hex via
# io_type_name(), which is the interesting case on this hub - anything not
# listed here is worth investigating.
IO_TYPE_NAMES = {
    0x0001: "Motor",
    0x0002: "SystemTrainMotor",
    0x0005: "Button",
    0x0008: "LEDLight",
    0x0014: "Voltage",
    0x0015: "Current",
    0x0016: "PiezoTone",
    0x0017: "RGBLight",
    0x0022: "ExternalTiltSensor",
    0x0023: "MotionSensor",
    0x0024: "VisionSensor",
    0x0025: "ExternalMotorWithTacho",
    0x0026: "InternalMotorWithTacho",
    0x0027: "InternalTilt",
    0x002E: "TechnicLargeLinearMotor",
    0x002F: "TechnicXLLinearMotor",
    0x0036: "TechnicHubGestSensor",
    0x0037: "RemoteControlButton",
    0x0038: "RemoteControlRSSI",
    0x0039: "TechnicHubAccelerometer",
    0x003A: "TechnicHubGyroSensor",
    0x003B: "TechnicHubTiltSensor",
    0x003C: "TechnicHubTemperatureSensor",
    0x0040: "TechnicColorSensor",
    0x0041: "TechnicDistanceSensor",
    0x0042: "TechnicForceSensor",
    0x0043: "Technic3x3ColorLightMatrix",
    0x0046: "TechnicMediumAngularMotor",
    0x0047: "TechnicLargeAngularMotor",
    0x004B: "TechnicLargeAngularMotorGrey",
    0x004C: "TechnicXLAngularMotorGrey",
}

IO_EVENT_DETACHED = 0x00
IO_EVENT_ATTACHED = 0x01
IO_EVENT_ATTACHED_VIRTUAL = 0x02


def io_type_name(type_id):
    return IO_TYPE_NAMES.get(type_id, "Unknown(0x{:04X})".format(type_id))


# ---------------------------------------------------------------------------
# MESSAGE BUILDERS
# ---------------------------------------------------------------------------
def hub_property(prop_id, operation):
    """Request or subscribe to a hub property (battery, versions, ...)."""
    return bytearray([0x05, 0x00, MSG_HUB_PROPERTIES, prop_id, operation])


def port_input_format_setup(port_id, mode=0x00, delta_interval=1, notify=True):
    """Subscribe to continuous value updates from a sensor port.

    Format: len, hub_id, 0x41, port, mode, delta (4B LE), notify (1B)
    """
    return bytearray(
        bytes([0x0A, 0x00, MSG_PORT_INPUT_FORMAT_SETUP, port_id, mode])
        + int(delta_interval).to_bytes(4, "little")
        + bytes([0x01 if notify else 0x00])
    )


def port_info_request(port_id, info_type=PORT_INFO_MODE):
    """Ask a port to describe itself. Attached ports reply with 0x43;
    empty ports reply with a GenericError (0x05) or stay silent."""
    return bytearray([0x05, 0x00, MSG_PORT_INFO_REQUEST, port_id, info_type])


def port_mode_info_request(port_id, mode, info_type):
    """Ask for one attribute (name, unit, value format, ...) of one mode."""
    return bytearray([0x06, 0x00, MSG_PORT_MODE_INFO_REQUEST, port_id, mode, info_type])


# ---------------------------------------------------------------------------
# PRIMITIVE DECODERS
# ---------------------------------------------------------------------------
def int16_le(data, offset):
    value = data[offset] | (data[offset + 1] << 8)
    return value - 65536 if value >= 32768 else value


def int32_le(data, offset):
    value = int.from_bytes(bytes(data[offset:offset + 4]), "little")
    return value - (1 << 32) if value >= (1 << 31) else value


def float_le(data, offset):
    return struct.unpack_from("<f", bytes(data[offset:offset + 4]))[0]


def _bcd(value, digits):
    """Binary-coded decimal: each nibble is one decimal digit."""
    return "".join(str((value >> shift) & 0xF)
                   for shift in range(4 * (digits - 1), -1, -4))


def decode_version(payload):
    """Decode a 4-byte LWP3 version into 'major.minor.bugfix.build'.

    The encoding is not four plain bytes: it is a little-endian int32 whose
    top nibble is the major version, the next nibble the minor, then two BCD
    digits of bugfix and four BCD digits of build.
    """
    if len(payload) < 4:
        return "--"
    v = int.from_bytes(bytes(payload[:4]), "little")
    major = (v >> 28) & 0x07
    minor = (v >> 24) & 0x0F
    return "{}.{}.{}.{}".format(major, minor, _bcd((v >> 16) & 0xFF, 2),
                                _bcd(v & 0xFFFF, 4))


# ---------------------------------------------------------------------------
# MESSAGE PARSERS - each returns a dict, or None if the message is not of
# that type / is too short to trust.
# ---------------------------------------------------------------------------
def parse_hub_attached_io(data):
    if len(data) < 5 or data[2] != MSG_HUB_ATTACHED_IO:
        return None
    event = data[4]
    out = {"port": data[3], "event": event}
    if event in (IO_EVENT_ATTACHED, IO_EVENT_ATTACHED_VIRTUAL) and len(data) >= 7:
        out["io_type"] = data[5] | (data[6] << 8)
    return out


def parse_port_information(data):
    """0x43 with info_type 0x01 describes a port's capabilities and modes."""
    if len(data) < 5 or data[2] != MSG_PORT_INFORMATION:
        return None
    out = {"port": data[3], "info_type": data[4]}
    if out["info_type"] == PORT_INFO_MODE and len(data) >= 11:
        caps = data[5]
        out.update(
            output=bool(caps & 0x01),
            input=bool(caps & 0x02),
            combinable=bool(caps & 0x04),
            synchronizable=bool(caps & 0x08),
            mode_count=data[6],
            input_modes=data[7] | (data[8] << 8),
            output_modes=data[9] | (data[10] << 8),
        )
    return out


def parse_port_mode_information(data):
    """0x44 carries one attribute of one mode of one port."""
    if len(data) < 6 or data[2] != MSG_PORT_MODE_INFORMATION:
        return None
    out = {"port": data[3], "mode": data[4], "info_type": data[5]}
    payload = data[6:]
    info_type = out["info_type"]

    if info_type == MODE_INFO_NAME:
        out["name"] = bytes(payload).split(b"\x00")[0].decode("ascii", "replace").strip()
    elif info_type == MODE_INFO_SYMBOL:
        out["symbol"] = bytes(payload).split(b"\x00")[0].decode("ascii", "replace").strip()
    elif info_type in (MODE_INFO_RAW, MODE_INFO_PCT, MODE_INFO_SI) and len(payload) >= 8:
        out["min"] = float_le(payload, 0)
        out["max"] = float_le(payload, 4)
    elif info_type == MODE_INFO_VALUE_FORMAT and len(payload) >= 4:
        out.update(
            value_count=payload[0],
            data_type=payload[1],
            data_type_name=DATA_TYPE_NAMES.get(payload[1], "0x{:02X}".format(payload[1])),
            total_figures=payload[2],
            decimals=payload[3],
        )
    return out


def parse_generic_error(data):
    if len(data) < 5 or data[2] != MSG_GENERIC_ERROR:
        return None
    return {"command": data[3], "code": data[4]}


def decode_mode_values(payload, data_type, value_count):
    """Decode a PortValueSingle payload into a list of numbers, given the
    format reported by MODE_INFO_VALUE_FORMAT."""
    size = DATA_TYPE_SIZES.get(data_type)
    if size is None:
        return []
    readers = {0x00: lambda p, o: p[o] - 256 if p[o] >= 128 else p[o],
               0x01: int16_le, 0x02: int32_le, 0x03: float_le}
    reader = readers[data_type]
    values = []
    for i in range(value_count):
        offset = i * size
        if offset + size > len(payload):
            break
        values.append(reader(payload, offset))
    return values
