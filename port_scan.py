"""
port_scan.py - Ask the Technic Move Hub to describe every one of its ports.

Standalone diagnostic. Connects, sweeps ports 0x00-0x3F with LWP3
"Port Information Request" messages, and for every port that answers,
asks each of its modes for its name, unit symbol, value range and value
format. Prints a report and writes the raw findings to logs/.

This exists to settle two open questions in hub_service.py:

  1. Are the onboard sensor ports really 0x37/0x38/0x39/0x3A, and what
     are their true value formats (int8/int16/int32, how many values,
     how many decimals)? The current scaling there is assumed from other
     LEGO hubs, not measured.

  2. Do the drive motors expose rotation counts (a POS / APOS / SPEED
     mode)? If they do, real closed-loop speed control, distance-based
     maneuvers and slip detection all become possible. If they do not,
     any speed estimate has to come from the IMU or an external sensor.

The script only reads - it never sends a drive command.

    python port_scan.py

Remember the hub holds one BLE connection at a time: close the LEGO
Control+ app and stop main.py before running this.
"""

import asyncio
import json
import os
import sys
import time

from technicmovehub import technicmovehub

import lwp3

PORT_RANGE = range(0x00, 0x40)

# Pacing. The hub drops requests if you flood it, so requests are
# pipelined with a small gap and then given time to settle, rather than
# waiting for each reply in turn (which would take minutes).
INTER_REQUEST_DELAY = 0.02
SETTLE_TIME = 1.0
MODE_SETTLE_TIME = 1.2

MODE_ATTRS = (
    (lwp3.MODE_INFO_NAME, "name"),
    (lwp3.MODE_INFO_SYMBOL, "symbol"),
    (lwp3.MODE_INFO_RAW, "raw"),
    (lwp3.MODE_INFO_SI, "si"),
    (lwp3.MODE_INFO_VALUE_FORMAT, "format"),
)

# Mode names that indicate usable odometry / velocity feedback.
ODOMETRY_HINTS = ("POS", "APOS", "SPEED", "ROT", "DEG", "COUNT")


class _Collector:
    """Notification handler that routes replies to the futures waiting
    for them, and passively records anything else the hub volunteers."""

    def __init__(self):
        self.pending = {}
        self.attached = {}
        self.versions = {}
        self.unmatched = []
        self.errors = []

    def handle(self, _sender, data):
        data = bytearray(data)

        if len(data) >= 9 and data[2] == lwp3.MSG_HUB_PROPERTIES:
            if data[3] == lwp3.PROP_FW_VERSION:
                self.versions["firmware"] = lwp3.decode_version(data[5:])
                return
            if data[3] == lwp3.PROP_HW_VERSION:
                self.versions["hardware"] = lwp3.decode_version(data[5:])
                return

        io = lwp3.parse_hub_attached_io(data)
        if io is not None:
            self.attached[io["port"]] = io
            return

        info = lwp3.parse_port_information(data)
        if info is not None:
            self._resolve(("info", info["port"]), info)
            return

        mode = lwp3.parse_port_mode_information(data)
        if mode is not None:
            self._resolve(("mode", mode["port"], mode["mode"], mode["info_type"]), mode)
            return

        err = lwp3.parse_generic_error(data)
        if err is not None:
            self.errors.append(err)
            return

        self.unmatched.append(bytes(data).hex(" "))

    def _resolve(self, key, value):
        fut = self.pending.pop(key, None)
        if fut is not None and not fut.done():
            fut.set_result(value)


async def _request_batch(client, collector, requests, settle):
    """Fire a batch of requests, wait once, and collect whatever answered.

    requests: iterable of (key, message). Returns {key: reply or None}.
    """
    loop = asyncio.get_running_loop()
    futures = {}
    for key, message in requests:
        fut = loop.create_future()
        collector.pending[key] = fut
        futures[key] = fut
        try:
            await client.write_gatt_char(lwp3.CHAR_UUID, message)
        except Exception as e:
            print("  write failed for {}: {}".format(key, e))
        await asyncio.sleep(INTER_REQUEST_DELAY)

    await asyncio.sleep(settle)

    results = {}
    for key, fut in futures.items():
        results[key] = fut.result() if fut.done() else None
        collector.pending.pop(key, None)
    return results


async def _scan(client):
    collector = _Collector()
    await client.start_notify(lwp3.CHAR_UUID, collector.handle)

    # Give the hub a moment to volunteer its HubAttachedIO announcements.
    await asyncio.sleep(1.0)

    # Versions, for the record - firmware differences are exactly why
    # this scan is worth doing rather than hard-coding a port map.
    for prop in (lwp3.PROP_FW_VERSION, lwp3.PROP_HW_VERSION):
        await client.write_gatt_char(
            lwp3.CHAR_UUID, lwp3.hub_property(prop, lwp3.PROP_OP_REQUEST_UPDATE))
    await asyncio.sleep(0.5)

    print("Probing ports 0x00-0x3F...")
    port_replies = await _request_batch(
        client, collector,
        [(("info", p), lwp3.port_info_request(p, lwp3.PORT_INFO_MODE))
         for p in PORT_RANGE],
        SETTLE_TIME,
    )

    ports = {}
    for port in PORT_RANGE:
        info = port_replies.get(("info", port))
        if info is None or "mode_count" not in info:
            continue
        ports[port] = {"info": info, "modes": {}}

    print("Found {} responding port(s): {}".format(
        len(ports), ", ".join("0x{:02X}".format(p) for p in sorted(ports)) or "none"))

    for port, entry in sorted(ports.items()):
        mode_count = entry["info"]["mode_count"]
        print("Querying {} mode(s) on port 0x{:02X}...".format(mode_count, port))
        requests = []
        for mode in range(mode_count):
            for info_type, _label in MODE_ATTRS:
                requests.append((("mode", port, mode, info_type),
                                 lwp3.port_mode_info_request(port, mode, info_type)))
        replies = await _request_batch(client, collector, requests, MODE_SETTLE_TIME)

        for mode in range(mode_count):
            detail = {}
            for info_type, label in MODE_ATTRS:
                reply = replies.get(("mode", port, mode, info_type))
                if reply is None:
                    continue
                if label == "name":
                    detail["name"] = reply.get("name", "")
                elif label == "symbol":
                    detail["symbol"] = reply.get("symbol", "")
                elif label in ("raw", "si"):
                    if "min" in reply:
                        detail[label] = [reply["min"], reply["max"]]
                elif label == "format":
                    detail["value_count"] = reply.get("value_count")
                    detail["data_type"] = reply.get("data_type_name")
                    detail["decimals"] = reply.get("decimals")
            entry["modes"][mode] = detail

    try:
        await client.stop_notify(lwp3.CHAR_UUID)
    except Exception:
        pass

    return {
        "versions": collector.versions,
        "attached": collector.attached,
        "ports": ports,
        "errors": collector.errors,
        "unmatched": collector.unmatched,
    }


def _fmt_range(pair):
    if not pair:
        return ""
    return "{:g}..{:g}".format(pair[0], pair[1])


def print_report(result):
    print()
    print("=" * 78)
    print("PORT / MODE REPORT")
    print("=" * 78)

    versions = result.get("versions") or {}
    if versions:
        print("\nHub firmware {}  hardware {}".format(
            versions.get("firmware", "?"), versions.get("hardware", "?")))

    attached = result["attached"]
    if attached:
        print("\nDevices the hub announced on connect:")
        for port in sorted(attached):
            io = attached[port]
            if "io_type" in io:
                print("  port 0x{:02X}  {}".format(port, lwp3.io_type_name(io["io_type"])))
            else:
                print("  port 0x{:02X}  (detached)".format(port))
    else:
        print("\nNo HubAttachedIO announcements were captured (they may have")
        print("been sent before this script subscribed). The port probe below")
        print("is the authoritative result.")

    for port, entry in sorted(result["ports"].items()):
        info = entry["info"]
        io = attached.get(port, {})
        label = lwp3.io_type_name(io["io_type"]) if "io_type" in io else "unidentified device"
        caps = [name for name, on in (("input", info.get("input")),
                                      ("output", info.get("output")),
                                      ("combinable", info.get("combinable")),
                                      ("sync", info.get("synchronizable"))) if on]
        print("\nPort 0x{:02X}  {}".format(port, label))
        print("  capabilities: {}".format(", ".join(caps) or "none"))
        print("  {:<4} {:<12} {:<7} {:<18} {:<18} {}".format(
            "mode", "name", "unit", "raw range", "SI range", "format"))
        for mode in sorted(entry["modes"]):
            d = entry["modes"][mode]
            fmt = ""
            if d.get("value_count") is not None:
                fmt = "{} x {}".format(d["value_count"], d.get("data_type"))
                if d.get("decimals"):
                    fmt += " ({} dp)".format(d["decimals"])
            print("  {:<4} {:<12} {:<7} {:<18} {:<18} {}".format(
                mode, (d.get("name") or "?")[:12], (d.get("symbol") or "")[:7],
                _fmt_range(d.get("raw")), _fmt_range(d.get("si")), fmt))

    # The headline question.
    print("\n" + "-" * 78)
    hits = []
    for port, entry in sorted(result["ports"].items()):
        for mode, d in sorted(entry["modes"].items()):
            name = (d.get("name") or "").upper()
            if any(hint in name for hint in ODOMETRY_HINTS):
                hits.append((port, mode, d.get("name"), d.get("symbol")))
    if hits:
        print("ODOMETRY / VELOCITY FEEDBACK FOUND:")
        for port, mode, name, symbol in hits:
            print("  port 0x{:02X} mode {}  '{}'  unit '{}'".format(
                port, mode, name, symbol or "-"))
        print("\nSubscribe to these with lwp3.port_input_format_setup(port, mode)")
        print("to get closed-loop speed/position feedback.")
    else:
        print("No position/speed modes found. Any speed estimate will have to")
        print("come from the IMU or an external sensor (e.g. overhead camera).")
    print("-" * 78)


def save(result):
    os.makedirs("logs", exist_ok=True)
    path = os.path.join("logs", "port_scan_{}.json".format(time.strftime("%Y%m%d_%H%M%S")))
    serialisable = {
        "versions": result.get("versions", {}),
        "attached": {"0x{:02X}".format(p): v for p, v in result["attached"].items()},
        "ports": {"0x{:02X}".format(p): v for p, v in result["ports"].items()},
        "errors": result["errors"],
        "unmatched": result["unmatched"],
    }
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(serialisable, fh, indent=2, default=str)
    return path


def main():
    hub = technicmovehub()
    print("Connecting to Technic Move Hub (make sure the LEGO app is closed)...")
    if hub.connect(timeout=60.0) is None:
        print("Could not find/connect to the hub.")
        return 1

    try:
        hub._run_async_in_thread(hub._client.pair())
    except Exception as e:
        print("Pairing warning: {}".format(e))

    try:
        result = hub._run_async_in_thread(_scan(hub._client))
        print_report(result)
        print("\nRaw findings written to {}".format(save(result)))
    finally:
        try:
            hub.disconnect()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
