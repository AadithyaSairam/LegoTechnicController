# LEGO Porsche GT4 e-Performance (42176) - Controller + Telemetry

Drive the Technic Move Hub from an Xbox/PlayStation controller, with a
live dashboard, per-tick CSV telemetry logging, and a motor watchdog.

    pip install -r requirements.txt
    python main.py

## Files

| File | Purpose |
| --- | --- |
| `main.py` | Entry point: starts the service thread, runs the dashboard, shuts down cleanly. |
| `hub_service.py` | BLE, calibration, controller reading, the fixed-rate control loop, the watchdog. No UI code. |
| `ui.py` | Tkinter dashboard. Pure display layer - polls `hub_service.get_state()`. |
| `lwp3.py` | LEGO Wireless Protocol v3 message builders and parsers. |
| `telemetry_log.py` | Buffered CSV writer, one row per control-loop tick. |
| `port_scan.py` | Standalone tool: asks the hub to describe all of its ports and modes. |
| `bt_check.py` | Is Bluetooth usable on this machine, and is the hub advertising? Scan only. |
| `test_tier0.py` | Hardware-free checks for the protocol, logging and safety code. |

## Control loop

The loop runs at a fixed `CONTROL_HZ` (default 20 Hz) using deadline-based
timing rather than `sleep(period)`, so the period does not drift by however
long the BLE write took, and `dt` is **measured** rather than assumed. Any
derivative or integral term added later depends on that being true.

The dashboard shows the achieved rate, jitter, overrun count and BLE write
duration. If the write latency exceeds the loop budget the rate degrades
honestly (e.g. 16.7 Hz) and the overrun counter climbs, rather than the loop
silently falling behind.

## Safety

Three independent guards, because a runaway RC car is not a theoretical
problem:

- **Watchdog.** A separate thread stops the motors if the control loop has
  not issued a command within `WATCHDOG_TIMEOUT` (300 ms). It arms only
  after calibration, i.e. only once the car can actually move.
- **Contained sends.** The `technicmovehub` library calls `sys.exit(1)`
  from inside its BLE thread when the hub is not connected, which surfaces
  as `SystemExit` - a `BaseException` that a plain `except Exception` does
  not catch. `safe_send()` contains it so a BLE dropout cannot kill the
  control loop and leave the car driving.
- **Ordered shutdown.** Closing the window stops the motors and disconnects
  *before* the process exits. Previously the daemon thread was killed
  mid-drive and the hub kept executing its last command.

Controller unplugs are handled too: the loop zeroes the commands and retries
the connection every couple of seconds.

## Telemetry logs

Every tick is written to `logs/drive_YYYYMMDD_HHMMSS.csv` - commands, raw
controller inputs, loop timing, and all sensor values as **raw numbers**
(unit formatting happens in the UI). Load directly with pandas:

```python
import pandas as pd
df = pd.read_csv("logs/drive_20260919_120000.csv")
df.plot(x="t_mono", y=["drive_cmd", "accel_x"])
```

Columns are documented in `telemetry_log.py`. Use `t_mono` (seconds since
loop start) for analysis and `t_wall` for correlating against video.

## Finding out what the hub can actually do

    python port_scan.py

Sweeps every port and asks each of its modes for its name, unit, range and
value format, then prints a report and writes JSON to `logs/`. It only
reads - it never sends a drive command.

Two things this settles:

1. **Sensor scaling.** The onboard sensor ports in `hub_service.py`
   (`0x37` temperature, `0x38` accelerometer, `0x39` gyro, `0x3A` tilt) come
   from [pybricks discussion #1733](https://github.com/orgs/pybricks/discussions/1733),
   but the payload scaling is *assumed* from other LEGO hubs, not measured.
   The scan makes the hub report its own value formats.
2. **Odometry.** Whether the drive motors expose a `POS` / `APOS` / `SPEED`
   mode. If they do, closed-loop speed control, distance-based maneuvers
   and slip detection all become possible; if not, any speed estimate has
   to come from the IMU or an external sensor. The report calls this out
   explicitly at the end.

## Tests

    python test_tier0.py

Covers message encoding (including that declared length bytes match real
lengths), version decoding, the message parsers, CSV logging, command
clamping, and that `safe_send` contains `SystemExit`. No hub or controller
required.

## If Bluetooth doesn't work at all

    python bt_check.py

Tells you which of three things is wrong: no radio visible to Windows, a
working radio but no hub advertising, or everything fine.

`No Bluetooth adapter found` is a machine problem, not a code problem. On
Intel combo WiFi/BT cards (AX200/AX210, as used on Gigabyte WiFi 6/6E
cards) **Bluetooth runs over USB while WiFi runs over PCIe**, so WiFi can
work perfectly while Bluetooth is completely absent. Check, in order:

1. Settings > Network & Internet > Airplane mode. If there is no Bluetooth
   toggle under "Wireless devices", Windows sees no radio at all.
2. On a PCIe add-in card: the small USB cable from the card to a 9-pin
   USB 2.0 motherboard header. Unplugged = exactly this symptom.
3. BIOS/UEFI onboard-device settings, and USB configuration (the BT radio
   sits on an internal USB port that some boards can disable).
4. `pnputil /enum-devices /class Bluetooth` - "Disconnected" means the
   device node exists but the hardware is not enumerating, which points at
   2 or 3 rather than at drivers.

Failing all that, a USB BLE dongle (Bluetooth 5.0, Realtek RTL8761B
chipset is reliable on Windows 10) works - but disable the dead onboard
radio in Device Manager first, since Windows exposes only one Bluetooth
radio to the stack bleak uses.

Note that this project cannot run on a phone: bleak supports Windows,
macOS and Linux only, with no Android or iOS backend.

## Important notes

- The hub holds ONE Bluetooth connection at a time - disconnect it from the
  LEGO Control+ app before running any script, and don't run `main.py` and
  `port_scan.py` at the same time.
- Steering calibration must be sent after every connect or the hub ignores
  drive commands entirely. `hub_service.calibrate_steering()` does this.
- Tested on Windows 10; some users report BLE pairing issues on Windows 11,
  so if connect fails that's the first thing to check.
- Xbox and PlayStation controllers both work with pygame on Windows with no
  special driver. Trigger axis mapping differs between them; `read_triggers()`
  handles both common layouts.

## Prior art / protocol references

- [DanieleBenedettelli/TechnicMoveHub](https://github.com/DanieleBenedettelli/TechnicMoveHub) -
  reverse-engineered Move Hub (88019) protocol, source of the combined
  drive/steer/lights packet and the calibration sequence.
- [toorisrael/LEGO-Porsche-Controller](https://github.com/toorisrael/LEGO-Porsche-Controller) -
  a more feature-complete controller built on bleak + pygame.
- [LWP3 specification](https://lego.github.io/lego-ble-wireless-protocol-docs/)
