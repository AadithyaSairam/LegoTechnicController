"""
bt_check.py - Is Bluetooth usable on this machine, and can it see the hub?

Run this after any BIOS change, cable reseat, driver install or dongle
swap. It answers, in order:

  1. Does bleak see a Bluetooth radio at all?
  2. Does a BLE scan work?
  3. Is the Technic Move Hub advertising?

It never connects or sends anything - purely a scan.

    python bt_check.py

If step 1 fails, the problem is Windows/hardware, not this project. On an
Intel combo WiFi/BT card (AX200/AX210), Bluetooth runs over USB while WiFi
runs over PCIe, so WiFi can work perfectly while Bluetooth is absent - check
the card's USB header cable and the BIOS before suspecting anything else.
A missing Bluetooth toggle under Settings > Network & Internet > Airplane
mode means Windows sees no radio.
"""

import asyncio
import sys

SCAN_SECONDS = 8.0
HUB_NAME = "Technic Move"          # as advertised, per technicmovehub
LWP3_SERVICE = "00001623-1212-efde-1623-785feabcd123"


async def main():
    try:
        from bleak import BleakScanner
    except ImportError:
        print("bleak is not installed. Run: pip install -r requirements.txt")
        return 2

    print("Scanning for {:.0f}s...".format(SCAN_SECONDS))
    try:
        devices = await BleakScanner.discover(timeout=SCAN_SECONDS, return_adv=True)
    except Exception as e:
        name = type(e).__name__
        print("\nNO BLUETOOTH ADAPTER AVAILABLE ({}: {})".format(name, e))
        print("\nWindows is not exposing a Bluetooth radio to the BLE stack.")
        print("This is a machine problem, not a problem with this project:")
        print("  - Settings > Network & Internet > Airplane mode: is there a")
        print("    Bluetooth toggle under 'Wireless devices'? If not, Windows")
        print("    sees no radio.")
        print("  - Intel combo cards run Bluetooth over USB: check the card's")
        print("    USB header cable, and the BIOS onboard-device settings.")
        print("  - Device Manager > View > Show hidden devices, to spot a")
        print("    greyed-out or errored radio.")
        return 1

    print("\nBluetooth adapter works. {} device(s) seen.\n".format(len(devices)))

    hits = []
    for device, adv in devices.values():
        name = device.name or adv.local_name or ""
        uuids = [u.lower() for u in (adv.service_uuids or [])]
        is_hub = HUB_NAME.lower() in name.lower() or LWP3_SERVICE in uuids
        if is_hub:
            hits.append((device, adv, name))
        print("  {}  {:<28} rssi {}".format(
            device.address, (name or "(no name)")[:28],
            adv.rssi if adv.rssi is not None else "?"))

    print()
    if hits:
        for device, adv, name in hits:
            print("FOUND THE HUB: {} at {} (rssi {})".format(
                name or HUB_NAME, device.address, adv.rssi))
        print("\nYou are good to go:  python port_scan.py")
        return 0

    print("Adapter is fine, but no Technic Move Hub is advertising.")
    print("  - Press the hub button so it blinks (it only advertises when")
    print("    it is not already connected to something).")
    print("  - Close the LEGO Control+ app and disconnect the hub from any")
    print("    phone - it accepts ONE connection at a time.")
    print("  - Make sure main.py or port_scan.py is not already running.")
    return 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
