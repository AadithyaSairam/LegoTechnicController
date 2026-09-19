"""
ui.py - Tkinter dashboard for the LEGO Technic Porsche GT4 e-Performance (42176)

Pure display layer: polls hub_service.get_state() every 100ms and renders
it. Contains NO BLE, protocol, or controller-reading code - all of that
lives in hub_service.py.

The service stores sensor readings as raw numbers (or None); all unit
formatting happens here.
"""

import tkinter as tk
from tkinter import font as tkfont

import hub_service

BG = "#111111"
ACCENT = "#00e0ff"
DIM = "#666666"
GOOD = "#33ff33"
WARN = "#ffcc00"
BAD = "#ff3333"


def _fmt_pct(value):
    return "--" if value is None else "{}%".format(value)


def _fmt_temp(value):
    return "--" if value is None else "{:.1f}°C".format(value)


def _fmt_xyz(triple, unit):
    if triple is None:
        return "--"
    return "{} {}".format(", ".join(str(v) for v in triple), unit)


def launch(on_close=None):
    root = tk.Tk()
    root.title("LEGO Porsche GT4 e-Performance - Live Dashboard")
    root.geometry("480x760")
    root.configure(bg=BG)

    big_font = tkfont.Font(family="Segoe UI", size=22, weight="bold")
    mid_font = tkfont.Font(family="Segoe UI", size=14, weight="bold")
    label_font = tkfont.Font(family="Segoe UI", size=12)
    status_font = tkfont.Font(family="Segoe UI", size=10)
    debug_font = tkfont.Font(family="Consolas", size=9)

    def row(label_text, row_idx, value_font=big_font):
        tk.Label(root, text=label_text, font=label_font, fg="#aaaaaa", bg=BG,
                 anchor="w").grid(row=row_idx, column=0, sticky="w", padx=20, pady=5)
        value_lbl = tk.Label(root, text="--", font=value_font, fg=ACCENT, bg=BG, anchor="e")
        value_lbl.grid(row=row_idx, column=1, sticky="e", padx=20, pady=5)
        return value_lbl

    battery_lbl = row("Battery", 0)
    fw_lbl = row("Firmware", 1, mid_font)
    hw_lbl = row("Hardware", 2, mid_font)
    drive_lbl = row("Drive Power", 3)
    steer_lbl = row("Steer Power", 4)
    headlights_lbl = row("Headlights", 5, mid_font)
    brake_lbl = row("Brake Lights", 6, mid_font)
    temp_lbl = row("Temperature", 7, mid_font)
    accel_lbl = row("Accelerometer", 8, mid_font)
    gyro_lbl = row("Gyro", 9, mid_font)
    tilt_lbl = row("Tilt", 10, mid_font)
    loop_lbl = row("Control Loop", 11, mid_font)
    send_lbl = row("BLE Write", 12, mid_font)

    failsafe_lbl = tk.Label(root, text="", font=mid_font, fg=BAD, bg=BG)
    failsafe_lbl.grid(row=13, column=0, columnspan=2, sticky="w", padx=20, pady=(10, 0))

    status_lbl = tk.Label(root, text="Starting...", font=status_font, fg=WARN,
                          bg=BG, wraplength=420, justify="left")
    status_lbl.grid(row=14, column=0, columnspan=2, sticky="w", padx=20, pady=10)

    log_lbl = tk.Label(root, text="", font=debug_font, fg=DIM, bg=BG,
                       wraplength=420, justify="left")
    log_lbl.grid(row=15, column=0, columnspan=2, sticky="w", padx=20, pady=2)

    debug_lbl = tk.Label(root, text="", font=debug_font, fg="#555555",
                         bg=BG, wraplength=420, justify="left")
    debug_lbl.grid(row=16, column=0, columnspan=2, sticky="w", padx=20, pady=2)

    conn_dot = tk.Label(root, text="●", font=("Segoe UI", 16), fg=BAD, bg=BG)
    conn_dot.grid(row=17, column=0, sticky="w", padx=20, pady=10)
    conn_txt = tk.Label(root, text="Disconnected", font=status_font, fg="#aaaaaa", bg=BG)
    conn_txt.grid(row=17, column=1, sticky="w")

    for i in range(2):
        root.grid_columnconfigure(i, weight=1)

    def refresh():
        s = hub_service.get_state()
        battery_lbl.config(text=_fmt_pct(s["battery_pct"]))
        fw_lbl.config(text=s["fw_version"])
        hw_lbl.config(text=s["hw_version"])
        drive_lbl.config(text=str(s["drive_value"]))
        steer_lbl.config(text=str(s["steer_value"]))
        headlights_lbl.config(text="ON" if s["headlights_on"] else "OFF",
                              fg=GOOD if s["headlights_on"] else DIM)
        brake_lbl.config(text="ON" if s["braking"] else "OFF",
                         fg=BAD if s["braking"] else DIM)
        temp_lbl.config(text=_fmt_temp(s["temperature_c"]))
        accel_lbl.config(text=_fmt_xyz(s["accel"], "mg"))
        gyro_lbl.config(text=_fmt_xyz(s["gyro"], "dps"))
        tilt_lbl.config(text=_fmt_xyz(s["tilt"], "°"))

        # Loop health: rate well below target, or climbing overruns, means
        # the BLE write is eating the budget - worth seeing at a glance.
        hz = s["loop_hz"]
        loop_lbl.config(
            text="{:.1f} Hz  ±{:.1f} ms  ({} over)".format(
                hz, s["loop_jitter_ms"], s["loop_overruns"]),
            fg=GOOD if hz >= hub_service.CONTROL_HZ * 0.9 else WARN)
        send_lbl.config(text="{:.1f} ms".format(s["send_ms"]),
                        fg=WARN if s["send_ms"] > 1000.0 / hub_service.CONTROL_HZ else ACCENT)

        failsafe_lbl.config(text="⚠  FAILSAFE - MOTORS STOPPED" if s["failsafe"] else "")
        status_lbl.config(text=s["status"])

        if s["log_path"]:
            log_lbl.config(text="Log: {}  ({} rows)".format(s["log_path"], s["log_rows"]))
        debug_lbl.config(text="Axes: {}".format(s["axes_debug"]) if s["axes_debug"] else "")

        if s["connected"]:
            conn_dot.config(fg=GOOD)
            conn_txt.config(text="Connected")
        else:
            conn_dot.config(fg=BAD)
            conn_txt.config(text="Disconnected")
        root.after(100, refresh)

    def handle_close():
        # Stop the car BEFORE tearing down the window. Closing the window
        # kills the daemon service thread, and the hub keeps executing the
        # last command it was given, so the shutdown has to be ordered.
        status_lbl.config(text="Stopping motors and disconnecting...")
        root.update_idletasks()
        if on_close is not None:
            on_close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", handle_close)
    refresh()
    root.mainloop()
