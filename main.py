"""
main.py - entry point. Starts the hub service on a background thread and
runs the dashboard on the main thread.

Closing the window calls hub_service.request_stop(), which stops the
motors and disconnects cleanly before the process exits.
"""

import threading

import hub_service
import ui

if __name__ == "__main__":
    t = threading.Thread(target=hub_service.run, daemon=True)
    t.start()
    try:
        ui.launch(on_close=hub_service.request_stop)
    finally:
        hub_service.request_stop()
        t.join(timeout=3.0)
