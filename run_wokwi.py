import argparse
import socket
import time
import tkinter as tk

PORT = 4000

parser = argparse.ArgumentParser(description="Connect the dashboard to Wokwi RFC2217")
parser.add_argument("--port", type=int, default=PORT)
parser.add_argument("--timeout", type=float, default=45.0)
parser.add_argument("--gui", action="store_true", help="launch the Tk dashboard")
args = parser.parse_args()
PORT = args.port

deadline = time.time() + args.timeout

while time.time() < deadline:
    try:
        s = socket.create_connection(("127.0.0.1", PORT), timeout=0.4)
        s.close()
        break
    except OSError:
        time.sleep(0.5)
else:
    print(f"Wokwi RFC2217 server was not detected on localhost:{PORT}.")
    raise SystemExit(1)

if not args.gui:
    print(f"Wokwi RFC2217 server is ready on localhost:{PORT}.")
    raise SystemExit(0)

from air_mouse import AirMouseApp

root = tk.Tk()
app = AirMouseApp(root)

try:
    app.source_var.set("Wokwi")
    app.change_source()
    app.connect_wokwi()
except Exception as exc:
    print("Wokwi connection:", repr(exc))

root.mainloop()
