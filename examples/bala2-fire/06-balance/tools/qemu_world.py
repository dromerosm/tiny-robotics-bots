#!/usr/bin/env python3
"""A world of walls for the firmware that runs in QEMU, to test explorer.py with it.

The firmware in QEMU (built with VARIANT=qemu) has a simulated robot that
balances, drives and turns, and a network. It has no world: its distance
sensor reads what the command `simtof <mm>` tells it. This script is the
world. It stands between the tool and the firmware:

    explorer.py  -->  qemu_world.py (port 8142)  -->  firmware in QEMU (port 8099)

  - `GET /state` and `POST /cmd` go to the firmware as they are.
  - 10 times a second it reads the robot's place and heading, moves the
    robot in the world, works out what the sensor sees, and sends it.
  - `GET /truth` gives the real place and the world, as sim_robot.py does.

    python3 qemu_world.py --robot 127.0.0.1:8099 --world rooms
    BALA2_HOST=127.0.0.1:8142 python3 explorer.py scan
    python3 explorer_test.py --robot 127.0.0.1:8142      the whole test

The worlds and the sensor's cone are those of sim_robot.py. `--drift` and
`--scale` give the robot's own numbers an error, as on the real floor: the
world moves the robot a bit less than the firmware reports. Low things do not
work here: nothing in this setup can hold the wheels of the firmware's robot.

Python 3 standard library only.
"""
import argparse
import json
import math
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import sim_robot


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", default="127.0.0.1:8099", help="the firmware in QEMU")
    ap.add_argument("--port", type=int, default=8142)
    ap.add_argument("--world", default="rooms", help="one of: " + ", ".join(sim_robot.WORLDS) +
                    ", or a JSON file with walls and low")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--drift", type=float, default=2.0,
                    help="degrees the reported heading is off after a turn of 360")
    ap.add_argument("--scale", type=float, default=1.03,
                    help="reported distance / real distance")
    args = ap.parse_args()
    args.speed, args.short = 1, 0
    world = sim_robot.WORLDS.get(args.world) or json.load(open(args.world))
    sim = sim_robot.Sim(world, args)  # only for its place, its zone and its sensor
    base = "http://" + args.robot
    lock = threading.Lock()
    last = {}

    def robot(path, body=None):
        req = urllib.request.Request(base + path, data=body,
                                     headers={"X-Bala2": "1"} if body is not None else {})
        with urllib.request.urlopen(req, timeout=3) as r:
            return r.read()

    def follow():
        """Move the robot in the world by what the firmware says it moved."""
        while True:
            start = time.time()
            try:
                s = json.loads(robot("/state"))
                pos, head = s["pos"], s["head"][0]  # the wheels: no noise in them
                with lock:
                    if s["state"] != "balancing":
                        last.clear()
                    elif not last:
                        last.update(pos=pos, head=head)
                    else:
                        dp, dh = pos - last["pos"], head - last["head"]
                        last.update(pos=pos, head=head)
                        th = math.radians(sim.th + dh * (1 - args.drift / 360.0) / 2)
                        sim.x += dp / args.scale * math.cos(th)
                        sim.y += dp / args.scale * math.sin(th)
                        sim.th += dh * (1 - args.drift / 360.0)
                    sim.t = s["up"]
                    for k in ("tz", "tzr", "tcol", "trow"):
                        sim.vars[k] = s["vars"][k]
                    mm = sim.tof()
                robot("/cmd", f"simtof {mm}".encode())
            except (OSError, ValueError, KeyError) as e:
                print("qemu_world:", e, flush=True)
                time.sleep(1)
            time.sleep(max(0.0, 0.1 - (time.time() - start)))

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def reply(self, data, code=200):
            self.send_response(code)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            try:
                if self.path == "/truth":
                    with lock:
                        self.reply(json.dumps({"x": sim.x, "y": sim.y, "heading": sim.th,
                                               "world": world, "t": sim.t}).encode())
                else:
                    self.reply(robot(self.path))
            except OSError as e:
                self.reply(str(e).encode(), 502)

        def do_POST(self):
            body = self.rfile.read(int(self.headers.get("Content-Length", 0)))
            if not self.headers.get("X-Bala2"):
                return self.reply(b"bad command\n", 400)
            try:
                self.reply(robot(self.path, body))
            except OSError as e:
                self.reply(str(e).encode(), 502)

    threading.Thread(target=follow, daemon=True).start()
    ThreadingHTTPServer(("127.0.0.1", args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
