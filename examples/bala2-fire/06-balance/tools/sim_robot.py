#!/usr/bin/env python3
"""A stand-in for 06-balance in a flat world, to test explorer.py without the robot.

It answers `GET /state` and `POST /cmd` like the firmware, with the same
`drive` rules: an order ends by itself, the speed changes slowly, the distance
sensor slows a forward order and the goal never passes the stop line. It does
not balance: it only moves a point with a heading through a world of walls.

    python3 sim_robot.py --port 8099 --world rooms
    BALA2_HOST=127.0.0.1:8099 python3 explorer.py scan

What it copies from the real robot (tests 38 to 41), so the tool meets the
same trouble:
  - it stops a few cm before its goal (`--short`), and runs behind it,
  - the tilt, the speed and the turn rate sway all the time,
  - the heading it reports drifts (`--drift`, degrees per turn of 360) and
    the distance it reports is a bit off (`--scale`),
  - the sensor sees a cone as wide as its zone (`tz`, `tzr`) and gives the
    nearest thing in it; a small zone sees less far,
  - low things (`low` in a world) stop the wheels and the sensor misses them.

`GET /truth` gives the real place and the world, for tests. `--speed 5` makes
the clock run 5 times faster; give explorer.py the same value in
BALA2_TIME_SCALE.

Python 3 standard library only.
"""
import argparse
import json
import math
import random
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

DT = 0.005
RADIUS = 0.08  # m, the robot as a circle
ZONE_DEG = 1.7


def box(x1, y1, x2, y2):
    return [(x1, y1, x2, y1), (x2, y1, x2, y2), (x2, y2, x1, y2), (x1, y2, x1, y1)]


# Walls are segments (x1, y1, x2, y2) in metres. The robot starts at (0, 0), looking along x.
WORLDS = {
    # One room, 3.0 x 2.4 m, with a box in it.
    "room": {"walls": box(-1.0, -1.2, 2.0, 1.2) + box(0.9, -0.3, 1.2, 0.1), "low": []},
    # Two rooms joined by a door 0.6 m wide, a box, and a low bar the sensor misses.
    "rooms": {"walls": box(-1.0, -1.2, 3.2, 1.2) + [(1.5, -1.2, 1.5, -0.1), (1.5, 0.5, 1.5, 1.2)] +
              box(0.3, 0.7, 0.6, 1.0),
              "low": [(2.2, -1.2, 2.2, -0.7)]},
    # A wall 0.45 m ahead with two gaps in it: 0.30 m (the robot fits) and 0.12 m (it does not).
    "gaps": {"walls": box(-1.0, -1.2, 2.0, 1.2) + [(0.45, -1.2, 0.45, -0.25), (0.45, 0.05, 0.45, 0.30),
                                                     (0.45, 0.42, 0.45, 1.2)], "low": []},
    # Nothing near: every direction reads "nothing".
    "empty": {"walls": box(-9, -9, 9, 9), "low": []},
}


def hit(px, py, dx, dy, seg):
    """Distance along the ray (px, py) + t (dx, dy) to the segment, or None."""
    x1, y1, x2, y2 = seg
    ex, ey = x2 - x1, y2 - y1
    den = dx * ey - dy * ex
    if abs(den) < 1e-9:
        return None
    t = ((x1 - px) * ey - (y1 - py) * ex) / den
    u = ((x1 - px) * dy - (y1 - py) * dx) / den
    return t if t > 0 and 0 <= u <= 1 else None


def near(px, py, seg):
    """Distance from a point to a segment."""
    x1, y1, x2, y2 = seg
    ex, ey = x2 - x1, y2 - y1
    u = max(0.0, min(1.0, ((px - x1) * ex + (py - y1) * ey) / (ex * ex + ey * ey or 1e-9)))
    return math.hypot(px - x1 - u * ex, py - y1 - u * ey)


class Sim:
    def __init__(self, world, args):
        self.world, self.a = world, args
        self.rng = random.Random(args.seed)
        self.x = self.y = self.th = 0.0  # the real place (m) and heading (degrees)
        self.pos = self.head = 0.0       # what the robot reports
        self.xg = self.hg = 0.0          # its goals, in what it reports
        self.v = self.w = 0.0            # speeds it follows now
        self.dv = self.dw = 0.0          # the order
        self.end = 0.0
        self.speed = self.yaw = 0.0
        self.t = 0.0
        self.state, self.reason = "balancing", "balancing"
        self.slows = False
        self.vars = dict(ts=0.2, tw=0.5, dv=0.25, dacc=0.3, tz=4, tzr=16, tcol=14, trow=7)
        self.mm = -1
        self.last_wall = time.time()
        self.lock = threading.Lock()

    def tof(self):
        """The nearest wall in the sensor's cone, in mm, or -1 for nothing."""
        v = self.vars
        rows = v["tzr"] or v["tz"]
        width = rows * ZONE_DEG
        mid = (v["trow"] - 7.5 if rows < 16 else 0) * ZONE_DEG
        reach = min(4.0, 0.25 * math.sqrt(rows * v["tz"]))  # a small zone sees less far
        best = None
        n = max(1, int(width / 2))
        for k in range(n + 1):
            ang = math.radians(self.th + mid - width / 2 + width * k / n)
            dx, dy = math.cos(ang), math.sin(ang)
            for seg in self.world["walls"]:
                d = hit(self.x, self.y, dx, dy, seg)
                if d is not None and (best is None or d < best):
                    best = d
        if best is None or best > reach:
            return -1
        return max(0, int(best * 1000 * (1 + self.rng.gauss(0, 0.01))))

    def step(self):
        a, v = self.a, self.vars
        self.t += DT
        if int(self.t / 0.1) != int((self.t - DT) / 0.1):
            self.mm = self.tof()  # the sensor measures 10 times a second
        if self.state != "balancing":
            return
        on = self.t < self.end
        want_v = max(-v["dv"], min(v["dv"], self.dv)) if on else 0.0
        want_w = self.dw if on else 0.0
        most, room = 0.25, 10.0
        if v["ts"] > 0 and self.mm >= 0:
            d = self.mm / 1000
            most *= min(1.0, max(0.0, (d - v["ts"]) / max(v["tw"] - v["ts"], 0.01)))
            room = max(0.0, d - v["ts"])
        self.slows = want_v > most
        want_v = min(want_v, most)
        self.v += max(-v["dacc"] * DT, min(v["dacc"] * DT, want_v - self.v))
        self.w += max(-180 * DT, min(180 * DT, want_w - self.w))
        self.xg = min(self.xg + self.v * DT, self.pos + room)
        self.hg += self.w * DT
        if abs(self.pos - self.xg) > 0.3:
            self.state, self.reason = "stopped", "moved too far"
            return
        # Follow the goals: behind them while it drives, and a little short at the end.
        ex = self.xg - self.pos
        dp = 0.0 if abs(ex) < a.short else (ex - math.copysign(a.short, ex)) * min(1.0, 5 * DT)
        dh = (self.hg - self.head) * min(1.0, 8 * DT)
        th = math.radians(self.th)
        real = dp / a.scale
        nx, ny = self.x + real * math.cos(th), self.y + real * math.sin(th)
        segs = self.world["walls"] + self.world["low"]
        if real and any(near(nx, ny, s) < RADIUS <= near(self.x, self.y, s) or
                        near(nx, ny, s) < min(RADIUS, near(self.x, self.y, s)) for s in segs):
            dp = 0.0  # something holds the wheels
        else:
            self.x, self.y = nx, ny
        self.pos += dp
        self.head += dh
        self.th += dh * (1 - a.drift / 360.0)  # the real turn is a bit less than reported
        self.speed, self.yaw = dp / DT, dh / DT

    def advance(self):
        now = time.time()
        steps = int((now - self.last_wall) * self.a.speed / DT)
        self.last_wall += steps * DT / self.a.speed
        for _ in range(min(steps, 20000)):
            self.step()

    def state_json(self):
        g = self.rng.gauss
        return {
            "state": self.state, "reason": self.reason, "tilt": round(5.5 + g(0, 1.0), 2),
            "speed": round(self.speed + g(0, 0.02), 3), "battery": 75, "charging": False,
            "vars": self.vars, "rate": 0, "pos": round(self.pos, 3), "u": 0, "setpoint": 5.5,
            "heading": round(self.head + g(0, 0.4), 1), "yaw": round(self.yaw + g(0, 6), 1),
            "turn": 0, "drive": [round(self.v, 3), round(self.w, 1)],
            "goal": [round(self.xg, 3), round(self.hg, 1)],
            # The wheels alone (what it reports) and the gyro alone (the real turn, 0.5% high).
            "head": [round(self.head, 1), round(self.th * 1.005, 1)],
            "tof": self.mm if self.state == "balancing" else -1, "tof_slows": self.slows,
            "acc": [0, 0, 1], "gyro": [0, 0, 0], "temp": 40, "enc": [0, 0], "i2c_err": 0,
            "rssi": -50, "up": int(self.t), "heap": 100000, "fw": "sim_robot.py", "stored": 0,
        }

    def command(self, line):
        part = line.split()
        if not part:
            return
        if part[0] == "drive":
            num = [float(p) for p in part[1:4]]
            num += [0.0, 0.0, 0.5][len(num):]
            self.dv = max(-0.25, min(0.25, num[0]))
            self.dw = max(-90.0, min(90.0, num[1]))
            self.end = self.t + min(max(num[2], 0.0), 5.0)
        elif part[0] == "stop":
            self.state, self.reason = "stopped", "button B"
        elif part[0] in self.vars and len(part) > 1:
            self.vars[part[0]] = type(self.vars[part[0]])(float(part[1]))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8099)
    ap.add_argument("--world", default="rooms", help="one of: " + ", ".join(WORLDS) +
                    ", or a JSON file with walls and low")
    ap.add_argument("--speed", type=float, default=1, help="the clock runs this many times faster")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--short", type=float, default=0.035, help="m it stops before its goal")
    ap.add_argument("--drift", type=float, default=2.0,
                    help="degrees the reported heading is off after a turn of 360")
    ap.add_argument("--scale", type=float, default=1.03,
                    help="reported distance / real distance (wheels that slip a little)")
    args = ap.parse_args()
    world = WORLDS.get(args.world) or json.load(open(args.world))
    sim = Sim(world, args)

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def reply(self, body, code=200):
            data = body.encode()
            self.send_response(code)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            with sim.lock:
                sim.advance()
                if self.path == "/truth":
                    self.reply(json.dumps({"x": sim.x, "y": sim.y, "heading": sim.th,
                                           "world": world, "t": sim.t}))
                else:
                    self.reply(json.dumps(sim.state_json()))

        def do_POST(self):
            line = self.rfile.read(int(self.headers.get("Content-Length", 0))).decode()
            if not self.headers.get("X-Bala2"):
                return self.reply("bad command\n", 400)
            with sim.lock:
                sim.advance()
                sim.command(line)
            self.reply("ok\n")

    ThreadingHTTPServer(("127.0.0.1", args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
