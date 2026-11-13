#!/usr/bin/env python3
"""Drive 06-balance as an explorer, one order at a time. Python 3 standard library only.

Each command does one whole thing (a turn, a stretch forward, a look around),
waits until the robot is still again, prints one JSON object and ends. So a
slow caller, like an AI agent that needs seconds between two orders, can drive
the robot. The robot keeps every safety rule: this tool only sends `drive`
orders that end by themselves, and reads `GET /state`.

    explorer.py status               all the sensors, the pose and the warnings
    explorer.py watch 5              5 s of sensor statistics, without moving
    explorer.py scan                 turn once around and list the free sectors
    explorer.py scan --arc 120       only the 120 degrees ahead
    explorer.py look                 a small fan ahead with the sensor's zones, no movement
    explorer.py turn 90              turn left 90 degrees (negative: right)
    explorer.py turn --to 0          turn to a heading
    explorer.py forward 0.4          go 0.4 m ahead; stops by itself at a wall
    explorer.py halt                 end the order now (the robot keeps balancing)
    explorer.py map                  what the scans have seen so far
    explorer.py origin               forget the map; here is (0, 0)
    explorer.py params               every value you can change, here and on the robot
    explorer.py set scan_rate 20     change one (this tool's, or the robot's: `set ts 0.25`)
    explorer.py cmd "save"           send any command line to the robot

Angles are in degrees and grow to the left. Distances are in metres, the
sensor's in millimetres. `--set name=value` changes a value for one call only.

Exit code: 0 done, 1 no robot or a bad call, 2 the robot is not balancing,
3 the way was closed (the JSON says why in "stopped").

This tool's own values and the map live in ~/.bala2-explorer (or the folder in
BALA2_EXPLORER_DIR). The robot's address comes from --host, BALA2_HOST,
bala2.local or 192.168.4.1, in that order.
"""
import argparse
import http.client
import json
import math
import os
import statistics
import sys
import time

from bala2_link import find_robot

# This tool's own values: name -> (start value, what it is).
PARAMS = {
    "poll_hz": (10, "times a second it reads /state while the robot moves"),
    "order_s": (1.5, "s, how long one drive order lasts; if this tool dies the robot stops this soon"),
    "refresh_s": (0.5, "s between two drive orders"),
    "settle_s": (1.0, "s the robot must be still before a command ends"),
    "settle_speed": (0.015, "m/s, a mean speed over settle_s under this counts as still (it always sways)"),
    "settle_yaw": (4, "deg/s, a mean turn rate over settle_s under this counts as not turning"),
    "settle_off": (0.04, "m, a command also waits until the robot is this close to its goal"),
    "settle_timeout": (5, "s, wait at most this long for it to be still"),
    "turn_rate": (45, "deg/s for `turn` (the robot allows 90)"),
    "turn_tol": (3, "deg, a turn this close to its goal is good"),
    "turn_tries": (2, "small extra turns to reach the goal"),
    "turn_fix_rate": (20, "deg/s for those extra turns"),
    "fwd_speed": (0.10, "m/s for `forward` (the robot allows `dv`)"),
    "blind_max": (0.15, "m, the most it goes back, or ahead without a sensor"),
    "stall_lag": (0.08, "m behind its goal and slower than stall_speed: something holds it"),
    "stall_back": (0.10, "m, after a stall the goal goes this far behind the robot, so it moves away"),
    "stall_speed": (0.03, "m/s, mean speed of the last second under this counts as held"),
    "stall_hard": (0.22, "m behind its goal at any speed: end the order (the robot gives up at 0.30 and falls)"),
    "tof_stop_m": (0.08, "m past the robot's stop line `ts`: this close, `forward` ends as stopped by the sensor"),
    "tilt_abort": (12, "deg away from the tilt at the start: end the order"),
    "min_battery": (25, "percent, no moving under this unless it charges"),
    "min_rssi": (-82, "dBm, `forward` ends when the Wi-Fi is weaker than this"),
    "scan_rate": (30, "deg/s for `scan`"),
    "scan_bin": (10, "deg, width of one direction in a scan"),
    "scan_lag_s": (0.1, "s, age of a distance when we read it (moves its direction back)"),
    "scan_tilt_tol": (4, "deg, drop scan samples taken this far from the usual tilt"),
    "free_mm": (2000, "mm, the distance we give a direction where the sensor sees nothing"),
    "clear_mm": (600, "mm, a direction is free if nothing is closer"),
    "robot_w_mm": (150, "mm, width the robot needs to pass (measure yours)"),
    "margin_mm": (50, "mm of room on each side"),
    "tof_offset_mm": (0, "mm from the middle of the wheels to the sensor"),
    "tof_settle_s": (0.3, "s to wait after the sensor's zone changes"),
    "look_from": (2, "first zone row of `look`"),
    "look_to": (13, "last zone row of `look`"),
    "look_step": (2, "rows between two looks"),
    "look_sign": (1, "1 or -1: which way a higher row looks (depends on the mount)"),
    "map_cell": (0.1, "m, one character of `map`"),
}
HOME = os.environ.get("BALA2_EXPLORER_DIR") or os.path.expanduser("~/.bala2-explorer")
ZONE_DEG = 1.7  # one detector of the sensor, from M5Stack's 27 degrees for 16


class Fail(Exception):
    def __init__(self, code, error, **more):
        super().__init__(error)
        self.code, self.out = code, {"error": error, **more}


def load(name, default):
    try:
        with open(os.path.join(HOME, name)) as f:
            return json.load(f)
    except (OSError, ValueError):
        return default


def store(name, data):
    os.makedirs(HOME, exist_ok=True)
    tmp = os.path.join(HOME, name + ".tmp")
    with open(tmp, "w") as f:
        json.dump(data, f)
    os.replace(tmp, os.path.join(HOME, name))


class P:
    """This tool's values: the start ones, then the saved ones, then --set."""

    def __init__(self, once):
        self.saved = {k: v for k, v in load("params.json", {}).items() if k in PARAMS}
        for k, (v, _) in PARAMS.items():
            setattr(self, k, self.saved.get(k, v))
        for item in once or []:
            k, _, v = item.partition("=")
            if k not in PARAMS:
                raise Fail(1, f"--set {k}: not one of this tool's values (see `params`)")
            setattr(self, k, float(v))


def norm(deg):
    """An angle in -180 .. 180."""
    return (deg + 180) % 360 - 180


class Robot:
    def __init__(self, host):
        name, _, port = (host or "").partition(":")
        self.host = find_robot(name or None) + (":" + port if port else "")
        self.conn = None
        self.ses = load("session.json", {})
        self.ses.setdefault("pose", [0.0, 0.0])
        self.ses.setdefault("points", [])
        self.ses.setdefault("path", [[0.0, 0.0]])
        self.fresh = False  # a new balance run began: the old map is gone
        self.last = None

    def _http(self, method, path, body=None, headers=None):
        for again in (True, False):
            try:
                if self.conn is None:
                    self.conn = http.client.HTTPConnection(self.host, timeout=2)
                self.conn.request(method, path, body=body, headers=headers or {})
                resp = self.conn.getresponse()
                data = resp.read().decode(errors="replace")
            except (OSError, http.client.HTTPException) as e:
                if self.conn:
                    self.conn.close()
                self.conn = None
                if not again:
                    raise Fail(1, f"no answer from the robot at {self.host}: {e}")
                continue
            if resp.status != 200:
                raise Fail(1, f"{path}: {resp.status} {data.strip()}")
            return data

    def cmd(self, line):
        self._http("POST", "/cmd", line.encode(), {"X-Bala2": "1"})

    def sample(self):
        """Read /state, and move our own pose (x, y) by what the wheels did."""
        s = json.loads(self._http("GET", "/state"))
        ses, old = self.ses, self.ses.get("last")
        if s["state"] != "balancing":
            ses["last"] = None
        else:
            if not old or s["up"] < old["up"]:
                # The robot counts its place and heading from each start.
                self.fresh = bool(ses["points"] or len(ses["path"]) > 1)
                ses.update(pose=[0.0, 0.0], points=[], path=[[0.0, 0.0]])
            else:
                d = s["pos"] - old["pos"]
                mid = math.radians((s["heading"] + old["heading"]) / 2)
                ses["pose"][0] += d * math.cos(mid)
                ses["pose"][1] += d * math.sin(mid)
            ses["last"] = {"pos": s["pos"], "heading": s["heading"], "up": s["up"]}
        self.last = s
        return s

    def ready(self, p):
        """The state, or a Fail if the robot cannot move now."""
        s = self.sample()
        if s["state"] != "balancing":
            raise Fail(2, f"the robot is not balancing: {s['state']}, {s['reason']}")
        if s["battery"] < p.min_battery and not s["charging"]:
            raise Fail(2, f"battery at {s['battery']}%, under min_battery")
        return s

    def pose(self):
        x, y = self.ses["pose"]
        return {"x": round(x, 3), "y": round(y, 3), "heading": round(self.last["heading"], 1)}

    def mark(self):
        x, y = self.ses["pose"]
        px, py = self.ses["path"][-1]
        if math.hypot(x - px, y - py) > 0.03:
            self.ses["path"].append([round(x, 3), round(y, 3)])

    def close(self):
        self.ses["path"] = self.ses["path"][-2000:]
        self.ses["points"] = self.ses["points"][-5000:]
        store("session.json", self.ses)


def motion(bot, p, v, w, seconds, check=None):
    """Drive at v (m/s) and w (deg/s) for `seconds`, then wait until the robot
    is still. The robot times each order, so a slow link does not change how
    far it goes. `check(sample, dt)` may return a reason to end early.
    Returns (samples, reason or None)."""
    period = 1.0 / p.poll_hz
    samples, why = [], None
    t0 = time.monotonic()
    next_send, last_t = 0.0, 0.0
    sending = seconds > 0.02
    try:
        while True:
            t = time.monotonic() - t0
            if sending and t >= next_send:
                left = seconds - t
                if left > 0.02:
                    bot.cmd(f"drive {v:.3f} {w:.1f} {min(p.order_s, left):.2f}")
                sending = left > p.order_s  # the last order covers the rest
                next_send = t + p.refresh_s
            s = bot.sample()
            s["t"] = round(t, 2)
            samples.append(s)
            if s["state"] != "balancing":
                raise Fail(2, f"the robot stopped balancing: {s['reason']}", pose=bot.pose())
            dt, last_t = t - last_t, t
            if why is None and check:  # also while it settles: a held robot shows late
                why = check(s, dt)
                if why:
                    bot.cmd("drive 0 0")
                    sending, seconds = False, t
            if t >= seconds:
                # The robot sways all the time, so look at means over the last settle_s.
                win = [x for x in samples if x["t"] > t - p.settle_s and x["t"] >= seconds]
                calm = (t - seconds >= p.settle_s and len(win) >= 3 and
                        all(abs(x["drive"][0]) < 0.005 and abs(x["drive"][1]) < 0.5 for x in win) and
                        abs(statistics.fmean(x["speed"] for x in win)) < p.settle_speed and
                        (why or "goal" not in s or abs(statistics.fmean(
                            x["pos"] - x["goal"][0] for x in win)) < p.settle_off) and
                        abs(statistics.fmean(x["yaw"] for x in win)) < p.settle_yaw)
                if calm or t > seconds + p.settle_timeout:
                    break
            time.sleep(max(0.0, period - (time.monotonic() - t0 - t)))
    except KeyboardInterrupt:
        bot.cmd("drive 0 0")
        raise
    bot.mark()
    return samples, why


def turn_by(bot, p, angle, rate=None):
    """Turn `angle` degrees from the heading now. Returns the heading error left."""
    target = bot.last["heading"] + angle
    rate = min(abs(rate or p.turn_rate), 90) or 1
    if abs(angle) > 0.5:
        motion(bot, p, 0, math.copysign(rate, angle), abs(angle) / rate)
    tries = 0
    while abs(target - bot.last["heading"]) > p.turn_tol and tries < p.turn_tries:
        err = target - bot.last["heading"]
        motion(bot, p, 0, math.copysign(p.turn_fix_rate, err), abs(err) / p.turn_fix_rate)
        tries += 1
    return round(target - bot.last["heading"], 1), tries


def warnings(s, p):
    out = []
    if s["state"] != "balancing":
        out.append(f"not balancing: {s['reason']}")
    if s["tof"] == -2:
        out.append("no distance sensor: forward is blind")
    if s["battery"] < p.min_battery and not s["charging"]:
        out.append("battery low")
    if s["rssi"] < p.min_rssi:
        out.append("weak Wi-Fi")
    if s["i2c_err"]:
        out.append(f"{s['i2c_err']} I2C errors since the start")
    if "link lost" in s["reason"]:
        out.append("no UDP heartbeat (normal for this tool; the servo stays stopped)")
    return out


def c_status(bot, p, a):
    s = bot.sample()
    if a.raw:
        return s
    keys = ("state", "reason", "tilt", "setpoint", "speed", "pos", "heading", "yaw", "drive", "goal",
            "tof", "tof_slows", "battery", "charging", "acc", "gyro", "temp", "enc", "u",
            "rssi", "i2c_err", "heap", "up", "fw")
    out = {k: s[k] for k in keys if k in s}
    out["pose"] = bot.pose() if s["state"] == "balancing" else None
    out["tof_zone"] = {k: s["vars"].get(k) for k in ("tz", "tzr", "tcol", "trow", "ts", "tw")}
    out["warnings"] = warnings(s, p)
    return out


def c_watch(bot, p, a):
    rows, t0 = [], time.monotonic()
    while time.monotonic() - t0 < a.seconds:
        rows.append(bot.sample())
        time.sleep(1.0 / p.poll_hz)
    out = {"samples": len(rows), "state": rows[-1]["state"], "reason": rows[-1]["reason"]}
    for k in ("tilt", "speed", "pos", "heading", "yaw", "tof", "u", "rssi"):
        vals = [r[k] for r in rows if not (k == "tof" and r[k] < 0)]
        if vals:
            out[k] = {"min": min(vals), "max": max(vals), "mean": round(statistics.fmean(vals), 3),
                      "sd": round(statistics.pstdev(vals), 3)}
    out["tof_nothing"] = sum(r["tof"] == -1 for r in rows)
    out["warnings"] = warnings(rows[-1], p)
    return out


def c_turn(bot, p, a):
    s = bot.ready(p)
    if (a.angle is None) == (a.to is None):
        raise Fail(1, "give an angle, or --to a heading")
    angle = a.angle if a.to is None else norm(a.to - s["heading"])
    h0 = s["heading"]
    err, tries = turn_by(bot, p, angle, a.rate)
    return {"asked": round(angle, 1), "turned": round(bot.last["heading"] - h0, 1), "error": err,
            "extra_turns": tries, "tof": bot.last["tof"], "pose": bot.pose()}


def c_forward(bot, p, a):
    s = bot.ready(p)
    dist, sign = a.distance, 1 if a.distance >= 0 else -1
    speed = min(abs(a.speed or p.fwd_speed), s["vars"].get("dv", 0.25)) or 0.05
    note = None
    if (sign < 0 or s["tof"] == -2) and abs(dist) > p.blind_max:
        dist = sign * p.blind_max
        note = "nothing looks that way: cut to blind_max"
    run = {"goal": 0.0, "lag": 0.0, "slow": 0.0, "speeds": []}
    pos0, tilt0 = s["pos"], s["tilt"]

    def check(st, dt):
        run["goal"] += st["drive"][0] * dt
        if "goal" in st:  # the robot says where its goal is
            lag = sign * (st["goal"][0] - st["pos"])
        else:  # older firmware: count it ourselves, from 0
            lag = sign * (run["goal"] - (st["pos"] - pos0))
        run["lag"] = max(run["lag"], lag)
        # The robot's own brake: no speed left, or the stop line (`ts`) is here.
        near = 0 <= st["tof"] < (st["vars"].get("ts", 0.2) + p.tof_stop_m) * 1000
        braked = st["tof_slows"] and (abs(st["drive"][0]) < 0.01 or near)
        run["slow"] = run["slow"] + dt if braked else 0.0
        run["speeds"] = (run["speeds"] + [sign * st["speed"]])[-max(3, int(p.poll_hz)):]
        held = statistics.fmean(run["speeds"]) < p.stall_speed and st["t"] > 1.5
        if lag > p.stall_hard or lag > p.stall_lag and held:
            return "stall"
        if abs(st["tilt"] - tilt0) > p.tilt_abort:
            return "tilt"
        if run["slow"] > 0.5:
            return "tof"
        if st["rssi"] < p.min_rssi:
            return "rssi"
        return None

    # The robot holds its place loosely, so it may stand off its goal already.
    # Ask for the distance from where it is, not from where its goal is.
    off = s["goal"][0] - s["pos"] if "goal" in s else 0.0
    todo = dist - off
    if todo * sign <= 0.005:
        todo = 0.0  # its goal is already that far ahead: just wait for it
    _, why = motion(bot, p, sign * speed, 0, abs(todo) / speed, check)
    last = bot.last
    out = {"asked": round(dist, 3), "moved": round(last["pos"] - pos0, 3),
           "stopped": why, "max_lag": round(run["lag"], 3), "tof": last["tof"],
           "pose": bot.pose()}
    if "goal" in last:
        out["off_goal"] = round(last["pos"] - last["goal"][0], 3)
    if note:
        out["note"] = note
    if why == "stall":
        # Its goal is ahead of it, behind the thing it pushes. Bring the goal back.
        back = run["lag"] + p.stall_back
        motion(bot, p, -sign * speed, 0, back / speed)
        out.update(backed=round(back, 3), pose=bot.pose())
    if why:
        raise Fail(3, {"stall": "something the sensor did not see holds the robot",
                       "tilt": "the robot leaned too far", "tof": "something is ahead",
                       "rssi": "the Wi-Fi got weak"}[why], **out)
    return out


def c_scan(bot, p, a):
    s = bot.ready(p)
    if s["tof"] == -2:
        raise Fail(1, "no distance sensor")
    arc = min(max(a.arc, p.scan_bin), 360)
    full = arc >= 360
    h0 = s["heading"]
    if not full:
        turn_by(bot, p, -arc / 2)
    rate = min(abs(a.rate or p.scan_rate), 90) or 1
    samples, _ = motion(bot, p, 0, rate, arc / rate)
    turn_by(bot, p, h0 + (360 if full else 0) - bot.last["heading"])

    usual = statistics.median(x["tilt"] for x in samples)
    bins, dropped = {}, 0
    for x in samples:
        if abs(x["tilt"] - usual) > p.scan_tilt_tol:
            dropped += 1
            continue
        rel = x["heading"] - x["yaw"] * p.scan_lag_s - h0
        rel = norm(rel) if full else rel
        if not full and abs(rel) > arc / 2 + p.scan_bin:
            continue
        i = math.floor(rel / p.scan_bin)
        mm = p.free_mm if x["tof"] < 0 else min(x["tof"], p.free_mm)
        b = bins.setdefault(i, [mm, 0])
        b[0], b[1] = min(b[0], mm), b[1] + 1
    half = math.ceil((180 if full else arc / 2) / p.scan_bin)
    idx = list(range(-half, half))
    dist = [bins[i][0] if i in bins else None for i in idx]
    center = lambda i: (i + 0.5) * p.scan_bin

    # Free sectors: runs of directions with nothing closer than clear_mm.
    free = [d is not None and d >= p.clear_mm for d in dist]
    runs, k, n = [], 0, len(idx)
    while k < n:
        if free[k]:
            j = k
            while j + 1 < n and free[j + 1]:
                j += 1
            runs.append([k, j])
            k = j + 1
        else:
            k += 1
    if full and len(runs) > 1 and runs[0][0] == 0 and runs[-1][1] == n - 1:
        runs[0][0] = runs.pop()[0] - n  # the run goes on across the back
    need = 2 * math.degrees(math.atan((p.robot_w_mm / 2 + p.margin_mm) / p.clear_mm))
    turned = bot.last["heading"] - h0 - (360 if full else 0)
    sectors = []
    for k, j in runs:
        width = (j - k + 1) * p.scan_bin
        mid = norm(idx[0] * p.scan_bin + k * p.scan_bin + width / 2)
        sectors.append({"center": round(mid, 1), "width": width,
                        "min_mm": min(dist[m % n] for m in range(k, j + 1)),
                        "passable": width >= need, "turn": round(norm(mid - turned), 1)})
    ok = [x for x in sectors if x["passable"]]
    best = max(ok, key=lambda x: (x["min_mm"], x["width"]), default=None)
    hits = [(d, center(i)) for i, d in zip(idx, dist) if d is not None and d < p.free_mm]
    x0, y0 = bot.ses["pose"]
    for d, c in hits:
        r, th = (d + p.tof_offset_mm) / 1000.0, math.radians(h0 + c)
        bot.ses["points"].append([round(x0 + r * math.cos(th), 3), round(y0 + r * math.sin(th), 3)])
    return {"arc": arc, "rate": rate, "samples": len(samples), "dropped_for_tilt": dropped,
            "bins": [[round(center(i), 1), d, bins[i][1] if i in bins else 0]
                     for i, d in zip(idx, dist)],
            "bins_are": "[degrees from the heading at the start, mm (null: not seen), samples]",
            "nearest": min(({"mm": d, "at": round(c, 1)} for d, c in hits),
                           key=lambda x: x["mm"], default=None),
            "need_deg": round(need, 1), "sectors": sectors, "best": best, "pose": bot.pose()}


def c_look(bot, p, a):
    s = bot.sample()
    if s["tof"] == -2:
        raise Fail(1, "no distance sensor")
    row0, rows0, out = s["vars"]["trow"], s["vars"].get("tzr"), []
    try:
        if rows0:
            bot.cmd("tzr 0")  # a narrow square, so each row looks one way
        row = int(p.look_from)
        while row <= p.look_to:
            bot.cmd(f"trow {row}")
            time.sleep(p.tof_settle_s)
            got = []
            for _ in range(3):
                got.append(bot.sample()["tof"])
                time.sleep(0.1)
            out.append([row, round(p.look_sign * (row - 7.5) * ZONE_DEG, 1),
                        int(statistics.median(got))])
            row += max(1, int(p.look_step))
    finally:
        bot.cmd(f"trow {row0}")
        if rows0:
            bot.cmd(f"tzr {rows0}")
    return {"rows": out, "rows_are": "[row, degrees to the left, mm (-1: nothing seen)]",
            "note": "which side is left depends on the mount: check look_sign once with a box",
            "zone": {k: s["vars"].get(k) for k in ("tz", "tcol")}}


def c_halt(bot, p, a):
    bot.cmd("drive 0 0")
    s = bot.sample()
    return {"state": s["state"], "reason": s["reason"], "drive": s["drive"]}


def c_map(bot, p, a):
    s = bot.sample()
    ses, cell = bot.ses, p.map_cell
    x, y = ses["pose"]
    if a.points:
        return {"pose": bot.pose(), "points": ses["points"], "path": ses["path"]}
    every = ses["points"] + ses["path"] + [[x, y]]
    to = lambda v: int(math.floor(v / cell + 0.5))
    cx = [to(q[0]) for q in every]
    cy = [to(q[1]) for q in every]
    x1, x2, y1, y2 = min(cx), max(cx), min(cy), max(cy)
    if x2 - x1 > 100 or y2 - y1 > 60:
        raise Fail(1, "the map is too big to draw: raise map_cell, or use --points")
    grid = [[" "] * (y2 - y1 + 1) for _ in range(x2 - x1 + 1)]
    for q in ses["path"]:
        grid[to(q[0]) - x1][y2 - to(q[1])] = "."
    for q in ses["points"]:
        grid[to(q[0]) - x1][y2 - to(q[1])] = "#"
    grid[to(0) - x1][y2 - to(0)] = "o"
    grid[to(x) - x1][y2 - to(y)] = "^<v>"[int(((s["heading"] + 45) % 360) // 90)]
    # Forward at the start (x) is up, left (y) is to the left.
    rows = ["".join(r) for r in reversed(grid)]
    return {"pose": bot.pose(), "cell_m": cell, "points": len(ses["points"]),
            "legend": "# seen, . path, o start, arrow robot; up is forward at the start",
            "map": rows}


def c_origin(bot, p, a):
    bot.sample()
    bot.ses.update(pose=[0.0, 0.0], points=[], path=[[0.0, 0.0]])
    return {"pose": [0.0, 0.0], "note": "map cleared; the heading still counts from the robot's start"}


def c_params(bot, p, a):
    out = {"tool": {k: {"value": getattr(p, k), "start": v, "what": w}
                    for k, (v, w) in PARAMS.items()}}
    try:
        out["robot"] = bot.sample()["vars"]
        out["robot_note"] = "`?` in bala2_link.py and the README say what each one is"
    except Fail as e:
        out["robot"] = str(e)
    return out


def c_set(bot, p, a):
    if len(a.pairs) % 2 or not a.pairs:
        raise Fail(1, "give pairs: name value [name value ...]")
    out = {}
    for name, value in zip(a.pairs[::2], a.pairs[1::2]):
        if name in PARAMS:
            if value == "start":
                p.saved.pop(name, None)
                out[name] = PARAMS[name][0]
            else:
                p.saved[name] = out[name] = float(value)
            store("params.json", p.saved)
            continue
        before = bot.sample()["vars"]
        if name not in before:
            raise Fail(1, f"{name}: not a value of this tool or of the robot (see `params`)")
        bot.cmd(f"{name} {float(value):g}")
        time.sleep(0.1)
        out[name] = bot.sample()["vars"][name]  # the robot keeps it inside its limits
    return {"set": out, "note": "robot values last until it restarts; `cmd save` keeps them"}


def c_cmd(bot, p, a):
    if a.line.split()[:1] in (["save"], ["defaults"]) and bot.sample()["state"] == "balancing":
        raise Fail(2, f"the robot refuses `{a.line}` while it balances: lay it down first")
    bot.cmd(a.line)
    time.sleep(0.1)
    s = bot.sample()
    return {"sent": a.line, "state": s["state"], "reason": s["reason"], "vars": s["vars"]}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default=os.environ.get("BALA2_HOST"),
                    help="robot name or address, with :port if it is not 80")
    ap.add_argument("--set", action="append", metavar="NAME=VALUE",
                    help="one of this tool's values, for this call only")
    sub = ap.add_subparsers(dest="command", required=True)
    both = argparse.ArgumentParser(add_help=False)  # so they also work after the command
    both.add_argument("--host", default=argparse.SUPPRESS, help=argparse.SUPPRESS)
    both.add_argument("--set", action="append", default=argparse.SUPPRESS, help=argparse.SUPPRESS)
    add = lambda name, fn: (sp := sub.add_parser(name, parents=[both]), sp.set_defaults(fn=fn))[0]
    add("status", c_status).add_argument("--raw", action="store_true", help="all of /state")
    add("watch", c_watch).add_argument("seconds", type=float, nargs="?", default=3)
    sp = add("turn", c_turn)
    sp.add_argument("angle", type=float, nargs="?", help="degrees, left is positive")
    sp.add_argument("--to", type=float, help="turn to this heading")
    sp.add_argument("--rate", type=float, help="deg/s")
    sp = add("forward", c_forward)
    sp.add_argument("distance", type=float, help="metres, negative goes back")
    sp.add_argument("--speed", type=float, help="m/s")
    sp = add("scan", c_scan)
    sp.add_argument("--arc", type=float, default=360, help="degrees, centred ahead")
    sp.add_argument("--rate", type=float, help="deg/s")
    add("look", c_look)
    add("halt", c_halt)
    add("map", c_map).add_argument("--points", action="store_true", help="the numbers, no drawing")
    add("origin", c_origin)
    add("params", c_params)
    add("set", c_set).add_argument("pairs", nargs="*", help="name value; `start` puts ours back")
    add("cmd", c_cmd).add_argument("line", help='a command line for the robot, like "servo 1500"')
    a = ap.parse_args()

    bot, code = None, 0
    try:
        p = P(a.set)
        bot = Robot(a.host)
        out = a.fn(bot, p, a)
    except Fail as e:
        out, code = e.out, e.code
    except KeyboardInterrupt:
        out, code = {"error": "interrupted; sent drive 0 0"}, 1
    if bot:
        if bot.fresh:
            out["new_run"] = "the robot started again: the map and the pose begin here"
        bot.close()
    if "map" in out and sys.stdout.isatty():
        print("\n".join(out.pop("map")))
    print(json.dumps(out, separators=(",", ":") if not sys.stdout.isatty() else None))
    sys.exit(code)


if __name__ == "__main__":
    main()
