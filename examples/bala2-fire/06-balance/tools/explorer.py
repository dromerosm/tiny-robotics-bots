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
    explorer.py goto 1.0 0.5         drive to a place, through cells the map says are free
    explorer.py explore --steps 3    scan, go to the nearest edge of the map, and again
    explorer.py map                  the map so far: taken, free and not seen
    explorer.py near                 a slow turn with the narrow zone: the map of 0.5 m around, and its gaps
    explorer.py gaps                 the gaps again, with what a person said about them
    explorer.py measure              sweep ahead slowly: the nearest thing, its width, and the room each side
    explorer.py approach 2           go toward gap 2 and stop 0.3 m before it, to ask a person
    explorer.py mark 2 real          a person saw gap 2 (or `false`: something closes it)
    explorer.py pass 2               go through gap 2, only if it is marked real
    explorer.py calibrate spin       measure the model: degrees (spin), metres (wall), drift (still)
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

import navmap as nm
from bala2_link import find_robot

# For tests: sim_robot.py --speed 5 runs its clock 5 times faster, and so must we.
SCALE = float(os.environ.get("BALA2_TIME_SCALE") or 1)


def now():
    return time.monotonic() * SCALE


def nap(seconds):
    time.sleep(max(0.0, seconds) / SCALE)


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
    "robot_w_mm": (90, "mm, width the robot needs to pass (ours: 90, wheel edge to wheel edge)"),
    "margin_mm": (50, "mm of room on each side"),
    "tof_offset_mm": (0, "mm from the middle of the wheels to the sensor"),
    "tof_settle_s": (0.3, "s to wait after the sensor's zone changes"),
    "look_from": (2, "first zone row of `look`"),
    "look_to": (13, "last zone row of `look`"),
    "look_step": (2, "rows between two looks"),
    "look_sign": (1, "1 or -1: which way a higher row looks (depends on the mount)"),
    "map_cell": (0.1, "m, one cell of the map and one character of `map`"),
    "odo_dist": (1.0, "the robot's model: real metres for each metre it reports (see `calibrate wall`)"),
    "odo_head": (1.0, "the robot's model: real degrees for each degree it reports (see `calibrate spin`)"),
    "beam_deg": (0, "deg, width of the sensor's cone for the map; 0 takes it from the zone (tz, tzr)"),
    "reach_mm": (1500, "mm, how far a reading clears the map; \"nothing\" clears this far too"),
    "near_m": (0.5, "m, how far around the robot `near` maps"),
    "near_cell": (0.05, "m, one cell of the near map"),
    "near_rate": (15, "deg/s for `near` (slow: the narrow zone needs many readings)"),
    "near_step": (2, "deg between two directions when `gaps` looks for ways out"),
    "measure_arc": (90, "deg, the sector ahead that `measure` sweeps"),
    "measure_rate": (10, "deg/s for `measure` (slow: many readings with the narrow zone)"),
    "measure_rows": (8, "zone rows across for `measure`: 0 = the narrow square (tz); 8 sees dark things farther, and errs wide"),
    "edge_shift": (0.0, "share of the zone's width to move an edge inward (0: the real sensor; 0.5 for a nearest-in-cone sensor)"),
    "edge_mm": (120, "mm, a jump this big between two readings is the edge of a thing"),
    "approach_m": (0.3, "m from the middle of a gap where `approach` stops"),
    "approach_leg": (0.1, "m, one stretch of `approach`"),
    "pass_m": (0.3, "m past the middle of a gap where `pass` ends"),
    "goto_tol": (0.10, "m, `goto` is there when it is this close"),
    "goto_tries": (8, "stretches `goto` may drive before it gives up"),
    "leg_max": (0.6, "m, the longest stretch `goto` drives in one go"),
    "frontier_min": (3, "cells, `explore` skips edges of the map smaller than this"),
    "fit_deg": (12, "degrees, most a full scan may correct the heading by the walls; 0: off"),
    "fit_step": (3, "degrees, most one scan corrects"),
    "fit_agree": (3, "walls that must agree before their direction is the reference"),
    "fit_flank": (22, "degrees past the flat stretch of a wall that must fit, on each side"),
    "fit_miss": (0.04, "share of the distance the scan may be off the shape of a wall"),
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
    def __init__(self, host, p):
        self.p = p
        name, _, port = (host or "").partition(":")
        self.host = find_robot(name or None) + (":" + port if port else "")
        self.conn = None
        self.ses = load("session.json", {})
        self.ses.setdefault("pose", [0.0, 0.0])
        self.ses.setdefault("points", [])
        self.ses.setdefault("path", [[0.0, 0.0]])
        self.ses.setdefault("grid", {})
        self.ses.setdefault("tried", {})
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
                self.fresh = bool(ses["grid"] or len(ses["path"]) > 1)
                ses.update(pose=[0.0, 0.0], points=[], path=[[0.0, 0.0]], grid={}, tried={},
                           h_fix=0.0, wall_ref=None, wall_seen=[], near=None, gaps=[])
            else:
                # The model: what the robot reports, times what one unit really is.
                d = (s["pos"] - old["pos"]) * self.p.odo_dist
                mid = math.radians((s["heading"] + old["heading"]) / 2 * self.p.odo_head +
                                   ses.get("h_fix", 0.0))
                ses["pose"][0] += d * math.cos(mid)
                ses["pose"][1] += d * math.sin(mid)
            ses["last"] = {"pos": s["pos"], "heading": s["heading"], "up": s["up"]}
            s["px"], s["py"] = ses["pose"]
        # The real heading: by the model, plus what the map has corrected (fit_scan).
        s["H"] = s["heading"] * self.p.odo_head + ses.get("h_fix", 0.0)
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
        return {"x": round(x, 3), "y": round(y, 3), "heading": round(self.last["H"], 1)}

    def mark(self):
        x, y = self.ses["pose"]
        px, py = self.ses["path"][-1]
        if math.hypot(x - px, y - py) > 0.03:
            self.ses["path"].append([round(x, 3), round(y, 3)])

    def close(self):
        self.ses["path"] = self.ses["path"][-2000:]
        self.ses["points"] = self.ses["points"][-5000:]
        self.ses["grid"] = {k: v for k, v in self.ses["grid"].items() if v}
        store("session.json", self.ses)


def motion(bot, p, v, w, seconds, check=None):
    """Drive at v (m/s) and w (deg/s) for `seconds`, then wait until the robot
    is still. The robot times each order, so a slow link does not change how
    far it goes. `check(sample, dt)` may return a reason to end early.
    Returns (samples, reason or None)."""
    period = 1.0 / p.poll_hz
    samples, why = [], None
    t0 = now()
    next_send, last_t = 0.0, 0.0
    sending = seconds > 0.02
    try:
        while True:
            t = now() - t0
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
            nap(max(0.0, period - (now() - t0 - t)))
    except KeyboardInterrupt:
        bot.cmd("drive 0 0")
        raise
    bot.mark()
    return samples, why


def turn_by(bot, p, angle, rate=None):
    """Turn `angle` real degrees from the heading now. Returns the error left."""
    k = p.odo_head or 1.0
    target = bot.last["heading"] + angle / k  # in the degrees the robot reports
    rate = min(abs(rate or p.turn_rate), 90) or 1
    if abs(angle) > 0.5:
        motion(bot, p, 0, math.copysign(rate, angle), abs(angle / k) / rate)
    tries = 0
    while abs(target - bot.last["heading"]) * k > p.turn_tol and tries < p.turn_tries:
        err = target - bot.last["heading"]
        motion(bot, p, 0, math.copysign(p.turn_fix_rate, err), abs(err) / p.turn_fix_rate)
        tries += 1
    return round((target - bot.last["heading"]) * k, 1), tries


def beam(bot, p):
    v = bot.last["vars"]
    return p.beam_deg or (v.get("tzr") or v.get("tz", 4)) * ZONE_DEG


def readings(p, samples):
    """The samples as the map wants them: (x, y, bearing, m or None), and the samples used."""
    got = [s for s in samples if "px" in s and s["tof"] != -2]
    if not got:
        return [], []
    usual = statistics.median(s["tilt"] for s in got)
    rs = []
    for s in got:
        if abs(s["tilt"] - usual) > p.scan_tilt_tol:
            continue  # it leaned: the sensor may have seen the floor
        b = s["H"] - s["yaw"] * p.scan_lag_s
        off, th = p.tof_offset_mm / 1000.0, math.radians(b)
        rs.append((s["px"] + off * math.cos(th), s["py"] + off * math.sin(th), b,
                   None if s["tof"] < 0 else s["tof"] / 1000.0))
    return rs, got


def fit(bot, p, samples):
    """Correct the heading with the walls. The walls of a room stand at right
    angles to each other, so the directions that look straight at a wall are
    all the same, counted in turns of 90 degrees. The first scan sets that
    direction; each later scan says how far the heading has drifted from it.
    Returns what it found, or None."""
    if p.fit_deg <= 0:
        return None
    rs, _ = readings(p, samples)
    walls = nm.wall_bearings([(r[2], r[3]) for r in rs], beam(bot, p), p.fit_flank, p.fit_miss)
    if not walls:
        return {"walls": 0, "applied": False}
    ses = bot.ses
    ref = ses.get("wall_ref")
    if ref is None:
        # One wall is not enough to set the direction: a corner of a box can
        # look like a wall. Wait until three of them agree, over the first scans.
        seen = ses.setdefault("wall_seen", [])
        seen += [round(w[0] % 90, 2) for w in walls]
        away = lambda a, b: (a - b + 45) % 90 - 45
        group = max(([away(x, c) for x in seen if abs(away(x, c)) <= p.fit_step] for c in seen),
                    key=len)
        if len(group) < p.fit_agree:
            return {"walls": len(walls), "seen": len(seen), "applied": False}
        c = next(c for c in seen
                 if len([x for x in seen if abs(away(x, c)) <= p.fit_step]) == len(group))
        ses["wall_ref"] = c + sum(group) / len(group)
        return {"walls": len(walls), "wall_ref": round(ses["wall_ref"], 1), "applied": False}
    off = sorted((w[0] - ref + 45) % 90 - 45 for w in walls)
    off = [o for o in off if abs(o) <= p.fit_deg]
    if not off:
        return {"walls": len(walls), "applied": False}
    # One scan corrects only a little: the drift of one scan is small, and a
    # corner of a box can look like a wall. A real error goes away over some scans.
    turn = max(-p.fit_step, min(p.fit_step, -statistics.median(off)))
    ses["h_fix"] = ses.get("h_fix", 0.0) + turn
    for s in samples + [bot.last]:
        if not s.get("fitted"):
            s["H"] += turn
            s["fitted"] = True
    return {"walls": len(walls), "used": len(off), "off": [round(o, 1) for o in off],
            "turn": round(turn, 2), "applied": True}


def learn(bot, p, samples, arcs):
    """Put what the sensor read on the way into the map."""
    grid, cell = bot.ses["grid"], p.map_cell
    rs, got = readings(p, samples)
    if not got:
        return 0, 0
    out = nm.add_readings(grid, cell, rs, beam(bot, p), p.reach_mm / 1000.0, arcs)
    for s in got[::4]:
        nm.add_robot(grid, cell, s["px"], s["py"], p.robot_w_mm / 2000.0)
    return out


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
    rows, t0 = [], now()
    while now() - t0 < a.seconds:
        rows.append(bot.sample())
        nap(1.0 / p.poll_hz)
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
    angle = a.angle if a.to is None else norm(a.to - s["H"])
    h0 = s["H"]
    err, tries = turn_by(bot, p, angle, a.rate)
    return {"asked": round(angle, 1), "turned": round(bot.last["H"] - h0, 1), "error": err,
            "extra_turns": tries, "tof": bot.last["tof"], "pose": bot.pose()}


def do_forward(bot, p, dist, speed=None, blind_ok=False):
    """Go `dist` real metres. Returns the result; "stopped" says why it ended early."""
    s = bot.last
    k = p.odo_dist or 1.0
    sign = 1 if dist >= 0 else -1
    speed = min(abs(speed or p.fwd_speed), s["vars"].get("dv", 0.25)) or 0.05
    note = None
    if (sign < 0 or s["tof"] == -2) and abs(dist) > p.blind_max and not blind_ok:
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
    todo = dist / k - off  # in the metres the robot reports
    if todo * sign <= 0.005:
        todo = 0.0  # its goal is already that far ahead: just wait for it
    samples, why = motion(bot, p, sign * speed, 0, abs(todo) / speed, check)
    last = bot.last
    out = {"asked": round(dist, 3), "moved": round((last["pos"] - pos0) * k, 3),
           "stopped": why, "max_lag": round(run["lag"], 3), "tof": last["tof"],
           "pose": bot.pose()}
    if "goal" in last:
        out["off_goal"] = round(last["pos"] - last["goal"][0], 3)
    if note:
        out["note"] = note
    learn(bot, p, samples, arcs=False)
    if why == "stall":
        # Something the sensor missed. Put it in the map, right in front of the robot.
        x, y = bot.ses["pose"]
        nm.add_bump(bot.ses["grid"], p.map_cell, x, y, last["H"] + (0 if sign > 0 else 180),
                    p.robot_w_mm / 2000.0 + p.map_cell / 2)
        # Its goal is ahead of it, behind the thing it pushes. Bring the goal back.
        back = run["lag"] + p.stall_back
        motion(bot, p, -sign * speed, 0, back / speed)
        out.update(backed=round(back, 3), pose=bot.pose())
    return out


WHY = {"stall": "something the sensor did not see holds the robot",
       "tilt": "the robot leaned too far", "tof": "something is ahead",
       "rssi": "the Wi-Fi got weak"}


def c_forward(bot, p, a):
    bot.ready(p)
    out = do_forward(bot, p, a.distance, a.speed)
    if out["stopped"]:
        raise Fail(3, WHY[out["stopped"]], **out)
    return out


def do_scan(bot, p, arc=360, rate=None):
    """Turn on the spot, read the sensor all the way, and put it in the map."""
    s = bot.last
    if s["tof"] == -2:
        raise Fail(1, "no distance sensor")
    arc = min(max(arc, p.scan_bin), 360)
    full = arc >= 360
    k = p.odo_head or 1.0
    h0 = s["H"]
    if not full:
        turn_by(bot, p, -arc / 2)
    rate = min(abs(rate or p.scan_rate), 90) or 1
    samples, _ = motion(bot, p, 0, rate, arc / k / rate)
    turn_by(bot, p, h0 + (360 if full else 0) - bot.last["H"])
    fitted = fit(bot, p, samples) if full else None
    if fitted and fitted["applied"]:
        h0 += fitted["turn"]  # the start of the scan was off by this too
    free, taken = learn(bot, p, samples, arcs=True)

    usual = statistics.median(x["tilt"] for x in samples)
    bins, dropped = {}, 0
    for x in samples:
        if abs(x["tilt"] - usual) > p.scan_tilt_tol:
            dropped += 1
            continue
        rel = x["H"] - x["yaw"] * p.scan_lag_s - h0
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
    free_dir = [d is not None and d >= p.clear_mm for d in dist]
    runs, j0, n = [], 0, len(idx)
    while j0 < n:
        if free_dir[j0]:
            j = j0
            while j + 1 < n and free_dir[j + 1]:
                j += 1
            runs.append([j0, j])
            j0 = j + 1
        else:
            j0 += 1
    if full and len(runs) > 1 and runs[0][0] == 0 and runs[-1][1] == n - 1:
        runs[0][0] = runs.pop()[0] - n  # the run goes on across the back
    need = 2 * math.degrees(math.atan((p.robot_w_mm / 2 + p.margin_mm) / p.clear_mm))
    turned = bot.last["H"] - h0 - (360 if full else 0)
    sectors = []
    for a0, a1 in runs:
        width = (a1 - a0 + 1) * p.scan_bin
        mid = norm(idx[0] * p.scan_bin + a0 * p.scan_bin + width / 2)
        sectors.append({"center": round(mid, 1), "width": width,
                        "min_mm": min(dist[m % n] for m in range(a0, a1 + 1)),
                        "passable": width >= need, "turn": round(norm(mid - turned), 1)})
    ok = [x for x in sectors if x["passable"]]
    best = max(ok, key=lambda x: (x["min_mm"], x["width"]), default=None)
    hits = [(d, center(i)) for i, d in zip(idx, dist) if d is not None and d < p.free_mm]
    return {"arc": arc, "rate": rate, "samples": len(samples), "dropped_for_tilt": dropped,
            "bins": [[round(center(i), 1), d, bins[i][1] if i in bins else 0]
                     for i, d in zip(idx, dist)],
            "bins_are": "[degrees from the heading at the start, mm (null: not seen), samples]",
            "nearest": min(({"mm": d, "at": round(c, 1)} for d, c in hits),
                           key=lambda x: x["mm"], default=None),
            "need_deg": round(need, 1), "sectors": sectors, "best": best,
            "map": {"free": free, "taken": taken}, "fit": fitted, "pose": bot.pose()}


def c_scan(bot, p, a):
    bot.ready(p)
    out = do_scan(bot, p, a.arc, a.rate)
    out["map_cells"] = out.pop("map")  # "map" is the drawing, in `map`
    return out


def keep_out(p):
    return (p.robot_w_mm / 2 + p.margin_mm) / 1000.0


def no_go(bot, p, start):
    """Cells the robot's middle must stay out of. Never the ones it stands on."""
    blocked = nm.blocked_cells(bot.ses["grid"], p.map_cell, keep_out(p))
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            blocked.discard((start[0] + dx, start[1] + dy))
    return blocked


def do_goto(bot, p, gx, gy, legs):
    """Drive to (gx, gy) through cells the map says are free. Returns how it ended:
    "arrived", "no way", "gave up", or why the robot had to stop."""
    grid, cell = bot.ses["grid"], p.map_cell
    for _ in range(int(p.goto_tries)):
        x, y = bot.ses["pose"]
        if math.hypot(gx - x, gy - y) <= p.goto_tol:
            return "arrived"
        nm.add_robot(grid, cell, x, y, p.robot_w_mm / 2000.0)
        start, goal = nm.cell_of(cell, x, y), nm.cell_of(cell, gx, gy)
        way = [goal] if start == goal else nm.route(grid, start, goal, no_go(bot, p, start))
        if not way:
            return "no way"
        tx, ty = (gx, gy) if way[0] == goal else (way[0][0] * cell, way[0][1] * cell)
        turn_by(bot, p, norm(math.degrees(math.atan2(ty - y, tx - x)) - bot.last["H"]))
        out = do_forward(bot, p, min(math.hypot(tx - x, ty - y), p.leg_max))
        legs.append({"to": [round(tx, 2), round(ty, 2)], "moved": out["moved"],
                     "stopped": out["stopped"]})
        if out["stopped"] == "tof":
            do_scan(bot, p)  # see what it is, then look for another way
        elif out["stopped"] in ("tilt", "rssi"):
            return out["stopped"]
    x, y = bot.ses["pose"]
    return "arrived" if math.hypot(gx - x, gy - y) <= p.goto_tol else "gave up"


def map_numbers(bot, p):
    grid = bot.ses["grid"]
    return {"free": sum(v <= nm.FREE for v in grid.values()),
            "taken": sum(v >= nm.OCC for v in grid.values()), "cell_m": p.map_cell}


def c_goto(bot, p, a):
    bot.ready(p)
    legs = []
    end = do_goto(bot, p, a.x, a.y, legs)
    out = {"asked": [a.x, a.y], "ended": end, "legs": legs, "pose": bot.pose()}
    if end != "arrived":
        raise Fail(3, f"goto ended: {end}", **out)
    return out


def c_explore(bot, p, a):
    """Scan, go to the nearest edge of what the map knows, and again."""
    bot.ready(p)
    grid, cell, tried = bot.ses["grid"], p.map_cell, bot.ses["tried"]
    steps, done = [], False
    for _ in range(a.steps):
        scan = do_scan(bot, p)
        x, y = bot.ses["pose"]
        start = nm.cell_of(cell, x, y)
        cost = nm.reach(grid, start, no_go(bot, p, start))
        edges = [g for g in nm.frontiers(grid, cost, int(p.frontier_min))
                 if tried.get(nm.key(g[1]), 0) < 2]
        if not edges:
            done = True
            break
        _, target, far = edges[0]
        tried[nm.key(target)] = tried.get(nm.key(target), 0) + 1
        legs = []
        end = do_goto(bot, p, target[0] * cell, target[1] * cell, legs)
        steps.append({"nearest": scan["nearest"], "fit": scan["fit"], "edges": len(edges),
                      "target": [round(target[0] * cell, 2), round(target[1] * cell, 2)],
                      "ended": end, "legs": len(legs),
                      "stops": [g["stopped"] for g in legs if g["stopped"]]})
        if end in ("tilt", "rssi"):
            break
    return {"steps": steps, "done": done, "map_cells": map_numbers(bot, p), "pose": bot.pose(),
            "note": "done: no edge of the map is left that the robot can get to"}


def slope(ts, vs):
    """Least squares slope of vs over ts: steadier than last minus first."""
    mt, mv = statistics.fmean(ts), statistics.fmean(vs)
    den = sum((x - mt) ** 2 for x in ts)
    return sum((x - mt) * (v - mv) for x, v in zip(ts, vs)) / den if den else 0.0


def average(bot, p, seconds=3.0):
    rows, t0 = [], now()
    while now() - t0 < seconds:
        rows.append(bot.sample())
        rows[-1]["t"] = now() - t0
        nap(1.0 / p.poll_hz)
    tof = [r["tof"] for r in rows if r["tof"] >= 0]
    return (statistics.fmean(r["pos"] for r in rows),
            statistics.fmean(tof) if len(tof) > len(rows) / 2 else None, rows)


def c_calibrate(bot, p, a):
    """Measure the robot's model with its own sensors."""
    s = bot.ready(p)
    if a.what == "still":
        # Standing: how much do the heading and the place drift by themselves?
        _, _, rows = average(bot, p, a.seconds)
        ts = [r["t"] for r in rows]
        out = {"seconds": a.seconds, "samples": len(rows),
               "heading_drift_deg_per_min": round(60 * slope(ts, [r["heading"] for r in rows]), 2),
               "pos_drift_m_per_min": round(60 * slope(ts, [r["pos"] for r in rows]), 3)}
        if "head" in rows[0]:
            for i, name in enumerate(("wheels", "gyro")):
                out[f"{name}_drift_deg_per_min"] = round(
                    60 * slope(ts, [r["head"][i] for r in rows]), 2)
        return out
    if s["tof"] == -2:
        raise Fail(1, "no distance sensor: it is the ruler for this")
    if a.what == "spin":
        # Turn more than once. The room repeats after one real turn, so the
        # distances say how many reported degrees one real turn is.
        h0 = s["H"]
        samples, _ = motion(bot, p, 0, p.scan_rate, a.turns * 360.0 / p.scan_rate)
        vals = [min(x["tof"], p.reach_mm * 2) if x["tof"] >= 0 else p.reach_mm * 2 for x in samples]
        period, sure = nm.turn_period([x["heading"] for x in samples], vals)
        turn_by(bot, p, norm(h0 - bot.last["H"]))
        if not period:
            raise Fail(1, "the turn was too short to compare two rounds")
        out = {"turns": a.turns, "samples": len(samples), "reported_deg_per_real_turn": period,
               "odo_head": round(360.0 / period, 4), "sure": sure,
               "sure_is": "error at other periods over the error at this one; under 2, do not trust it",
               "was": p.odo_head}
        if "head" in samples[0]:
            # The same for the wheels alone and for the gyro alone.
            span = samples[-1]["heading"] - samples[0]["heading"]
            for i, name in enumerate(("wheels", "gyro")):
                part = (samples[-1]["head"][i] - samples[0]["head"][i]) / span * period
                out[f"{name}_deg_per_real_turn"] = round(part, 2)
            out["track_m_should_be"] = "TRACK_M x wheels_deg_per_real_turn / 360"
        name, value = "odo_head", out["odo_head"]
    else:
        # A wall ahead is the ruler: drive at it and back, and compare what the
        # wheels say with what the sensor says.
        if not 350 <= s["tof"] <= 1800:
            raise Fail(1, "face a wall 0.5 to 1.5 m ahead first", tof=s["tof"])
        turn_by(bot, p, -30)
        samples, _ = motion(bot, p, 0, 15, 60.0 / 15)
        seen = [(x["tof"], x["H"]) for x in samples if x["tof"] >= 0]
        if len(seen) < 8:
            raise Fail(1, "the sensor lost the wall while it looked for it")
        least = min(d for d, _ in seen)
        square = statistics.fmean(h for d, h in seen if d <= least * 1.02)
        turn_by(bot, p, norm(square - bot.last["H"]))
        d = min(a.distance, (bot.last["tof"] - 300) / 1000.0 if bot.last["tof"] > 0 else 0.1)
        if d < 0.1:
            raise Fail(1, "too close to the wall to drive at it")
        ratios, legs = [], []
        pos0, tof0, _ = average(bot, p)
        for way in [1, -1] * a.rounds:
            do_forward(bot, p, way * d, blind_ok=True)
            pos1, tof1, _ = average(bot, p)
            if tof0 is None or tof1 is None or abs(pos1 - pos0) < 0.05:
                legs.append("lost")
            else:
                r = -(tof1 - tof0) / 1000.0 / (pos1 - pos0)
                ratios.append(r)
                legs.append(round(r, 4))
            pos0, tof0 = pos1, tof1
        if len(ratios) < 2:
            raise Fail(1, "not enough good runs", legs=legs)
        out = {"runs": legs, "odo_dist": round(statistics.median(ratios), 4),
               "spread": round(max(ratios) - min(ratios), 4), "was": p.odo_dist,
               "note": "the sensor is the ruler: its own error (a few percent) is in this number"}
        name, value = "odo_dist", out["odo_dist"]
    if a.apply:
        p.saved[name] = value
        store("params.json", p.saved)
        out["applied"] = True
    else:
        out["to_use_it"] = f"set {name} {value}"
    return out


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
            nap(p.tof_settle_s)
            got = []
            for _ in range(3):
                got.append(bot.sample()["tof"])
                nap(0.1)
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


# --- Near map and gaps -------------------------------------------------------
# A small map of what is around the robot, in cells of 5 cm, from a slow turn
# with the narrow zone (about 7 degrees, so about 6 cm wide at 0.5 m; the wide
# zone is 24 cm there, as wide as the gap the robot needs). Then the ways out
# between the things it saw, and how wide each one is. A person says if a gap
# is real (`mark`): the sensor misses low, thin and dark things.

def need_mm(p):
    return p.robot_w_mm + 2 * p.margin_mm


def find_gaps(grid, cell, x, y, p, old=()):
    """Ways out of the near map: runs of directions with nothing in near_m,
    with something on both sides. Returns them, the nearest first."""
    # A little past near_m, or a wall right at the edge comes and goes.
    step, reach = p.near_step, p.near_m + 2 * cell
    rays = []
    for i in range(int(round(360 / step))):
        b = i * step
        th = math.radians(b)
        r, hit = p.robot_w_mm / 2000.0, None
        while r <= reach:
            px, py = x + r * math.cos(th), y + r * math.sin(th)
            if nm.is_occ(grid, nm.cell_of(cell, px, py)):
                hit = (px, py)
                break
            r += cell / 2
        rays.append((b, hit))
    n = len(rays)
    if all(h is None for _, h in rays):
        return []
    j0 = next(j for j in range(n) if rays[j][1] is not None)
    gaps, j = [], j0 + 1
    while j < j0 + n:
        if rays[j % n][1] is not None:
            j += 1
            continue
        k = j
        while rays[k % n][1] is None:
            k += 1
        right, left = rays[(j - 1) % n][1], rays[k % n][1]  # angles grow to the left
        arc = (k - j) * step
        if arc < 180:
            mx, my = (left[0] + right[0]) / 2, (left[1] + right[1]) / 2
            width = math.hypot(left[0] - right[0], left[1] - right[1]) * 1000
            kind = "gap"
        else:
            # Most of the circle is open: that is open floor, not a gap. Its
            # middle is the middle of the open directions, near_m away.
            th = math.radians((j + k - 1) / 2 * step)
            mx, my = x + p.near_m * math.cos(th), y + p.near_m * math.sin(th)
            width, kind = 2000 * p.near_m, "open"
        gaps.append({"kind": kind, "bearing": round(math.degrees(math.atan2(my - y, mx - x)), 1),
                     "dist_m": round(math.hypot(mx - x, my - y), 3), "width_mm": int(width),
                     "arc_deg": arc, "passable": width >= need_mm(p),
                     "mid": [round(mx, 3), round(my, 3)],
                     "edges": [[round(right[0], 3), round(right[1], 3)],
                               [round(left[0], 3), round(left[1], 3)]]})
        j = k
    gaps.sort(key=lambda g: g["dist_m"])
    for i, g in enumerate(gaps, 1):
        g["id"] = i
        # Keep what a person said about the same gap before.
        for o in old:
            if o.get("mark") and math.hypot(o["mid"][0] - g["mid"][0], o["mid"][1] - g["mid"][1]) < 0.08:
                g["mark"] = o["mark"]
    return gaps


def blocking(grid, cell, x, y, heading, p):
    """What stands in the way straight ahead: the thing that touches the strip
    as wide as the robot needs, and how far it reaches to each side. In metres,
    in the robot's frame: ahead, and left (+) / right (-). None if the way is free."""
    th = math.radians(heading)
    c, s_ = math.cos(th), math.sin(th)
    occ = {nm.unkey(k) for k, v in grid.items() if v >= nm.OCC}
    frame = lambda q: ((q[0] * cell - x) * c + (q[1] * cell - y) * s_,
                       -(q[0] * cell - x) * s_ + (q[1] * cell - y) * c)
    half = need_mm(p) / 2000.0
    seeds = [q for q in occ if 0 < frame(q)[0] <= p.near_m + 2 * cell and abs(frame(q)[1]) <= half]
    if not seeds:
        return None
    # The whole thing: the taken cells that touch the nearest one in the strip.
    first = min(seeds, key=lambda q: frame(q)[0])
    todo, part = [first], {first}
    while todo:
        q = todo.pop()
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                nb = (q[0] + dx, q[1] + dy)
                if nb in occ and nb not in part:
                    part.add(nb)
                    todo.append(nb)
    pts = [frame(q) for q in part]
    a, l = [q[0] for q in pts], [q[1] for q in pts]
    return {"ahead_m": round(min(a), 3), "far_m": round(max(a), 3),
            "left_m": round(max(l), 3), "right_m": round(min(l), 3), "cells": len(part),
            "note": "far_m is the far side the sensor saw; the back of a thing can be farther"}


def show_gaps(bot, p):
    h = bot.last["H"]
    out = []
    for g in bot.ses.get("gaps") or []:
        out.append({"id": g["id"], "kind": g.get("kind", "gap"),
                    "turn": round(norm(g["bearing"] - h), 1), "dist_m": g["dist_m"],
                    "width_mm": g["width_mm"], "passable": g["passable"], "mark": g.get("mark")})
    return {"gaps": out, "need_mm": need_mm(p),
            "gaps_are": "turn: degrees to its middle from where the robot looks (left +); "
                        "width_mm: between the things on each side, from 5 cm cells, so it can be "
                        "some cm short; open: most of the circle is free; "
                        "mark: what a person said (real, false) or null"}


def sweep(bot, p, arc, rate, rows=0):
    """Turn slowly over `arc` degrees centred ahead with the narrow zone, and
    come back. Returns (samples, beam in degrees, heading at the start)."""
    s = bot.last
    if s["tof"] == -2:
        raise Fail(1, "no distance sensor")
    rows0 = s["vars"].get("tzr")
    h0 = s["H"]
    k = p.odo_head or 1.0
    try:
        if rows0 != rows:
            bot.cmd(f"tzr {int(rows)}")  # 0: the narrow square
            nap(p.tof_settle_s)
        bot.sample()
        width = p.beam_deg or (int(rows) or bot.last["vars"].get("tz", 4)) * ZONE_DEG
        turn_by(bot, p, -arc / 2)
        samples, _ = motion(bot, p, 0, rate, arc / k / rate)
    finally:
        if rows0 != rows:
            bot.cmd(f"tzr {rows0}")
    turn_by(bot, p, norm(h0 - bot.last["H"]))
    return samples, width, h0


def measure_front(rs, beam_deg, x0, y0, h0, p):
    """The nearest thing ahead in a sweep, from its edges: where the distance
    jumps by edge_mm or more, or the sensor sees nothing past it.

    rs: readings (x, y, bearing, m or None) from `readings`, each from where
    the robot was then, so a robot that slides does not smear the thing.
    Returns the thing and the room on each side, in the frame of the start
    (x0, y0, h0): metres ahead, and to the left (+) / right (-)."""
    pts = sorted(rs, key=lambda r: norm(r[2] - h0))
    if not pts:
        return None
    th0 = math.radians(h0)
    c0, s0 = math.cos(th0), math.sin(th0)

    def frame(px, py):
        return ((px - x0) * c0 + (py - y0) * s0, -(px - x0) * s0 + (py - y0) * c0)

    def hit(r, shift=0.0):
        b = math.radians(r[2] + shift)
        return frame(r[0] + r[3] * math.cos(b), r[1] + r[3] * math.sin(b))

    jump = p.edge_mm / 1000.0
    # The thing: the nearest reading near the middle, then the readings next to
    # it as long as the distance changes smoothly.
    mid = [i for i, r in enumerate(pts) if r[3] is not None and abs(norm(r[2] - h0)) <= 15]
    if not mid:
        return {"thing": None, "note": "nothing ahead within 15 degrees"}
    i0 = min(mid, key=lambda i: pts[i][3])
    lo = hi = i0
    def step(i, d):
        """The next reading on the same thing from i, going d (+1 or -1), or None.
        One empty reading in between does not end it: the face of a box at an
        angle sends little light back."""
        for j in (i + d, i + 2 * d):
            if not 0 <= j < len(pts):
                return None
            if pts[j][3] is not None:
                return j if abs(pts[j][3] - pts[i][3]) < jump else None
        return None

    while (j := step(lo, -1)) is not None:
        lo = j
    while (j := step(hi, +1)) is not None:
        hi = j

    def edge(i, j):
        """The sensor gives the distance of what fills most of its zone, so the
        reading changes when the middle of the zone crosses the edge: between
        reading i (on the thing) and j (off it). edge_shift moves it inward."""
        b = pts[i][2] if j is None else pts[i][2] + norm(pts[j][2] - pts[i][2]) / 2
        r = pts[i]
        sign = 1 if j is None or norm(pts[j][2] - pts[i][2]) > 0 else -1
        return hit((r[0], r[1], b, r[3]), -sign * p.edge_shift * beam_deg)

    right = edge(lo, lo - 1 if lo > 0 else None)  # angles grow to the left
    left = edge(hi, hi + 1 if hi < len(pts) - 1 else None)
    open_right, open_left = lo == 0, hi == len(pts) - 1  # the thing goes past the sweep
    body = [hit(pts[i]) for i in range(lo, hi + 1) if pts[i][3] is not None]
    thing = {"ahead_m": round(min(b[0] for b in body), 3),
             "left_m": round(left[1], 3), "right_m": round(right[1], 3),
             "width_mm": int(round((left[1] - right[1]) * 1000)),
             "readings": hi - lo + 1,
             "sure": not (open_left or open_right),
             "note": "the width is across the line ahead; only the front is seen, not the depth"}

    def side(idx, sign):
        """Past one edge: the next thing the sweep saw, and the room between."""
        rest = [pts[i] for i in idx]
        # Only what stands where the robot will pass: up to 0.5 m past the front.
        reach_x = thing["ahead_m"] + 0.5
        far = [q for q in (hit(r) for r in rest if r[3] is not None) if q[0] <= reach_x]
        nothing = sum(1 for r in rest if r[3] is None)
        edge = left[1] if sign > 0 else right[1]
        out = {"readings": len(rest), "nothing": nothing}
        if not rest:
            return {**out, "room_mm": None, "note": "the sweep ended at the edge: turn and look"}
        if far:
            # Of what lies to that side, the nearest across the line: that is the room.
            nearest = min(far, key=lambda q: sign * (q[1] - edge))
            out.update(next_ahead_m=round(nearest[0], 3), next_side_m=round(nearest[1], 3),
                       room_mm=int(round(sign * (nearest[1] - edge) * 1000)))
        else:
            out.update(room_mm=None, note="nothing seen past the edge, as far as the sensor sees")
        return out

    need = need_mm(p)
    sides = {"left": side(range(hi + 1, len(pts)), +1), "right": side(range(0, lo), -1)}
    for v in sides.values():
        # Nothing seen is not free: a dark box may send no light back. Then ask.
        v["fits"] = None if v.get("room_mm") is None else v["room_mm"] >= need
    return {"thing": thing, "sides": sides, "need_mm": need}


def c_measure(bot, p, a):
    """Measure the thing ahead: how far, how wide, and the room on each side."""
    s = bot.ready(p)
    x0, y0 = bot.ses["pose"]
    samples, width, h0 = sweep(bot, p, a.arc or p.measure_arc, p.measure_rate, p.measure_rows)
    rs, _ = readings(p, samples)
    nm.add_readings(bot.ses["grid"], p.map_cell, rs, width, p.reach_mm / 1000.0, arcs=True)
    out = measure_front(rs, width, x0, y0, h0, p) or {"thing": None}
    xs = [q["px"] for q in samples if "px" in q]
    ys = [q["py"] for q in samples if "py" in q]
    out.update(samples=len(rs), beam_deg=round(width, 1),
               slid_m=round(math.hypot(max(xs) - min(xs), max(ys) - min(ys)), 3) if xs else None,
               pose=bot.pose())
    if a.profile:
        out["profile"] = [[round(norm(r[2] - h0), 1), None if r[3] is None else int(r[3] * 1000)]
                          for r in sorted(rs, key=lambda r: norm(r[2] - h0))]
        out["profile_is"] = "[degrees from the heading at the start (left +), mm or null for nothing]"
    return out


def c_near(bot, p, a):
    s = bot.ready(p)
    if s["tof"] == -2:
        raise Fail(1, "no distance sensor")
    rows0 = s["vars"].get("tzr")
    k = p.odo_head or 1.0
    h0 = s["H"]
    try:
        if rows0:
            bot.cmd("tzr 0")  # the narrow zone
            nap(p.tof_settle_s)
        bot.sample()
        width = p.beam_deg or bot.last["vars"].get("tz", 4) * ZONE_DEG
        samples, _ = motion(bot, p, 0, p.near_rate, 360 / k / p.near_rate)
    finally:
        if rows0:
            bot.cmd(f"tzr {rows0}")
    turn_by(bot, p, h0 + 360 - bot.last["H"])
    rs, _ = readings(p, samples)
    x, y = bot.ses["pose"]
    grid = {}
    nm.add_readings(grid, p.near_cell, rs, width, p.near_m + 3 * p.near_cell, arcs=True)
    nm.add_robot(grid, p.near_cell, x, y, p.robot_w_mm / 2000.0)
    nm.add_readings(bot.ses["grid"], p.map_cell, rs, width, p.reach_mm / 1000.0, arcs=True)
    bot.ses["near"] = {"grid": grid, "x": x, "y": y, "cell": p.near_cell}
    bot.ses["gaps"] = find_gaps(grid, p.near_cell, x, y, p, bot.ses.get("gaps") or [])
    out = {"samples": len(rs), "beam_deg": round(width, 1),
           "blocking": blocking(grid, p.near_cell, x, y, bot.last["H"], p),
           **show_gaps(bot, p), "pose": bot.pose()}
    rows = nm.draw(grid, p.near_cell, (x, y), bot.last["H"])
    if rows:
        out["map"] = rows
    return out


def c_gaps(bot, p, a):
    bot.sample()
    if not bot.ses.get("near"):
        raise Fail(1, "no near map yet: run `near` first")
    return show_gaps(bot, p)


def gap(bot, n):
    for g in bot.ses.get("gaps") or []:
        if g["id"] == n:
            return g
    raise Fail(1, f"no gap {n}: see `gaps`")


def c_approach(bot, p, a):
    """Go toward a gap in short stretches, and stop approach_m from its middle."""
    bot.ready(p)
    g = gap(bot, a.id)
    legs = []
    for _ in range(20):
        x, y = bot.ses["pose"]
        mx, my = g["mid"]
        d = math.hypot(mx - x, my - y)
        if d <= p.approach_m + 0.05:  # the robot stops some cm short of a goal
            break
        turn_by(bot, p, norm(math.degrees(math.atan2(my - y, mx - x)) - bot.last["H"]))
        out = do_forward(bot, p, min(p.approach_leg, d - p.approach_m))
        legs.append({"moved": out["moved"], "stopped": out["stopped"], "tof": out["tof"]})
        if out["stopped"] or abs(out["moved"]) < 0.01:
            break
    x, y = bot.ses["pose"]
    mx, my = g["mid"]
    turn_by(bot, p, norm(math.degrees(math.atan2(my - y, mx - x)) - bot.last["H"]))
    look = c_look(bot, p, a)
    return {"gap": a.id, "width_mm": g["width_mm"], "passable": g["passable"],
            "dist_m": round(math.hypot(mx - x, my - y), 3), "legs": legs, "tof": bot.last["tof"],
            "look": look["rows"], "look_is": look["rows_are"], "pose": bot.pose(),
            "next": f"ask: is gap {a.id} real, about {g['width_mm']} mm wide, straight ahead? "
                    f"then `mark {a.id} real` or `mark {a.id} false`"}


def c_mark(bot, p, a):
    """What a person saw: the gap is there (real), or something closes it (false)."""
    bot.sample()
    g = gap(bot, a.id)
    g["mark"] = a.what
    grid, cell = bot.ses["grid"], p.map_cell
    (x1, y1), (x2, y2) = g["edges"]
    n = max(2, int(math.hypot(x2 - x1, y2 - y1) / (cell / 2)))
    line = [(x1 + (x2 - x1) * i / n, y1 + (y2 - y1) * i / n) for i in range(1, n)]
    near = bot.ses.get("near")
    for px, py in line:
        if a.what == "false":
            nm.bump(grid, nm.cell_of(cell, px, py), nm.TOP)  # goto will keep away
            if near:
                nm.bump(near["grid"], nm.cell_of(near["cell"], px, py), nm.TOP)
    return {"gap": a.id, "mark": a.what, "note": "false: the map has it closed now" if a.what == "false"
            else "real: `pass` may go through it"}


def c_pass(bot, p, a):
    """Go straight through a gap a person said is real, to pass_m past its middle."""
    bot.ready(p)
    g = gap(bot, a.id)
    if g.get("mark") != "real":
        raise Fail(1, f"gap {a.id} is not marked real: ask first, then `mark {a.id} real`")
    if not g["passable"]:
        raise Fail(1, f"gap {a.id} is {g['width_mm']} mm, the robot needs {need_mm(p)}")
    x, y = bot.ses["pose"]
    mx, my = g["mid"]
    turn_by(bot, p, norm(math.degrees(math.atan2(my - y, mx - x)) - bot.last["H"]))
    out = do_forward(bot, p, math.hypot(mx - x, my - y) + p.pass_m)
    out["gap"] = a.id
    if out["stopped"]:
        raise Fail(3, WHY[out["stopped"]], **out)
    return out


def c_halt(bot, p, a):
    bot.cmd("drive 0 0")
    s = bot.sample()
    return {"state": s["state"], "reason": s["reason"], "drive": s["drive"]}


def c_map(bot, p, a):
    s = bot.sample()
    ses = bot.ses
    x, y = ses["pose"]
    out = {"pose": bot.pose(), **map_numbers(bot, p)}
    if a.points:
        out.update(grid=ses["grid"], path=ses["path"],
                   grid_is="cell 'ix,iy' (times cell_m): score, 2 and up taken, -1 and down free")
        return out
    rows = nm.draw(ses["grid"], p.map_cell, (x, y), s.get("H", 0.0), ses["path"])
    if rows is None:
        raise Fail(1, "the map is too big to draw: raise map_cell, or use --points")
    out["legend"] = "# taken, . free, : path, o start, arrow robot; up is forward at the start"
    out["map"] = rows
    return out


def c_origin(bot, p, a):
    bot.sample()
    bot.ses.update(pose=[0.0, 0.0], points=[], path=[[0.0, 0.0]], grid={}, tried={},
                   near=None, gaps=[])
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
        nap(0.1)
        out[name] = bot.sample()["vars"][name]  # the robot keeps it inside its limits
    return {"set": out, "note": "robot values last until it restarts; `cmd save` keeps them"}


def c_cmd(bot, p, a):
    if a.line.split()[:1] in (["save"], ["defaults"]) and bot.sample()["state"] == "balancing":
        raise Fail(2, f"the robot refuses `{a.line}` while it balances: lay it down first")
    bot.cmd(a.line)
    nap(0.1)
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
    sp = add("goto", c_goto)
    sp.add_argument("x", type=float, help="metres ahead of the start")
    sp.add_argument("y", type=float, help="metres to the left of the start")
    add("explore", c_explore).add_argument("--steps", type=int, default=1,
                                           help="scans and moves to do in this call")
    sp = add("calibrate", c_calibrate)
    sp.add_argument("what", choices=("spin", "wall", "still"))
    sp.add_argument("--turns", type=float, default=2.5, help="spin: turns on the spot")
    sp.add_argument("--distance", type=float, default=0.4, help="wall: metres of each run")
    sp.add_argument("--rounds", type=int, default=3, help="wall: runs there and back")
    sp.add_argument("--seconds", type=float, default=20, help="still: how long")
    sp.add_argument("--apply", action="store_true", help="keep the result as the model")
    add("near", c_near)
    add("gaps", c_gaps)
    sp = add("measure", c_measure)
    sp.add_argument("--arc", type=float, help="degrees to sweep (measure_arc)")
    sp.add_argument("--profile", action="store_true", help="also every reading of the sweep")
    add("approach", c_approach).add_argument("id", type=int, help="a gap from `near` or `gaps`")
    sp = add("mark", c_mark)
    sp.add_argument("id", type=int)
    sp.add_argument("what", choices=("real", "false"), help="what a person saw")
    add("pass", c_pass).add_argument("id", type=int, help="a gap marked real")
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
        bot = Robot(a.host, p)
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
