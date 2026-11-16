#!/usr/bin/env python3
"""Test explorer.py from end to end against sim_robot.py: no robot needed.

It starts the stand-in robot in a world of two rooms, measures the robot's
model (`calibrate spin`, `calibrate wall`), explores until the map has no edge
left, and then compares the result with the truth the stand-in knows:

  - where the tool thinks the robot is, against where it is,
  - the cells the map says are taken: how many are really near a wall,
  - the floor the robot could reach: how much of it the map says is free.

    python3 explorer_test.py                 about 4 minutes
    python3 explorer_test.py --no-calibrate  the same with the model left at 1.0
    python3 explorer_test.py --robot 127.0.0.1:8142
                                             the same against the firmware in QEMU,
                                             behind qemu_world.py (about 20 minutes)

Exit code 0 if every check passes. Python 3 standard library only.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time
import urllib.request

import navmap as nm
import sim_robot

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default="rooms")
    ap.add_argument("--port", type=int, default=8141)
    ap.add_argument("--speed", type=float, default=5)
    ap.add_argument("--steps", type=int, default=30)
    ap.add_argument("--no-calibrate", action="store_true")
    ap.add_argument("--robot", help="host:port of a robot that already runs, with /truth "
                    "(qemu_world.py); no stand-in is started and the clock is the real one")
    ap.add_argument("--keep", help="folder for the tool's files (default: a temporary one)")
    a = ap.parse_args()

    home = a.keep or tempfile.mkdtemp(prefix="explorer-test-")
    host = a.robot or f"127.0.0.1:{a.port}"
    env = dict(os.environ, BALA2_HOST=host, BALA2_EXPLORER_DIR=home,
               BALA2_TIME_SCALE="1" if a.robot else str(a.speed))
    sim = None
    if not a.robot:
        sim = subprocess.Popen([sys.executable, os.path.join(HERE, "sim_robot.py"), "--port",
                                str(a.port), "--world", a.world, "--speed", str(a.speed)])

    def tool(*args, ok=(0,)):
        r = subprocess.run([sys.executable, os.path.join(HERE, "explorer.py"), *args],
                           env=env, capture_output=True, text=True)
        out = json.loads(r.stdout) if r.stdout.strip() else {"error": r.stderr[-300:]}
        if r.returncode not in ok:
            raise SystemExit(f"explorer.py {' '.join(args)} failed ({r.returncode}): {out}")
        return out

    def truth():
        with urllib.request.urlopen(f"http://{host}/truth", timeout=3) as r:
            return json.load(r)

    try:
        time.sleep(0.8)
        if a.robot:
            tool("cmd", "tzr 16")  # the wide zone: the stand-in starts with it
        model = {}
        if not a.no_calibrate:
            spin = tool("calibrate", "spin", "--apply")
            tool("turn", "--to", "180")  # the wall behind the start is the ruler
            wall = tool("calibrate", "wall", "--apply")
            tool("turn", "--to", "0")
            model = {"odo_head": spin["odo_head"], "sure": spin["sure"],
                     "odo_dist": wall["odo_dist"]}
            print("model:", model, flush=True)
        steps, done = 0, False
        while steps < a.steps and not done:
            out = tool("explore", "--steps", "3")
            steps += len(out["steps"])
            done = out["done"]
            print(f"explored {steps} steps, cells {out['map_cells']}, done {done}", flush=True)
        final = tool("map", "--points")
        true = truth()
        print("\n".join(tool("map")["map"]))
    finally:
        if sim:
            sim.terminate()

    world, cell, grid = true["world"], final["cell_m"], final["grid"]
    walls = [tuple(w) for w in world["walls"]]
    low = [tuple(w) for w in world["low"]]
    pose = final["pose"]
    pose_err = math.hypot(pose["x"] - true["x"], pose["y"] - true["y"])
    head_err = abs(nm.norm(pose["heading"] - true["heading"]))

    taken = [nm.unkey(k) for k, v in grid.items() if v >= nm.OCC]
    good = sum(min(sim_robot.near(c[0] * cell, c[1] * cell, w) for w in walls + low) <= 0.2
               for c in taken)
    # The floor the robot could reach in the real world: flood from the start.
    keep = 0.15
    ok = lambda c: min(sim_robot.near(c[0] * cell, c[1] * cell, w) for w in walls + low) > keep
    floor, todo = {(0, 0)}, [(0, 0)]
    while todo:
        c = todo.pop()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nb = (c[0] + dx, c[1] + dy)
            if nb not in floor and abs(nb[0]) < 200 and abs(nb[1]) < 200 and ok(nb):
                floor.add(nb)
                todo.append(nb)
    seen = sum(grid.get(nm.key(c), 0) <= nm.FREE for c in floor)

    checks = [
        ("the exploration ended by itself", done, f"{steps} steps"),
        ("place error under 0.25 m", pose_err < 0.25, f"{pose_err:.3f} m"),
        ("heading error under 8 degrees", head_err < 8, f"{head_err:.1f} degrees"),
        ("taken cells near a real wall, 85% or more", good >= 0.85 * len(taken),
         f"{good} of {len(taken)}"),
        ("reachable floor mapped as free, 85% or more", seen >= 0.85 * len(floor),
         f"{seen} of {len(floor)}"),
    ]
    if not a.no_calibrate:
        want_head = 1 - 2.0 / 360  # sim_robot.py: --drift 2, --scale 1.03
        checks += [
            ("odo_head within 0.3% of the truth", abs(model["odo_head"] / want_head - 1) < 0.003,
             f"{model['odo_head']} for {want_head:.4f}"),
            ("odo_dist within 2% of the truth", abs(model["odo_dist"] * 1.03 - 1) < 0.02,
             f"{model['odo_dist']} for {1 / 1.03:.4f}"),
        ]
    bad = 0
    for name, passed, detail in checks:
        print(("ok    " if passed else "FAIL  ") + name + ": " + detail)
        bad += not passed
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
