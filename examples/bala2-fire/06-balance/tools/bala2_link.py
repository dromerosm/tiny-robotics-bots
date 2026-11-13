#!/usr/bin/env python3
"""Talk to 06-balance over Wi-Fi. Python 3 standard library only.

The robot joins your home Wi-Fi if it was built with WIFI_ENV and the
network is in range; then it is bala2.local. If not, join the robot's own
network (BALA2-xxxx, the key on its screen); it is then at 192.168.4.1. This
tool tries bala2.local first, then 192.168.4.1.

    python3 bala2_link.py                    show telemetry, type commands
    python3 bala2_link.py --log run.csv      also save every line to a file
    BALA2_TOKEN=<key> python3 bala2_link.py --ota build/balance.bin
                                             send new firmware (app image)

The update token is the key on the robot's screen. Put it in the
environment variable BALA2_TOKEN, not on the command line.

While it runs, it sends a heartbeat ("hb") 5 times a second. If the robot
hears nothing for 1 second, it stops the servo and keeps balancing in place.
So stopping this tool (Ctrl+C) does not make the robot fall. To stop the
robot, type "stop" or press its button B.

Commands: a name and a value (kp, ki, kd, rf, kpos, kvel, kt, off, ramp, kh,
vmax, max, gw, kw, tp, mt, mk, ks, kr, kst, as, amin, av, ts, tw, tz, tzr, tcol,
trow, bp, servo, bl), ? to list them, rec N (N seconds of 200 Hz
samples, sent as they come), fall, stop
"""
import argparse
import os
import socket
import sys
import threading
import time
import urllib.request

HOSTS = ("bala2.local", "192.168.4.1")
PORT = 4210


def find_robot(host):
    """The robot's IP address: the given host, or bala2.local, or 192.168.4.1."""
    for name in (host,) if host else HOSTS:
        try:
            return socket.gethostbyname(name)
        except OSError:
            pass
    return HOSTS[-1]


def ota(path, robot):
    data = open(path, "rb").read()
    print(f"sending {len(data)} bytes to http://{robot}/update ...")
    req = urllib.request.Request(f"http://{robot}/update", data=data, method="POST")
    req.add_header("Content-Type", "application/octet-stream")
    req.add_header("X-Token", os.environ.get("BALA2_TOKEN", ""))
    try:
        with urllib.request.urlopen(req, timeout=120) as resp:
            print(resp.read().decode().strip())
    except urllib.error.HTTPError as e:
        print(f"refused: {e.code} {e.read().decode().strip()}")
        sys.exit(1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", help="save every line from the robot to this file")
    ap.add_argument("--ota", help="send this app image and exit (token in BALA2_TOKEN)")
    ap.add_argument("--host", help="robot name or address (default: bala2.local, then 192.168.4.1)")
    args = ap.parse_args()
    ip = find_robot(args.host)
    print(f"robot at {ip}")

    if args.ota:
        ota(args.ota, ip)
        return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.5)
    robot = (ip, PORT)
    log = open(args.log, "a", buffering=1) if args.log else None
    running = True

    def send(data):
        # While the robot restarts, its network is gone for a moment and
        # sendto() fails. Keep going: a dead heartbeat would stop the servo.
        try:
            sock.sendto(data, robot)
        except OSError as e:
            print(f"(send failed: {e})")

    def heartbeat():
        while running:
            send(b"hb\n")
            time.sleep(0.2)

    def receive():
        quiet_since = time.time()
        while running:
            try:
                data, _ = sock.recvfrom(2048)
            except OSError:  # timeout, or the robot is restarting
                if time.time() - quiet_since > 2:
                    print("(no data from the robot: is this Mac on the same Wi-Fi?)")
                    quiet_since = time.time()
                time.sleep(0.05)
                continue
            quiet_since = time.time()
            for line in data.decode(errors="replace").splitlines():
                print(line)
                if log:
                    log.write(f"{time.time():.3f} {line}\n")

    threading.Thread(target=heartbeat, daemon=True).start()
    threading.Thread(target=receive, daemon=True).start()
    try:
        for line in sys.stdin:
            send(line.encode())
            if log:
                log.write(f"{time.time():.3f} >> {line}")
    except KeyboardInterrupt:
        pass
    # stdin closed (or Ctrl+C): keep the heartbeat going until Ctrl+C, so a
    # tool started without a terminal still keeps the link alive.
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        running = False


if __name__ == "__main__":
    main()
