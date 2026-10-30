#!/usr/bin/env python3
"""Talk to 06-balance over Wi-Fi. Python 3 standard library only.

First join the robot's Wi-Fi network (BALA2-xxxx, password on its screen).

    python3 bala2_link.py                    show telemetry, type commands
    python3 bala2_link.py --log run.csv      also save every line to a file
    python3 bala2_link.py --ota build/balance.bin
                                             send new firmware (app image)

While it runs, it sends a heartbeat ("hb") 5 times a second. If the robot
balances and hears nothing for 1 second, it stops. So stopping this tool
(Ctrl+C) also stops the robot.

Commands: kp, ki, kd, kv, kx, kpos, kvel, kt, off, ramp, vmax, max (each with a value), ?, stop
"""
import argparse
import socket
import sys
import threading
import time
import urllib.request

ROBOT = "192.168.4.1"
PORT = 4210


def ota(path):
    data = open(path, "rb").read()
    print(f"sending {len(data)} bytes to http://{ROBOT}/update ...")
    req = urllib.request.Request(f"http://{ROBOT}/update", data=data, method="POST")
    req.add_header("Content-Type", "application/octet-stream")
    try:
        with urllib.request.urlopen(req, timeout=120) as resp:
            print(resp.read().decode().strip())
    except urllib.error.HTTPError as e:
        print(f"refused: {e.code} {e.read().decode().strip()}")
        sys.exit(1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", help="save every line from the robot to this file")
    ap.add_argument("--ota", help="send this app image and exit")
    args = ap.parse_args()

    if args.ota:
        ota(args.ota)
        return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.5)
    robot = (ROBOT, PORT)
    log = open(args.log, "a", buffering=1) if args.log else None
    running = True

    def send(data):
        # While the robot restarts, its network is gone for a moment and
        # sendto() fails. Keep going: a dead heartbeat would stop the robot.
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
                data, _ = sock.recvfrom(256)
            except OSError:  # timeout, or the robot is restarting
                if time.time() - quiet_since > 2:
                    print("(no data from the robot: is this Mac on its Wi-Fi?)")
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
