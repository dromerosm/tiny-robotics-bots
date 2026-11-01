#!/usr/bin/env python3
"""Make apple-touch-icon.png (180 x 180) from the shapes of favicon.svg.

Safari asks for /apple-touch-icon.png when you save the page to the home
screen or show it in the tab overview. It does not take an SVG, so we draw
the same robot here, with Python only (no image library):

    python3 components/tr_ota/touch-icon.py

Change favicon.svg and this file together.
"""
import struct
import zlib
from pathlib import Path

SIZE = 180
SCALE = SIZE / 64 * 0.8  # the 64 x 64 drawing, at 80 % of the icon
OFFSET = SIZE * 0.1      # centred
SAMPLES = 4              # 4 x 4 samples per pixel, for smooth edges

BG = (0xFA, 0xF9, 0xF5)
ORANGE = (0xC4, 0x62, 0x3F)
DARK = (0x1F, 0x1E, 0x1D)
LIGHT = (0xFA, 0xF9, 0xF5)


def rrect(x, y, w, h, r):
    def inside(px, py):
        if not (x <= px <= x + w and y <= py <= y + h):
            return False
        cx = min(max(px, x + r), x + w - r)
        cy = min(max(py, y + r), y + h - r)
        return (px - cx) ** 2 + (py - cy) ** 2 <= r * r
    return inside


def circle(cx, cy, r):
    return lambda px, py: (px - cx) ** 2 + (py - cy) ** 2 <= r * r


# favicon.svg, in its order: later shapes are drawn on top.
SHAPES = [
    (rrect(30, 4, 4, 10, 2), ORANGE),  # the antenna line (4 wide, round caps)
    (circle(32, 5, 4), ORANGE),
    (rrect(6, 14, 52, 44, 12), ORANGE),
    (rrect(13, 22, 38, 24, 6), DARK),
    (circle(24, 34, 4.5), LIGHT),
    (circle(40, 34, 4.5), LIGHT),
    (rrect(26, 50, 12, 3, 1.5), DARK),
]


def color_at(px, py):
    x, y = (px - OFFSET) / SCALE, (py - OFFSET) / SCALE
    c = BG
    for inside, fill in SHAPES:
        if inside(x, y):
            c = fill
    return c


def main():
    rows = bytearray()
    for py in range(SIZE):
        rows.append(0)  # PNG filter: none
        for px in range(SIZE):
            acc = [0, 0, 0]
            for sy in range(SAMPLES):
                for sx in range(SAMPLES):
                    c = color_at(px + (sx + 0.5) / SAMPLES, py + (sy + 0.5) / SAMPLES)
                    for i in range(3):
                        acc[i] += c[i]
            rows.extend(round(v / SAMPLES ** 2) for v in acc)

    def chunk(kind, data):
        body = kind + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body))

    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", SIZE, SIZE, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(bytes(rows), 9))
           + chunk(b"IEND", b""))
    out = Path(__file__).with_name("apple-touch-icon.png")
    out.write_bytes(png)
    print(f"{out}: {len(png)} bytes")


if __name__ == "__main__":
    main()
