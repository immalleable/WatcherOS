#!/usr/bin/env python3
"""Convert the sim's RGB565 dumps into round 412x412 PNGs for the README.
Applies a circular alpha mask (transparent outside the bezel) plus a thin
ring, matching the physical round LCD. Writes to ../docs/."""
import struct, os
from PIL import Image, ImageDraw
W = H = 412
HERE = os.path.dirname(os.path.abspath(__file__))
DOCS = os.path.join(HERE, "..", "docs")
os.makedirs(DOCS, exist_ok=True)

def load565(path):
    with open(path, "rb") as f:
        data = f.read()
    px = struct.unpack("<%dH" % (W*H), data)
    img = Image.new("RGB", (W, H))
    out = img.load()
    for i, v in enumerate(px):
        r = (v >> 11) & 0x1F; g = (v >> 5) & 0x3F; b = v & 0x1F
        out[i % W, i // W] = (r*255//31, g*255//63, b*255//31)
    return img

def roundify(img, name):
    mask = Image.new("L", (W, H), 0)
    d = ImageDraw.Draw(mask)
    d.ellipse((0, 0, W-1, H-1), fill=255)
    img = img.convert("RGBA"); img.putalpha(mask)
    ring = ImageDraw.Draw(img)
    ring.ellipse((1, 1, W-2, H-2), outline=(60, 60, 66, 255), width=3)
    p = os.path.join(DOCS, name)
    img.save(p)
    print("wrote", os.path.relpath(p, HERE))

for src, name in [("out_radar.565","radar.png"),
                  ("out_timer.565","timer.png"),
                  ("out_wifi.565","wifi.png")]:
    s = os.path.join(HERE, src)
    if os.path.exists(s):
        roundify(load565(s), name)
    else:
        print("missing", src)
