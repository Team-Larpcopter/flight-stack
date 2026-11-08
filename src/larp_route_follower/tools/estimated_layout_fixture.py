#!/usr/bin/env python3
"""Build a schematic changed-entrance test map. Never a deployment map."""
import json
import sys
import numpy as np
from pathlib import Path
cad, layout, output = map(Path, sys.argv[1:])
rows = np.loadtxt(cad, skiprows=1)
cfg = json.loads(layout.read_text())
lo, hi = np.array(cfg['clear_box'])
rows[np.all((rows[:, :3] >= lo) & (rows[:, :3] <= hi), axis=1), 3] = 0
for a, b in cfg['walls']:
    a, b = np.array(a), np.array(b)
    delta = b-a
    t = np.clip((rows[:, :2]-a) @ delta / np.dot(delta, delta), 0, 1)
    distance = np.linalg.norm(rows[:, :2] - (a+t[:, None]*delta), axis=1)
    hit = ((distance <= cfg['thickness']/2) & (rows[:, 2] >= cfg['height'][0]) &
           (rows[:, 2] <= cfg['height'][1]))
    rows[hit, 3] = 1
with output.open('w') as f:
    f.write(cad.open().readline())
    np.savetxt(f, rows, fmt=['%.3f', '%.3f', '%.3f', '%d'])
print(output)
