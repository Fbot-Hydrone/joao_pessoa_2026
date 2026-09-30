"""Run phase4_maze_node's planner on a saved map and draw it.

    python3 plan_offline.py map_dir out.png start_x,start_y goal_x,goal_y [fly_z radius fence]
"""
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
sys.path.insert(0, '/ws/src/hydrone_lio')
sys.path.insert(0, '/ws/src/hydrone_mission')
from hydrone_lio.lio_map_node import read_pcd  # noqa: E402
from hydrone_mission.phase4_maze_node import astar, inflate, nearest_free  # noqa: E402

d, out = sys.argv[1], sys.argv[2]
start = [float(v) for v in sys.argv[3].split(',')]
goal = [float(v) for v in sys.argv[4].split(',')]
fz = float(sys.argv[5]) if len(sys.argv) > 5 else 0.2
radius = float(sys.argv[6]) if len(sys.argv) > 6 else 0.25
fence = [float(v) for v in sys.argv[7].split(',')] if len(sys.argv) > 7 else [-0.9, 2.4, -7.9, -0.3]
res, below, above = 0.1, 0.55, 0.6
_, v = read_pcd(f'{d}/voxels.pcd')
names, raw = read_pcd(f'{d}/voxels.pcd')
hits = raw[:, 3]
min_hits = int(sys.argv[8]) if len(sys.argv) > 8 else 1
v = v[(v[:, 2] > fz - below) & (v[:, 2] < fz + above) & (hits >= min_hits)]
x0, x1, y0, y1 = fence
h, w = int(round((x1 - x0) / res)) + 1, int(round((y1 - y0) / res)) + 1
occ = np.zeros((h, w), dtype=bool)
i = np.floor((v[:, 0] - x0) / res).astype(int)
j = np.floor((v[:, 1] - y0) / res).astype(int)
ok = (i >= 0) & (i < h) & (j >= 0) & (j < w)
occ[i[ok], j[ok]] = True
cells = int(np.ceil(radius / res))
blocked = inflate(np.pad(occ, cells + 1), cells)[cells + 1:-cells - 1, cells + 1:-cells - 1]
cell = lambda xy: (int(np.clip(round((xy[0] - x0) / res), 0, h - 1)), int(np.clip(round((xy[1] - y0) / res), 0, w - 1)))  # noqa: E731
s, g = cell(start), cell(goal)
print('start blocked', blocked[s], 'goal blocked', blocked[g])
s, g = nearest_free(blocked, s), nearest_free(blocked, g)
path = astar(blocked, s, g)
print('path', None if path is None else f'{len(path)} cells, {len(path) * res:.1f} m')
fig, ax = plt.subplots(figsize=(16, 7))
ax.imshow(blocked.astype(int) + occ.astype(int), origin='lower', cmap='Greys',
          extent=[y0 - res / 2, y1 + res / 2, x0 - res / 2, x1 + res / 2])
if path:
    p = np.array([(x0 + a * res, y0 + b * res) for a, b in path])
    ax.plot(p[:, 1], p[:, 0], 'r-', lw=2)
ax.plot(start[1], start[0], 'go', ms=10)
ax.plot(goal[1], goal[0], 'bs', ms=10)
ax.set_xlim(y1, y0)
ax.set_xlabel('y'); ax.set_ylabel('x'); ax.grid(True, alpha=0.3)
ax.set_title(f'fly_z {fz}, radius {radius}, min_hits {min_hits}')
plt.savefig(out, dpi=90, bbox_inches='tight')
print(out)
