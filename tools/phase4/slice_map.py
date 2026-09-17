"""Height slices of a region of a saved map: python3 slice_map.py map_dir out.png xmin xmax ymin ymax"""
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
sys.path.insert(0, '/ws/src/hydrone_lio')
from hydrone_lio.lio_map_node import read_pcd  # noqa: E402

d, out = sys.argv[1], sys.argv[2]
x0, x1, y0, y1 = (float(v) for v in sys.argv[3:7])
_, v = read_pcd(f'{d}/voxels.pcd')
v = v[(v[:, 0] > x0) & (v[:, 0] < x1) & (v[:, 1] > y0) & (v[:, 1] < y1)]
bands = [(-0.3, 0.1), (0.1, 0.3), (0.3, 0.5), (0.5, 0.7), (0.7, 0.9), (0.9, 1.2), (1.2, 1.6), (1.6, 3.0)]
fig, axs = plt.subplots(2, 4, figsize=(22, 9))
for ax, (a, b) in zip(axs.ravel(), bands):
    s = v[(v[:, 2] >= a) & (v[:, 2] < b)]
    ax.scatter(s[:, 1], s[:, 0], s=2, c='k')
    ax.set_title(f'z [{a}, {b}) : {len(s)}')
    ax.set_xlim(y1, y0); ax.set_ylim(x0, x1); ax.set_aspect('equal'); ax.grid(True, alpha=0.4)
    ax.set_xticks(np.arange(np.ceil(y0), y1 + 0.01, 0.5)); ax.set_yticks(np.arange(np.ceil(x0), x1 + 0.01, 0.5))
    ax.tick_params(labelsize=6)
plt.tight_layout(); plt.savefig(out, dpi=100); print(out)
