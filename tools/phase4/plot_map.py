"""Top-down view of a saved phase 4 map (voxels.pcd), for finding structures.

    python3 plot_map.py maps/phase4 out.png [z_min z_max]

Colors voxels by height inside [z_min, z_max] (odom frame, metres above the
takeoff point's base_link). Origin = takeoff, x forward, y left.
"""
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, '/ws/src/hydrone_lio')
from hydrone_lio.lio_map_node import read_pcd  # noqa: E402

map_dir, out = sys.argv[1], sys.argv[2]
z_min = float(sys.argv[3]) if len(sys.argv) > 3 else 0.2
z_max = float(sys.argv[4]) if len(sys.argv) > 4 else 3.0
names, v = read_pcd(f'{map_dir}/voxels.pcd')
v = v[(v[:, 2] > z_min) & (v[:, 2] < z_max)]
fig, ax = plt.subplots(figsize=(14, 14))
sc = ax.scatter(v[:, 1], v[:, 0], c=v[:, 2], s=1, cmap='viridis')
ax.plot(0, 0, 'r^', markersize=12)
ax.set_xlabel('y (left +) [m]')
ax.set_ylabel('x (forward +) [m]')
ax.invert_xaxis()  # left on the left
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
ax.set_xticks(np.arange(np.floor(v[:, 1].min()), np.ceil(v[:, 1].max()) + 1, 1))
ax.set_yticks(np.arange(np.floor(v[:, 0].min()), np.ceil(v[:, 0].max()) + 1, 1))
plt.colorbar(sc, label='z [m]')
ax.set_title(f'{map_dir}: {len(v)} voxels, z in [{z_min}, {z_max}]')
plt.savefig(out, dpi=110, bbox_inches='tight')
print(out)
