"""Replay a recorded bag through the maze core: grid, covered region, openings.

tools/ isn't mounted in the stack container, so pipe it in:

    docker exec -i -u hydrone joao_pessoa_2026-hydrone-1 bash -lc \
      'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; \
       python3 - /ws/maps/bags/survey1 /ws/maps/replay_survey1' < tools/phase4/replay_maze_bag.py

    optional: flight_z=0.5 every=1 until=<s from start> gt=1

Reads /cloud_registered (camera_init; odom = camera_init + (0, 0, 0.1)) and
/hydrone/lio/odom_raw (odom -> base_link), feeds OccupancyGrid +
detect_openings exactly like the mission does, and writes
<out>_grid.png (free/occupied/unknown, covered, openings, track) and
<out>_layers.png (roof and low-band layers) plus a text summary on stdout.
"""
import sys
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, '/ws/src/hydrone_mission')
from hydrone_mission.maze.arena import arena_from_grid  # noqa: E402
from hydrone_mission.maze.grid import GridParams, OccupancyGrid  # noqa: E402
from hydrone_mission.maze.openings import detect_openings, window_extent  # noqa: E402
from hydrone_mission.maze import geometry_fallback as gf  # noqa: E402

LIDAR_UP = 0.1          # Mid-360 above base_link
CAMERA_INIT_TO_ODOM = np.array([0.0, 0.0, 0.1])


def cloud_xyz(msg):
    offs = {f.name: f.offset for f in msg.fields}
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
    return np.stack([raw[:, offs[c]:offs[c] + 4].copy().view(np.float32)[:, 0] for c in 'xyz'], axis=1)


def read_bag(path, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import yaml
    with open(f'{path}/metadata.yaml') as f:
        storage = yaml.safe_load(f)['rosbag2_bagfile_information']['storage_identifier']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        yield topic, deserialize_message(data, get_message(types[topic])), stamp * 1e-9


def main():
    bag = sys.argv[1] if len(sys.argv) > 1 else '/ws/maps/bags/survey1'
    out = sys.argv[2] if len(sys.argv) > 2 else '/ws/maps/replay_survey1'
    opts = dict(a.split('=') for a in sys.argv[3:])
    flight_z = float(opts.get('flight_z', 0.5))
    every = int(opts.get('every', 1))
    until = float(opts.get('until', 1e9))

    grid = OccupancyGrid(GridParams(flight_z=flight_z))
    pose, track, gt = None, [], []
    recent = []
    ms, n_scans, t0 = [], 0, None
    ops = []
    for topic, msg, t in read_bag(bag, ['/cloud_registered', '/hydrone/lio/odom_raw',
                                        '/biguasim/uav0_id0/DynamicsSensor/Odom']):
        t0 = t if t0 is None else t0
        if t - t0 > until:
            break
        if topic == '/hydrone/lio/odom_raw':
            p = msg.pose.pose.position
            pose = np.array([p.x, p.y, p.z])
            track.append(pose)
        elif topic == '/biguasim/uav0_id0/DynamicsSensor/Odom':
            p = msg.pose.pose.position
            gt.append([p.x, p.y, p.z])
        elif topic == '/cloud_registered' and pose is not None:
            n_scans += 1
            if n_scans % every:
                continue
            pts = cloud_xyz(msg) + CAMERA_INIT_TO_ODOM
            origin = pose + np.array([0, 0, LIDAR_UP])
            c0 = time.perf_counter()
            grid.update(pts, origin)
            ms.append((time.perf_counter() - c0) * 1e3)
            near = pts[np.linalg.norm(pts[:, :2] - pose[:2], axis=1) < 4.0]
            recent.append(near[::2])
            recent = recent[-40:]
    if not ms:
        print('no scans with a pose in', bag)
        return
    viewer = pose[:2]
    ops, covered, crop = detect_openings(grid, viewer=viewer)
    arena = arena_from_grid(grid)
    buf = np.vstack(recent)
    print(f'{len(ms)} scans, grid update {np.mean(ms):.1f} ms mean, {np.percentile(ms, 95):.1f} p95, '
          f'floor {grid.floor_z}, last pose {np.round(pose, 2)}')
    for o in ops:
        bot, top = window_extent(buf, o, flight_z)
        print(f'  {o.kind:8s} centre {np.round(o.center, 2)} width {o.width:.2f} normal {np.round(o.normal, 2)} '
              f'sill_hits {o.sill_hits} passes {o.low_passes} flanks {o.flanks} confirmed {o.confirmed} '
              f'z {None if bot is None else round(bot, 2)}..{None if top is None else round(top, 2)}')
    if arena is not None:
        print(f'  arena centre {np.round(arena.center, 2)} size {np.round(arena.size, 2)}')
    ij = np.argwhere(covered) + np.array([crop[0], crop[2]])
    if len(ij):
        def free_score(pts):
            c = grid.to_cell(np.asarray(pts))
            ok = grid.inside(c)
            return int(grid.free()[c[ok, 0], c[ok, 1]].sum())
        box = gf.fit_box(grid.cell_center(ij), np.zeros(2), free_score=free_score)
        if box is not None:
            pred = gf.predictions(box)
            print(f'  fallback box near end {np.round(box.near_end, 2)} axis {np.round(box.d, 2)}; '
                  f'entrance {np.round(pred["entrance"].center, 2)} exit {np.round(pred["exit"].center, 2)} '
                  f'landing {np.round(pred["landing"], 2)}')

    i0, i1, j0, j1 = crop
    free = grid.free()[i0:i1, j0:j1]
    occ = grid.occupied()[i0:i1, j0:j1]
    img = np.full(free.shape + (3,), 0.6)
    img[free] = (1, 1, 1)
    img[free & covered] = (0.7, 0.85, 1.0)
    img[occ] = (0, 0, 0)
    # odom x up, y left: rows = -x, cols = -y
    ext = [grid.origin + j1 * grid.p.res, grid.origin + j0 * grid.p.res,
           grid.origin + i0 * grid.p.res, grid.origin + i1 * grid.p.res]
    show = lambda a: np.flip(np.flip(a, 0), 1)  # noqa: E731
    fig, ax = plt.subplots(figsize=(12, 12))
    ax.imshow(show(img), extent=ext, interpolation='nearest')
    tr = np.array(track)
    ax.plot(tr[:, 1], tr[:, 0], 'm-', lw=1, label='LIO track')
    ax.plot(0, 0, 'r^', ms=10)
    colors = {'window': 'r', 'door': 'b', 'unknown': 'g'}
    for o in ops:
        c = colors.get(o.kind, 'k')
        ax.plot(o.center[1], o.center[0], 'o', color=c, ms=10, mfc='none' if not o.confirmed else c)
        ax.arrow(o.center[1], o.center[0], o.normal[1] * 0.5, o.normal[0] * 0.5, color=c, width=0.02)
        ax.annotate(f'{o.kind} {o.width:.2f}', (o.center[1], o.center[0]), color=c, fontsize=9,
                    xytext=(5, 5), textcoords='offset points')
    if arena is not None:
        cs = np.vstack([arena.corners(), arena.corners()[:1]])
        ax.plot(cs[:, 1], cs[:, 0], 'c--')
    ax.set_xlabel('y (left +) [m]')
    ax.set_ylabel('x (forward +) [m]')
    ax.set_title(f'{bag}: flight band {flight_z}+-{grid.p.band}, light blue = covered, '
                 'o filled = confirmed opening (red window, blue door, green unknown)', fontsize=9)
    ax.grid(True, alpha=0.3)
    fig.savefig(f'{out}_grid.png', dpi=90)

    fig, axs = plt.subplots(1, 2, figsize=(18, 9))
    for a, layer, name in ((axs[0], grid.roof, 'roof hits'), (axs[1], grid.low, 'low-band (vertical) hits')):
        im = a.imshow(show(np.minimum(layer[i0:i1, j0:j1], 20)), extent=ext, cmap='viridis',
                      interpolation='nearest')
        a.set_title(name)
        a.grid(True, alpha=0.3)
        fig.colorbar(im, ax=a, shrink=0.7)
    fig.savefig(f'{out}_layers.png', dpi=80)
    print('wrote', f'{out}_grid.png', f'{out}_layers.png')


if __name__ == '__main__':
    main()
