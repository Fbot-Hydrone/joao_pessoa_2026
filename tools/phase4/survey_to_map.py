#!/usr/bin/env python3
"""Turn a teleport survey (survey_teleport.py) into a measured KnownMap.

1. body -> world through each pose's ground truth (BiguaSim: right-handed,
   x forward, y left, z up; rotations roll/pitch/yaw in degrees)
2. world -> map: start from the survey's guess, then fit (x, y, yaw) so the
   wall-height returns sit on the reference map's walls (2D ICP), and the
   floor height from the floor returns
3. for every wall of the reference map, an occupancy image over (along the
   wall, height) from the returns within its thickness: the empty rectangles
   are its openings, measured
4. write the map (walls from the reference, openings as measured, takeoff
   from the spawn) + height slices as PNGs for a look

    python3 tools/phase4/survey_to_map.py survey.npz out_dir [--ref rulebook.yaml]
"""
import argparse
import copy
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, 'src', 'hydrone_confined'))

from hydrone_confined.core.known_map import KnownMap  # noqa: E402

sys.path.insert(0, HERE)
from survey_teleport import FLOOR_Z, MAP_TO_WORLD_X, MAP_TO_WORLD_Y, SPAWN, rpy_matrix  # noqa: E402


def world_points(d):
    if 'world_frame' in d.files:
        return np.concatenate([d[f'p{k}'] for k in range(int(d['n'])) if len(d[f'p{k}'])])
    out = []
    for k in range(int(d['n'])):
        p = d[f'p{k}']
        if not len(p):
            continue
        R = rpy_matrix(*d['rot'][k])
        out.append(p @ R.T + d['loc'][k])
    return np.concatenate(out)


def guess_to_map(w):
    """The survey's own guess: a 180 deg turn."""
    return np.column_stack([MAP_TO_WORLD_X - w[:, 0], MAP_TO_WORLD_Y - w[:, 1], w[:, 2] - FLOOR_Z])


def fit_2d(pts, km, band=(0.3, 1.3), iters=30):
    """(dx, dy, dyaw) that moves pts (map guess) onto km's walls: point-to-segment ICP."""
    segs = km.segments(*band)
    a, b = segs[:, 0], segs[:, 1]
    m = (pts[:, 2] > band[0]) & (pts[:, 2] < band[1])
    x0, x1, y0, y1 = km.structure
    m &= (pts[:, 0] > x0 - 0.3) & (pts[:, 0] < x1 + 0.3) & (pts[:, 1] > y0 - 0.3) & (pts[:, 1] < y1 + 0.3)
    P = pts[m, :2]
    if len(P) > 30000:
        P = P[np.random.default_rng(0).choice(len(P), 30000, replace=False)]
    # the walls repeat every ~1 m along the structure, so ICP alone can lock a
    # period off: first a coarse search of the shift on a wall raster
    occ, org, res = km.rasterize(band, res=0.05, margin=1.5)
    best = (-1, 0.0, 0.0)
    for dx in np.arange(-1.5, 1.51, 0.05):
        for dy in np.arange(-0.4, 0.41, 0.05):
            ij = np.floor((P + [dx, dy] - org) / res).astype(int)
            ok = (ij[:, 0] >= 0) & (ij[:, 0] < occ.shape[0]) & (ij[:, 1] >= 0) & (ij[:, 1] < occ.shape[1])
            sc = int(occ[ij[ok, 0], ij[ok, 1]].sum())
            if sc > best[0]:
                best = (sc, dx, dy)
    T = np.array([best[1], best[2], 0.0])
    for it in range(iters):
        c, s = np.cos(T[2]), np.sin(T[2])
        Q = P @ np.array([[c, s], [-s, c]]) + T[:2]
        ab = b - a
        L2 = np.maximum((ab ** 2).sum(1), 1e-12)
        u = np.clip(((Q[:, None, :] - a[None]) * ab[None]).sum(-1) / L2[None], 0, 1)
        proj = a[None] + u[..., None] * ab[None]
        dist = np.linalg.norm(Q[:, None, :] - proj, axis=-1)
        k = dist.argmin(1)
        dmin = dist[np.arange(len(Q)), k]
        keep = dmin < (0.3 if it < 10 else 0.1)
        src, dst = Q[keep], proj[np.arange(len(Q)), k][keep]
        # closed-form 2D rigid alignment of src onto dst
        ms, md = src.mean(0), dst.mean(0)
        H = (src - ms).T @ (dst - md)
        ang = np.arctan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
        ca, sa = np.cos(ang), np.sin(ang)
        R = np.array([[ca, -sa], [sa, ca]])
        t = md - ms @ R.T
        # compose with T
        Rt = np.array([[np.cos(T[2]), -np.sin(T[2])], [np.sin(T[2]), np.cos(T[2])]])
        T = np.array([*(R @ T[:2] + t), T[2] + ang])
        if abs(ang) < 1e-5 and np.linalg.norm(t) < 1e-4:
            break
    return T, float(np.median(dmin[keep])) if keep.any() else np.inf


def apply_2d(pts, T):
    c, s = np.cos(T[2]), np.sin(T[2])
    out = pts.copy()
    out[:, :2] = pts[:, :2] @ np.array([[c, s], [-s, c]]) + T[:2]
    return out


def measure_openings(pts, w, res=0.03, z_top=1.5, z_seen=0.0, margin=0.035, min_w=0.3, min_h=0.3):
    """Empty rectangles in a wall's (u, z) occupancy image: [(u0, u1, z0, z1)].

    Only z_seen..z_top was observed; a rectangle starting at z_seen goes to the floor.
    """
    t, n = w.tangent, w.normal
    rel = pts[:, :2] - w.a
    u = rel @ t
    d = np.abs(rel @ n)
    m = (d < w.t / 2 + margin) & (u > -0.02) & (u < w.length + 0.02) & (pts[:, 2] > z_seen) & (pts[:, 2] < z_top)
    nu, nz = int(np.ceil(w.length / res)), int(np.ceil(z_top / res))
    img = np.zeros((nu, nz), dtype=int)
    iu = np.clip((u[m] / res).astype(int), 0, nu - 1)
    iz = np.clip((pts[m, 2] / res).astype(int), 0, nz - 1)
    np.add.at(img, (iu, iz), 1)
    occ = img > 0
    # close pinholes: a cell counts as wall if most of its 3x3 neighbourhood is
    pad = np.pad(occ, 1)
    nb = sum(pad[1 + a:1 + a + nu, 1 + b:1 + b + nz] for a in (-1, 0, 1) for b in (-1, 0, 1))
    wall = nb >= 3
    free = ~wall
    free[:, :int(np.ceil(z_seen / res))] = False      # never seen: not evidence of anything
    out = []
    # grow maximal empty rectangles column by column (the openings are axis-aligned)
    seen = np.zeros_like(free)
    for i in range(nu):
        for j in range(nz):
            if not free[i, j] or seen[i, j]:
                continue
            j1 = j
            while j1 + 1 < nz and free[i, j1 + 1]:
                j1 += 1
            i1 = i
            while i1 + 1 < nu and free[i1 + 1, j:j1 + 1].all():
                i1 += 1
            seen[i:i1 + 1, j:j1 + 1] = True
            wu, wz = (i1 - i + 1) * res, (j1 - j + 1) * res
            z0 = 0.0 if j * res <= z_seen + res else j * res
            if wu >= min_w and (j1 + 1) * res - z0 >= min_h:
                out.append((round(i * res, 2), round((i1 + 1) * res, 2), round(z0, 2),
                            round(min((j1 + 1) * res, z_top), 2)))
    return out, int(m.sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('survey')
    ap.add_argument('out_dir')
    ap.add_argument('--ref', default=os.path.join(ROOT, 'src', 'hydrone_mission', 'config', 'phase4',
                                                   'known_maps', 'rulebook.yaml'))
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    d = np.load(a.survey)
    km = KnownMap.from_yaml(a.ref)
    w = world_points(d)
    print(f'{len(w)} points from {int(d["n"])} poses')
    g = guess_to_map(w)
    # the floor is hardly seen (lowest ray -4 deg); take the survey's guess and
    # check it against the ceiling, the densest layer over the structure
    floor = 0.0
    x0, x1, y0, y1 = km.structure
    zin = g[(g[:, 0] > x0 + 0.3) & (g[:, 0] < x1 - 0.3) & (g[:, 1] > y0 + 0.2) & (g[:, 1] < y1 - 0.2), 2]
    h, e = np.histogram(zin, bins=np.arange(0.5, 2.5, 0.01))
    ceiling = float(e[h.argmax()] + 0.005)
    print(f'ceiling {ceiling:.3f} m over the floor guess')
    # below the lowest sensor the walls were never seen: openings reaching down
    # to it are floor-level holes
    z_seen = float(d['loc'][:, 2].min() - FLOOR_Z + 0.1 - 0.04)
    g[:, 2] -= floor
    T, resid = fit_2d(g, km)
    print(f'floor at guess z {floor:+.3f}; fit dx {T[0]:+.3f} dy {T[1]:+.3f} yaw {np.degrees(T[2]):+.2f} deg, '
          f'median wall residual {resid:.3f} m')
    pm = apply_2d(g, T)
    np.save(os.path.join(a.out_dir, 'survey_map_points.npy'), pm[::3].astype(np.float32))

    with open(a.ref) as f:
        doc = yaml.safe_load(f)
    doc = copy.deepcopy(doc)
    doc['name'] = 'cbr2026-biguasim'
    report = []
    for wd, wobj in zip(doc['walls'], km.walls):
        ops, n = measure_openings(pm, wobj, z_top=min(ceiling, 1.5) - 0.02, z_seen=z_seen)
        ref = [(o.name, o.u, o.z) for o in wobj.openings]
        report.append(f'{wobj.name:12s} pts {n:6d} measured {ops}  ref {[(nm, u, z) for nm, u, z in ref]}')
        # match each reference opening to the measured one overlapping it most; keep names/roles
        new = []
        used = set()
        for o in wd.get('openings', []):
            best, bo = None, 0.0
            for k, (u0, u1, z0, z1) in enumerate(ops):
                ov = max(0.0, min(u1, o['u'][1]) - max(u0, o['u'][0]))
                if ov > bo:
                    best, bo = k, ov
            if best is not None:
                used.add(best)
                u0, u1, z0, z1 = ops[best]
                o = dict(o, u=[u0, u1], z=[max(z0, 0.0), z1])
                o.pop('soft', None)
            else:
                o = dict(o, note='not seen in the survey')
            new.append(o)
        for k, (u0, u1, z0, z1) in enumerate(ops):
            if k not in used:
                new.append({'name': f'{wobj.name}_extra{k}', 'kind': 'hole' if z0 < 0.2 else 'window',
                            'u': [u0, u1], 'z': [z0, z1]})
        if new:
            wd['openings'] = new
    # where BiguaSim spawns the drone, in this map
    sp = apply_2d(guess_to_map(np.array([SPAWN])), T)[0]
    doc['takeoff']['center'] = [round(float(sp[0]), 3), round(float(sp[1]), 3)]
    doc['takeoff']['yaw_deg'] = round(float(90.0 + np.degrees(T[2])), 2)
    doc['takeoff']['top'] = round(float(sp[2] - floor - 0.08), 3)
    doc['survey'] = {'world_to_map': {'guess': [MAP_TO_WORLD_X, MAP_TO_WORLD_Y, FLOOR_Z],
                                      'fit_dx_dy_yawdeg': [round(float(T[0]), 4), round(float(T[1]), 4),
                                                           round(float(np.degrees(T[2])), 3)],
                                      'floor': round(floor, 4), 'wall_residual': round(resid, 4)}}
    out_yaml = os.path.join(a.out_dir, 'biguasim.yaml')
    with open(out_yaml, 'w') as f:
        f.write('# CBR 2026 Phase 4 structure as built in BiguaSim, measured by a teleport survey\n'
                '# (tools/phase4/survey_teleport.py + survey_to_map.py). Walls from the rulebook,\n'
                '# openings and the takeoff pose measured.\n')
        yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=None)
    print('\n'.join(report))
    print('wrote', out_yaml)
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        bands = [(0.1, 0.3), (0.35, 0.55), (0.65, 0.85), (0.95, 1.15), (1.2, 1.4)]
        fig, ax = plt.subplots(len(bands), 1, figsize=(13, 3.2 * len(bands)))
        x0, x1, y0, y1 = km.structure
        for axx, (z0, z1) in zip(ax, bands):
            s = pm[(pm[:, 2] > z0) & (pm[:, 2] < z1)]
            s = s[(s[:, 0] > x0 - 1.5) & (s[:, 0] < x1 + 1) & (s[:, 1] > y0 - 0.5) & (s[:, 1] < y1 + 1.5)]
            axx.scatter(s[:, 0], s[:, 1], s=0.3, c='k')
            for wobj in km.walls:
                axx.plot([wobj.a[0], wobj.b[0]], [wobj.a[1], wobj.b[1]], 'r-', lw=0.6, alpha=0.6)
            axx.set_title(f'survey, z {z0}..{z1} (red: rulebook wall lines)')
            axx.set_aspect('equal')
            axx.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(os.path.join(a.out_dir, 'survey_slices.png'), dpi=90)
    except ImportError:
        pass


if __name__ == '__main__':
    main()
