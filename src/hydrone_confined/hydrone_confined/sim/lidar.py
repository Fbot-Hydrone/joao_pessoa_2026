"""Mid-360-like ray caster against axis-aligned boxes and the floor (slab method)."""
import numpy as np

EL_MIN, EL_MAX = np.deg2rad(-7.0), np.deg2rad(52.0)


def directions(n, rng):
    """Random unit rays over the Mid-360 FOV: 360 deg azimuth, -7..52 deg elevation."""
    az = rng.uniform(0, 2 * np.pi, n)
    el = rng.uniform(EL_MIN, EL_MAX, n)
    ce = np.cos(el)
    return np.stack([ce * np.cos(az), ce * np.sin(az), np.sin(el)], axis=1)


def cast(origin, dirs, lo, hi, max_range=40.0, skip=None):
    """Ranges (inf = no return) and the index of the box hit (-1 floor, -2 none).

    skip: bool mask over boxes to ignore.
    """
    o = np.asarray(origin, dtype=float)
    n = len(dirs)
    best = np.full(n, np.inf)
    which = np.full(n, -2, dtype=int)
    with np.errstate(divide='ignore', invalid='ignore'):
        inv = 1.0 / dirs
        down = dirs[:, 2] < 0
        tf = np.where(down, -o[2] / dirs[:, 2], np.inf)
    tf = np.where(tf > 0, tf, np.inf)
    best = np.minimum(best, tf)
    which[np.isfinite(tf)] = -1
    for k in range(len(lo)):
        if skip is not None and skip[k]:
            continue
        with np.errstate(invalid='ignore'):
            t1 = (lo[k] - o) * inv
            t2 = (hi[k] - o) * inv
        tmin = np.nan_to_num(np.minimum(t1, t2), nan=-np.inf).max(axis=1)
        tmax = np.nan_to_num(np.maximum(t1, t2), nan=np.inf).min(axis=1)
        hit = (tmax >= np.maximum(tmin, 0)) & (tmin > 1e-6)
        t = np.where(hit, tmin, np.inf)
        closer = t < best
        best[closer] = t[closer]
        which[closer] = k
    best[best > max_range] = np.inf
    which[~np.isfinite(best)] = -2
    return best, which


def scan(origin, lo, hi, n_rays, rng, noise=0.02, max_range=40.0, drop_tags=None, tags=None):
    """World-frame points of one sweep. drop_tags: returns off boxes with these tags are lost."""
    dirs = directions(n_rays, rng)
    r, which = cast(origin, dirs, lo, hi, max_range)
    ok = np.isfinite(r)
    if drop_tags and tags is not None:
        bad = np.array([t in drop_tags for t in tags])
        ok &= ~((which >= 0) & bad[np.maximum(which, 0)])
    r = r[ok] + rng.normal(0, noise, ok.sum())
    return np.asarray(origin) + dirs[ok] * r[:, None]
