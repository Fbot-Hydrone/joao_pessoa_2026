"""phase2_core — the Phase 2 decisions that need no ROS.

Pure functions, so they run in a plain pytest and in any other mission:

  base_entry / merge_positions  which base positions were given, which are not
  scan_path                     Phase 1's map sweep, for the bases nobody gave
  fill_missing                  found bases -> the missing pickups/deliveries
  clamp_to_disc                 keep the touch point where the legs stay on
  turn_to_axis                  heading that puts the kit along the gripper
  takeover                      did someone else take the vehicle?
"""

import math

import yaml

from hydrone_nav import coverage
from hydrone_nav.arena import _point


# ── where the bases are ─────────────────────────────────────────────────────

SCAN = "scan"     # a per-base parameter set to this forces that base missing


def base_entry(text, layout_entry, grid):
    """One base from its parameter, else from the layout file.

    `text` is the launch/node parameter: "" keeps the layout's entry, "scan"
    marks the base unknown, anything else is YAML for one layout entry
    ("[x, y]", "[x, y, z]" or "{grid: [col, row]}"), arena frame.
    Returns (x, y, ...) or None when the base is unknown.
    """
    text = (text or "").strip()
    if text.lower() == SCAN:
        return None
    if not text:
        return tuple(layout_entry) if layout_entry is not None else None
    return _point(yaml.safe_load(text), grid)


def merge_positions(texts, layout_list, grid):
    """[param per slot] + the layout's list -> [entry or None per slot]."""
    out = []
    for i, t in enumerate(texts):
        lay = layout_list[i] if i < len(layout_list) else None
        out.append(base_entry(t, lay, grid))
    return out


# ── the scan: Phase 1's map sweep ───────────────────────────────────────────

def scan_path(plan_bounds, home_yaw, here, *, side="right", inset_m=1.2,
              z=2.5, swath_m=None, overlap=0.25):
    """The closed perimeter, then lanes spaced by the belly camera's swath.

    The same two levels phase1_mission_node flies in search_mode map_sweep
    (perimeter_sweep + camera_lawnmower, built in the sweep frame of the
    takeoff heading). No swath -> perimeter only. The lanes fly on one
    heading.
    plan_bounds is [x0, y0, z0, x1, y1, z1] in map.
    """
    b = ((plan_bounds[0], plan_bounds[1], plan_bounds[2]),
         (plan_bounds[3], plan_bounds[4], plan_bounds[5]))
    framed = coverage.sweep_frame_bounds(b, home_yaw, side)
    (x0, y0, _), (x1, y1, _) = framed
    corners = [(x0 + inset_m, y0 + inset_m), (x1 - inset_m, y0 + inset_m),
               (x1 - inset_m, y1 - inset_m), (x0 + inset_m, y1 - inset_m)]
    start = coverage.nearest_corner_index(
        corners, coverage.sweep_frame_xy(here, home_yaw, side))
    perim = coverage.sweep_frame_path(
        coverage.perimeter_sweep(framed, inset_m=inset_m, z=z,
                                 start_corner=start), home_yaw, side)
    if not swath_m:
        return perim
    lanes = coverage.sweep_frame_path(
        coverage.camera_lawnmower(framed, swath_m=swath_m, z=z,
                                  overlap=overlap, margin_m=inset_m),
        home_yaw, side)
    # Phase 1 flips 180 deg at every lane end; a belly camera sees the same
    # strip either way, so the lanes keep the perimeter's last heading.
    yaw = perim[-1][3] if perim else home_yaw
    return perim + [(x, y, zz, yaw) for x, y, zz, _ in lanes]


def fill_missing(pick, deliv, pads, kits_xy, *, home, ground_z=0.0,
                 min_obs=3, kit_radius_m=0.6, min_kit_hits=3,
                 pickup_min_top_m=1.0, known_gap_m=0.8, home_gap_m=1.0):
    """Fill the None slots of `pick`/`deliv` (map xy) from the scan.

    pads     [(x, y, top_z, observations, is_takeoff_base), ...] from pad_map
    kits_xy  [(x, y), ...] every kit the belly saw during the scan

    A pickup is a base with a KIT on it (rules: deliveries are empty); when
    too few kits were seen, the tallest bases on the 1.5 m block fill in.
    Deliveries are the bases with no kit, most-seen first. A slot nobody can
    fill stays None. Returns (pick, deliv, notes).
    """
    pick, deliv = list(pick), list(deliv)
    known = [p for p in pick + deliv if p is not None]

    def near(a, b, r):
        return math.hypot(a[0] - b[0], a[1] - b[1]) <= r

    cands = []
    for x, y, top, obs, takeoff in pads:
        if takeoff or obs < min_obs or near((x, y), home, home_gap_m):
            continue
        if any(near((x, y), k, known_gap_m) for k in known):
            continue
        hits = sum(1 for k in kits_xy if near((x, y), k, kit_radius_m))
        cands.append({"xy": (x, y), "top": top - ground_z, "obs": obs,
                      "hits": hits})
    notes = [f"base at ({c['xy'][0]:.2f}, {c['xy'][1]:.2f}) top "
             f"{c['top']:.2f} m, {c['obs']} looks, {c['hits']} kit sightings"
             for c in cands]

    def take(slots, pool):
        for i, s in enumerate(slots):
            if s is None and pool:
                c = pool.pop(0)
                slots[i] = c["xy"]
                cands.remove(c)

    with_kit = sorted((c for c in cands if c["hits"] >= min_kit_hits),
                      key=lambda c: (-c["hits"], -c["obs"]))
    take(pick, with_kit)
    tall = sorted((c for c in cands if c["top"] >= pickup_min_top_m),
                  key=lambda c: (-c["hits"], -c["top"]))
    take(pick, tall)
    empty = sorted((c for c in cands if c["hits"] < min_kit_hits),
                   key=lambda c: -c["obs"])
    take(deliv, empty)
    return pick, deliv, notes


# ── geometry ────────────────────────────────────────────────────────────────

def clamp_to_disc(p, centre, r):
    """`p` pulled inside the disc of radius `r` around `centre`."""
    dx, dy = p[0] - centre[0], p[1] - centre[1]
    d = math.hypot(dx, dy)
    if d <= r or d == 0.0:
        return tuple(p)
    k = r / d
    return (centre[0] + dx * k, centre[1] + dy * k)


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def wrap_half_pi(a):
    """An axis angle (defined mod pi) into [-pi/2, pi/2)."""
    return (a + math.pi / 2.0) % math.pi - math.pi / 2.0


def mean_axis(angles):
    """Mean of axis angles (mod pi), by doubling. None for an empty list."""
    if not angles:
        return None
    s = sum(math.sin(2.0 * a) for a in angles)
    c = sum(math.cos(2.0 * a) for a in angles)
    return 0.5 * math.atan2(s, c)


def turn_to_axis(axis_img, axes, gripper_axis=0.0):
    """Yaw change that lays the kit along the gripper's grabbing axis.

    axis_img      the kit's long axis in the image, rad, from +u towards +v
    axes          VisualDescent.axes (body y per +u, body x per +v), FLU
    gripper_axis  body direction the kit's long axis must lie along, rad
                  (0 = nose to tail: the fingers close sideways)
    The smallest turn, in [-pi/2, pi/2): an axis has no front.
    """
    bx = axes[1] * math.sin(axis_img)
    by = axes[0] * math.cos(axis_img)
    return wrap_half_pi(math.atan2(by, bx) - gripper_axis)


# ── the pilot ───────────────────────────────────────────────────────────────

FLYING = ("TAKEOFF", "SCAN", "TURN", "GOTO", "DESCEND", "ALIGN", "TOUCH",
          "VERIFY")
ON_BASE = ("WAIT", "ACT")


def takeover(state, armed, mode, *, ours, airborne, landed_mode="LAND"):
    """Why the vehicle is no longer the mission's, or None.

    ours      the mission armed it and has not landed it yet
    airborne  off the base (a refused takeoff disarms on the base: not a pilot)
    The RC is kill-only (and in Phase 2 a human may send it home): any arm,
    disarm or mode change the mission did not ask for ends the run for good.
    """
    if state in FLYING and ours:
        if not armed and (state != "TAKEOFF" or airborne):
            return f"disarmed in {state} without being told"
        if armed and mode != "GUIDED":
            return f"mode {mode} in {state}, not GUIDED"
    if state in ON_BASE and not ours:
        if armed:
            return f"armed in {state} by someone else"
        if state == "ACT" and mode not in (landed_mode, "GUIDED"):
            return f"mode changed to {mode} while sitting on a base"
    return None
