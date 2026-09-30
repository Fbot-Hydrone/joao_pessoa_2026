"""Rulebook fallback: fit the 2 x 6 m structure and predict its openings.

The rules: a 2 x 6 x 1.5 m box along an arena wall, the takeoff pad beside one
end. The entrance window is in that near end wall, the door in the open long
face near it, the exit window in the open long face at the far end. The arena
is 8 x 8 m with the pad + structure spanning one wall.

Only used when the generic pipeline times out or can't decide; the mission logs
every time it does.
"""
from dataclasses import dataclass

import numpy as np

from hydrone_mission.maze.arena import robust_rect
from hydrone_mission.maze.openings import Opening


@dataclass
class FallbackParams:
    length: float = 6.0
    width: float = 2.0
    pad: float = 2.0            # takeoff pad side, along the wall
    arena: float = 8.0
    exit_from_end: float = 0.6  # exit window centre from the far end
    door_from_end: float = 1.5
    min_roof_cells: int = 15
    match_tol: float = 0.5      # a detected opening this close to a prediction is it


@dataclass
class BoxFit:
    near_end: np.ndarray        # (2,) centre of the end wall beside the pad
    d: np.ndarray               # unit, near end -> far end
    open_side: np.ndarray       # unit, from the box axis toward the open long face
    length: float
    width: float

    @property
    def center(self):
        return self.near_end + self.d * self.length / 2

    def corners(self):
        w = self.open_side * self.width / 2
        a, b = self.near_end, self.near_end + self.d * self.length
        return np.array([a + w, b + w, b - w, a - w])


def fit_box(roof_xy, takeoff_xy, params=None, free_score=None):
    """BoxFit from roof-evidence points, or None.

    free_score(xy_points) -> number of observed-free cells there; used to tell
    the open long face (arena side, free) from the one on the arena wall.
    """
    p = params or FallbackParams()
    roof_xy = np.asarray(roof_xy, dtype=float)
    if len(roof_xy) < p.min_roof_cells:
        return None
    rect = robust_rect(roof_xy, trim=1.0)
    if rect is None:
        return None
    u, v = rect.axes
    tk = np.asarray(takeoff_xy, dtype=float) - rect.center
    # the long axis is the one the pad sits beyond; else the longer side
    beyond_u = abs(tk @ u) - rect.size[0] / 2
    beyond_v = abs(tk @ v) - rect.size[1] / 2
    if beyond_u > 0 and beyond_v <= 0.5:
        axis, size, across, across_size = u, rect.size[0], v, rect.size[1]
    elif beyond_v > 0 and beyond_u <= 0.5:
        axis, size, across, across_size = v, rect.size[1], u, rect.size[0]
    elif rect.size[0] >= rect.size[1]:
        axis, size, across, across_size = u, rect.size[0], v, rect.size[1]
    else:
        axis, size, across, across_size = v, rect.size[1], u, rect.size[0]
    s = 1.0 if tk @ axis > 0 else -1.0
    near_end = rect.center + axis * s * size / 2
    d = -s * axis
    # across: trust the observed centre if the width is seen, else slide it
    # so the long face with more free space outside stays where it was seen
    open_side = across
    if free_score is not None:
        probe = lambda side: free_score(  # noqa: E731
            [near_end + d * k + side * (across_size / 2 + 0.6) for k in np.linspace(0.5, size - 0.5, 8)])
        open_side = across if probe(across) >= probe(-across) else -across
    if across_size < p.width - 0.4:
        # seen from outside, the open face is the part we saw: keep its line
        near_end = near_end + open_side * (across_size / 2 - p.width / 2)
    return BoxFit(near_end=near_end, d=d, open_side=open_side, length=p.length, width=p.width)


def predictions(box, params=None):
    """Predicted entrance / door / exit Openings and the landing point."""
    p = params or FallbackParams()
    w = box.open_side
    face = box.near_end + w * box.width / 2
    entrance = Opening(center=box.near_end.copy(), width=0.8, normal=box.d.copy(),
                       confidence=0.2, kind='predicted')
    door = Opening(center=face + box.d * p.door_from_end, width=0.8, normal=-w,
                   confidence=0.2, kind='predicted')
    exit_ = Opening(center=face + box.d * (box.length - p.exit_from_end), width=0.8, normal=-w,
                    confidence=0.2, kind='predicted')
    wall_line = box.near_end - w * box.width / 2
    landing = wall_line - box.d * p.pad + box.d * p.arena / 2 + w * p.arena / 2
    return {'entrance': entrance, 'door': door, 'exit': exit_, 'landing': landing}


def match(pred, openings, params=None, wall_span=None, skip_doors=True):
    """The detected opening that sits on the predicted wall, or None.

    A match must face the same way and lie on the same wall line, within
    wall_span along it (default: anywhere within match_tol of the centre).
    skip_doors=False trusts the rulebook over a door classification (the
    near end wall has the window, whatever the sill evidence said).
    """
    p = params or FallbackParams()
    t = np.array([-pred.normal[1], pred.normal[0]])
    best, best_d = None, np.inf
    for o in openings:
        if skip_doors and o.kind == 'door':
            continue
        off = o.center - pred.center
        if abs(off @ pred.normal) > p.match_tol or abs(np.dot(o.normal, pred.normal)) < 0.7:
            continue
        along = abs(off @ t)
        if along > (wall_span if wall_span is not None else p.match_tol):
            continue
        if along < best_d:
            best, best_d = o, along
    return best
