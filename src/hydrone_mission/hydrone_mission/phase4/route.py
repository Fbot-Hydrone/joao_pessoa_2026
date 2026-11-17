"""The route through a known maze: every room, every wall in view, then out.

Rooms and passages come from the KnownMap. Two rooms are connected by a
passage (an interior opening between them) or openly, when their rectangles
touch and nothing walls the seam at cruise height (the two arms of an L).

The tour is a depth-first walk from the entry room over that graph (nearest
neighbour first), with the walk back out of dead ends, ending in the room with
the exit. Each room gets look points: its centre, or two along the long side
when it is long. At a look point the drone turns once around so a camera
sees every wall (QR codes).

Legs, in order:
  ('look', room, xy)            go there inside the room, turn around
  ('cross', passage, a, b)      from room a through the passage into room b
  ('move', room, xy)            open seam: just fly
  ('exit', passage, room)       through the exit window, out
"""
from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass
class Edge:
    a: str
    b: str
    kind: str                   # passage / open
    passage: object = None      # known_map.PassageRef
    seam: np.ndarray = None     # a point on the open seam
    blocked: bool = False

    def other(self, r):
        return self.b if r == self.a else self.a


class RoomGraph:

    def __init__(self, km, cruise_z=0.75):
        self.km = km
        self.rooms = {r.name: r for r in km.rooms}
        self.edges = []
        for p in km.passages('interior'):
            a = km.room_at(p.center + p.normal * 0.25)
            b = km.room_at(p.center - p.normal * 0.25)
            if a is not None and b is not None and a.name != b.name:
                self.edges.append(Edge(a.name, b.name, 'passage', passage=p))
        names = list(self.rooms)
        linked = {frozenset((e.a, e.b)) for e in self.edges}
        for i, na in enumerate(names):
            for nb in names[i + 1:]:
                if frozenset((na, nb)) in linked:
                    continue        # a door is already a passage between them
                seam = self._open_seam(self.rooms[na], self.rooms[nb], cruise_z)
                if seam is not None:
                    self.edges.append(Edge(na, nb, 'open', seam=seam))
        self.entry = self._room_behind('entry')
        self.exit = self._room_behind('exit')

    def _room_behind(self, role):
        ps = self.km.passages(role)
        if not ps:
            return None
        p = ps[0]
        for s in (1, -1):
            r = self.km.room_at(p.center + s * p.normal * 0.3)
            if r is not None:
                return r.name
        return None

    def _open_seam(self, ra, rb, z, gap=0.15, min_run=0.4):
        """Middle of the unwalled part of the seam between two touching rooms, or None."""
        for ax in (0, 1):
            other = 1 - ax
            lo_a, hi_a = (ra.x, ra.y)[ax]
            lo_b, hi_b = (rb.x, rb.y)[ax]
            if 0 <= lo_b - hi_a <= gap:
                at = 0.5 * (hi_a + lo_b)
            elif 0 <= lo_a - hi_b <= gap:
                at = 0.5 * (hi_b + lo_a)
            else:
                continue
            s0 = max((ra.x, ra.y)[other][0], (rb.x, rb.y)[other][0])
            s1 = min((ra.x, ra.y)[other][1], (rb.x, rb.y)[other][1])
            if s1 - s0 < min_run:
                continue
            s = np.arange(s0, s1, 0.02)
            pts = np.zeros((len(s), 2))
            pts[:, ax], pts[:, other] = at, s
            free = ~self._occupied(pts, z)
            # longest free run along the seam
            best, cur, best_end = 0, 0, 0
            for k, f in enumerate(free):
                cur = cur + 1 if f else 0
                if cur > best:
                    best, best_end = cur, k
            if best * 0.02 >= min_run:
                mid = pts[best_end - best // 2]
                return mid
        return None

    def _occupied(self, pts, z):
        occ, origin, res = self.km.rasterize((z - 0.1, z + 0.1), res=0.02, margin=0.2)
        ij = np.floor((pts - origin) / res).astype(int)
        ij[:, 0] = np.clip(ij[:, 0], 0, occ.shape[0] - 1)
        ij[:, 1] = np.clip(ij[:, 1], 0, occ.shape[1] - 1)
        return occ[ij[:, 0], ij[:, 1]]

    def neighbours(self, r):
        return [(e.other(r), e) for e in self.edges if not e.blocked and r in (e.a, e.b)]

    def dist(self, a, b):
        return float(np.linalg.norm(self.rooms[a].center - self.rooms[b].center))

    def shortest(self, a, b):
        """Rooms a..b (Dijkstra on centre distances), with the edges between, or None."""
        best = {a: (0.0, None, None)}
        todo = [(0.0, a)]
        while todo:
            todo.sort()
            d, r = todo.pop(0)
            if r == b:
                break
            if d > best[r][0]:
                continue
            for n, e in self.neighbours(r):
                nd = d + self.dist(r, n)
                if n not in best or nd < best[n][0]:
                    best[n] = (nd, r, e)
                    todo.append((nd, n))
        if b not in best:
            return None
        rooms, edges = [b], []
        while rooms[-1] != a:
            _, prev, e = best[rooms[-1]]
            rooms.append(prev)
            edges.append(e)
        return rooms[::-1], edges[::-1]

    def reachable(self, a):
        seen, q = {a}, deque([a])
        while q:
            r = q.popleft()
            for n, _ in self.neighbours(r):
                if n not in seen:
                    seen.add(n)
                    q.append(n)
        return seen


def look_points(room, long_split=1.3):
    """Where to stand in a room to see all of it."""
    w, h = room.x[1] - room.x[0], room.y[1] - room.y[0]
    c = room.center
    if max(w, h) <= long_split:
        return [c]
    ax = np.array([1.0, 0.0]) if w >= h else np.array([0.0, 1.0])
    L = max(w, h)
    return [c - ax * L / 4, c + ax * L / 4]


def plan_tour(graph, start, visited=(), end=None):
    """Legs covering every reachable, not yet visited room from `start`, ending in `end`."""
    end = end or graph.exit
    legs = []
    seen = set(visited)

    def step_to(a, e):
        b = e.other(a)
        if e.kind == 'passage':
            legs.append(('cross', e.passage, a, b))
        else:
            legs.append(('move', b, e.seam))
        return b

    def visit(r):
        seen.add(r)
        for q in look_points(graph.rooms[r]):
            legs.append(('look', r, q))

    def dfs(r):
        if r not in seen:
            visit(r)
        nbrs = sorted(graph.neighbours(r), key=lambda ne: graph.dist(r, ne[0]))
        # the end room last, so the tour finishes there
        nbrs.sort(key=lambda ne: ne[0] == end)
        for n, e in nbrs:
            if n in seen:
                continue
            step_to(r, e)
            last = dfs(n)
            if any(m not in seen for m in graph.reachable(start)):
                step_to(n, e)       # walk back out of the dead end
            else:
                return last
        return r

    here = dfs(start)
    if end is not None and here != end:
        path = graph.shortest(here, end)
        if path is not None:
            rooms, edges = path
            for a, e in zip(rooms[:-1], edges):
                step_to(a, e)
            if end not in seen:
                visit(end)
    return legs


def exit_legs(graph, here):
    """Shortest way from room `here` to the exit room, then out."""
    path = graph.shortest(here, graph.exit)
    if path is None:
        return None
    rooms, edges = path
    legs = []
    for a, e in zip(rooms[:-1], edges):
        legs.append(('cross', e.passage, a, e.other(a)) if e.kind == 'passage'
                    else ('move', e.other(a), e.seam))
    return legs
