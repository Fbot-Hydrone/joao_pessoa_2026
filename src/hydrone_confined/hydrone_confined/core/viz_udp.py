"""Debug stream to tools/maze_viewer: JSON datagrams over UDP.

Fire and forget: nothing listening costs nothing, and a slow viewer never
slows the stack. Messages (one datagram each, keep them under ~60 kB):

  {"type": "map", walls: [{a, b, t, kind, openings: [{u, z, kind, role, soft, name}]}],
   rooms: [{name, x, y}], structure: [x0, x1, y0, y1], arena: [...],
   takeoff: {center, size}, landing: [x, y]}
  {"type": "frame", t, phase, mode, note, truth: [x, y, z, yaw] (sim only),
   est: [x, y, z, yaw], sensor: [x, y], beams: [[world angle, range]...],
   sectors: [[world angle, range]...], particles: [[x, y]...], path: [[x, y]...],
   goal: [x, y], qr_world: {letter: [x, y]}, qr_seen: [letters]}
  {"type": "reset"}

The map is re-sent every few seconds so a viewer started late still gets it.
"""
import json
import socket
import time

import numpy as np


def _f(x, nd=3):
    return None if x is None else [round(float(v), nd) for v in np.ravel(x)]


def map_message(km):
    walls = []
    for w in km.walls:
        walls.append({'a': _f(w.a), 'b': _f(w.b), 't': float(w.t), 'kind': w.kind, 'name': w.name,
                      'openings': [{'u': _f(o.u), 'z': _f(o.z), 'kind': o.kind, 'role': o.role,
                                    'soft': bool(o.soft), 'name': o.name} for o in w.openings]})
    to = km.takeoff or {}
    return {'type': 'map', 'walls': walls,
            'rooms': [{'name': r.name, 'x': _f(r.x), 'y': _f(r.y)} for r in km.rooms],
            'structure': _f(km.structure), 'arena': _f(km.arena),
            'takeoff': {'center': _f(to.get('center')), 'size': _f(to.get('size'))} if 'center' in to else {},
            'landing': _f(km.landing)}


class VizUdp:

    def __init__(self, host='127.0.0.1', port=47800, hz=15.0, map_every=3.0):
        self.addr = (host, int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.period = 1.0 / hz if hz > 0 else 0.0
        self.map_every = map_every
        self._map = None
        self._t_map = -1e9
        self._t_frame = -1e9

    def _send(self, msg):
        try:
            self.sock.sendto(json.dumps(msg, separators=(',', ':')).encode(), self.addr)
        except (BlockingIOError, OSError):
            pass            # nobody listening, or the buffer is full: drop it

    def set_map(self, km):
        self._map = map_message(km)
        self._send(self._map)
        self._t_map = time.monotonic()

    def reset(self):
        self._send({'type': 'reset'})

    def frame(self, t, est, phase='', mode='', truth=None, fix=None, particles=None, path=None,
              goal=None, qr_world=None, qr_seen=(), note='', mount=(0.0, 0.0), max_particles=150):
        now = time.monotonic()
        if self._map is not None and now - self._t_map > self.map_every:
            self._send(self._map)
            self._t_map = now
        if now - self._t_frame < self.period:
            return
        self._t_frame = now
        msg = {'type': 'frame', 't': round(float(t), 2), 'phase': phase, 'mode': mode, 'note': note,
               'est': _f(est), 'truth': _f(truth), 'qr_seen': list(qr_seen)}
        if est is not None:
            c, s = np.cos(est[3]), np.sin(est[3])
            sensor = np.array([est[0] + c * mount[0] - s * mount[1], est[1] + s * mount[0] + c * mount[1]])
            msg['sensor'] = _f(sensor)
            if fix is not None and fix.beams is not None:
                b = fix.beams
                msg['beams'] = [[round(float(est[3] + a), 3), round(float(r), 3) if np.isfinite(r) else 1e6]
                                for a, r in zip(b.angles, b.ranges)]
            if fix is not None and fix.sectors is not None:
                sc = fix.sectors
                msg['sectors'] = [[round(float(est[3] + a), 3), round(float(r), 3) if np.isfinite(r) else 1e6]
                                  for a, r in zip(sc.angles, sc.ranges)]
        if particles is not None and len(particles):
            p = np.asarray(particles)
            if len(p) > max_particles:
                p = p[np.linspace(0, len(p) - 1, max_particles).astype(int)]
            msg['particles'] = [[round(float(x), 3), round(float(y), 3)] for x, y in p[:, :2]]
        if path is not None:
            msg['path'] = [[round(float(x), 3), round(float(y), 3)] for x, y in np.asarray(path)[:, :2]]
        if goal is not None:
            msg['goal'] = _f(goal)
        if qr_world:
            msg['qr_world'] = {k: _f(v[0] if isinstance(v, tuple) else v) for k, v in qr_world.items()}
        self._send(msg)
