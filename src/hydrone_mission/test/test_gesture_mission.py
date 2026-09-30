"""Phase 3 gesture mission core, flown against a point-mass drone.

    python3 -m pytest src/hydrone_mission/test/test_gesture_mission.py -q
"""
import math

import numpy as np
import pytest

from hydrone_mission.gesture.mission import (Debouncer, Mission, MissionParams, Observation,
                                             clearance, wrap)

DT = 0.05


class Drone:
    """Kinematic stand-in for GUIDED velocity control: it flies what it is told.

    Takeoff climbs at 0.5 m/s to the target; land descends and disarms on the floor.
    """

    def __init__(self, mission, z=0.0):
        self.m = mission
        self.pos = np.array([0.0, 0.0, z])
        self.yaw = 0.0
        self.armed = False
        self.t = 0.0
        self.obs = None                     # callable t -> Observation | None
        self.obstacles_world = np.zeros((0, 2))
        self.cmds = []
        self.log = []
        self.floor = z

    def body_obstacles(self):
        rel = self.obstacles_world - self.pos[:2]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.stack([c * rel[:, 0] + s * rel[:, 1], -s * rel[:, 0] + c * rel[:, 1]], axis=1)

    def run(self, seconds, until=None):
        for _ in range(int(round(seconds / DT))):
            self.t += DT
            obs = self.obs(self.t) if self.obs else None
            cmd = self.m.step(self.t, (self.pos.copy(), self.yaw), obs,
                              self.body_obstacles(), self.t, self.armed)
            self.cmds.append(cmd)
            if cmd.kind == "takeoff":
                self.armed = True
                self.pos[2] = min(cmd.z, self.pos[2] + 0.5 * DT)
            elif cmd.kind == "velocity":
                self.pos += np.asarray(cmd.velocity) * DT
                self.yaw = wrap(self.yaw + cmd.yaw_rate * DT)
            elif cmd.kind == "land":
                self.pos[2] = max(self.floor, self.pos[2] - 0.5 * DT)
                if self.pos[2] <= self.floor + 1e-6:
                    self.armed = False
            if until is not None and until(self):
                return True
        return False


def person(gesture="HOVER", x=0.0):
    return lambda t: Observation(t, gesture, True, x)


def fly_to_find(**kw):
    m = Mission(MissionParams(**kw), logger=lambda s: None)
    d = Drone(m)
    m.start()
    assert d.run(30, until=lambda d: m.state == "FIND")
    return m, d


# ── debouncer ───────────────────────────────────────────────────────────────

def test_debouncer_needs_the_hold_time():
    db = Debouncer()
    assert db.update("DIREITA", 0.0) == ("HOVER", False)
    assert db.update("DIREITA", 0.4) == ("HOVER", False)
    assert db.update("DIREITA", 0.5) == ("DIREITA", True)
    assert db.update("NENHUM", 0.6) == ("DIREITA", False)
    assert db.update("NENHUM", 0.85) == ("HOVER", True)


def test_debouncer_rides_over_a_one_frame_dropout():
    db = Debouncer()
    db.update("SUBIR", 0.0)
    db.update("SUBIR", 0.2)
    db.update("NENHUM", 0.25)             # one bad frame
    assert db.update("SUBIR", 0.5) == ("SUBIR", True)


def test_debouncer_land_is_slow():
    db = Debouncer()
    for k in range(11):
        assert db.update("POUSAR", k * 0.1)[0] == "HOVER"
    assert db.update("POUSAR", 1.2) == ("POUSAR", True)


def test_clearance_is_a_corridor():
    pts = np.array([[2.0, 0.1], [1.0, 1.0], [-0.5, 0.0]])
    assert clearance(pts, (1.0, 0.0), 0.45) == pytest.approx(2.0)
    assert clearance(pts, (0.0, 1.0), 0.45) == math.inf
    assert clearance(pts, (-1.0, 0.0), 0.45) == pytest.approx(0.5)


# ── the scripted opening ────────────────────────────────────────────────────

def test_takeoff_forward_one_metre_then_turn_right():
    m, d = fly_to_find()
    kinds = [c.kind for c in d.cmds]
    assert kinds[0] == "idle" and "takeoff" in kinds
    assert d.pos[0] == pytest.approx(1.0, abs=0.13)
    assert abs(d.pos[1]) < 0.05
    assert d.pos[2] == pytest.approx(1.0, abs=0.1)
    assert math.degrees(d.yaw) == pytest.approx(-90.0, abs=5.0)


def test_find_waits_then_sweeps_then_lands_on_timeout():
    m, d = fly_to_find(find_timeout_s=20.0)
    yaws = []
    d.run(19.0, until=lambda d: yaws.append(d.yaw) or m.state != "FIND")
    off = [math.degrees(wrap(y - yaws[0])) for y in yaws]
    assert max(off) > 40 and min(off) < -40
    assert max(abs(o) for o in off[:int(2.5 / DT)]) < 0.1   # still before find_wait_s
    assert d.run(3.0, until=lambda d: m.state in ("LAND", "LANDED"))


def test_found_person_is_centred_and_gestures_begin():
    m, d = fly_to_find()
    d.obs = person("HOVER", x=0.5)        # right of centre
    d.run(0.5)
    assert d.cmds[-1].yaw_rate < 0        # turning right toward them
    assert d.run(2.0, until=lambda d: m.state == "GESTURE")


# ── gestures ────────────────────────────────────────────────────────────────

def into_gestures(**kw):
    m, d = fly_to_find(**kw)
    d.obs = person()
    assert d.run(2.0, until=lambda d: m.state == "GESTURE")
    return m, d


def test_direita_moves_to_the_drones_right_and_stops_on_hover():
    m, d = into_gestures()
    start = d.pos.copy()
    d.obs = person("DIREITA")
    d.run(3.0)
    # facing -y after the right turn, so the drone's right is -x
    assert d.pos[0] - start[0] < -0.5
    assert abs(d.pos[1] - start[1]) < 0.05
    d.obs = person("HOVER")
    d.run(0.4)
    here = d.pos.copy()
    d.run(3.0)
    assert np.linalg.norm(d.pos - here) < 0.15   # holds where it stopped


def test_aproximar_afastar_subir_descer():
    m, d = into_gestures()
    for g, axis, sign in (("APROXIMAR", 1, -1), ("AFASTAR", 1, +1),
                          ("SUBIR", 2, +1), ("DESCER", 2, -1)):
        start = d.pos.copy()
        d.obs = person(g)
        d.run(2.5)
        assert sign * (d.pos[axis] - start[axis]) > 0.25, g
        d.obs = person("HOVER")
        d.run(1.0)


def test_nenhum_and_a_dead_camera_are_hover():
    m, d = into_gestures()
    d.obs = person("DIREITA")
    d.run(2.0)
    d.obs = person("NENHUM")
    d.run(0.5)
    assert m.debounce.active == "HOVER"
    d.obs = person("DIREITA")
    d.run(2.0)
    last = d.cmds[-1]
    d.obs = lambda t: Observation(t - 5.0, "DIREITA", True)   # frozen camera
    d.run(0.5)
    assert m.debounce.active == "HOVER"
    assert np.linalg.norm(d.cmds[-1].velocity) < np.linalg.norm(last.velocity)


def test_lidar_stops_the_approach_short_of_the_operator():
    m, d = into_gestures()
    # the operator stands 2.5 m in front of the nose (-y)
    d.obstacles_world = np.array([[d.pos[0], d.pos[1] - 2.5]])
    d.obs = person("APROXIMAR")
    d.run(8.0)
    gap = abs(d.obstacles_world[0, 1] - d.pos[1])
    assert 1.0 < gap < 1.3
    assert "obstacle" in d.cmds[-1].blocked


def test_altitude_and_radius_limits():
    m, d = into_gestures(max_alt=1.5, max_radius=1.0)
    d.obs = person("SUBIR")
    d.run(8.0)
    assert d.pos[2] < 1.55
    d.obs = person("HOVER")
    d.run(1.0)
    d.obs = person("AFASTAR")
    d.run(10.0)
    assert np.linalg.norm(d.pos[:2] - m.anchor) < 1.05


def test_lost_operator_goes_back_to_find():
    m, d = into_gestures()
    d.obs = lambda t: Observation(t, "NENHUM", False)
    assert d.run(5.0, until=lambda d: m.state == "FIND")


def test_pousar_lands_and_subir_takes_off_again():
    m, d = into_gestures()
    d.obs = person("POUSAR")
    assert d.run(2.0, until=lambda d: m.state == "LAND")
    assert d.run(10.0, until=lambda d: m.state == "LANDED")
    assert not d.armed and d.pos[2] == pytest.approx(0.0, abs=0.01)
    d.obs = person("SUBIR")
    d.run(1.0)
    assert m.state == "LANDED"            # not held long enough yet
    assert d.run(1.0, until=lambda d: m.state == "TAKEOFF")
    assert d.cmds[-1].takeoff_seq == 1
    d.obs = person("HOVER")
    assert d.run(10.0, until=lambda d: m.state == "GESTURE")
