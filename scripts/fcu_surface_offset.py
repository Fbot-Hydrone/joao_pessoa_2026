#!/usr/bin/env python3
"""fcu_surface_offset.py — set the Pixhawk up for the Orin's floor-height feed.

    python3 scripts/fcu_surface_offset.py status
    python3 scripts/fcu_surface_offset.py apply     # backup, set, reboot, check
    python3 scripts/fcu_surface_offset.py revert    # back to the backup

Run it on the Orin with MAVROS STOPPED (it talks MAVLink to the FCU over the
Ethernet link itself) and the vehicle DISARMED.

apply:
  RNGFND2_*  <- the current RNGFND1_* (the real DroneCAN sensor moves to 2)
  RNGFND1_TYPE = 10 (MAVLink), ORIENT/POS/GNDCLR/MIN/MAX unchanged: the
              Orin's surface_offset_node fills it, so the EKF's (POSZ=2) z
              becomes height above the floor.
  RNGFND2_ORIENT = 24 (UP — a label only, the sensor still points down).
              EKF3's readRangeFinder takes EVERY downward (PITCH_270)
              rangefinder, up to two, into one buffer: left at 25, the raw
              sensor is fused alongside the corrected one and z settles in
              between (bench 2026-11-18: fed 0.68 and 0.18, z wandered
              0.17-0.32). With PRX*_TYPE=0 and SURFTRAK_MODE=1 nothing in
              ArduCopter reads an upward rangefinder; ArduPilot still reports
              it as DISTANCE_SENSOR id 1, which is what the Orin needs.
revert:
  every RNGFND1_* / RNGFND2_* back to the values in the newest backup.

Backups: ~/logs/fcu_rngfnd_backup_<date>.json (RNGFND* only) and
         ~/logs/fcu_params_full_<date>.txt (every parameter, for the record).

A TYPE change only takes effect after a reboot, and backend-specific
parameters (RNGFNDx_ADDR for DroneCAN) only exist once the type is set, so
apply goes TYPE -> reboot -> the rest -> reboot.
"""

import glob
import json
import os
import sys
import time

from pymavlink import mavutil

FCU = os.environ.get("FCU_URL", "udpout:192.168.144.14:14550")
LOGS = os.path.expanduser("~/logs")
COPY = ("TYPE", "ORIENT", "POS_X", "POS_Y", "POS_Z", "GNDCLR", "MIN", "MAX",
        "ADDR", "OFFSET", "SCALING", "FUNCTION", "PIN", "RMETRIC",
        "STOP_PIN", "WSP_MAVG", "WSP_MEDF", "WSP_FRQ", "WSP_AVG", "WSP_THR",
        "WSP_BAUD", "RECV_ID", "SNR_MIN")


class Fcu:
    def __init__(self):
        self.m = mavutil.mavlink_connection(FCU, source_system=251)
        self.hb()
        if not self.m.wait_heartbeat(timeout=10):
            sys.exit("no heartbeat from the FCU — is MAVROS stopped?")

    def hb(self):
        for _ in range(3):
            self.m.mav.heartbeat_send(6, 8, 0, 0, 0)
            time.sleep(0.1)

    def armed(self):
        """The AUTOPILOT's armed bit. Other heartbeats share the link — QGC's
        (255/190) carries base_mode 192 and would read as armed."""
        t = time.time()
        while time.time() - t < 5:
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if (hb and hb.get_srcSystem() == self.m.target_system
                    and hb.get_srcComponent() == self.m.target_component):
                return bool(hb.base_mode &
                            mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        sys.exit("no autopilot heartbeat — cannot tell if it is armed")

    def all_params(self):
        self.m.mav.param_request_list_send(self.m.target_system,
                                           self.m.target_component)
        out, t = {}, time.time()
        total = None
        while time.time() - t < 60:
            r = self.m.recv_match(type="PARAM_VALUE", blocking=True,
                                  timeout=2)
            if r is None:
                if total and len(out) >= total:
                    break
                continue
            out[r.param_id] = r.param_value
            total = r.param_count
            if len(out) >= total:
                break
        return out

    def get(self, name):
        for _ in range(4):
            self.m.mav.param_request_read_send(
                self.m.target_system, self.m.target_component,
                name.encode(), -1)
            t = time.time()
            while time.time() - t < 1.5:
                r = self.m.recv_match(type="PARAM_VALUE", blocking=True,
                                      timeout=0.5)
                if r and r.param_id == name:
                    return r.param_value
        return None

    def set(self, name, value):
        self.m.mav.param_set_send(self.m.target_system,
                                  self.m.target_component, name.encode(),
                                  float(value),
                                  mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.3)
        got = self.get(name)
        ok = got is not None and abs(got - float(value)) < 1e-4
        print(f"  {name:16s} -> {value:<10g} {'ok' if ok else f'GOT {got}'}")
        return ok

    def reboot(self):
        if self.armed():
            sys.exit("ARMED — refusing to reboot the FCU.")
        print("  rebooting the FCU...")
        self.m.mav.command_long_send(self.m.target_system,
                                     self.m.target_component,
                                     mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                     0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(8)
        for _ in range(20):
            self.hb()
            if self.m.wait_heartbeat(timeout=3):
                time.sleep(3)
                return
        sys.exit("FCU did not come back after the reboot")


def rng(params, i):
    return {k[len(f"RNGFND{i}_"):]: v for k, v in params.items()
            if k.startswith(f"RNGFND{i}_")}


def status(f):
    p = f.all_params()
    for i in (1, 2):
        d = rng(p, i)
        print(f"RNGFND{i}: " + ", ".join(f"{k}={d[k]:g}" for k in COPY
                                          if k in d))
    print(f"EK3_SRC1_POSZ={p.get('EK3_SRC1_POSZ')}")
    t1, t2 = p.get("RNGFND1_TYPE"), p.get("RNGFND2_TYPE")
    o2 = p.get("RNGFND2_ORIENT")
    mode = ("SURFACE_OFFSET (Orin feeds RNGFND1)"
            if t1 == 10 and t2 == 24 and o2 == 24
            else "SURFACE_OFFSET BUT RNGFND2 STILL DOWNWARD — the EKF fuses "
                 "both; set RNGFND2_ORIENT=24"
            if t1 == 10 and t2 == 24
            else "PLAIN (DroneCAN is RNGFND1)" if t1 == 24 and not t2
            else "MIXED — check")
    print("mode:", mode)
    return p


def apply(f):
    if f.armed():
        sys.exit("ARMED — refusing.")
    p = f.all_params()
    if p.get("RNGFND1_TYPE") == 10:
        sys.exit("RNGFND1 is already MAVLink — already applied? (status)")
    stamp = time.strftime("%Y-%m-%d_%H%M%S")
    os.makedirs(LOGS, exist_ok=True)
    with open(f"{LOGS}/fcu_params_full_{stamp}.txt", "w") as fh:
        for k in sorted(p):
            fh.write(f"{k},{p[k]:.7g}\n")
    backup = {k: v for k, v in p.items() if k.startswith("RNGFND")}
    with open(f"{LOGS}/fcu_rngfnd_backup_{stamp}.json", "w") as fh:
        json.dump(backup, fh, indent=1, sort_keys=True)
    print(f"backup: {LOGS}/fcu_rngfnd_backup_{stamp}.json "
          f"({len(p)} params in fcu_params_full_{stamp}.txt)")
    r1 = rng(p, 1)
    print("step 1: RNGFND2_TYPE <- RNGFND1_TYPE, RNGFND1_TYPE = 10")
    f.set("RNGFND2_TYPE", r1["TYPE"])
    f.set("RNGFND1_TYPE", 10)
    f.reboot()
    print("step 2: the rest of RNGFND1_* onto RNGFND2_*")
    for k in COPY:
        if k == "TYPE" or k not in r1:
            continue
        if f.get(f"RNGFND2_{k}") is not None:
            f.set(f"RNGFND2_{k}", 24 if k == "ORIENT" else r1[k])
    f.reboot()
    status(f)


def revert(f):
    if f.armed():
        sys.exit("ARMED — refusing.")
    files = sorted(glob.glob(f"{LOGS}/fcu_rngfnd_backup_*.json"))
    if not files:
        sys.exit("no backup to revert to")
    with open(files[-1]) as fh:
        backup = json.load(fh)
    print(f"reverting to {files[-1]}")
    f.set("RNGFND1_TYPE", backup["RNGFND1_TYPE"])
    f.set("RNGFND2_TYPE", backup.get("RNGFND2_TYPE", 0))
    f.reboot()
    for k, v in sorted(backup.items()):
        if k.endswith("_TYPE"):
            continue
        if f.get(k) is not None:
            f.set(k, v)
    f.reboot()
    status(f)


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "status"
    fcu = Fcu()
    {"status": status, "apply": apply, "revert": revert}[cmd](fcu)
