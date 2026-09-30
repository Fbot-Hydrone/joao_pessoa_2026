"""Probe BiguaSim's RaycastLidar on the KopisX8, no SITL.

Ticks the world with zero motor commands (drone sits on the ground) and records:
tick rate with/without the lidar, point counts, a few scans + one depth frame
for a geometry cross-check. Output: npz in the dir given as argv[1].

    python3 probe_raycast_lidar.py /out [lidar|depth|both] [ticks]
"""
import sys, time
import numpy as np
import biguasim
from biguasim.ardubridge import ArduBiguaSimRunner, VEHICLE_REGISTRY

out = sys.argv[1]
mode = sys.argv[2] if len(sys.argv) > 2 else "both"
ticks = int(sys.argv[3]) if len(sys.argv) > 3 else 600
jitter = "jitter" in sys.argv

PROFILE = VEHICLE_REGISTRY["KopisX8"]
TPS = 200
sc = ArduBiguaSimRunner.build_scenario(
    PROFILE, package_name="Competition", world="CompetionMap",
    agent_name="uav0", location=[3, 3.4, 0.78], rotation=[0, 0, -90],
    ticks_per_sec=TPS)
sensors = sc["agents"][0]["sensors"]
if mode in ("lidar", "both"):
    sensors.append({
        "sensor_type": "RaycastLidar", "sensor_name": "Lidar",
        "location": [0, 0, 0.5], "Hz": 200,
        "configuration": {
            "Channels": 40, "Range": 7000, "PointsPerSecond": 200000,
            "RotationFrequency": 10, "UpperFovLimit": 52, "LowerFovLimit": -7,
            "HorizontalFov": 360, "NoiseStdDev": 0.0, "DropOffGenRate": 0.0,
            "DropOffIntensityLimit": 1.0, "DropOffAtZeroIntensity": 0.0,
            "ShowDebugPoints": False}})
if mode in ("depth", "both"):
    sensors.append({
        "sensor_type": "DepthCamera", "sensor_name": "Depth",
        "location": [0, 0, 0.5], "Hz": 20,
        "configuration": {"CaptureWidth": 256, "CaptureHeight": 256, "FOV": 90}})

env = biguasim.make(scenario_cfg=sc, show_viewport=False, verbose=False)
agent = sc["main_agent"]
scans, depth, rates = [], None, []
with env:
    env.step([0.0] * 4)
    t0 = time.time()
    for i in range(ticks):
        if jitter and mode != "depth":
            next(iter(env.agents.values())).sensors["Lidar"].rotate(
                [3.0 * np.sin(i * 0.37), 3.0 * np.cos(i * 0.23), 0.0])
        s = env.step([0.0] * 4)[agent][0]
        if "Lidar" in s and i > 200:
            scans.append(np.array(s["Lidar"], dtype=np.float32))
        if "Depth" in s and i > 200:
            depth = np.array(s["Depth"], dtype=np.float32)
        if i == 200:
            t1 = time.time()
    dt = time.time() - t1
print(f"mode={mode} ticks/s={(ticks-200)/dt:.1f} scans={len(scans)} "
      f"pts/scan mean={np.mean([len(x) for x in scans]) if scans else 0:.0f}")
np.savez_compressed(f"{out}/probe_{mode}{'_jit' if jitter else ''}.npz",
                    depth=depth if depth is not None else np.zeros(1),
                    **{f"scan{k}": v for k, v in enumerate(scans[-400:])})
