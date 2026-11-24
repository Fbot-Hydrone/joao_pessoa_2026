# AX-12A Payload Gripper — Architecture, Logic & Usage Guide

Documentation for the real-hardware 3D-printed payload gripper (20 cm travel) on the Orin. Written on the Orin (uncommitted there, 2026-11-24), brought into `cbr/phase2`.

---

## 1. System Overview

### Hardware Stack
- **Actuator**: Dynamixel **AX-12A** (ID `1`, Joint Mode `0–300°` / `0–1023` ticks, `9–12 V` bus, `1 Mbps` half-duplex TTL).
- **Controller / Bridge**: **ArbotiX-M** (ATmega644P @ 16 MHz) flashed with stock Vanadium Labs `ros.ino` firmware (`/home/ivo/arbotix_fw`).
- **USB Interface**: FTDI FT232R USB-UART cable (`115200` baud, 8N1), auto-detected via `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_*` (fallback `/dev/ttyUSB0`).
- **Mechanics**: 3D-printed linkage, `20 cm` side-to-side jaw travel (CW / decreasing ticks = open, CCW / increasing ticks = close).

### Software Architecture
```text
phase2_mission_node (autonomy state machine)
  └─ hydrone_controller.gripper.Gripper  (ActionClient → non-blocking Job)
       └─ /hydrone/gripper/command  (hydrone_msgs/action/Gripper)
            ├─ [SIM]  ardubridge_node → BiguaSim world command
            └─ [REAL] gripper_dynamixel_node (hydrone_bringup)
                        ├─ /hydrone/gripper/grab       (Trigger: close + torque ramp → hold)
                        ├─ /hydrone/gripper/release    (Trigger: open + relax)
                        ├─ /hydrone/gripper/calibrate  (Trigger: dynamic torque homing)
                        ├─ /hydrone/gripper/state      (JointState: pos, speed, load)
                        └─ dynamixel_sdk (Protocol 1.0)
                             └─ FTDI FT232R (115200 baud)
                                  └─ ArbotiX-M `ros.ino` (UART0 ↔ UART1 1 Mbps)
                                       └─ AX-12A (ID 1)
```

---

## 2. Communication Protocol & ArbotiX-M Bridge Details

Both the Jetson (`dynamixel_sdk.Protocol1PacketHandler`) and the ArbotiX-M firmware (`ros.ino` + `Bioloid/ax12.cpp`) use **Dynamixel Protocol 1.0**:

```text
Instruction: [0xFF] [0xFF] [ID] [LEN] [INS] [PARAM_0 ... PARAM_N] [CHECKSUM]
Status:      [0xFF] [0xFF] [ID] [LEN] [ERR] [PARAM_0 ... PARAM_M] [CHECKSUM]
  CHECKSUM = ~(ID + LEN + INS/ERR + sum(PARAMS)) & 0xFF
```

### Atomic Telemetry Read
Instead of polling `Moving`, `Present Position`, and `Present Load` separately, the driver reads **11 contiguous RAM bytes (`addr 36..46`) in one single packet**:

| Address | Size | Field |
|---|---|---|
| `36–37` | 2 B | `Present Position` (`0..1023` over `0–300°`) |
| `38–39` | 2 B | `Present Speed` (bits `0..9` = magnitude) |
| `40–41` | 2 B | `Present Load` (bits `0..9` = torque magnitude `0..1023`) |
| `42` | 1 B | `Present Voltage` (`value / 10.0` V) |
| `43` | 1 B | `Present Temperature` (°C) |
| `46` | 1 B | `Moving` (`0` or `1`) |
| Header `ERR` | 1 B | AX-12A hardware error flags (`0x20` = Overload, `0x04` = Overheating) |

### Critical ArbotiX-M Firmware Quirks Handled by the Driver
1. **DTR Reset & Bootloader Window**: Opening the FTDI port toggles DTR and resets the ArbotiX-M. Any bytes sent during the bootloader window trap the board in bootloader mode. The driver stays silent for `boot_wait_s` (`7.5 s` default, covering both ~2 s and ~6.5 s bootloaders) before probing.
2. **No `PING` Support**: `ros.ino` only passes through `AX_READ_DATA` (`0x02`) and `AX_WRITE_DATA` (`0x03`). Startup readiness is probed by reading registers `36..46`.
3. **Synthetic Write ACKs**: `ros.ino` returns a synthetic `statusPacket(id, 0)` on writes without reading the AX-12A reply, and only supports 1- or 2-byte writes. All register writes are retried (3× with `clearPort()` + 50 ms backoff) and checked for return status; true servo health is confirmed on subsequent telemetry reads.
4. **Polling Rate**: Polls every `poll_interval_s = 0.18 s` (single 11-byte packet) so the ArbotiX-M serial parser never falls out of sync.

---

## 3. Control Logic

### A. Dynamic Torque-Limited Calibration (`auto_calibrate` / `/hydrone/gripper/calibrate`)
Because the jaws may start at any arbitrary position at power-on (or after reassembly), calibration discovers both mechanical stops dynamically without forcing the 3D-printed linkage:
1. Drives toward the open limit at `calib_torque_limit = 400`, `calib_speed = 150`.
2. After the breakaway inrush window (`calib_inrush_s = 0.40 s`), stops only when **both** `load >= calib_load_threshold (280)` **and** motion has slowed (`abs(pos - last_pos) <= slow_delta (8)`), or on 3 consecutive stalled samples, or Overload. Mid-travel linkage friction while moving smoothly is ignored.
3. Backs off `limit_margin = 10` ticks → records `open_pos`.
4. Repeats in the close direction → backs off `10` ticks → records `closed_pos`.
5. **Obstruction guard**: If usable travel `< min_calib_travel (100)` ticks, logs a warning and falls back to the configured `open_position` / `closed_position`.
6. Returns cleanly to `open_pos`.

### B. 3D-Print-Safe Dynamic Torque Ramp & Stiffness Verification (`_close_and_hold`)
Used by both `/hydrone/gripper/grab` and `/hydrone/gripper/command` (`close=True`):

1. **Gentle approach**: Starts closing toward `closed_pos` at `close_start_torque = 350`. If it reaches `closed_pos` within `reach_tolerance (20)` ticks → relaxes motor and reports `EMPTY`.
2. **Dynamic torque escalation & movement analysis** (when slowed short of `closed_pos`):
   - Steps torque up by `torque_ramp_step = 80` up to the **3D-print-safe ceiling** `close_max_torque = 720`, waits `ramp_step_s = 0.30 s`, and measures jaw displacement `moved` and torque change `dload`:
     - **Moving without torque rise** (`moved > firm_move_tol (8)` and `dload < min_torque_rise (40)`): Not firmly grabbed (3D-print drag, edge graze, or object shifting) → continues closing at the higher torque.
     - **Firm grasp confirmed** (`|moved| <= 8` **and** `dload >= 40`, or solid load `>= 260` at `close_max_torque`): Object is solidly seated → latches hold.
   - Every ramp step is logged as:
     `grab ramp: torque=... (cap 720) | pos A->B (moved=...) | load X->Y (dload=...)`
3. **3D-print-safe hold & active Overload relief**:
   - Sets `Goal Position = clamp(pos + close_sign * hold_squeeze (24))` (inside the AX-12A compliance slope).
   - Sets dynamic hold torque `min(confirmed_torque + 110, hold_torque_limit (780))` (capped at `780` to protect the 3D-printed jaws).
   - Runs a 3-step relief check (`0.35 s` each): if `load > max_hold_load (730)` or `ERR_OVERLOAD (0x20)` appears, automatically backs off `relief_step = 8` ticks toward open and re-arms.
4. **Empty / Open relaxation**:
   - An empty close relaxes `Goal Position` off the closed stop so idle load drops to ~0.
   - Opening monitors `open_torque_threshold (350)` and backs off `open_backoff = 10` ticks if it hits a shifted open stop early.

### C. Mission Integration (`phase2_mission_node.py`, branch `cbr/phase2`)
- **`OPENING` state**: waits for `gripper.open()` to finish before arming at startup, and re-opens the gripper while hovering before retrying or skipping a failed pickup (`_grab_failed`, `_do_verify`), so closed jaws never come down on the kit again.
- **One close, on the ground by default** (`close_in_touch:=false`): TOUCH descends on the kit to `touch_range_m` and lands; ACT closes once with the kit between the legs. Closing in the air at `touch_range_m` (0.22, 0.17 landed) leaves the finger tips 1-2.5 cm above the 7 cm kit.
- `close_in_touch:=true` is the Orin version of 2026-11-24: close in TOUCH, keep the result (`_touch_grab`) and reuse it in ACT instead of a second close on the ground.
- On the drone the whole run is `orin_flight/run_phase2.sh` (see `orin_flight/PHASE2.md`): `sources_real ... gripper:=true` homes the gripper at boot, so the jaws must be empty when it starts.

---

## 4. ROS 2 Interfaces

| Interface | Type | Purpose |
|---|---|---|
| `/hydrone/gripper/command` | Action `hydrone_msgs/action/Gripper` | Goal `bool close`; Result `success`, `holding`, `message`; Feedback `position` (`0.0–1.0`) |
| `/hydrone/gripper/grab` | Service `std_srvs/srv/Trigger` | Close with dynamic torque ramp & hold object (`success=True` when `HOLDING`, `False` when `EMPTY`) |
| `/hydrone/gripper/release` | Service `std_srvs/srv/Trigger` | Open jaws and release held object |
| `/hydrone/gripper/calibrate` | Service `std_srvs/srv/Trigger` | Run dynamic torque-limited homing and update `open_pos` / `closed_pos` |
| `/hydrone/gripper/state` | Topic `sensor_msgs/msg/JointState` | Live `position` (ticks), `velocity` (speed), `effort` (load `0..1023`) |

---

## 5. Key Parameters (`src/hydrone_bringup/config/gripper.yaml`)

All gripper parameters live in `src/hydrone_bringup/config/gripper.yaml` (loaded automatically by `sources_real.launch.py` and usable standalone with `--params-file`):

| Parameter | Default | Description |
|---|---|---|
| `port` | `/dev/ttyUSB0` | Serial port (auto-detected in `sources_real.launch.py`) |
| `baudrate` | `115200` | USB baudrate to ArbotiX-M |
| `id` | `1` | AX-12A servo ID |
| `boot_wait_s` | `7.5` | Silent bootloader wait after FTDI open |
| `open_position` / `closed_position` | `264` / `755` | Initial / fallback limits (ticks) |
| `auto_calibrate` | `true` | Run dynamic homing on node startup |
| `calib_torque_limit` | `400` | Torque limit during calibration homing |
| `calib_speed` | `150` | Speed during calibration homing |
| `calib_load_threshold` | `280` | Load threshold (paired with `slow_delta`) during calibration |
| `calib_stall_samples` | `3` | Consecutive stalled polls to detect calibration stop |
| `calib_inrush_s` | `0.40` | Breakaway current ignore window during calibration |
| `limit_margin` | `10` | Safety ticks backed off each mechanical stop |
| `close_start_torque` | `350` | Initial gentle closing torque |
| `torque_ramp_step` | `80` | Torque increase per stiffness probe step |
| `close_max_torque` | `720` | Maximum closing torque (3D-print cap) |
| `hold_torque_limit` | `780` | Maximum holding torque (3D-print cap) |
| `hold_torque_margin` | `110` | Margin added above confirmed grasp torque |
| `max_hold_load` | `730` | Relief threshold during active hold |
| `hold_squeeze` | `24` | Compliance-slope ticks past contact while holding |
| `relief_step` | `8` | Ticks eased per relief iteration |
| `firm_move_tol` | `8` | Max ticks moved during ramp step to confirm stopped on object |
| `min_torque_rise` | `40` | Min load rise during ramp step to confirm firm grip |
| `slow_delta` | `8` | Max ticks/poll to count as slowed (rejects mid-travel friction) |
| `close_torque_threshold` | `260` | Initial contact load threshold during close |
| `open_torque_threshold` | `350` | Torque stop guard during open |
| `open_backoff` | `10` | Ticks backed off if open hits stop early |
| `torque_limit` | `550` | General moving torque (used for open) |
| `moving_speed` | `200` | Normal open/close speed (`0..1023`) |

---

## 6. How to Use

### Setup Environment (on the Jetson)
```zsh
source /opt/ros/humble/setup.zsh
source ~/mavros_ws/install/setup.zsh
source ~/Documents/joao_pessoa_2026/install/setup.zsh
```

### Option 1 — Full Drone Stack Launch (Phase 2)
```zsh
ros2 launch hydrone_bringup sources_real.launch.py gripper:=true
```
Useful launch arguments:
- `gripper_auto_calibrate:=false` — skip startup homing and use fallback `264 / 755`
- `gripper_port:=/dev/ttyUSB0` — override FTDI auto-detection

### Option 2 — Run Gripper Node Standalone (Bench Testing)
```zsh
PORT=$(ls /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_* | head -n 1)
ros2 run hydrone_bringup gripper_dynamixel_node --ros-args \
  --params-file ~/Documents/joao_pessoa_2026/src/hydrone_bringup/config/gripper.yaml \
  -p port:="$PORT"
```
Override any parameter live without rebuilding, e.g.:
```zsh
ros2 run hydrone_bringup gripper_dynamixel_node --ros-args \
  --params-file ~/Documents/joao_pessoa_2026/src/hydrone_bringup/config/gripper.yaml \
  -p port:="$PORT" \
  -p hold_torque_limit:=800 \
  -p max_hold_load:=750
```

### Trigger Services & Action CLI
```zsh
# Dynamic torque-limited calibration (keep jaws empty!)
ros2 service call /hydrone/gripper/calibrate std_srvs/srv/Trigger {}

# Close with dynamic torque ramp + movement verification and hold object
ros2 service call /hydrone/gripper/grab std_srvs/srv/Trigger {}

# Open and release object
ros2 service call /hydrone/gripper/release std_srvs/srv/Trigger {}

# Or call the Phase 2 ROS 2 Action directly:
ros2 action send_goal --feedback /hydrone/gripper/command hydrone_msgs/action/Gripper "{close: true}"
ros2 action send_goal --feedback /hydrone/gripper/command hydrone_msgs/action/Gripper "{close: false}"

# Watch live position, speed, and load (effort):
ros2 topic echo /hydrone/gripper/state
```

### Standalone Calibration Script (`~/calibrate_gripper.py`)
Requires only `pyserial` (stop `gripper_dynamixel_node` first so the serial port is free):
```zsh
# Manual hand-sweep monitor (torque OFF — prints live pos, deg, load, V, T, min/max)
python3 ~/calibrate_gripper.py

# Active torque-limited auto-probe (empty jaws — finds open & closed limits automatically)
python3 ~/calibrate_gripper.py --probe
```

### Rebuilding & Running Unit Tests (on the Jetson)
```zsh
cd ~/Documents/joao_pessoa_2026
source /opt/ros/humble/setup.zsh && source ~/mavros_ws/install/setup.zsh
colcon build --packages-select hydrone_bringup hydrone_mission
source install/setup.zsh
python3 -m pytest src/hydrone_bringup/test/test_gripper_logic.py \
                  src/hydrone_mission/test/test_phase2_mission.py \
                  src/hydrone_bringup/test/test_launch_arguments.py -v
```

---

## 7. Quick Troubleshooting

- **`AX-12A id 1 not answering`**: Check 12 V servo power (`Present Voltage` should be `~11–12.5 V`), confirm FTDI appears under `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_*`, and make sure no other process (`calibrate_gripper.py` / another node) holds the port.
- **Calibration stops early (`measured travel < 100`)**: Check the logged `[open_reason]` / `[closed_reason]` and increase `-p calib_torque_limit:=450 -p calib_load_threshold:=320` if linkage friction is higher.
- **Object slips / prematurely says `HOLDING`**: Lower `firm_move_tol` (e.g. `6`) or raise `min_torque_rise` (e.g. `55`) so stricter stiffness is required before latching.
- **Grip too tight for 3D-printed parts**: Lower `-p close_max_torque:=600 -p hold_torque_limit:=620 -p max_hold_load:=580 -p hold_squeeze:=15`.
- **Grip not tight enough on a heavy kit**: Raise `hold_squeeze` slightly (`25`) and `hold_torque_limit` / `max_hold_load` carefully while watching the `grab ramp:` and `hold=... load=...` logs.
