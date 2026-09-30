---
tags: [hydrone, phase3, mission]
---
# Phase 3 HRI Mission

Back to [[Hydrone]]. Phase 3: land on all six bases guided **only** by an
operator's gestures, then fly home to the takeoff base on its own. The markers
on the bases are hidden, human tracking is forbidden, and every recognised
gesture has to be printed on the team computer for the judge.

## What the drone does

| step | who drives | what |
|---|---|---|
| takeoff | team computer | `auto_start:=true` (default), or `/hydrone/phase3/start` (`iniciar` in the terminal) |
| approach | autonomous | `approach_forward_m` (2 m) straight ahead, then `approach_turn_deg` (90°) to the **right** — the operator is there |
| detect | camera | any `HumanGesture` message = operator detected; printed with position and skeleton size |
| fly | **gestures only** | one gesture = one step; LAND puts it down on the base under it; TAKEOFF lifts it again |
| home | autonomous | after `target_bases` landings (6; 3 each with two drones), or on a `return_home` gesture: take off, fly over the takeoff base, land |

Nothing between *detect* and *home* moves the drone except a gesture: no base
detection, no tracking of the operator.

## Running it in BiguaSim

**What it took on 2026-09-30** (the first run on this machine):

```bash
ROS_DOMAIN_ID=77 BS_SIM_DIR=~/Documents/bs-drone-competition-phase4 \
    ./scripts/docker_up.sh --phase3 --no-build
```

- **The simulator client matters.** The GitHub `main` of `bs-drone-competition`
  spawns the Kopis but flips it on takeoff; the copy phase 4 was developed on
  (with `remote_runner.py`, `server/`, `client/`) flies it, and the LIO then
  stays within 6–9 cm of ground truth.
- **The world must know the Kopis**: `Competition` from 2026-09-04 or later.
  The 2026-08-30 build dies in UE5 on `Assertion failed: SpawnedAgent` a few
  seconds in, which shows up downstream only as SITL's endless
  `No JSON sensor message received`.
- **Pick your own `ROS_DOMAIN_ID`.** The container uses host networking; on
  domain 42 another robot on the lab network was flooding every node with
  `sequence size exceeds remaining buffer`. `phase3_terminal.sh` execs into the
  container, so it inherits the domain.
- `--no-build` while the image cannot rebuild (MAVROS humble binaries left apt;
  the source-build fix lives on `phase3-human`, 99dd9d43 + e345d08f).

Flown that way: takeoff, 2 m approach, 90° right turn, operator detection,
forward / turn-right steps, LAND on a gesture, TAKEOFF on a gesture — all as
the fake-FCU tests predicted.

```bash
./scripts/docker_up.sh --phase3            # sim + LIO + mission (Kopis X8, like phase 4)
./scripts/phase3_terminal.sh               # second terminal: you are the camera
```

On the host instead: `ros2 launch hydrone_bringup phase3_sim.launch.py`, then
`ros2 run hydrone_mission gesture_terminal`. The terminal needs a real TTY, so
it is never started from a launch file.

Tuning goes on the command line (`step_m:=1.0 takeoff_alt:=1.8 target_bases:=3`);
the step sizes can also be changed mid-flight from the terminal (`passo 1.0`,
`altura 0.5`, `giro 90`). Watch the state with `ros2 topic echo /hydrone/phase3/status`.

`phase3_sim.launch.py` is phase 4's bring-up unchanged (`phase:=3`,
`mission:=none`, `map_name:=phase3`) plus `phase3.launch.py`. The EKF flies on
the LIO, as it will on the real aircraft. It spawns where phase 4 spawns, so
the first 2 m and the turn happen wherever that is in the Competition world —
there is no phase 3 arena in the simulator yet.

## The gesture contract

The camera pipeline, when it arrives, only has to publish
`hydrone_msgs/HumanGesture` on `/hydrone/vision/human_gesture` — the topic and
message `hydrone_vision` already uses. `gesture_terminal` publishes exactly
that, so swapping the keyboard for the camera changes nothing downstream.

| terminal | `gesture_name` | action |
|---|---|---|
| `w` / `frente` | `arms_forward` | MOVE_FORWARD `step_m` |
| `s` / `tras` | `arms_back` | MOVE_BACK |
| `a` / `esquerda` | `left_arm_point` | MOVE_LEFT |
| `d` / `direita` | `right_arm_point` | MOVE_RIGHT |
| `r` / `subir` | `move_up` | MOVE_UP `step_z_m` |
| `f` / `descer` | `move_down` | MOVE_DOWN |
| `q` / `girar_esq` | `turn_left` | TURN_LEFT `yaw_step_deg` |
| `e` / `girar_dir` | `turn_right` | TURN_RIGHT |
| `x` / `parar` | `arms_cross` | STOP — hold where it is |
| `p` / `pousar` | `both_arms_up` (or `thumbs_down`) | LAND on the base below |
| `t` / `decolar` | `thumbs_up` | TAKEOFF after a landing |
| `h` / `voltar` | `return_home` | autonomous return + land |
| `o` / `operador` | `human_detected` | operator seen, no command |

The first eight names are the ones `vision_node`'s classifier already emits;
the rest are new and should be emitted verbatim. An action name
(`MOVE_UP`, …) is also accepted. Source of truth:
`hydrone_mission/phase3/core.py`.

- **Directions are the drone's**: MOVE_RIGHT is the drone's right, which is
  the operator's left while it faces them. Mapping arms to directions is the
  detector's call.
- **Holding a pose keeps stepping.** A camera republishes the same gesture
  every frame; the mission takes a new move only once the last one has
  arrived, and the same gesture again only after `repeat_cooldown_s`. STOP,
  LAND and RETURN_HOME are taken at any time.
- Messages under `min_confidence` are dropped; `unknown` is ignored.
- Height is clamped to `min_alt`..`max_alt` above the takeoff base.

## Tested

Against a fake MAVROS/FCU (setpoint follower + takeoff/LAND/disarm) in a
throwaway container: approach lands at (0, 2, 1.5) facing 90° right of the
start; forward/right/up/turn steps arrive where the body-frame math says; a
second move while one is flying is ignored; land → takeoff → return-home ends
within 8 cm of home; `target_bases` landings trigger the return on their own;
`iniciar` starts an `auto_start:=false` run. `test/test_phase3_core.py` covers
the vocabulary and step math headless.

In BiguaSim (see above) every step reported "concluído" within `arrive_tol_m`.
Not flown yet there: RETURN_HOME and the automatic return after
`target_bases`, and the spawn is phase 4's (there is no phase 3 arena).
