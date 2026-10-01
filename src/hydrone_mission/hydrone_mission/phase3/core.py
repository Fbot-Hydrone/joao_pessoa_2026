"""
Phase 3 (human-swarm interaction) — the pure-Python half, no ROS.

The CONTRACT between whatever recognises the operator (the camera pipeline, or
the terminal stand-in while flying in BiguaSim) and the mission:

    topic  /hydrone/vision/human_gesture   (hydrone_msgs/HumanGesture)
    field  gesture_name  one of the keys of GESTURE_ACTION below
           confidence    0..1; below the mission's min_confidence it is ignored
           human_position, skeleton_keypoints  printed for the judge, not flown on

Any message at all while the mission is looking for the operator counts as
"operator detected" (that is what the camera publishes when it sees a person).
After that, each gesture_name maps to one ACTION.

Directions are in the DRONE's body frame: MOVE_RIGHT is the drone's right.
The drone faces the operator, so that is the operator's LEFT — the detector
decides which arm means which, the mission only ever sees the action.

A gesture moves the drone one STEP (step_m, step_z_m, yaw_step_deg). A
recogniser that publishes every frame while the operator holds a pose simply
keeps the drone stepping: the mission takes a new movement only once the last
one has arrived, and the same gesture again only after a cooldown.
"""

import math

# ── Actions ──────────────────────────────────────────────────────────────────
MOVE_FORWARD = "MOVE_FORWARD"
MOVE_BACK = "MOVE_BACK"
MOVE_LEFT = "MOVE_LEFT"
MOVE_RIGHT = "MOVE_RIGHT"
MOVE_UP = "MOVE_UP"
MOVE_DOWN = "MOVE_DOWN"
TURN_LEFT = "TURN_LEFT"
TURN_RIGHT = "TURN_RIGHT"
STOP = "STOP"
LAND = "LAND"
TAKEOFF = "TAKEOFF"
RETURN_HOME = "RETURN_HOME"
HUMAN = "HUMAN_DETECTED"     # operator seen, no command

MOVES = (MOVE_FORWARD, MOVE_BACK, MOVE_LEFT, MOVE_RIGHT, MOVE_UP, MOVE_DOWN,
         TURN_LEFT, TURN_RIGHT)

# gesture_name -> action. The first eight are the names hydrone_vision's
# classifier already emits (vision_node.GESTURE_LABELS); the rest are new and
# the detector should emit them verbatim.
GESTURE_ACTION = {
    "arms_forward": MOVE_FORWARD,
    "arms_back": MOVE_BACK,
    "left_arm_point": MOVE_LEFT,
    "right_arm_point": MOVE_RIGHT,
    "arms_cross": STOP,
    "both_arms_up": LAND,          # "land here" — on the base under the drone
    "thumbs_down": LAND,
    "thumbs_up": TAKEOFF,
    "move_up": MOVE_UP,
    "move_down": MOVE_DOWN,
    "turn_left": TURN_LEFT,
    "turn_right": TURN_RIGHT,
    "return_home": RETURN_HOME,
    "human_detected": HUMAN,
}


def action_for(gesture_name):
    """The action a gesture asks for, or None for 'unknown'/anything unmapped.

    An action name itself (e.g. 'MOVE_UP') is accepted too, so a detector can
    skip the pose vocabulary altogether.
    """
    name = (gesture_name or "").strip()
    if name in GESTURE_ACTION:
        return GESTURE_ACTION[name]
    if name in FRL_ACTION:
        return FRL_ACTION[name]
    if name.upper() in GESTURE_ACTION.values():
        return name.upper()
    return None


# ── The camera's vocabulary (frl_core): hold to move ─────────────────────────
# gesture_camera streams frl_core's DEBOUNCED gesture every frame. Its movement
# gestures are velocities, not steps: held, the drone keeps going at
# vel_speed; HOVER / STOP brake. frl_core.COMMANDS is the one table of
# directions (+lateral = the drone's right, +frente = towards the operator).
HOLD = "HOLD"                 # HOVER: neutral, brake and hold
FRL_ACTION = {"HOVER": HOLD, "NENHUM": HOLD, "STOP": STOP, "POUSAR": LAND}
FRL_TAKEOFF = "SUBIR"         # held on the ground for takeoff_hold_s => TAKEOFF
# every name the camera streams (frl_core.COMMANDS keys, plus NENHUM)
STREAMED = ("HOVER", "NENHUM", "STOP", "POUSAR", "DIREITA", "ESQUERDA",
            "APROXIMAR", "AFASTAR", "SUBIR", "DESCER")


def velocity_for(gesture_name):
    """(lateral, frente, vertical) for a moving frl_core gesture, else None."""
    from hydrone_mission.phase3.frl_core import COMMANDS
    cmd = COMMANDS.get((gesture_name or "").strip())
    return cmd if cmd is not None and any(cmd) else None


def velocity_enu(cmd, yaw, speed, speed_z):
    """Body-frame (lateral, frente, vertical) -> MAVROS local ENU (vx, vy, vz).

    Right of a vehicle at `yaw` (ENU, CCW from east) is (sin yaw, -cos yaw).
    """
    lat, fwd, vert = cmd
    vx = speed * (fwd * math.cos(yaw) + lat * math.sin(yaw))
    vy = speed * (fwd * math.sin(yaw) - lat * math.cos(yaw))
    return vx, vy, speed_z * vert


def wrap_pi(angle):
    """Fold an angle into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def step_setpoint(sp, action, step_m, step_z_m, yaw_step_rad):
    """The next [x, y, z, yaw] setpoint (MAVROS local ENU) for a MOVE action.

    Translations are in the body frame of the setpoint's own yaw, so a step is
    exact however far the vehicle is from the setpoint when it is asked.
    Right/clockwise is NEGATIVE yaw in ENU.
    """
    x, y, z, yaw = sp
    fx, fy = math.cos(yaw), math.sin(yaw)      # forward
    lx, ly = -fy, fx                           # left
    if action == MOVE_FORWARD:
        return [x + step_m * fx, y + step_m * fy, z, yaw]
    if action == MOVE_BACK:
        return [x - step_m * fx, y - step_m * fy, z, yaw]
    if action == MOVE_LEFT:
        return [x + step_m * lx, y + step_m * ly, z, yaw]
    if action == MOVE_RIGHT:
        return [x - step_m * lx, y - step_m * ly, z, yaw]
    if action == MOVE_UP:
        return [x, y, z + step_z_m, yaw]
    if action == MOVE_DOWN:
        return [x, y, z - step_z_m, yaw]
    if action == TURN_LEFT:
        return [x, y, z, wrap_pi(yaw + yaw_step_rad)]
    if action == TURN_RIGHT:
        return [x, y, z, wrap_pi(yaw - yaw_step_rad)]
    raise ValueError(f"not a move: {action}")


# ── Terminal stand-in for the camera ─────────────────────────────────────────
# (terminal word, what it publishes as gesture_name, help text)
TERMINAL_COMMANDS = [
    (("w", "frente", "forward"), "arms_forward", "anda para frente (step_m)"),
    (("s", "tras", "trás", "back"), "arms_back", "anda para trás"),
    (("a", "esquerda", "left"), "left_arm_point", "anda para a esquerda do drone"),
    (("d", "direita", "right"), "right_arm_point", "anda para a direita do drone"),
    (("r", "subir", "up"), "move_up", "sobe (step_z_m)"),
    (("f", "descer", "down"), "move_down", "desce"),
    (("q", "girar_esq", "gira_esq", "turn_left"), "turn_left", "gira à esquerda (yaw_step_deg)"),
    (("e", "girar_dir", "gira_dir", "turn_right"), "turn_right", "gira à direita"),
    (("x", "parar", "stop"), "arms_cross", "para e segura onde está"),
    (("p", "pousar", "land"), "both_arms_up", "pousa na base embaixo do drone"),
    (("t", "decolar", "takeoff"), "thumbs_up", "decola de novo (depois de um pouso)"),
    (("h", "voltar", "home"), "return_home", "volta sozinho à base de decolagem e pousa"),
    (("o", "operador", "humano", "human"), "human_detected", "simula a câmera vendo o operador"),
]


def parse_terminal(word):
    """Terminal word -> gesture_name, or None. A raw gesture_name also works."""
    w = (word or "").strip().lower()
    for words, gesture, _ in TERMINAL_COMMANDS:
        if w in words:
            return gesture
    if w in GESTURE_ACTION:
        return w
    return None
