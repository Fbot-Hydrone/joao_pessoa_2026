"""frl_core.py - núcleo puro (sem câmera, sem ROS): keypoints -> gesto -> comando -> PWM.

Convenção de imagem: x para a direita, y para baixo. Keypoints no padrão COCO-17.

Vocabulário de gestos (braços esticados; ângulo medido a partir do braço pendurado):
  braços para baixo ................ HOVER     (neutro, nivela e freia)
  um braço na horizontal ........... DIREITA / ESQUERDA (o drone vai para o lado do braço, visto na imagem)
  os dois na horizontal (T) ........ STOP      (nivela e freia, explícito)
  os dois para cima (Y) ............ SUBIR     (no chão, mantido 1,5 s => DECOLAR)
  os dois na diagonal p/ baixo (A) . DESCER
  um braço para cima, outro baixo .. AFASTAR   (drone vai para trás, se afastando do operador)
  um braço na diagonal p/ cima ..... APROXIMAR (drone vai para frente, em direção ao operador)
  um braço para cima + outro na horizontal ... POUSAR (mantido 1,2 s)
Qualquer coisa fora das faixas (braço dobrado, zona morta, keypoint incerto) => NENHUM => vira HOVER.
"""
import math

L_SH, R_SH, L_EL, R_EL, L_WR, R_WR = 5, 6, 7, 8, 9, 10

# (nome, min, max) em graus: 0 = braço pendurado, 90 = horizontal, 180 = vertical para cima.
# As zonas mortas entre as faixas são propositais: caiu nelas => gesto ignorado.
BINS = (("DOWN", 0, 22), ("DIAG_DOWN", 33, 62), ("SIDE", 75, 105),
        ("DIAG_UP", 118, 147), ("UP", 158, 180))
EXT_MIN = 0.75   # |ombro->pulso| / (|ombro->cotovelo| + |cotovelo->pulso|): braço precisa estar esticado
MIN_CONF = 0.4   # confiança mínima de cada keypoint usado


def arm_state(sh, el, wr, confs):
    """Retorna (estado, sinal_x, ângulo). sinal_x = +1 se o pulso está à direita do ombro na imagem."""
    if min(confs) < MIN_CONF:
        return "UNK", 0, None
    reach = math.dist(sh, el) + math.dist(el, wr)
    vx, vy = wr[0] - sh[0], wr[1] - sh[1]
    if reach < 1e-6 or math.hypot(vx, vy) / reach < EXT_MIN:
        return "BENT", 0, None
    theta = math.degrees(math.atan2(abs(vx), vy))
    for name, lo, hi in BINS:
        if lo <= theta <= hi:
            return name, (1 if vx > 0 else -1), theta
    return "UNK", 0, theta


def classify(kpts, conf):
    """kpts: (17,2) em pixels; conf: (17,). Retorna (gesto, (estado_L, estado_R), (ang_L, ang_R))."""
    l = arm_state(kpts[L_SH], kpts[L_EL], kpts[L_WR], (conf[L_SH], conf[L_EL], conf[L_WR]))
    r = arm_state(kpts[R_SH], kpts[R_EL], kpts[R_WR], (conf[R_SH], conf[R_EL], conf[R_WR]))
    sl, sr = l[0], r[0]

    def has(a, b):
        return (sl, sr) in ((a, b), (b, a))

    if has("DOWN", "DOWN"):
        g = "HOVER"
    elif has("SIDE", "SIDE"):
        g = "STOP"
    elif has("UP", "UP"):
        g = "SUBIR"
    elif has("DIAG_DOWN", "DIAG_DOWN"):
        g = "DESCER"
    elif has("UP", "SIDE"):
        g = "POUSAR"
    elif has("UP", "DOWN"):
        g = "AFASTAR"
    elif has("DIAG_UP", "DOWN"):
        g = "APROXIMAR"
    elif has("SIDE", "DOWN"):
        d = l[1] if sl == "SIDE" else r[1]
        g = "DIREITA" if d > 0 else "ESQUERDA"
    else:
        g = "NENHUM"
    return g, (sl, sr), (l[2], r[2])


# (lateral, frente, vertical): +lateral = direita do drone; +frente = drone anda para frente (rumo ao operador)
COMMANDS = {
    "HOVER": (0, 0, 0), "STOP": (0, 0, 0), "POUSAR": (0, 0, 0),
    "DIREITA": (1, 0, 0), "ESQUERDA": (-1, 0, 0),
    "APROXIMAR": (0, 1, 0), "AFASTAR": (0, -1, 0),
    "SUBIR": (0, 0, 1), "DESCER": (0, 0, -1),
}
HOLD_S = {"HOVER": 0.25, "STOP": 0.25, "POUSAR": 1.2}  # tempo que o gesto precisa ser mantido
DEFAULT_HOLD = 0.5


class Debouncer:
    """Só confirma um gesto depois de mantido por HOLD_S; voltar ao neutro é mais rápido (segurança)."""

    def __init__(self):
        self.active, self.since = "HOVER", 0.0
        self.cand, self.t0 = None, 0.0

    def force(self, g, t=0.0):
        if g != self.active:
            self.active, self.since = g, t
        self.cand = None

    def update(self, g, t):
        """Retorna (comando_ativo, mudou?)."""
        if g == "NENHUM":
            g = "HOVER"
        if g == self.active:
            self.cand = None
            return self.active, False
        if g != self.cand:
            self.cand, self.t0 = g, t
        if t - self.t0 >= HOLD_S.get(g, DEFAULT_HOLD):
            self.active, self.since, self.cand = g, t, None
            return self.active, True
        return self.active, False


G = 9.81


class Mover:
    """Comando (lateral, frente, vertical) -> PWM de RC para ALT_HOLD.

    Controla uma velocidade ESTIMADA por modelo (integra a inclinação comandada, com arrasto).
    Não é medição: deriva com vento/trim. O humano corrige com gestos.
    ATENÇÃO: angle_max_deg deve ser igual ao ANGLE_MAX do ArduPilot (ANGLE_MAX = angle_max_deg*100).
    """

    def __init__(self, tilt_deg=5.0, angle_max_deg=15.0, vmax=0.5, kp=8.0, drag=0.4,
                 climb_pwm=200, yaw_pwm_max=60):
        self.tilt, self.angle_max, self.vmax = tilt_deg, angle_max_deg, vmax
        self.kp, self.drag, self.climb_pwm, self.yaw_pwm_max = kp, drag, climb_pwm, yaw_pwm_max
        self.v = [0.0, 0.0]

    def reset(self):
        self.v = [0.0, 0.0]

    def step(self, cmd, dt, yaw=0.0):
        lat, fwd, vert = cmd
        tilts = []
        for i, c in enumerate((lat, fwd)):
            t = max(-self.tilt, min(self.tilt, self.kp * (c * self.vmax - self.v[i])))
            self.v[i] += (G * math.tan(math.radians(t)) - self.drag * self.v[i]) * dt
            tilts.append(t)
        k = 500.0 / self.angle_max
        roll = 1500 + k * tilts[0]          # + = direita
        pitch = 1500 - k * tilts[1]         # PWM menor = nariz para baixo = frente
        thr = 1500 + self.climb_pwm * vert  # ALT_HOLD: fora da zona morta = razão de subida/descida
        yawp = 1500 + self.yaw_pwm_max * max(-1.0, min(1.0, yaw))
        return tuple(int(round(x)) for x in (roll, pitch, thr, yawp))
