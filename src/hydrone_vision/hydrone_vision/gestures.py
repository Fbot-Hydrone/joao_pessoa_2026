"""
gestures — pure core of the Phase 3 gesture reader: keypoints -> gesture.

No camera, no ROS, no model: tested headless in test/test_gestures.py.
gesture_detector_node turns an image into COCO-17 keypoints in PIXELS (MediaPipe
Pose, mapped by mediapipe_to_coco); everything here works on those.

Image convention: x to the right, y down. Keypoints are COCO-17.

Vocabulary (straight arms; angle measured from the hanging arm):
  both down ...................... HOVER      (neutral: hold position)
  one horizontal ................. DIREITA / ESQUERDA (drone goes to the side
                                   the arm points to, AS SEEN IN THE IMAGE)
  both horizontal (T) ............ STOP       (hold position, explicit)
  both up (Y) .................... SUBIR      (on the ground, held => take off)
  both diagonal down (A) ......... DESCER
  one up, the other down ......... AFASTAR    (drone backs away from the operator)
  one diagonal up, other down .... APROXIMAR  (drone moves toward the operator)
  one up + the other horizontal .. POUSAR     (land)
Anything outside the bins (bent arm, dead zone, unsure keypoint) is NENHUM,
which the mission treats as HOVER.

"As seen in the image" is also as seen by the drone: the camera looks along
the drone's nose, so image right is the drone's right. Facing the drone, the
operator's LEFT arm is on image right, so the drone moves where the arm points.
"""

import math

# COCO-17 indices
NOSE = 0
L_SH, R_SH, L_EL, R_EL, L_WR, R_WR = 5, 6, 7, 8, 9, 10
L_HIP, R_HIP = 11, 12

# MediaPipe Pose (33 landmarks) index for each COCO-17 keypoint
MP_TO_COCO = (0, 2, 5, 7, 8, 11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28)

# (name, min, max) in degrees: 0 = arm hanging, 90 = horizontal, 180 = straight up.
# The gaps between bins are on purpose: an arm in one of them is ignored.
BINS = (("DOWN", 0, 22), ("DIAG_DOWN", 33, 62), ("SIDE", 75, 105),
        ("DIAG_UP", 118, 147), ("UP", 158, 180))
EXT_MIN = 0.75   # |shoulder->wrist| / (|shoulder->elbow| + |elbow->wrist|): arm must be straight
MIN_CONF = 0.4   # minimum confidence of every keypoint used

GESTURES = ("HOVER", "STOP", "SUBIR", "DESCER", "POUSAR", "AFASTAR", "APROXIMAR",
            "DIREITA", "ESQUERDA", "NENHUM")


def arm_state(sh, el, wr, confs):
    """(state, x_sign, angle). x_sign = +1 when the wrist is right of the shoulder in the image."""
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
    """kpts: (17,2) in pixels; conf: (17,). Returns (gesture, (state_L, state_R), (ang_L, ang_R))."""
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


def mediapipe_to_coco(landmarks, width, height):
    """MediaPipe's 33 normalised landmarks -> COCO-17 (kpts in pixels, conf).

    `landmarks` is anything indexable whose items have .x, .y, .visibility
    (MediaPipe's NormalizedLandmark) or are (x, y, visibility) tuples.
    Scaling to pixels is not cosmetic: the arm angles are only right in a
    frame whose axes have the same unit, and a 640x480 image stretches x.
    """
    kpts, conf = [], []
    for i in MP_TO_COCO:
        lm = landmarks[i]
        if hasattr(lm, "x"):
            x, y, v = lm.x, lm.y, getattr(lm, "visibility", 1.0)
        else:
            x, y, v = lm
        kpts.append((float(x) * width, float(y) * height))
        conf.append(float(v))
    return kpts, conf


def person_center(kpts, conf, width, height):
    """(x, size) of the operator in the image, or None.

    x is the shoulder midpoint in [-1, 1], + to the right of the image centre
    (hips when the shoulders are unsure). size is the shoulder width over the
    image width: it grows as the operator gets closer.
    """
    for a, b in ((L_SH, R_SH), (L_HIP, R_HIP)):
        if conf[a] >= MIN_CONF and conf[b] >= MIN_CONF:
            cx = (kpts[a][0] + kpts[b][0]) / 2.0
            size = abs(kpts[a][0] - kpts[b][0]) / float(width)
            return 2.0 * cx / float(width) - 1.0, size
    return None

