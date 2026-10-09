"""Turn generate.py's (empty, kit) image pairs into a YOLO detection dataset.

    python3 scripts/kit_dataset/label.py ~/Documents/kit_dataset_sim

THE LABEL IS THE DIFFERENCE. Both images of a pair were rendered from the same
pose under the same light, one before the kits were spawned and one after, so
every pixel that changed belongs to a kit — or to its SHADOW. A shadow is the
same surface, darker by a roughly constant factor in all three channels; a kit
pixel is a different colour. Shadow pixels are dropped before boxing.

A pair is DISCARDED (not mislabelled) when too much of the frame changed: that
is a render that did not repeat (the sim settling, an exposure step), and a box
drawn from it would be noise.

Output (YOLO, one class `lipo`, same name the current model uses):
    images/{train,val}/*.png   labels/{train,val}/*.txt   data.yaml
    previews/                  a sample with the boxes drawn, to eyeball
NEGATIVES: a share of the EMPTY images go in with an empty label file — the
base with the window's reflection and no kit, the exact thing the current
model calls a kit.
AUGMENTED copies (brightness, contrast, colour cast, noise, blur) stand in for
the lighting the simulator could not vary (set_day_time hangs in this world).
"""
import argparse
import glob
import os
import random
import shutil

import cv2
import numpy as np

DIFF_T = 28            # per-channel max abs difference that counts as change
MIN_AREA = 80          # px; smaller blobs are render noise
MIN_SIDE = 8           # px; a kit thinner than this is noise or unreadable
MAX_CHANGED = 0.12     # fraction of the frame; more = the pair did not repeat


def kit_mask(empty, kit):
    e = empty.astype(np.float32) + 1.0
    k = kit.astype(np.float32) + 1.0
    changed = np.abs(k - e).max(axis=2) > DIFF_T
    # Shadow: every channel darker by a similar ratio.
    ratio = k / e
    darker = (ratio < 0.92).all(axis=2)
    flat = (ratio.max(axis=2) - ratio.min(axis=2)) < 0.12
    shadow = darker & flat
    # ...but a grey kit on a sunlit white floor is ALSO darker by a flat
    # ratio. What a shadow cannot do is create EDGES: it darkens a surface
    # smoothly, a kit brings its own outline and print. Pixels that changed
    # AND gained strong gradient are kit even when they look like shadow.
    gk = _grad(kit)
    ge = _grad(empty)
    textured = cv2.dilate(((gk - ge) > 40).astype(np.uint8),
                          np.ones((5, 5), np.uint8)).astype(bool)
    m = (changed & (~shadow | textured)).astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    return m, changed.mean()


def _grad(img):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)
    return cv2.magnitude(cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3),
                         cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3))


def boxes(mask):
    n, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    out = []
    for i in range(1, n):
        x, y, w, h, a = stats[i]
        if a >= MIN_AREA and min(w, h) >= MIN_SIDE:
            out.append((x, y, w, h))
    return out


def augment(img, rng):
    """One photometric variant: brightness/contrast, colour cast, noise, blur."""
    out = img.astype(np.float32)
    out = out * rng.uniform(0.6, 1.35) + rng.uniform(-30, 30)
    out *= np.array([rng.uniform(0.85, 1.15) for _ in range(3)], np.float32)
    if rng.random() < 0.5:
        out += np.random.default_rng(rng.randrange(1 << 30)).normal(
            0, rng.uniform(2, 8), out.shape)
    out = np.clip(out, 0, 255).astype(np.uint8)
    if rng.random() < 0.3:
        k = rng.choice([3, 5])
        out = cv2.GaussianBlur(out, (k, k), 0)
    return out


def yolo_line(b, w, h):
    x, y, bw, bh = b
    return f"0 {(x + bw / 2) / w:.6f} {(y + bh / 2) / h:.6f} {bw / w:.6f} {bh / h:.6f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root", help="folder holding raw/ (from generate.py)")
    ap.add_argument("--val", type=float, default=0.2)
    ap.add_argument("--negatives", type=float, default=0.3,
                    help="share of EMPTY images kept as negatives")
    ap.add_argument("--augment", type=int, default=1,
                    help="photometric copies per positive image")
    ap.add_argument("--seed", type=int, default=7)
    a = ap.parse_args()
    root = os.path.expanduser(a.root)
    raw = os.path.join(root, "raw")
    rng = random.Random(a.seed)
    for sub in ("images", "labels", "previews"):
        shutil.rmtree(os.path.join(root, sub), ignore_errors=True)
    for split in ("train", "val"):
        os.makedirs(os.path.join(root, "images", split))
        os.makedirs(os.path.join(root, "labels", split))
    os.makedirs(os.path.join(root, "previews"))

    stats = dict(pairs=0, positives=0, boxes=0, negatives=0, discarded=0,
                 no_kit_in_view=0, augmented=0)

    def write(name, img, lines, split):
        cv2.imwrite(os.path.join(root, "images", split, name + ".png"), img)
        with open(os.path.join(root, "labels", split, name + ".txt"), "w") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))

    for kit_path in sorted(glob.glob(os.path.join(raw, "*_kit.png"))):
        stem = os.path.basename(kit_path)[:-len("_kit.png")]
        empty_path = os.path.join(raw, stem + "_empty.png")
        if not os.path.exists(empty_path):
            continue
        stats["pairs"] += 1
        empty, kit = cv2.imread(empty_path), cv2.imread(kit_path)
        h, w = kit.shape[:2]
        # Split by POSE, so a pose's image, its negative and its augmented
        # copies never straddle train/val.
        split = "val" if rng.random() < a.val else "train"
        mask, frac = kit_mask(empty, kit)
        if frac > MAX_CHANGED:
            stats["discarded"] += 1
            continue
        bs = boxes(mask)
        if rng.random() < a.negatives:
            write(stem + "_neg", empty, [], split)
            stats["negatives"] += 1
        if not bs:
            stats["no_kit_in_view"] += 1
            continue
        lines = [yolo_line(b, w, h) for b in bs]
        write(stem, kit, lines, split)
        stats["positives"] += 1
        stats["boxes"] += len(bs)
        for j in range(a.augment):
            write(f"{stem}_aug{j}", augment(kit, rng), lines, split)
            stats["augmented"] += 1
        if stats["positives"] % 25 == 1:
            prev = kit.copy()
            for x, y, bw, bh in bs:
                cv2.rectangle(prev, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            cv2.imwrite(os.path.join(root, "previews", stem + ".png"), prev)

    with open(os.path.join(root, "data.yaml"), "w") as f:
        f.write(f"path: {root}\ntrain: images/train\nval: images/val\n\n"
                "nc: 1\nnames: ['lipo']\n")
    print(stats)


if __name__ == "__main__":
    main()
