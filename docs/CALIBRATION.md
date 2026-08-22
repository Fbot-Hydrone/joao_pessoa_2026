# Calibrating the belly camera

The belly camera is the one that decides **where the drone lands**. It has no
depth: `pad_map_node` turns a pad's pixel position into a place in the world by
projecting through the `camera_info` this camera publishes. Until it is
calibrated that `camera_info` is a *guess* — a nominal 60° pinhole with zero
distortion and a perfectly centred sensor — and every pad position inherits the
error.

`down_cam_usb_node` says so once a second while `calibrated` is false. This
document is how to make it stop.

---

## 1. What you need

| | |
|---|---|
| target | a **ChArUco** board — a chessboard with an ArUco marker in each white square. `camera_calibration` supports chessboard, circles, acircles and charuco; it does **not** support a plain ArUco grid |
| the board's four numbers | squares across × down, checker size, marker size, dictionary. A calib.io board prints all four along its bottom edge |
| a flat mount | see §5 — this is the largest error source and the easiest to get wrong |
| a display | the calibrator is a GUI. Use the Jetson's own monitor, or `ssh -X` |

The board in use here reads:

```
www.calib.io | 11x9 | Checker Size: 22 mm | Marker Size: 16 mm | Dictionary: AruCo DICT_4X4
```

which becomes `--size 9x11 --square 0.022 --charuco_marker_size 0.016
--aruco_dict 4x4_250`. Two of those need explaining, and both are traps.

### `--size` counts SQUARES for charuco, not interior corners

For `-p chessboard` the size is the number of **interior corners** — the
familiar `8x6` for a 9×7 board. For `-p charuco` it is the number of
**squares**. `cameracalibrator.py:150` passes the pair straight into
`cv2.aruco.CharucoBoard`, and `calibrator.py:125` then expects `(x-1)*(y-1)`
corners from it. Pass interior corners here and the board is simply never
detected — no error, no warning, the GUI just never registers a sample and it
looks like a camera or lighting fault.

Note also that the printed `11x9` is rows × columns, while `--size` is
**columns × rows**. Run the probe (§2) rather than trusting either the label or
your own counting.

### `--aruco_dict` has its own spelling

`cameracalibrator` accepts only `aruco_orig`, `4x4_250`, `5x5_250`, `6x6_250`,
`7x7_250`. A board printed from `DICT_4X4_50` still works with `4x4_250`,
because OpenCV's predefined dictionaries are nested — the first 50 markers of
`DICT_4X4_250` *are* `DICT_4X4_50`.

---

## 2. Check the numbers before you spend twenty minutes waving a board

```bash
# hold the board in front of the belly camera, then:
docker run --rm --privileged -v /dev:/dev -v $PWD:/repo -w /repo \
  hydrone-jetson:humble \
  python3 scripts/charuco_probe.py --save /repo/probe.png
```

It reports which dictionary detects markers, which `--size` is right, and
prints the exact `cameracalibrator` command. It exists because every one of
those parameters fails **silently** when wrong.

It does *not* pick `--size` by corner count, which cannot tell `9x11` from
`11x9` — both give `(9-1)*(11-1) = 80`. It fits a **homography** from each
candidate's object points to the detected image points. The board is planar, so
the correct layout fits to a fraction of a pixel and a transposed one scrambles
the correspondence entirely. Measured on a synthetic board: **0.268 px** for
`9x11` against **111.013 px** for `11x9`.

Self-test without a camera or a board:

```bash
python3 -c "import cv2; d=cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250); \
  cv2.imwrite('/tmp/synth.png', \
  cv2.aruco.CharucoBoard_create(9,11,0.022,0.016,d).draw((640,780)))"
python3 scripts/charuco_probe.py --image /tmp/synth.png
```

---

## 3. Run it

```bash
./scripts/jetson_up.sh --calibrate
```

That starts the belly camera and the calibrator GUI together, with the board
parameters above already filled in (override with `CAL_SIZE`, `CAL_SQUARE`,
`CAL_MARKER`, `CAL_DICT` in the environment for a different target).

It deliberately does **not** use `sources_real.launch.py`, which would also
start the ZED, its depth computation and MAVROS. On a Tegra X1 that is most of
the CPU, and the calibrator wants frames promptly or the GUI feels broken.

### Where the window appears, and the cookie that stops it

`--calibrate` passes `$DISPLAY` through and mounts the X cookie. Two routes
work: the Jetson's own screen (`DISPLAY=:0`), or `ssh -X` from your desk
(`DISPLAY=localhost:10.0`, reachable because the container is on `--network
host`).

The cookie is the part that bites. `$HOME/.Xauthority` is very often **stale** —
on this board it was two months old and held cookies for a display that no
longer existed. The symptom is

```
Invalid MIT-MAGIC-COOKIE-1 key
cv2.error: ... Can't initialize GTK backend in function 'cvInitSystem'
```

which reads like missing X libraries rather than an authorisation failure. The
script therefore asks the running X server which `-auth` file it was started
with (`ps -o args= -C Xorg`) instead of guessing; here that is
`/run/user/1000/gdm/Xauthority`. If it still fails, `xhost +local:` on whichever
machine owns the display.

`--no-service-check` because `down_cam_usb_node` takes its intrinsics as ROS
**parameters** and offers no `set_camera_info` service. The calibrator's
**COMMIT** button therefore does nothing useful here; use **SAVE**, which
writes `/tmp/calibrationdata.tar.gz`, and copy the numbers out by hand (§4).

That file is written **inside the container**, which runs with `--rm`. Copy it
out from a second terminal *before* stopping the GUI:

```bash
docker cp hydrone-jetson:/tmp/calibrationdata.tar.gz .
```

Run the GUI **on the Jetson**, not on a laptop subscribing over the network:
the board is on wifi, and `/down_cam/image_raw` at 640×480×3 × 15 Hz is about
110 Mbit/s.

### Filling the coverage bars

The GUI shows X, Y, Size and Skew. All four must go green before **CALIBRATE**
lights up, and each one is a distinct kind of motion:

* **X / Y** — board at the left, right, top and bottom edges of the frame.
  This is what pins the principal point `cx, cy`.
* **Size** — board close, and board far. This is what separates focal length
  from distance; without it `fx, fy` are poorly determined.
* **Skew** — board tilted, not square-on. Tilt it around both axes, 30–45°.

Two things specific to this camera:

* The C270 is **fixed focus** and will not resolve anything closer than roughly
  40 cm. Its "close" is about 40–50 cm, not 10.
* Calibrate at the **resolution you fly at** — 640×480 here. `fx, fy, cx, cy`
  scale with resolution, so a 1280×720 calibration is wrong at 640×480 unless
  you scale it yourself.

Aim for 40+ samples spread over the frame, not 40 of the same pose.

---

## 4. Feeding the numbers back

The calibrator prints a YAML block. What matters:

```yaml
camera_matrix:                  distortion_coefficients:
  data: [fx,  0, cx,              data: [k1, k2, p1, p2, k3]
          0, fy, cy,
          0,  0,  1]
```

Pass them to the launch:

```bash
./scripts/jetson_up.sh \
    down_cam_calibrated:=true \
    down_cam_fx:=<fx> down_cam_fy:=<fy> \
    down_cam_cx:=<cx> down_cam_cy:=<cy>
```

and once you are happy, edit the defaults in
`src/hydrone_bringup/launch/sources_real.launch.py` so they survive without
being retyped. The once-a-second warning stops when `calibrated` is true, which
is the confirmation that the node took them.

**A good result** has reprojection error below ~0.5 px at 640×480. Above ~1.0
px, something is wrong — usually §5.

---

## 5. Flatness, and the error nobody looks for

A calibration target is assumed to be **perfectly planar**. Paper is not. A
sheet taped at the corners bows in the middle by a few millimetres, which on a
22 mm square is a several-percent geometric error, and the solver absorbs it
into the distortion coefficients — where it looks like a plausible answer
rather than a mistake.

Glue the print to glass, acrylic, or dibond. Foam board and cardboard warp with
humidity. Tape at the edges only is not enough. If you can see the print lift
anywhere when you look across it at a shallow angle, fix that before you start.

**Print scale, by contrast, does not matter for what we need.** Scaling the
object points uniformly scales only the translation vectors; `fx, fy, cx, cy`
and the distortion coefficients come out identical. So a printer that scaled
the page to 97% costs nothing here — measure the squares anyway, it is free,
but do not redo a calibration over it.

---

## 6. Checking it afterwards

```bash
ros2 run camera_calibration cameracheck --size 9x11 --square 0.022 \
    monocular:=/down_cam image:=/down_cam/image_raw
```

Then the check that actually matters for this drone: put a pad at a known place
on the floor, hover over it, and compare what `pad_map_node` publishes on
`/hydrone/pads/markers` against a tape measure. Intrinsics that look good and a
mount transform that is wrong produce a confident, repeatable, wrong answer —
and `down_cam_mount_xyz` / `down_cam_mount_rpy_deg` are still at BiguaSim's
virtual airframe values. See [`JETSON-REAL-STACK.md`](JETSON-REAL-STACK.md) §9.
