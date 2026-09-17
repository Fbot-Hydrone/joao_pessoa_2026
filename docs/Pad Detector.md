---
tags: [hydrone, vision, detector]
---
# Pad Detector

Back to [[Hydrone]]. The landing-pad computer-vision pipeline: pure OpenCV/numpy
in `hydrone_vision/pad_detector.py`, no ROS, no model files, deterministic cost
per frame. Used by both [[Landing Sites]]'s `pad_mission_node` and
[[Phase 1 Mission]]'s `phase1_mission_node` — same code, same tuning, different
flight logic around it.

The ROS wrapper is a separate file so the algorithm can be tested against
images directly.

There are **two pads**, and they are not found the same way. `field_mode`
picks: `blue` for BiguaSim's, `dark_blue` for the real arena's. Only the mask
stage differs — checks 2–6 below are shared code and shared thresholds.

BiguaSim's pad is a saturated blue field carrying a yellow ring with a yellow
cross through its centre, on brown-green ground. **Finding blue is not enough**
— a tarp, a shirt, a painted line, a puddle reflecting sky all pass that. The
structure is the identity of the pad, so most of the work goes into proving a
blue blob really carries a concentric ring-and-cross:

| # | check | rejects |
|---|---|---|
| 1 | HSV blue/yellow masks with a high **saturation floor** | washed-out look-alikes |
| 2 | area, **solidity**, aspect ratio | slivers, painted lines, bridged blobs |
| 3 | yellow fraction inside the blue footprint | plain blue objects; yellow ones with a blue rim |
| 4 | **concentricity** of the yellow and blue centroids | blue with an off-centre yellow smear |
| 5 | **ring coverage**: rays out of the centre hit yellow in *every* direction | one-sided smears |
| 6 | **cross arms**: halfway out along each ray, yellow appears in exactly four angular lobes | a solid yellow disc (yellow at every angle), a bare ring (yellow at none) |

Checks 5 and 6 come from one polar sweep of 72 rays. Each ray is measured against
**the ring radius found along that ray**, never a global circle — which is what
makes it hold when perspective squashes the pad into an ellipse, and what makes
it independent of the drone's heading.

## Confidence, and the 0.75 line

Checks 1–4 are hard gates. Checks 5–6 also gate, **but only once the pad is big
enough on screen** (`r95 ≥ structure_radius_px`, 22 px) for a missing ring or a
missing arm to actually be visible; below that, morphology has already fused ring
and arms into one blob and the measurement means nothing.

That produces a useful property, and the mission depends on it:

> The cross check carries weight 0.25 and is forced to zero when the structure
> cannot be resolved. The remaining weights sum to 0.75, so **confidence > 0.75
> implies the ring and the cross were actually verified.**

Which is what lets a mission treat a distant sighting as a *lead* worth flying
to and never as something to land on sight. `pad_mission_node` no longer uses
it (it only ever reads the belly camera, from directly above); it is
`phase1_mission_node` that depends on the property, in the split between the
forward camera identifying and the belly camera validating — see
[[Phase 1 Mission#Both cameras, and the two-stage decision|Phase 1 Mission §5]].
`test_pad_detector.py::test_high_confidence_implies_the_structure_was_verified`
pins this.

## Measured behaviour

From `src/hydrone_vision/test/test_pad_detector.py` (68 tests, synthetic
renders, `field_mode="blue"` unless said otherwise):

- **Detected** from a pad filling the frame down to ~**18 px across** — at
  640×480 / 90° that is a 1 m pad about 25 m away.
- **Rotation invariant** (0–138° checked), and correct under oblique
  (trapezoidal) views.
- Survives Gaussian noise to σ=20, exposure ×0.55–×1.25, and motion blur.
- **Rejects**: bare ground, a plain blue rectangle, blue with an off-centre
  yellow blob, blue with a *solid* yellow disc, blue with a ring but no cross,
  yellow alone, a thin blue line, and a desaturated print of the real pad.

## One documented miss

If the image border cuts **through the ring**, the ring stops enclosing
anything, the blue inside and outside merge, and no candidate carries a
concentric ring — the frame is missed. A pad merely clipped at the edge, ring
intact, is still found.

This is safe (a missed frame, never a false landing site) and it clears itself as
the pad moves inward, which is what happens while flying toward it. Pinned by
`test_pad_clipped_through_the_ring_is_a_known_miss` so it stays a known
behaviour rather than a surprise.

## The real arena: `field_mode="dark_blue"`

The real pad is **not the simulated pad recoloured**, and it is not found by
colour at all. Two earlier attempts were, and both silently detected nothing;
why is worth keeping, because it is the same trap for whoever retunes this next.

Measured off the arena photographs and off the ZED's own frames, 2026-08-22:

| | H | S | V |
|---|---|---|---|
| floor, phone camera | 109 | 142 | 193 |
| floor, ZED | 103–105 | 220 | 190 |
| pad field, phone | 107 | 104 | 158 |
| pad field, ZED | 101 | 172 | 146 |
| **markings, ZED** | **58** | **44** | **196** |

**The markings are not yellow to the ZED.** Its auto white balance throws a
heavy green cast and its exposure washes the paint out: hue 58 is *green*,
saturation 44 is almost none. The yellow band is H 18–38, S ≥ 110, and against
a real ZED frame it admits **zero pixels**. Every structural check reaches the
image only through that mask, so the cascade never ran — the detector returned
nothing and reported nothing. Pinned by
`test_sim_mode_finds_no_yellow_at_all_in_zed_colours`.

**Nor is the floor separable by colour.** Raw ZED capture (2026-08-23) has a
global *blue* cast, and in it the white wall reads more blue-dominant than the
far mat (opponent −39 against −28). The previous version gated on "markings
must lie on blue-dominant floor"; measured across 44 labelled frames that gate
selected **80–94% of every frame**, walls included. It has been removed rather
than retuned.

### What it does instead

Every candidate is an **ellipse**, verified by one polar sweep in that
ellipse's own normalised coordinates.

1. **Markings, by contrast.** Opponent channel `yb = (R+G)/2 − B`, minus its
   own local mean. A colour *difference* against a *local* reference, so white
   balance, exposure and the ZED's rolling-shutter banding all move signal and
   reference together and cancel.
2. **Two hypothesis families**, because the pad is two different things at the
   two ranges that matter:
   - **ring fit** — the circle resolves as its own marking component; fitting
     an ellipse to it puts the centre exactly on the pad. This is the belly
     camera at landing height.
   - **cluster fit** — border, ring and cross merge into one small patch (the
     forward ZED across the arena). Coarser, but it still carries the
     foreshortening a circular model throws away.

   A ring fit outranks a cluster fit over the same pad (`ring_bonus`), because
   the winner is the position that gets projected into the world.
3. **One verifier.** 72 rays to normalised radius 1.25:
   - `ring_cov` — rays whose marking **reaches out** to the ring. Not "the ray
     met marking somewhere": a cross has paint over the centre, so a plain
     any-hit test returns 1.00 for two crossing bars, and that alone scored a
     bare cross at 0.89.
   - `arms` — angular lobes of marking between 0.25 and 0.55 of the **median**
     outer radius. Measuring against the median rather than each ray's own
     radius is what stops a square border, whose radius swings by √2 between
     edge and corner, reporting eight arms for a four-armed cross.
   - `seen` — how much of the sweep stayed inside the image. **Per camera.**
4. **The field is darker than the mat.** median(inside) − median(outside) over
   non-marking pixels; negative for a pad under any illumination, and two
   medians from the same frame so a colour cast cancels. Every labelled pad
   −25..−2; the bright window lattice that was outranking pads, +7..+95.
5. **Strength** — how far the markings actually cleared `mark_delta`.
6. **Airframe** — `ignore_regions` blanks the parts of the frame the drone
   occupies. See below.

The square border needs no special handling any more: sweeping to 1.25 and
referencing the arms to the median outer radius covers both the case where the
outermost marking is the border and the case where it is the circle. The ROI
shrink, the rect-fill gate and the mat gate are all gone.

### Measured, on 44 labelled frames from the arena

Two ZED clips and two belly-camera clips (2026-08-23), hand-labelled. Scored on
whether the **best-ranked** detection is the pad, which is what the mission
acts on:

| | top-1 | mis-ranked | missed | on empty frames |
|---|---|---|---|---|
| shipped before | 8/42 | 4 | 30 | 0 |
| **this version** | **29/42** | **0** | 13 | 1 |

Centre error median **11.1 px**, p90 **30.9 px** (was p90 43.8 on the few it
found). Per camera:

| clip | top-1 | mis-ranked | centre err median |
|---|---|---|---|
| ZED clean | 6/11 | 0 | 18.8 px |
| ZED stained | 7/13 | 0 | 19.0 px |
| belly clean | 7/7 | 0 | 0.5 px |
| belly stained | 9/11 | 0 | 1.7 px |

The belly camera is **16/18 with nothing mis-ranked**, which is the number that
matters for confirmation. The forward camera is 13/24; what it misses is the
pad seen almost edge-on at the far wall, 9:1 foreshortened and a few thousand
pixels in area.

Cost, desktop CPU: **4.6 ms/frame at 672×376, 12.2 ms at 960×544.**

> The belly-camera centre errors are partly circular: those labels were
> produced by fitting ellipses to the rings and confirming the fits by eye,
> because reading coordinates off a contact sheet by hand put four of them out
> by ~130 px. Recall and mis-ranking counts there are honest; the sub-pixel
> centre figures are not independent evidence. The ZED labels *were* read
> independently, so their 12–21 px is a real number (and is inside the ±20 px
> the hand-reading is good to).

### The two cameras are configured differently

One parameter, opposite ends:

```
forward ZED   min_seen: 0.85     its answer becomes a WORLD POSITION
belly camera  min_seen: 0.30     it only ever answers yes/no
```

At landing height roughly a third of the belly frames showing a pad show only
part of one, sometimes with the centre outside the image. An arc of the circle
plus the cross is enough, and the fitted ellipse puts the centre where the
pad's centre really is.

A clipped pad always offers *two* readings — the arc, whose sweep runs off the
image but whose centre is right, and a compact cluster wholly inside the frame
whose centre is 60–80 px biased. `min_seen` chooses. The belly camera wants the
arc; the forward camera would rather have neither than the biased one.

**Cross alone is deliberately not enough.** Below the height where any of the
circle is in shot, all that is left is two crossing bars, and a mat seam
crossing another seam forges that. The belly camera's answer gates a landing,
so it says no. Cost, measured: about one frame in twenty.

### The airframe

The belly camera sees its own landing legs. A dark object with a bright edge on
blue foam passes every test in the detector — one scored **0.95**. Nothing in a
single frame separates them, so they go by position:

```
ignore_regions: [0.75, 0.0, 1.0, 0.22,     x0,y0,x1,y1 fractions
                 0.0, 0.78, 0.16, 1.0]
```

**These were measured off the 2026-08-23 belly footage. Re-measure them if the
camera or the legs move** — watch `/hydrone/pads/down/debug_image` on the
ground with the rotors stopped. Without the mask that camera goes from 0
mis-ranked and 1 ghost to 1 mis-ranked and 3 ghosts.

### Retuning it

ROS parameters on `pad_detector_node`, none of them a colour:

```
mark_delta          8.0    how far above its local mean the opponent channel
                           must rise to count as paint
mark_window_frac    0.06   size of that neighbourhood, as a fraction of the
                           frame's longer side
mark_contrast_mult  1.5    how far REAL paint must clear mark_delta
min_axis_px        18.0    smallest ellipse worth considering
min_seen            0.30   fraction of the sweep that must stay in frame
ignore_regions        []   airframe, as x0,y0,x1,y1 fractions
```

Watch `/hydrone/pads/<camera>/debug_image`. If the pad goes quiet, lower
`mark_delta` until the markings come back — no further. If stains and mat seams
start being reported, raise `mark_contrast_mult`; every detection carries its
measured contrast in `scores["contrast"]`, so both numbers can be read off the
same frame rather than guessed.

`mark_contrast_mult` was **2.5** in the previous version, calibrated on
photographs of a *monitor* showing the debug stream, which exaggerated marking
contrast to 25–192. On raw capture a real pad measures **19**, so the gate sat
one unit above the signal and cost one clip 64 of its 65 frames. Calibrate on
raw frames.

`blue_hsv_*` and `yellow_hsv_*` do **not** apply in this mode.

### Still open

- **The forward camera cannot tell a pad from wall clutter by appearance.**
  Measured: a solar panel leaning on the wall and a cable lying in a loop on
  the mat produce ring coverage, arm counts, residuals and contrast
  *statistically indistinguishable* from a real pad at that range. The
  darker-field gate removes the bright ones; the rest cannot be separated
  without geometry. The forward ZED already has depth, and a **ground-plane
  gate on the back-projected point in `pad_detector_node`** would remove the
  whole class. **NOT IMPLEMENTED** — this is the single highest-value thing
  left in the detector chain.
- The pad seen edge-on at the far wall (aspect 9:1, a few thousand pixels) is
  missed. It clears as the drone turns toward it.
- Raising the caller's threshold to 0.70 trades recall for accuracy as it
  should: 24/42, still nothing mis-ranked, centre error median 8.1 px. That is
  deliberate — see `ecc_penalty`, which pays for foreshortening in confidence
  so a slant sighting arrives as a lead rather than as a fix.

## Retuning the simulator: `field_mode="blue"`

The HSV bands are ROS parameters on `pad_detector_node`:

```
blue_hsv_low/high    library default [95,110,50] .. [135,255,255]
yellow_hsv_low/high  library default [18,110,90] .. [38,255,255]
```

**The sim needed this too.** Measured on BiguaSim's rendering (2026-08-18):
UE's tonemapping/bloom pushes the whole pad toward white, and the saturation
floor of 110 admitted **zero** pixels of either colour, so no pad was ever
detected from any altitude. Measured inside the pad's bounding box on a
lossless `/down_cam` frame at a 3 m hover:

```
blue field         S 37-75, mean 56
yellow ring+cross  S 38-59, mean 48
```

An earlier grab over the spawn pad put the yellow at S ≈ 82–109, so saturation
varies a lot with altitude and local lighting inside the map. **Both** floors
have to come down, not just yellow: `_detect_sim` runs `findContours` on the
BLUE mask and iterates over blue contours, so an empty blue mask means zero
candidates and the yellow, ring, cross and concentricity checks never run at
all.

`landing_sites.launch.py` and `phase1.launch.py` therefore both pass

```
blue_hsv_low:   [95, 30, 50]
yellow_hsv_low: [18, 30, 90]
```

S ≥ 30 covers every pixel of both grabs with margin and leaves the
discrimination to the structural checks, which is where it belongs — on a
confirmed detection ring coverage is 1.0, arms 4, concentricity offset 0.005.
The library default (and its tests) are unchanged.

---

## Pixel → world

Two routes, preferred in this order:

- **Depth** — back-project with the intrinsics through the registered depth
  image. Metric, assumption-free. The forward ZED normally uses this.
- **Ground plane** — intersect the pixel's ray with a horizontal plane at
  `ground_z`. The arena floor is flat and the belly camera points nearly
  straight at it, so this is accurate to centimetres and needs no depth sensor.

A ray at or above the horizon is refused outright rather than extrapolated into
a huge number that would poison the map.

**The world frame is the frame of `/mavros/local_position/pose`** (the FCU's
local ENU, normally `map`), not the VO `odom` frame — even though the two are
near-identical here. The mission's setpoints go to
`/mavros/setpoint_position/local`, which is interpreted in exactly that frame, so
composing the camera pose from the same source the controller uses removes a
whole class of "the map and the controller disagree" error. TF is consulted only
for the constant `base_link → <optical frame>` mount.

`src/hydrone_vision/test/test_pad_projection.py` works this chain against poses
whose answer can be done by hand: centre pixel straight below, right-of-image →
right-of-drone, up-of-image → ahead-of-drone, yaw following the airframe, depth
back-projection, the ground-plane fallback, and every refusal case.
