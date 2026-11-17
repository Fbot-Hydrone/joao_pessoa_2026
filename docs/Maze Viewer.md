---
tags: [hydrone, tools, viewer, phase4]
---
# Maze Viewer

Back to [[Hydrone]]. `tools/maze_viewer`: a top-down live view of [[Confined Navigation]] and [[Phase 4 Known Maze]]. It is a small C++ app on [opengl-quickstart](https://github.com/GatoMonocromatico/opengl-quickstart), a git submodule at `tools/maze_viewer/external/opengl-quickstart`. It uses the library's window, shader and camera classes, plus its own flat-colour shaders.

It listens for JSON datagrams on UDP 47800 (protocol in `hydrone_confined/core/viz_udp.py`). Anything that speaks that protocol can drive it: the headless sim (`tools/phase4/sim_known.py --viz`), the ROS `viz_udp_node` (kinematic sim or BiguaSim), or a bag replay. It has no ROS dependency.

## Build and run
```bash
git submodule update --init tools/maze_viewer/external/opengl-quickstart
cd tools/maze_viewer
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
./build/maze_viewer            # [port]
```
When the stack runs on another machine (BiguaSim on machine 1), pass `viz_host:=<this machine's IP>` to `phase4_sim.launch.py`.

## What's drawn
- **Map:** walls (outer ones brighter). Openings are coloured by kind: entrance green, exit magenta, forbidden door red, low holes orange, windows cyan, doors blue. Soft (guessed) openings are hatched.
- **Pads and rooms:** the takeoff and landing pads, and the room outlines.
- **Drone:** the true pose (green, sim only) and the estimated pose (blue), each with a trail. The pose is a 0.165 m disc with a nose.
- **Localizer:** the beams (yellow, with their endpoints), the guard sectors closer than 0.7 m (red arcs), and the particles (violet).
- **Mission:** the planned path and its goal, and the QR codes (brown, orange once read).
- **Title bar:** time, phase, pose source, QR count, the est-vs-true error, and "stale" when nothing has arrived for 2 s.

## Controls
- **Mouse wheel:** zoom. **Right/middle drag:** pan.
- **F:** follow the drone. **Home:** fit the structure.
- **1–6:** toggle beams / sectors / particles / path / trails / rooms. **C:** clear the trails.
- **F12:** save `screenshot.png`. `MV_SHOT=<png> MV_SHOT_AFTER=<s>` saves one frame and exits (scripted checks).

## Note
The submodule is pinned to `2df6889`, the "linkable package" restructure of opengl-quickstart. When this was written, that commit only existed in the local clone (`~/projects/c-projects/opengl-quickstart`), not on GitHub. Until it's pushed, a fresh clone can't fetch it.
