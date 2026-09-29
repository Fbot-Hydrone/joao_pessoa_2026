---
tags: [hydrone, config, sim]
---
# Sim Settings

Back to [[Hydrone]]. Related: [[Config Single Source]].

`src/biguasim-ros2/biguasim_main/config/sim_settings.yaml` holds simulator
settings that **don't depend on the airframe**. The per-airframe scenario still
lives in `config-<agent>.yaml`.

For now it only sets the UE5 viewport (window) resolution:

```yaml
viewport:
  width: 1280
  height: 720
```

## How it gets to UE5

```
sim_settings.yaml ──BiguaSimInterface.parse_scenario_yaml──> scenario['window_width'/'window_height']
                                                        └──> ardubridge_node copies them into ardu_scenario
biguasim.make(scenario_cfg=...) ──environments.py──> UE5 binary -ForceRes -ResX= -ResY=
```

- The interface looks for the file **next to** `config-<agent>.yaml`, so launch
  files didn't change.
- `ardubridge_node` rebuilds the scenario via `ArduBiguaSimRunner.build_scenario`,
  which drops unknown keys. That's why the two window keys are copied explicitly,
  like `octree_*` and `show_viewport`.
- Missing file or missing `viewport:` block → biguasim default, 1280×720.
- `width`/`height` must both be positive integers, or the node refuses to start
  with a message naming the file.

## Scope

- Works for `ardubridge.launch.py` and `biguasim.launch.py`, the paths that start
  UE5 themselves.
- **Not** `remote_ardubridge.launch.py`: it connects to a world that's already
  running, so the window size is set wherever that world was started.
- Only applies at engine start. Edit the file, then relaunch.
- With `show_viewport: false` there's no window to see.
- This is the viewport, not camera sensor resolution. Camera image sizes stay in
  each sensor's `configuration` in `config-<agent>.yaml`.

## Deploying

The file is installed by the `config/*.yaml` glob in `setup.py`. Existing config
files are live through the dev bind mount, but a **new** file isn't. After
pulling this change, run `scripts/dev_rebuild.sh` once (or rebuild the image).

## Known gap (not fixed)

`frames_per_sec` in `config-<agent>.yaml` looks unused on the ardubridge path.
`build_scenario` doesn't take it and the node doesn't copy it, the same thing
that used to happen to the resolution. Not checked at runtime.
