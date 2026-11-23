---
tags: [hydrone, phase4, hardware, leds]
---
# Camera Lights

Back to [[Hydrone]]. The maze in phase 4 is dark, so the Kopis carries an RGB
LED strip to light the QR codes for the camera. Three MOSFETs on our board,
one per strip channel, and each gate on a Jetson Nano GPIO. Every channel is
on or off, so the strip has 8 states: off, red, green, blue, yellow, cyan,
magenta, white.

Package: `src/hydrone_leds` (driver + CLI + ROS 2 node). The driver has no ROS
in it, so the CLI runs on the Nano host, which has no ROS 2.

## Wiring (Jetson Nano 4 GB, 40-pin header)

| MOSFET gate (strip channel) | header pin | sysfs GPIO |
|---|---|---|
| red | 29 | 149 |
| green | 31 | 200 |
| blue | 33 | 38 |
| board GND | 30 (or 34, 39) | |

- The strip supply's GND must join the Jetson GND (through the board). The strip
  voltage never goes near the header.
- The board has gate pull-downs, so the strip stays off while the Nano boots
  (the three pins come up as inputs) and whenever nothing drives them.
- Pins left alone on purpose: 8/10 (Cube UART), 3/5/27/28 (I2C, they have
  pull-ups and would light the strip at boot), the SPI pins.
- For another board, pass `--gpios` (CLI) or `gpios` (node) with the sysfs
  numbers for red, green, blue.

**MOSFET check.** We think it is a 2N7000. On our strip it turns fully on at
3.3 V (tested 2026-11-23), but keep the limits in mind for a longer strip or
another batch. Its gate threshold goes up to 3.0 V, and the Jetson drives
3.3 V, so some parts only half turn on. It is also rated
200 mA continuous: a 5050 strip draws ~20 mA per color per 3 LEDs, so around
0.5–1 m per channel at 30 LEDs/m. If the MOSFET runs hot or the strip is dim,
that is why. A logic-level part (IRLZ44N, AO3400) has neither limit.

## Use

On the Nano host, straight from the checkout:

```bash
src/hydrone_leds/leds white        # also: red, "red blue", rb, cyan, off ...
src/hydrone_leds/leds status
```

The color stays after the command exits: an exported sysfs pin keeps its
level. The pins are never unexported, because that makes them inputs and the
pull-downs switch the strip off.

As a node (inside the container):

```bash
ros2 launch hydrone_leds leds.launch.py                   # white
ros2 launch hydrone_leds leds.launch.py backend:=fake     # no hardware
ros2 topic pub --once /hydrone/leds/color std_msgs/String "{data: 'red blue'}"
```

It sets `color` at start, follows `/hydrone/leds/color`, publishes the result
on `/hydrone/leds/state`, and switches off when it stops. A bad color is
logged and ignored.

`phase4_real.launch.py` starts it by default: `leds:=white`, or any color, or
`leds:=none` for no node. If the image was built before `hydrone_leds`, the
launch prints a warning and runs without it. Run `jetson_up.sh --phase4
--rebuild` once, or rebuild the image with `--build34`.

## Permissions

The node and the CLI run as `jetson`, not root. That works because the
`gpio` group owns `/sys/class/gpio`, and the container gets the user's groups.
JetPack's `60-jetson-gpio-common.rules` sets that up and was already there on
the Nano. `src/hydrone_leds/setup_gpio.sh` adds the group, the membership and
the udev rule if any of them is missing, and does nothing otherwise.

Right after an export, udev needs a few ms to chgrp the new files. The driver
retries for up to 2 s before giving up.

## Dimming (not done)

The MOSFETs could dim the strip with PWM. That does not wear the LEDs: they
get their normal current while on and none while off, and switching at kHz is
nothing to them. What matters is the camera: PWM near the exposure time gives
dark bands (rolling shutter) or frame-to-frame flicker, so it has to be
≥ ~20 kHz hardware PWM. The Nano has only two such pins (32, 33, after a
pinmux change), so all three channels need an I2C PWM chip. For inspection,
full brightness with a short exposure (less motion blur) is probably what we
want anyway.

## Tested

2026-11-22 on the Kopis's Nano, over USB (192.168.55.1), with nothing wired to
the pins yet. The CLI on the host set all 8 states and read them back, with no
sudo, and the level held after exit. The node in `hydrone-jetson-lio:humble`
as `jetson` started on its `color`, followed the topic, ignored a bad color
and switched off on SIGINT. Unit tests (`test/test_core.py`, fake sysfs
tree): 12 pass.

2026-11-23 with the board wired: all 8 states showed in the right colors at
full brightness, and the strip stayed dark through the boot. Not run yet: the
full `phase4_real.launch.py` with the LEDs in it.
