"""RGB strip on three on/off channels: colors and the GPIO backends.

No ROS in here, so the CLI runs on a bare Python 3 (the Jetson Nano host has
no ROS 2) and the node just wraps it.
"""

import os
import time

CHANNELS = ('red', 'green', 'blue')

# every on/off combination of the three channels
COLORS = {
    'off': (0, 0, 0),
    'red': (1, 0, 0),
    'green': (0, 1, 0),
    'blue': (0, 0, 1),
    'yellow': (1, 1, 0),
    'cyan': (0, 1, 1),
    'magenta': (1, 0, 1),
    'white': (1, 1, 1),
}

# Jetson Nano (t210) 40-pin header -> sysfs GPIO number
JETSON_NANO_PINS = {29: 149, 31: 200, 33: 38}
JETSON_NANO_GPIOS = (149, 200, 38)  # red pin 29, green pin 31, blue pin 33


def parse_color(words):
    """'white', 'red blue', 'rb', 'r g' -> (r, g, b). Raises ValueError."""
    if isinstance(words, str):
        words = words.replace(',', ' ').replace('+', ' ').split()
    state = [0, 0, 0]
    if not words:
        raise ValueError('no color given')
    for w in words:
        w = w.lower()
        if w in COLORS:
            if len(words) > 1 and w in ('off', 'white'):
                raise ValueError("'%s' doesn't mix with other colors" % w)
            state = [a | b for a, b in zip(state, COLORS[w])]
        elif w and set(w) <= set('rgb'):
            for ch in w:
                state['rgb'.index(ch)] = 1
        else:
            raise ValueError('unknown color %r, use one of: %s or letters r/g/b'
                             % (w, ', '.join(COLORS)))
    return tuple(state)


def color_name(state):
    for name, s in COLORS.items():
        if s == tuple(state):
            return name
    return '?'


class FakeGpio:
    """No hardware, just remembers. For tests and desktop runs."""

    def __init__(self, gpios=JETSON_NANO_GPIOS):
        self.gpios = tuple(gpios)
        self.state = (0, 0, 0)

    def write(self, state):
        self.state = tuple(int(bool(v)) for v in state)

    def read(self):
        return self.state


class SysfsGpio:
    """Outputs through /sys/class/gpio.

    Exported pins keep their level after the process exits, which is what lets
    the CLI set a color and quit. Never unexported here on purpose: that would
    make them inputs again and the gate pull-downs would switch the strip off.
    """

    def __init__(self, gpios=JETSON_NANO_GPIOS, root='/sys/class/gpio',
                 udev_wait=2.0):
        self.gpios = tuple(gpios)
        self.root = root
        self.udev_wait = udev_wait

    def _path(self, gpio, name):
        return os.path.join(self.root, 'gpio%d' % gpio, name)

    def _export(self, gpio):
        if os.path.isdir(os.path.join(self.root, 'gpio%d' % gpio)):
            return
        try:
            with open(os.path.join(self.root, 'export'), 'w') as f:
                f.write(str(gpio))
        except PermissionError:
            raise PermissionError(
                'cannot write %s/export: add yourself to the gpio group '
                '(src/hydrone_leds/setup_gpio.sh) or use sudo' % self.root)

    def _write_file(self, path, text):
        # right after an export, udev still has to chgrp the new files
        deadline = time.monotonic() + self.udev_wait
        while True:
            try:
                with open(path, 'w') as f:
                    f.write(text)
                return
            except (PermissionError, FileNotFoundError):
                if time.monotonic() > deadline:
                    raise
                time.sleep(0.02)

    def write(self, state):
        for gpio, on in zip(self.gpios, state):
            self._export(gpio)
            # 'high'/'low' sets direction and level in one go: no glitch
            self._write_file(self._path(gpio, 'direction'), 'high' if on else 'low')

    def read(self):
        out = []
        for gpio in self.gpios:
            try:
                with open(self._path(gpio, 'direction')) as f:
                    if f.read().strip() != 'out':
                        out.append(0)  # input: the pull-down holds the gate low
                        continue
                with open(self._path(gpio, 'value')) as f:
                    out.append(int(f.read().strip()))
            except FileNotFoundError:
                out.append(0)  # never exported
        return tuple(out)


def make_backend(name, gpios=JETSON_NANO_GPIOS):
    if name == 'sysfs':
        return SysfsGpio(gpios)
    if name == 'fake':
        return FakeGpio(gpios)
    raise ValueError('unknown backend %r (sysfs, fake)' % name)


class RgbLeds:
    def __init__(self, backend):
        self.backend = backend

    def set(self, color):
        """color: a name, a word list for parse_color, or an (r, g, b) tuple."""
        state = tuple(color) if isinstance(color, (tuple, list)) and \
            all(isinstance(v, int) for v in color) else parse_color(color)
        self.backend.write(state)
        return state

    def off(self):
        return self.set(COLORS['off'])

    def get(self):
        return self.backend.read()
