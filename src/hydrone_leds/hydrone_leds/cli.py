"""Set the LED strip from a shell. The color stays after this exits.

    leds white            leds red blue      leds rb
    leds off              leds status        (no args = status)
"""

import argparse
import sys

from hydrone_leds.core import CHANNELS, COLORS, JETSON_NANO_GPIOS, RgbLeds, \
    color_name, make_backend


def main(argv=None):
    p = argparse.ArgumentParser(
        prog='leds', description='RGB strip on/off per channel.',
        epilog='colors: %s, or channel letters (rg, r b, ...)' % ', '.join(COLORS))
    p.add_argument('color', nargs='*', help="color name(s), letters, or 'status'")
    p.add_argument('--backend', default='sysfs', choices=('sysfs', 'fake'))
    p.add_argument('--gpios', default=','.join(map(str, JETSON_NANO_GPIOS)),
                   help='sysfs numbers for red,green,blue (default: Nano pins 29,31,33)')
    a = p.parse_args(argv)

    gpios = [int(g) for g in a.gpios.split(',')]
    if len(gpios) != 3:
        p.error('--gpios needs three numbers: red,green,blue')
    leds = RgbLeds(make_backend(a.backend, gpios))
    try:
        if a.color and a.color != ['status']:
            leds.set(a.color)
        state = leds.get()
    except (ValueError, OSError) as e:
        print('leds: %s' % e, file=sys.stderr)
        return 1
    on = [c for c, v in zip(CHANNELS, state) if v] or ['none']
    print('%s  (on: %s)' % (color_name(state), ' '.join(on)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
