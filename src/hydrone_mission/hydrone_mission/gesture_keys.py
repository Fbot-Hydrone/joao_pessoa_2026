#!/usr/bin/env python3
"""
gesture_keys — the operator's arms, from the keyboard (Phase 3 in the sim).

    ros2 run hydrone_mission gesture_keys

Publishes the chosen gesture on /hydrone/gesture/inject at 10 Hz for as long as
it stays chosen, exactly like an arm held in the air: phase3_gesture_node (with
allow_inject, the sim default) takes it instead of the camera and debounces it
the same way. The mission's state line is shown underneath.

  w APROXIMAR   s AFASTAR    a ESQUERDA   d DIREITA
  r SUBIR       f DESCER     t STOP       l POUSAR
  space HOVER (also "the operator is in view": FIND needs a person)
  0 nobody in view      q quit
"""

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

KEYS = {
    'w': 'APROXIMAR', 's': 'AFASTAR', 'a': 'ESQUERDA', 'd': 'DIREITA',
    'r': 'SUBIR', 'f': 'DESCER', 't': 'STOP', 'l': 'POUSAR',
    ' ': 'HOVER', '0': 'SEM_PESSOA',
}


class GestureKeys(Node):

    def __init__(self):
        super().__init__('gesture_keys')
        self.pub = self.create_publisher(String, '/hydrone/gesture/inject', 10)
        self.create_subscription(String, '/hydrone/gesture/state', self._state_cb, 10)
        self.gesture = 'SEM_PESSOA'
        self.state = '(no mission state yet)'

    def _state_cb(self, m):
        self.state = m.data

    def publish(self):
        self.pub.publish(String(data=self.gesture))


def main():
    rclpy.init()
    node = GestureKeys()
    fd = sys.stdin.fileno()
    if not sys.stdin.isatty():
        print('gesture_keys needs a terminal: docker compose exec -it ...', file=sys.stderr)
        return
    old = termios.tcgetattr(fd)
    print(__doc__.split('\n\n', 2)[2])
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                k = sys.stdin.read(1).lower()
                if k == 'q':
                    break
                if k in KEYS:
                    node.gesture = KEYS[k]
            node.publish()
            rclpy.spin_once(node, timeout_sec=0.0)
            sys.stdout.write(f'\r\033[K sending: {node.gesture:<11} | mission: {node.state}')
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
