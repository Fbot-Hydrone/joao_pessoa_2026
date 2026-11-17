#!/usr/bin/env python3
"""Carry the viewer's UDP stream over an ssh connection (stdout/stdin).

When the stack runs on another machine and UDP can't come back (a firewall
here, a NAT there), relay it through ssh instead:

    ssh user@remote 'python3 viz_relay.py out 47800' | python3 viz_relay.py in 47800

`out` listens on the remote's UDP port (point viz_host at 127.0.0.1 there) and
writes one JSON message per line; `in` reads those lines and sends them to the
local viewer's port.
"""
import socket
import sys


def out(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1', port))
    w = sys.stdout.buffer
    while True:
        data, _ = s.recvfrom(1 << 16)
        w.write(data.replace(b'\n', b' ') + b'\n')
        w.flush()


def inn(port, host='127.0.0.1'):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for line in sys.stdin.buffer:
        line = line.strip()
        if line:
            s.sendto(line, (host, port))


if __name__ == '__main__':
    mode, port = sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 47800
    try:
        out(port) if mode == 'out' else inn(port)
    except (KeyboardInterrupt, BrokenPipeError):
        pass
