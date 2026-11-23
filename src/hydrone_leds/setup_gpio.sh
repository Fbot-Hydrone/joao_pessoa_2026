#!/bin/bash
# One-time: let a non-root user drive /sys/class/gpio (the `leds` CLI, and the
# container, which runs as that user with its groups). Safe to re-run.
# JetPack's Jetson.GPIO already ships this as 60-jetson-gpio-common.rules; this
# only fills in what is missing.
set -e
USER_NAME=${1:-${SUDO_USER:-$USER}}
RULE=/etc/udev/rules.d/99-hydrone-gpio.rules

getent group gpio >/dev/null || sudo groupadd --system gpio
if ! id -nG "$USER_NAME" | tr ' ' '\n' | grep -qx gpio; then
    sudo usermod -aG gpio "$USER_NAME"
    echo "added $USER_NAME to gpio: log out and back in (or reboot) for it to apply"
fi

if [ "$(stat -c %G /sys/class/gpio/export)" = gpio ]; then
    echo "gpio sysfs already group-writable"
    exit 0
fi

sudo tee "$RULE" >/dev/null <<'RULES'
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", ACTION=="add", \
    PROGRAM="/bin/sh -c 'chown root:gpio /sys/class/gpio/export /sys/class/gpio/unexport; chmod 220 /sys/class/gpio/export /sys/class/gpio/unexport'"
SUBSYSTEM=="gpio", DEVPATH=="/*/gpiochip*/gpio/*", ACTION=="add", \
    PROGRAM="/bin/sh -c 'chown root:gpio /sys%p/active_low /sys%p/direction /sys%p/edge /sys%p/value; chmod 660 /sys%p/active_low /sys%p/direction /sys%p/edge /sys%p/value'"
RULES
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=gpio
sleep 1
echo "export now: $(stat -c '%G %A' /sys/class/gpio/export)  ($RULE)"
