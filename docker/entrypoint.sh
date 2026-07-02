#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

# The biguasim package (BiguaSim client + ArduPilot bridge runner) lives in
# the bs-drone-competition repo, mounted read-only by docker-compose. It is
# installed on first container start; the copy to a temp dir keeps pip from
# writing build artifacts into the mounted (host) repo.
if ! python3 -c 'import biguasim' 2>/dev/null; then
    if [ -d /opt/bs-drone-competition ]; then
        echo "[entrypoint] installing biguasim from /opt/bs-drone-competition ..."
        tmp=$(mktemp -d)
        cp -r /opt/bs-drone-competition/pyproject.toml \
              /opt/bs-drone-competition/setup.py \
              /opt/bs-drone-competition/README.md \
              /opt/bs-drone-competition/src \
              "$tmp"/
        pip3 install --no-cache-dir "$tmp"
        rm -rf "$tmp"
    else
        echo "[entrypoint] WARNING: biguasim not installed and /opt/bs-drone-competition not mounted." >&2
        echo "[entrypoint]          ardubridge_node will fail to import biguasim (see docker-compose.yml)." >&2
    fi
fi

# Unreal Engine refuses to start as root ("Refusing to run with the root
# privileges"), so drop to the unprivileged user, first granting it the
# groups that own the mounted /dev/dri render nodes (gids vary per host).
if [ "$(id -u)" = 0 ]; then
    for gid in $(stat -c %g /dev/dri/* 2>/dev/null | sort -u); do
        [ "$gid" = 0 ] && continue
        getent group "$gid" >/dev/null || groupadd -g "$gid" "dri$gid"
        usermod -aG "$gid" hydrone
    done
    export HOME=/home/hydrone
    cd /home/hydrone
    exec setpriv --reuid=hydrone --regid=hydrone --init-groups "$@"
fi

exec "$@"
