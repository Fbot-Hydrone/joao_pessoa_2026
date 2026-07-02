#!/usr/bin/env bash
# Bring up the full simulation stack in Docker.
set -e
cd "$(dirname "$0")/.."

# Let the containerized UE5 viewport open on the host X server
xhost +local:docker

# Make sure the shared asset dir exists so the bind mount doesn't create it root-owned
mkdir -p "$HOME/.local/share/biguasim"

docker compose up --build "$@"
