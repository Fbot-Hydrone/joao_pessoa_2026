#!/bin/bash

echo "Liberando a placa de vídeo para o Docker..."
xhost +local:docker

echo "Iniciando o ecossistema do Drone..."
docker compose up --build
