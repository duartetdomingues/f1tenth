#!/bin/bash

# Diretório onde o mapa será salvo
MAP_DIR="/home/jetson/f1tenth/maps"

# Nome do ficheiro com data e hora atual
TIMESTAMP=$(date +"%Y_%m_%d-%H_%M")
MAP_NAME="${MAP_DIR}/map_${TIMESTAMP}"

# Cria o diretório se não existir
mkdir -p "$MAP_DIR"

# Informa o utilizador
echo "[🗺️] Salvando mapa em: $MAP_NAME"

# Executa o comando de salvamento
ros2 run nav2_map_server map_saver_cli -f "$MAP_NAME"
