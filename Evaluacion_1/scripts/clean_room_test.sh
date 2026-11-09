#!/usr/bin/env bash
# Prueba en limpio: comprueba que el proyecto se instala, compila y funciona
# en un Ubuntu 24.04 + ROS 2 Jazzy recién instalado, siguiendo solo el README.
#
# Usa Docker con la imagen oficial ros:jazzy (no tiene Gazebo ni nada del
# proyecto). Dentro: descomprime el zip de entrega, rosdep install, colcon
# build, colcon test, lanza la simulación sin interfaz gráfica y ejecuta
# scripts/integration_test.py. Tarda ~10 min (descarga Gazebo).
#
# Uso:  ./scripts/clean_room_test.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORK="$(mktemp -d)"
"$ROOT/scripts/make_release.sh" "$WORK"
cp "$ROOT/scripts/clean_room_test_container.sh" "$ROOT/scripts/integration_test.py" "$WORK/"
DRI=""; [ -d /dev/dri ] && DRI="--device /dev/dri"   # GPU para la cámara, si existe
docker run --rm $DRI -v "$WORK:/release" ros:jazzy bash /release/clean_room_test_container.sh
rm -rf "$WORK"
