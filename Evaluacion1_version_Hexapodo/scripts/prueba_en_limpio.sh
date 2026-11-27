#!/usr/bin/env bash
# Prueba en limpio (E7): ¿funciona el proyecto en un PC recién instalado?
#
# Usa Docker con la imagen oficial ros:jazzy: Ubuntu 24.04 + ROS 2 Jazzy
# base, sin Gazebo ni nada del proyecto (como el PC del profesor). Copia el
# proyecto TAL COMO ESTÁ EN GIT (último commit, sin build/ ni install/) y
# dentro del contenedor sigue solo INICIO_RAPIDO.md:
#     1. ./instalar.sh --test   (rosdep + colcon build + colcon test)
#     2. display.launch.py      (modelo, sin pantalla)
#     3. bringup.launch.py headless + scripts/prueba_integracion.py
# Tarda ~10–15 min la primera vez (descarga Gazebo).
#
# Uso:  ./scripts/prueba_en_limpio.sh            (desde Evaluacion1_version_Hexapodo/)
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO="$(git -C "$ROOT" rev-parse --show-toplevel)"
SUB="$(realpath --relative-to="$REPO" "$ROOT")"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

echo "Proyecto: commit $(git -C "$REPO" rev-parse --short HEAD) ($SUB)"
git -C "$REPO" archive HEAD "$SUB" | tar -x -C "$WORK"

cat > "$WORK/en_contenedor.sh" <<IN
#!/bin/bash
set -o pipefail
export DEBIAN_FRONTEND=noninteractive
step() { echo; echo "=================== \$* ==================="; }
cp -r /release/$SUB /root/ && cd /root/$SUB

step "1. ./instalar.sh --test"
./instalar.sh --test 2>&1 | grep -vE "^(Get|Hit|Selecting|Preparing|Unpacking|Setting up|Processing)" | tail -40
source install/setup.bash

step "2. display.launch.py (sin pantalla: use_gui:=false; RViz falla sin pantalla, es esperado)"
ros2 daemon start >/dev/null 2>&1
timeout 15 ros2 launch hexapod_description display.launch.py use_gui:=false > /tmp/display.log 2>&1 &
sleep 10
echo "articulaciones en /joint_states: \$(ros2 topic echo --once /joint_states --field name 2>/dev/null | tr ',' '\n' | grep -c joint)"
wait

step "3. bringup.launch.py headless + prueba de integración"
ros2 launch hexapod_bringup bringup.launch.py headless:=true use_rviz:=false > /tmp/bringup.log 2>&1 &
python3 scripts/prueba_integracion.py
RES=\$?
ros2 control list_controllers

step "Errores en el log del launch"
grep -iE "\[ERROR\]|exception|traceback" /tmp/bringup.log | grep -v "Ogre\|rviz" | head -10 || true
kill -INT %1 2>/dev/null; sleep 5
echo; echo "FIN DE LA PRUEBA EN LIMPIO (prueba de integración: código \$RES)"
exit \$RES
IN
chmod +x "$WORK/en_contenedor.sh"

DRI=""; [ -d /dev/dri ] && DRI="--device /dev/dri"   # GPU para la cámara, si existe
docker run --rm $DRI -v "$WORK:/release:ro" ros:jazzy bash /release/en_contenedor.sh
