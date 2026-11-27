#!/usr/bin/env bash
# Instala todo lo necesario y compila el proyecto, en un solo comando:
#
#     ./instalar.sh            dependencias + compilación
#     ./instalar.sh --test     además corre los tests (colcon test)
#
# Requisito previo: Ubuntu 24.04 con ROS 2 Jazzy instalado en /opt/ros/jazzy
# (guía oficial: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
#
# Cómo funciona: cada paquete declara en su package.xml lo que necesita
# (Gazebo, ros2_control, numpy, ...). rosdep lee esos package.xml y le pide a
# apt los paquetes del sistema que falten. colcon compila los 3 paquetes.
# (El setup.py de los paquetes Python NO instala dependencias: solo dice
# qué archivos propios se instalan: nodos, launch y config.)

set -euo pipefail
cd "$(dirname "$0")"

# Como root (p. ej. en un contenedor) no hace falta sudo, y puede no existir.
SUDO="sudo"
[ "$(id -u)" -eq 0 ] && SUDO=""

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "No se encontró ROS 2 Jazzy en /opt/ros/jazzy."
  echo "Instálalo primero: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html"
  exit 1
fi
# setup.bash de ROS usa variables sin definir: se relaja -u solo para cargarlo
set +u
source /opt/ros/jazzy/setup.bash
set -u

echo "== 1/4 Herramientas: rosdep y colcon =="
$SUDO apt-get update
DEBIAN_FRONTEND=noninteractive $SUDO apt-get install -y python3-rosdep python3-colcon-common-extensions

echo "== 2/4 Base de datos de rosdep =="
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  $SUDO rosdep init
fi
rosdep update

echo "== 3/4 Dependencias del proyecto (leídas de los package.xml) =="
rosdep install --from-paths src --ignore-src -y

echo "== 4/4 Compilación =="
colcon build --symlink-install

if [ "${1:-}" = "--test" ]; then
  echo "== Tests =="
  colcon test
  colcon test-result --verbose
fi

echo
echo "Listo. En cada terminal nueva:"
echo "    source $(pwd)/install/setup.bash"
echo "Y para lanzar todo:"
echo "    ros2 launch hexapod_bringup bringup.launch.py"
