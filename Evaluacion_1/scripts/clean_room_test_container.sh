#!/bin/bash
# Prueba en limpio: sigue SOLO los pasos del README dentro de ros:jazzy.
export DEBIAN_FRONTEND=noninteractive
step() { echo; echo "=================== $* ==================="; }

step "0. Descomprimir el zip de entrega"
apt-get update -qq && apt-get install -y -qq unzip >/dev/null
cd /root && unzip -q /release/cesar_olivares_2026_2.zip && cd cesar_olivares_2026_2 && ls

step "README 2. Instalación"
apt-get install -y -qq python3-rosdep python3-colcon-common-extensions >/dev/null
source /opt/ros/jazzy/setup.bash
rosdep init 2>/dev/null || echo "(rosdep ya inicializado)"
rosdep update -q
rosdep install --from-paths src --ignore-src -y -q 2>&1 | tail -3
echo "rosdep exit: ${PIPESTATUS[0]}"

step "README 3. Compilación"
colcon build 2>&1 | tail -4
source install/setup.bash

step "Tests (colcon test)"
colcon test >/dev/null 2>&1; colcon test-result --verbose | tail -5

step "README 4.1 display.launch.py (sin GUI: use_gui:=false, RViz falla sin pantalla)"
ros2 daemon start >/dev/null 2>&1; timeout 15 ros2 launch octavius_description display.launch.py use_gui:=false > /tmp/display.log 2>&1 &
sleep 10; ros2 topic echo --once /joint_states --field name 2>&1 | head -3; wait

step "README 4.2 bringup.launch.py headless"
ros2 launch octavius_bringup bringup.launch.py headless:=true use_rviz:=false > /tmp/bringup.log 2>&1 &
for i in $(seq 1 120); do ros2 control list_controllers 2>/dev/null | grep -q "gripper_controller.*active" && break; sleep 1; done
ros2 control list_controllers
step "README 5. Verificación rápida (sensores)"
timeout 8 ros2 topic hz /octavius/camera/image 2>&1 | grep -m1 "average rate" || echo "camara: SIN DATOS"
timeout 6 ros2 topic hz /octavius/imu 2>&1 | grep -m1 "average rate" || echo "imu: SIN DATOS"

step "Prueba funcional teclado/joystick (fase3_test.py)"
python3 /release/integration_test.py

step "Errores en el log del launch"
grep -iE "\[ERROR\]|\[Err\]|exception" /tmp/bringup.log | head -10 || true
kill -INT %1 2>/dev/null; sleep 5
echo; echo "FIN DE LA PRUEBA EN LIMPIO"
