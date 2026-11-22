# Octavius — brazo manipulador simulado con ROS 2

Proyecto de la asignatura **Open Robotics** (2° semestre 2026) — Cesar Olivares Lavalovich.

Octavius es un brazo de **6 articulaciones** (base, hombro, codo, muñeca pitch, muñeca roll y garra de dos dedos) modelado en URDF/XACRO, visualizado en RViz y simulado en Gazebo Harmonic con `ros2_control`. Lleva una cámara en la garra y una IMU en la muñeca.

> Guía rápida para instalar y ejecutar. La explicación detallada de cada componente (qué es, por qué se usa, qué datos maneja y cómo los procesa), con ecuaciones, mediciones y decisiones de diseño, está en [`DOCUMENTACION.md`](DOCUMENTACION.md). El plan de trabajo está en [`PLAN_DE_TRABAJO.md`](PLAN_DE_TRABAJO.md).

---

## 1. Requisitos

| Software | Versión probada |
|---|---|
| Ubuntu | 24.04 LTS |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic (se instala con `ros-jazzy-ros-gz`) |

## 2. Instalación

```bash
# 1. Herramientas para resolver dependencias (si aún no están)
sudo apt update
sudo apt install python3-rosdep python3-colcon-common-extensions

# 2. Dependencias del proyecto, leídas desde los package.xml
cd Evaluacion_1
source /opt/ros/jazzy/setup.bash
sudo rosdep init   # solo la primera vez en el PC; si ya existe, ignorar el error
rosdep update
rosdep install --from-paths src --ignore-src -y
```

## 3. Compilación

```bash
cd Evaluacion_1
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

> Cada terminal nueva necesita `source install/setup.bash` antes de usar los paquetes.

## 4. Ejecución

### 4.1 Solo el modelo en RViz (con sliders)

```bash
ros2 launch octavius_description display.launch.py
```

Se abre RViz con el brazo y la ventana *Joint State Publisher* con un slider por articulación. Al mover un slider se mueve el modelo y su TF.

### 4.2 Simulación completa (Gazebo + RViz + control)

```bash
ros2 launch octavius_bringup bringup.launch.py
```

Levanta Gazebo, los controladores, RViz y el nodo `arm_commander`, que recibe los comandos del teclado y del joystick.
(`gazebo.launch.py` levanta lo mismo pero sin `arm_commander`.)

Argumentos opcionales (ver todos con `ros2 launch octavius_bringup bringup.launch.py --show-args`):

| Argumento | Defecto | Descripción |
|---|---|---|
| `use_rviz` | `true` | Abrir RViz |
| `headless` | `false` | Gazebo sin interfaz gráfica |
| `world` | `workcell.sdf` | Mundo de Gazebo |
| `use_microros` | `false` | Lanzar el agente micro-ROS (joystick de la ESP32) |
| `serial_port` | `/dev/ttyUSB0` | Puerto serial de la ESP32 |
| `agent` | `native` | Agente micro-ROS: `native` (compilado) o `docker` |

### 4.3 Control por teclado

En **otra terminal** (el nodo necesita leer el teclado, por eso no va dentro del launch):

```bash
source install/setup.bash
ros2 run octavius_bringup keyboard_teleop
```

| Tecla | Acción |
|---|---|
| `w` / `s` | Codo arriba / abajo |
| `r` / `f` | Muñeca pitch arriba / abajo |
| `q` / `e` | Girar la garra |
| `a` | Abrir / cerrar garra |
| `h` | Volver a home (todas las articulaciones a 0) |

Cada pulsación mueve 0,05 rad (≈ 2,9°). Mantener la tecla presionada repite el paso. Ninguna articulación pasa de su límite del URDF menos un margen de 0,02 rad (*límites blandos*, ver `limit_margin` en `config/teleop.yaml`). La base y el hombro se controlan con el joystick de la ESP32 (Fase 4).

Sin la ESP32, el joystick se puede simular publicando a mano (eje X → base, eje Y → hombro, valores entre -1 y 1):

```bash
ros2 topic pub -r 20 /octavius/joystick sensor_msgs/msg/Joy "{axes: [0.5, 0.0]}"
```

### 4.4 Mover el brazo directo a los controladores (sin arm_commander)

Con `gazebo.launch.py` corriendo, en otra terminal:

```bash
# Brazo: 5 ángulos [rad] en orden base, hombro, codo, muñeca pitch, muñeca roll
ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.6, 0.7, 0.8, 0.5, 0.0]}"

# Garra: apertura de cada dedo [m], de 0.0 (cerrada) a 0.025 (abierta); un valor por dedo, siempre iguales
ros2 topic pub --once /gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.02, 0.02]}"
```

### 4.5 Joystick con la ESP32 (micro-ROS)

Circuito, explicación del procesamiento y carga del firmware: [`docs/circuito_joystick.md`](docs/circuito_joystick.md).

**a) Agente micro-ROS.** Es el programa del PC que conecta la ESP32 con ROS 2. Hay dos opciones:

*Opción 1: compilarlo* (tutorial oficial de micro-ROS; se hace una sola vez, en un workspace aparte):

```bash
mkdir -p ~/microros_ws/src && cd ~/microros_ws
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
source /opt/ros/jazzy/setup.bash
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build && source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

Después, en cada terminal donde se lance Octavius con el joystick: `source ~/microros_ws/install/local_setup.bash`.

*Opción 2: Docker*, sin compilar. Se usa agregando `agent:=docker` al launch; la primera vez descarga la imagen oficial `microros/micro-ros-agent:jazzy`.

**b) Lanzar** con la ESP32 conectada por USB:

```bash
source ~/microros_ws/install/local_setup.bash     # solo con la opción 1
source install/setup.bash
ros2 launch octavius_bringup bringup.launch.py use_microros:=true
```

Si la placa aparece en otro puerto: `serial_port:=/dev/ttyACM0` (para verlo: `ls /dev/ttyUSB* /dev/ttyACM*`).

El LED de la ESP32 pasa de parpadear a quedar fijo cuando se conecta. Para comprobarlo:

```bash
ros2 node list                    # aparece /octavius_joystick
ros2 topic hz /octavius/joystick  # ~20 Hz
```

## 5. Verificación rápida

```bash
ros2 control list_controllers          # joint_state_broadcaster, arm_controller y gripper_controller: active
ros2 topic hz /octavius/camera/image   # cámara, ~15 Hz
ros2 topic hz /octavius/imu            # IMU, ~100 Hz
```

### 5.1 Pruebas automáticas

```bash
colcon test
colcon test-result --verbose
```

| Prueba | Qué verifica |
|---|---|
| `test_arm_logic.py` | Fórmulas de `arm_commander`: límites y límites blandos, zona muerta, integración del joystick, paso por tecla, retorno a home y mapa de teclas |
| `test_config_consistency.py` | Que URDF, `controllers.yaml`, `teleop.yaml` y `gz_bridge.yaml` usen los mismos nombres de articulaciones y tópicos |
| `flake8`, `pep257` | Estilo de código y docstrings de Python (estándar de ROS 2) |
| `copyright` | Licencia en cada archivo fuente |
| `xmllint`, `lint_cmake` | Sintaxis de `package.xml` y `CMakeLists.txt` |

### 5.2 Prueba de integración y prueba en limpio

```bash
# Con la simulación corriendo (bringup.launch.py), en otra terminal:
python3 scripts/integration_test.py

# Instalación completa en un Ubuntu + ROS 2 limpio (requiere Docker, ~10 min):
./scripts/clean_room_test.sh
```

## 6. Problemas frecuentes

| Síntoma | Causa / solución |
|---|---|
| `ros2 topic hz /octavius/camera/image` da menos de 15 Hz | El PC no tiene GPU (o Gazebo usa renderizado por software): la cámara se procesa más lento. No afecta al control del brazo |
| `ros2 topic echo` dice `does not appear to be published yet` justo después de lanzar | El *daemon* de la CLI de ROS 2 aún no descubre el tópico. Esperar unos segundos y repetir |
| `No se encontró el paquete micro_ros_agent` | Falta `source ~/microros_ws/install/local_setup.bash`, o usar `agent:=docker` |
| El agente no abre el puerto (`Permission denied`) | El usuario no está en el grupo `dialout`: `sudo usermod -aG dialout $USER` y volver a iniciar sesión |
| El LED sigue parpadeando | Puerto equivocado (`serial_port:=...`), o el Monitor Serie del Arduino IDE sigue abierto |
| El brazo se mueve solo con el joystick suelto | Se tocó el joystick mientras la ESP32 arrancaba (calibración). Reiniciar la placa sin tocarlo |
| Al cerrar con Ctrl+C aparece `gazebo-1 ... exit code -2` o `micro_ros_agent ... exit code -2` | Normal: Gazebo (`gz sim`) y el agente micro-ROS terminan por la propia señal de Ctrl+C en vez de capturarla. No es un error del proyecto |

## 7. Estructura

```
Evaluacion_1/
├── README.md
├── DOCUMENTACION.md               documentación técnica y tutorial completo
├── PLAN_DE_TRABAJO.md
├── docs/                          circuito del joystick y protocolo de pruebas con hardware
├── presentacion/                  presentación Beamer (plantilla USACH) y su PDF
├── firmware/octavius_joystick/    firmware de la ESP32 (micro_ros_arduino)
├── scripts/                       zip de entrega y pruebas de integración / en limpio
└── src/
    ├── octavius_description/      modelo URDF/XACRO + RViz
    │   ├── urdf/                  octavius.urdf.xacro (principal) y sus partes
    │   ├── launch/display.launch.py
    │   └── rviz/display.rviz
    └── octavius_bringup/          simulación y control
        ├── launch/                bringup.launch.py (principal) y gazebo.launch.py
        ├── octavius_bringup/      nodos: arm_commander, keyboard_teleop, arm_logic (cálculos)
        ├── config/                controladores, teleoperación, bridge Gazebo↔ROS, RViz
        ├── test/                  tests unitarios y linters
        └── worlds/workcell.sdf    mesa con objetos
```

## 8. Estado del proyecto

| Fase | Estado |
|---|---|
| 0 — Workspace y paquetes | ✅ |
| 1 — Modelo URDF/XACRO + RViz | ✅ |
| 2 — Gazebo + ros2_control | ✅ |
| 3 — Teclado + `arm_commander` | ✅ |
| 4 — micro-ROS + joystick | 🟡 código completo: firmware compilado sin advertencias y agente probado. Pruebas con la ESP32 pendientes: [`docs/pruebas_hardware.md`](docs/pruebas_hardware.md) |
| 5 — Launch integrado | ✅ `bringup.launch.py` (con `use_microros:=true` suma el agente) |
| 6 — Sensores básicos (cámara + IMU) | ✅ adelantada: publican y se ven en RViz |
| 7 — Calidad y prueba en limpio | ✅ probado en un contenedor `ros:jazzy` limpio siguiendo solo este README ([evidencia](docs/evidencia/prueba_en_limpio_2026-11-09.log)) |
| 8 — Documentación y entrega | ✅ [`DOCUMENTACION.md`](DOCUMENTACION.md) y [presentación](presentacion/Presentacion_Octavius.pdf) |
