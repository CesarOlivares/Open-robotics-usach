# Octavius — brazo manipulador simulado con ROS 2

Proyecto de la asignatura **Open Robotics** (2° semestre 2026) — Cesar Olivares Lavalovich.

Octavius es un brazo de **6 articulaciones** (base, hombro, codo, muñeca pitch, muñeca roll y garra) modelado en URDF/XACRO, visualizado en RViz y simulado en Gazebo Harmonic con `ros2_control`. Lleva una cámara en la garra y una IMU en la muñeca.

> Guía rápida para instalar y ejecutar. La explicación detallada de cada componente estará en `DOCUMENTACION.md`, y el plan de trabajo en [`PLAN_DE_TRABAJO.md`](PLAN_DE_TRABAJO.md).

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

### 4.2 Simulación completa en Gazebo + RViz

```bash
ros2 launch octavius_bringup gazebo.launch.py
```

Argumentos opcionales (ver todos con `ros2 launch octavius_bringup gazebo.launch.py --show-args`):

| Argumento | Defecto | Descripción |
|---|---|---|
| `use_rviz` | `true` | Abrir RViz |
| `headless` | `false` | Gazebo sin interfaz gráfica |
| `world` | `workcell.sdf` | Mundo de Gazebo |

### 4.3 Mover el brazo a mano (prueba de los controladores)

Con la simulación corriendo, en otra terminal:

```bash
# Brazo: 5 ángulos [rad] en orden base, hombro, codo, muñeca pitch, muñeca roll
ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.6, 0.7, 0.8, 0.5, 0.0]}"

# Garra: apertura de cada dedo [m], de 0.0 (cerrada) a 0.025 (abierta)
ros2 topic pub --once /gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.02]}"
```

## 5. Verificación rápida

```bash
ros2 control list_controllers          # joint_state_broadcaster, arm_controller y gripper_controller: active
ros2 topic hz /octavius/camera/image   # cámara, ~15 Hz
ros2 topic hz /octavius/imu            # IMU, ~100 Hz
```

## 6. Estructura

```
Evaluacion_1/
├── README.md
├── PLAN_DE_TRABAJO.md
└── src/
    ├── octavius_description/      modelo URDF/XACRO + RViz
    │   ├── urdf/                  octavius.urdf.xacro (principal) y sus partes
    │   ├── launch/display.launch.py
    │   └── rviz/display.rviz
    └── octavius_bringup/          simulación y control
        ├── launch/gazebo.launch.py
        ├── config/                controladores, bridge Gazebo↔ROS, RViz
        └── worlds/workcell.sdf    mesa con objetos
```

## 7. Estado del proyecto

| Fase | Estado |
|---|---|
| 0 — Workspace y paquetes | ✅ |
| 1 — Modelo URDF/XACRO + RViz | ✅ |
| 2 — Gazebo + ros2_control | ✅ |
| 3 — Teclado + `arm_commander` | ⏳ |
| 4 — micro-ROS + joystick | ⏳ |
| 5 — Launch integrado | ⏳ |
| 6 — Sensores básicos (cámara + IMU) | ✅ adelantada: publican y se ven en RViz |
| 7–8 — Calidad, documentación y entrega | ⏳ |
