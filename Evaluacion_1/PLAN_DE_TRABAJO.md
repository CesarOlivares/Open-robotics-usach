# Plan de trabajo — Proyecto **Octavius**: brazo manipulador simulado con ROS 2

| | |
|---|---|
| Asignatura | Open Robotics — 2° semestre 2026 |
| Estudiante | Cesar Olivares Lavalovich |
| Ubicación | `~/Open_robotics/Evaluacion_1/` |
| Entrega | `cesar_olivares_2026_2.zip` |

> Documento para revisión **antes** de empezar a programar. Cuando esté aprobado, se ejecuta fase por fase.

---

## 0. Principios del proyecto

**Nada de cajas negras.** Cada componente (paquete, plugin, controlador, mensaje) se documenta respondiendo 4 preguntas:

1. **Qué es** — de qué paquete oficial viene y qué hace.
2. **Por qué lo usamos** — qué problema resuelve y qué alternativa se descartó.
3. **Qué datos entran y salen** — tópicos, tipos de mensaje, unidades, frecuencias.
4. **Cómo procesamos esos datos** — fórmulas y pasos (numerados en el documento).

**Primero el núcleo, después los extras.** El orden de trabajo prioriza lo que sostiene todo el proyecto: **modelo → controladores → integración (Gazebo + teclado + micro-ROS + launch)**. Los sensores entran con su versión más simple, la que basta para cumplir el requisito. Los sensores avanzados (encoder de cuadratura, procesamiento de la IMU) quedan como **extensión opcional** (§12), que se hace solo si el núcleo está terminado y documentado.

---

## 1. Contexto y entorno objetivo

| Ítem | Valor (verificado en esta máquina) |
|---|---|
| Sistema operativo | Ubuntu 24.04 LTS (x86_64) |
| ROS 2 | **Jazzy Jalisco** (`/opt/ros/jazzy`) |
| Simulador | **Gazebo Harmonic** (vía `ros_gz`, ya instalado) |
| Control | `ros2_control`, `ros2_controllers` (`position_controllers`, `forward_command_controller`), `gz_ros2_control` (instalados) |
| Visualización | RViz2, `joint_state_publisher(_gui)`, `xacro`, `rqt_plot` (instalados) |
| micro-ROS | Agente compilado en `~/Open_robotics/Micro-ros`; ESP32 + joystick analógico, transporte **serial (USB)** |
| Trabajo previo de clases | `open_ws/src/arm_robot_description` (brazo 2 GDL) — referencia de estilo |

**Fuente de referencia oficial:** docs.ros.org/en/jazzy, control.ros.org, gazebosim.org (Harmonic) y micro.ros.org.

**Criterio de éxito global:** el profesor descomprime el proyecto, ejecuta `rosdep install`, `colcon build`, `ros2 launch …` y todo funciona sin pasos ocultos.

---

## 2. Trazabilidad de requerimientos

| # | Requerimiento | Cómo se cumple | Verificación |
|---|---|---|---|
| R1 | Crear paquete con sus directorios | Workspace colcon con `octavius_description` y `octavius_bringup` (§4) | `colcon build` sin errores ni warnings |
| R2 | Robot articulado en URDF/XACRO | XACRO modular de **Octavius**: 6 articulaciones (§3) | `check_urdf`, árbol con `urdf_to_graphviz` |
| R3 | Cargar modelo en RViz | `display.launch.py` con `robot_state_publisher` + RViz | Captura de RViz |
| R4 | `joint_state_publisher` para ver movimiento | `joint_state_publisher_gui` (un slider por articulación) | Mover sliders → se mueve el brazo |
| R5 | Percepción en RViz: RobotModel y TF | Config `.rviz` con RobotModel, TF e Image (cámara de la garra) | Captura |
| R6 | Cargar modelo en Gazebo | `ros_gz_sim create` desde `/robot_description`, brazo fijado sobre una mesa | Brazo aparece en Gazebo |
| R7 | ≥ 2 plugins de sensores | **Cámara RGB en la garra** + **IMU en la muñeca**, ambos plugins nativos de Gazebo, en su versión básica: publicar y visualizar (§6) | `ros2 topic hz /octavius/camera/image /octavius/imu` |
| R8 | `ros2_control` + teclado | `gz_ros2_control` con interfaz de posición; **teclado controla codo, muñeca y garra** (§5) | Teclas mueven esas articulaciones en Gazebo y RViz |
| R9 | Conectividad y control de articulaciones con micro-ROS | **ESP32 + joystick controla base (X) y hombro (Y)**, publicando por serial vía `micro_ros_agent` (§5) | Mover el joystick → giran base y hombro |
| R10 | Iniciar todo con un launchfile | `bringup.launch.py` único con argumentos | Un solo comando levanta todo |
| R11 | Estructura de carpetas recomendada | Estructura estándar ROS 2 (§4) | Revisión |
| R12 | Presentación (PDF + editable), ≤ 15 min | Plantilla con los ítems de formato exigidos | Checklist §10 |
| R13 | Tutorial paso a paso (PDF + editable) | Generado desde el `.md` maestro (pandoc → `.docx` + `.pdf`) | Checklist §10 |
| R14 | Entrega `.zip` | `make_release.sh` → `cesar_olivares_2026_2.zip` sin `build/ install/ log/` | Descomprimir en carpeta nueva y compilar |

> **Nota sobre R7:** el requisito de 2 sensores no se puede postergar. Lo que sí se posterga es el trabajo avanzado sobre ellos. Cámara e IMU son plugins nativos de Gazebo: agregarlos es declarar el sensor en el URDF y una línea en el bridge, sin código propio. Por eso son la opción más barata para cumplir.

---

## 3. Diseño del robot Octavius

5 grados de libertad de brazo + garra = **6 articulaciones** (decisión propia, más allá del mínimo pedido).

| # | Articulación | Tipo | Eje | Función | Controlado por |
|---|---|---|---|---|---|
| 1 | `base_joint` | revolute | Z | Giro de la base | Joystick X (ESP32) |
| 2 | `shoulder_joint` | revolute | Y | Hombro | Joystick Y (ESP32) |
| 3 | `elbow_joint` | revolute | Y | Codo | Teclado |
| 4 | `wrist_pitch_joint` | revolute | Y | Inclina la muñeca | Teclado |
| 5 | `wrist_roll_joint` | revolute | Z | Gira la garra sobre su eje | Teclado |
| 6 | `gripper_joint` | prismatic | Y | Abre/cierra la garra (2° dedo con `mimic`) | Teclado |

```
world
└── base_link (fixed; la mesa es parte del mundo de Gazebo, no del robot)
    └── base_joint ──► turret_link
        └── shoulder_joint ──► upper_arm_link
            └── elbow_joint ──► forearm_link
                └── wrist_pitch_joint ──► wrist_link
                    ├── imu_link (fixed)               ← SENSOR 2: IMU
                    └── wrist_roll_joint ──► gripper_base_link
                        ├── camera_link (fixed)         ← SENSOR 1: cámara RGB
                        │   └── camera_optical_link (fixed, convención REP-103)
                        ├── gripper_joint ──► finger_left_link
                        └── gripper_mimic_joint ──► finger_right_link (mimic)
```

- Dimensiones, masas y límites en **un solo archivo** de parámetros xacro.
- Límites articulares (`lower/upper/velocity/effort`) en cada junta; tabla de parámetros DH en la documentación.
- Inercias con macros xacro (cilindro/caja), con las fórmulas numeradas en el documento.
- Geometría con primitivas (no depende de mallas externas).
- Pose cero: brazo vertical apuntando a +Z; por eso el giro de la garra (roll) es en torno a Z.

---

## 4. Estructura del repositorio

```
Evaluacion_1/
├── README.md                         ← guía rápida: instalar, compilar, ejecutar
├── DOCUMENTACION.md                  ← documento maestro detallado (fuente del tutorial)
├── PLAN_DE_TRABAJO.md                ← este documento
├── src/
│   ├── octavius_description/         ← ament_cmake: solo el modelo
│   │   ├── urdf/
│   │   │   ├── octavius.urdf.xacro       (principal, incluye los demás)
│   │   │   ├── octavius_params.xacro     (dimensiones, masas, límites)
│   │   │   ├── macros/inertial.xacro
│   │   │   ├── arm.xacro
│   │   │   ├── gripper.xacro
│   │   │   ├── sensors.xacro             (cámara + IMU con sus plugins gz)
│   │   │   └── ros2_control.xacro
│   │   ├── rviz/display.rviz
│   │   ├── launch/display.launch.py      (RViz + joint_state_publisher_gui)
│   │   └── package.xml / CMakeLists.txt
│   └── octavius_bringup/             ← ament_python: simulación, control y nodos
│       ├── launch/
│       │   ├── gazebo.launch.py
│       │   ├── microros_agent.launch.py
│       │   └── bringup.launch.py         (lanza todo)
│       ├── config/
│       │   ├── controllers.yaml          (ros2_control)
│       │   ├── teleop.yaml               (velocidades, zona muerta, mapeo de teclas)
│       │   ├── gz_bridge.yaml            (ros_gz_bridge)
│       │   └── bringup.rviz
│       ├── worlds/workcell.sdf           (mesa + objetos, 100 % local)
│       ├── octavius_bringup/
│       │   ├── keyboard_teleop.py        (lee teclas → publica intención de movimiento)
│       │   └── arm_commander.py          (integra joystick + teclado → comando a los controladores)
│       ├── test/                         (flake8, pep257, copyright, tests unitarios)
│       └── package.xml / setup.py / setup.cfg
├── firmware/
│   └── octavius_joystick/octavius_joystick.ino   (ESP32 + micro_ros_arduino, serial)
├── docs/
│   ├── img/                              (capturas, rqt_graph, árbol TF, árbol URDF, circuito)
│   ├── tutorial.docx / tutorial.pdf
│   └── presentacion.pptx / presentacion.pdf
└── scripts/
    ├── install_deps.sh
    └── make_release.sh
```

---

## 5. Control de articulaciones

### 5.1 Interfaces de ros2_control y por qué no hay PID propio

En `ros2_control`, cada articulación del hardware expone:

| Interfaz | Qué es | Equivalente en un robot real |
|---|---|---|
| **comando** `position` | ángulo objetivo que se le ordena | señal PWM de ángulo a un servo |
| **estado** `position` | ángulo medido | lectura de un encoder |
| **estado** `velocity` | velocidad medida | derivada de la lectura del encoder |

En Gazebo, el plugin `gz_ros2_control` lee el ángulo de cada junta desde el motor de física y lo entrega como interfaz de estado. Se comporta como un **encoder ideal** (resolución infinita, sin ruido). Ese valor sale por `joint_state_broadcaster` → `/joint_states`, que usan RViz, la TF y nuestro nodo.

El lazo de posición lo cierra el hardware: en Gazebo el plugin, en un servo real su electrónica interna. Por eso **no se agrega un PID propio**. Solo tendría sentido si el actuador entregara torque/corriente en vez de posición.

> El encoder de cuadratura "realista", con el método de la clase del profesor, queda en la extensión opcional (§12.1).

### 5.2 Controladores elegidos

| Controlador | Tipo (paquete oficial) | Articulaciones | Qué hace exactamente |
|---|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster` | todas | Lee las interfaces de estado y las publica en `/joint_states` |
| `arm_controller` | `position_controllers/JointGroupPositionController` | 1–5 | Recibe un vector de 5 ángulos (`std_msgs/Float64MultiArray`) y **lo copia tal cual** a las interfaces de comando en cada ciclo. Sin interpolación ni lógica oculta |
| `gripper_controller` | `position_controllers/JointGroupPositionController` | 6 | Igual, con 1 valor: ángulo de la garra (abierta / cerrada) |

**Por qué este controlador y no `joint_trajectory_controller` (JTC):**

- **JTC** recibe una *trayectoria*: una lista de puntos, cada uno con posiciones y el instante en que hay que llegar. En cada ciclo (ej. 100 Hz) interpola entre puntos con un spline, escribe la posición intermedia en la interfaz de comando y verifica tolerancias de llegada. Es el estándar cuando se planifican movimientos completos (p. ej. con MoveIt).
- Aquí no planificamos trayectorias: el operador mueve el brazo en tiempo real con joystick y teclado. Nuestro nodo ya genera incrementos pequeños y suaves, así que la interpolación de JTC no aporta nada y solo agregaría una capa más que explicar.
- `JointGroupPositionController` es la pieza más simple de `ros2_controllers` y se explica en una línea: "lo que publico es lo que se aplica". JTC queda descrito en la documentación como la alternativa y cuándo convendría.

### 5.3 Flujo de datos

```
 ESP32 + joystick ─(USB serial)─► micro_ros_agent ─► /octavius/joystick (sensor_msgs/Joy) ─┐
                                                                                          ├─► arm_commander ─► /arm_controller/commands     ─┐
 Teclado ─► keyboard_teleop ─────────────────────► /octavius/keyboard (sensor_msgs/Joy) ─┘         ▲          /gripper_controller/commands ─┤
                                                                                                    │ (estado inicial)                      ▼
                                       /joint_states ◄── joint_state_broadcaster ◄──────────────────┴────────── controller_manager + gz_ros2_control
```

Joystick y teclado comandan articulaciones distintas, pero `arm_controller` necesita **siempre el vector completo de 5 ángulos**. Si cada fuente publicara directo al controlador, una sobrescribiría a la otra. Por eso un único nodo, `arm_commander`, mantiene el vector objetivo y lo actualiza con ambas fuentes.

### 5.4 Cómo se procesan los datos

**a) En la ESP32 (firmware)** — lee y normaliza; no toma decisiones:

1. Lee el ADC de 12 bits de cada eje: `raw ∈ [0, 4095]`, centro ≈ 2048.
2. Normaliza a `[-1, 1]`: `v = (raw − centro) / 2048`.
3. Aplica zona muerta (el joystick en reposo nunca da exactamente el centro): si `|v| < 0,1` → `v = 0`.
4. Publica `sensor_msgs/Joy` con `axes = [v_x, v_y]` a 20 Hz.

Se usa `sensor_msgs/Joy` porque es el mensaje estándar de ROS para mandos, con ejes ya normalizados en `[-1, 1]`.

**b) En `arm_commander` (ROS)** — convierte "intención" en ángulo:

1. Al arrancar, toma el ángulo actual de cada junta desde `/joint_states` como objetivo inicial. Así el brazo no salta.
2. En cada ciclo (Δt = 0,02 s): `q_objetivo ← q_objetivo + v · ω_max · Δt`, donde `ω_max` es la velocidad máxima configurada en `teleop.yaml` (rad/s).
3. Satura con los límites del URDF: `q_objetivo ← clamp(q_objetivo, q_min, q_max)`.
4. Publica el vector de 5 ángulos a `/arm_controller/commands`.
5. Si deja de recibir datos del joystick por más de 0,5 s (cable desconectado), deja de mover base y hombro (*watchdog* de seguridad).

Ejemplo: joystick a la derecha a tope (`v = 1`), `ω_max = 0,5 rad/s` → la base gira 0,01 rad por ciclo ≈ 28,6 °/s.

**c) Teclado (`keyboard_teleop`)** — mapa propuesto (configurable en `teleop.yaml`):

| Tecla | Acción |
|---|---|
| `a` | Abrir / cerrar garra (alterna) |
| `q` / `e` | Girar la garra sobre su eje (muñeca roll) − / + |
| `w` / `s` | Codo arriba / abajo |
| `r` / `f` | Muñeca pitch arriba / abajo |
| `h` | Volver a posición *home* |

Codo y muñeca pitch también quedan en el teclado, porque el joystick solo tiene 2 ejes.

---

## 6. Sensores (versión básica)

Alcance de esta versión: **declarar el sensor, publicarlo en ROS y visualizarlo**. Sin procesamiento propio.

| | Sensor 1: Cámara RGB | Sensor 2: IMU |
|---|---|---|
| Ubicación | `camera_link`, en la garra (eye‑in‑hand) | `imu_link`, en la muñeca |
| Plugin Gazebo | `<sensor type="camera">` + sistema `gz-sim-sensors-system` | `<sensor type="imu">` + sistema `gz-sim-imu-system` |
| Tópico ROS (vía `ros_gz_bridge`) | `/octavius/camera/image` (`sensor_msgs/Image`) + `/octavius/camera/camera_info` | `/octavius/imu` (`sensor_msgs/Imu`) |
| Datos | Imagen 640×480 RGB a 15 Hz + parámetros intrínsecos | Orientación (cuaternión), velocidad angular (rad/s), aceleración lineal (m/s²) |
| Visualización | Panel *Image* en RViz | `rqt_plot` de velocidad angular mientras se mueve la muñeca |

- `ros_gz_bridge` se configura por YAML: cada entrada indica tópico Gazebo ↔ tópico ROS, tipo de mensaje y dirección. Se explica línea por línea.
- Se agrega `/clock` al bridge y `use_sim_time:=true` en todos los nodos, para que los sellos de tiempo de los sensores coincidan con la simulación.

---

## 7. Fases de trabajo

Orden: **núcleo (fases 0–5) → sensores básicos (6) → calidad y entrega (7–8)**. Cada fase termina con una verificación concreta antes de pasar a la siguiente.

### Fase 0 — Preparación (≈ 0,5 día)
- Workspace en `Evaluacion_1/`, repositorio git con commits por fase.
- `package.xml` con todas las dependencias para `rosdep install --from-paths src -y --ignore-src`.
- ✅ *Hecho cuando:* `colcon build` de paquetes vacíos funciona.

### Fase 1 — Modelo URDF/XACRO + RViz (R2–R5) (≈ 1,5 días)
- XACRO modular de Octavius, macros de inercia, materiales.
- `display.launch.py`: `robot_state_publisher` + `joint_state_publisher_gui` + RViz (RobotModel, TF).
- Árbol URDF, árbol TF (`ros2 run tf2_tools view_frames`) y tabla DH.
- ✅ *Hecho cuando:* `check_urdf` pasa y los sliders mueven las 6 articulaciones en RViz.

### Fase 2 — Gazebo + ros2_control (R6, R8) (≈ 1,5 días)
- Mundo `workcell.sdf`: mesa y objetos hechos con primitivas.
- Spawn con `ros_gz_sim create`; bloque `<ros2_control>` con `gz_ros2_control/GazeboSimSystem` (comando: posición; estado: posición y velocidad).
- `controllers.yaml` con los tres controladores; spawners encadenados con `OnProcessExit`.
- ✅ *Hecho cuando:* `ros2 control list_controllers` muestra los 3 activos y `ros2 topic pub` a `/arm_controller/commands` mueve el brazo en Gazebo y RViz.

### Fase 3 — Teclado + `arm_commander` (R8) (≈ 1 día)
- Nodos `keyboard_teleop` y `arm_commander`, con parámetros en `teleop.yaml` y tests unitarios de la integración y la saturación de límites.
- ✅ *Hecho cuando:* el teclado mueve codo, muñeca y garra sin pasarse de los límites.

### Fase 4 — micro-ROS + joystick (R9) (≈ 1,5 días)
- Circuito: ejes del joystick en pines ADC1 de la ESP32 (p. ej. GPIO34/35). Se incluye diagrama de conexión.
- Firmware `octavius_joystick.ino`: transporte serial (`set_microros_transports()`), normalización y zona muerta (§5.4a), publica `sensor_msgs/Joy` a 20 Hz, LED indica conexión con el agente y reconecta si este se cae.
- `microros_agent.launch.py`: `micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200` (puerto configurable).
- ✅ *Hecho cuando:* el joystick gira la base (X) y mueve el hombro (Y) en Gazebo y RViz.

### Fase 5 — Launch integrado (R10) (≈ 0,5 día)
- `bringup.launch.py` con argumentos documentados (`--show-args`): `use_rviz`, `use_microros`, `serial_port`, `world`, `headless`.
- ✅ *Hecho cuando:* un comando levanta Gazebo + controladores + RViz + teclado (+ agente). **Aquí el núcleo queda terminado.**

### Fase 6 — Sensores básicos (R7) (≈ 0,5 día)
- Cámara e IMU en `sensors.xacro`, bridge por YAML, panel Image en RViz y `rqt_plot` de la IMU.
- ✅ *Hecho cuando:* `ros2 topic hz` muestra ambos sensores publicando y la imagen se ve en RViz.

### Fase 7 — Calidad y reproducibilidad (≈ 1 día)
- `ament_flake8` / `ament_pep257` / `xmllint`; `colcon test` en verde.
- Comentarios en todos los xacro/yaml/launch.
- Prueba en limpio: descomprimir el zip en otra carpeta (idealmente en un contenedor `ros:jazzy`) y seguir **solo** el README.
- ✅ *Hecho cuando:* la prueba en limpio funciona sin intervención.

### Fase 8 — Documentación y entregables (R12–R14) (≈ 1,5 días)
- `DOCUMENTACION.md` → `tutorial.docx` + `tutorial.pdf` (pandoc).
- Presentación de ~12–15 diapositivas con el formato exigido.
- `make_release.sh` → `cesar_olivares_2026_2.zip`.

**Estimado total del núcleo + entrega: ~9,5 días.** Extensión opcional (§12): +1,5 días.

---

## 8. Contenido del documento maestro (`DOCUMENTACION.md`)

1. Portada (título, estudiante, profesor, fecha) e índice
2. Introducción y objetivos
3. Requisitos de software y hardware + instalación paso a paso
4. Arquitectura del sistema (`rqt_graph`, árbol TF, árbol URDF, flujo de datos §5.3)
5. Tutorial paso a paso. Cada componente se explica con el esquema de 4 preguntas (§0):
   5.1 Workspace y paquetes · 5.2 URDF/XACRO de Octavius (links, joints, límites, inercias, DH) · 5.3 RViz y joint_state_publisher · 5.4 Gazebo y mundo · 5.5 ros2_control: interfaces y controladores · 5.6 Nodos propios y procesamiento de datos · 5.7 micro-ROS (circuito, firmware, agente serial) · 5.8 Launch integrado · 5.9 Sensores y bridge
6. Ejecución y uso (comandos, argumentos, mapa de teclas y joystick)
7. Verificación y resultados (capturas numeradas, tablas de tópicos y frecuencias)
8. Solución de problemas frecuentes (permisos de `/dev/ttyUSB0`, agente no conecta, controladores no cargan, etc.)
9. Conclusiones y trabajo futuro (extensiones de §12)
10. Referencias (IEEE)

---

## 9. Riesgos y mitigaciones

| Riesgo | Mitigación |
|---|---|
| El profesor no tiene la ESP32 | `use_microros:=false`; base y hombro también se pueden mover con `joint_state_publisher_gui` en RViz; video de la demo |
| Permisos del puerto serial (`/dev/ttyUSB0`) | Documentar `sudo usermod -aG dialout $USER`; puerto configurable por argumento |
| `micro_ros_agent` no está en apt para Jazzy | Instrucciones para compilarlo (como en clases) **o** imagen Docker oficial `microros/micro-ros-agent:jazzy`, ambas documentadas |
| El brazo vibra o cae en Gazebo | Inercias correctas, `damping` en juntas, ajuste de la ganancia de posición del plugin |
| Mundo dependiente de internet | Mundo hecho con primitivas, 100 % local |
| Diferencias de versión | Versiones exactas documentadas; dependencias vía `rosdep` |
| Gazebo lento en el PC del profesor | Tasas de sensores moderadas; argumento `headless` |

---

## 10. Checklist de formato (presentación y tutorial)

Membrete · logo universidad/departamento · título · nombre estudiante · nombre profesor · fecha · numeración de páginas · numeración de tablas, figuras y ecuaciones · tipografía uniforme · figuras en buena resolución · índice · introducción · conclusiones · referencias ISO 690 o IEEE.

---

## 11. Decisiones

| Tema | Decisión |
|---|---|
| Nombre | ✅ Octavius (`octavius_description`, `octavius_bringup`) |
| Zip | ✅ `cesar_olivares_2026_2.zip` |
| Articulaciones | ✅ base, hombro, codo, muñeca pitch, muñeca roll, garra |
| Joystick (ESP32, serial) | ✅ X → base, Y → hombro |
| Teclado | ✅ `a` garra, `q/e` giro de garra, y además codo y muñeca pitch (§5.4c) |
| Control | ✅ Interfaz de posición, sin PID propio; `JointGroupPositionController` |
| Motor de física | ✅ Bullet Featherstone: DART (el predeterminado) no soporta la junta `mimic` de la garra en Gazebo Harmonic — verificado en Fase 2 |
| Prioridad | ✅ Núcleo primero (modelo, controladores, integración); sensores básicos después |
| Sensores (requisito) | ✅ Cámara RGB + IMU, versión básica (§6) |
| Sensores avanzados | ⏸️ Postergados como extensión opcional (§12) |
| Presentación | ⏳ Nombre del profesor, plantilla oficial (logo/membrete), ¿PowerPoint (.pptx) como editable? |

---

## 12. Extensión opcional (solo con el núcleo terminado)

### 12.1 Encoders de cuadratura con el método de la clase del profesor

Gazebo no trae un sensor encoder. Se modelaría en dos capas: el plugin `gz-sim-joint-state-publisher-system` entrega el ángulo verdadero del eje, y un nodo `quadrature_encoder` (con la lógica pura en `quadrature.py`, testeable sin ROS) reproduce la clase:

1. Resolución con precisión cuádruple: R = r · mH · s = 45 · 11 · 4 = **1980 cuentas/vuelta** (≈ 0,18°/cuenta).
2. Ángulo → estado A/B (código Gray `00 → 01 → 11 → 10`).
3. Decodificación con la tabla de verdad de la clase: `+1` en 2→0, 0→1, 3→2, 1→3; `−1` en 1→0, 3→1, 0→2, 2→3; los saltos imposibles 0↔3 y 1↔2 se marcan como error `E`. Si entre dos muestras el eje avanza varias cuentas, se recorren los estados intermedios uno por uno, igual que la interrupción en cada flanco.
4. Posición `θ = n · 2π / R`; velocidad `ω = Δn · 2π / (R · Δt)` con Δt = 100 ms.
5. Referencia (homing): `n = 0` en la pose *home* de arranque.

Salidas: `/octavius/encoders` (`sensor_msgs/JointState`) y `/octavius/encoders/counts` (`std_msgs/Int32MultiArray`). Validación: ángulo verdadero vs encoder en `rqt_plot`, con error < 1 cuenta. Además `arm_commander` pasaría a tomar su estado inicial del encoder.

### 12.2 Procesamiento de la IMU

Comparar la orientación medida por la IMU con la orientación de `wrist_link` en la TF (cinemática directa). Si coinciden, valida a la vez el sensor y el modelo. Incluye ruido gaussiano configurado.
