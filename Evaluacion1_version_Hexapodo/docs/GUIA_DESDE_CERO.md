# Guía para aprender el proyecto desde cero (E10)

Esta guía explica **qué se construyó, por qué y cómo encaja todo**, partiendo
de lo básico de ROS 2. Se lee en orden; cada sección se apoya en la anterior.
Para *ejecutar* rápido está [`INICIO_RAPIDO.md`](../INICIO_RAPIDO.md); para el
registro de cada etapa con sus mediciones, [`DOCUMENTACION.md`](../DOCUMENTACION.md).

Índice:

1. Las piezas básicas de ROS 2
2. El workspace y los 3 paquetes
3. Cómo se describe el robot: URDF y xacro
4. Los parámetros (`hexapod_params.xacro`)
5. El árbol de marcos (TF)
6. Simulación: Gazebo y ros2_control
7. El cerebro: cinemática, marcha y nodos
8. Nodos y tópicos: quién habla con quién
9. Sensores
10. micro-ROS y el joystick
11. Los launch: qué arranca cada uno
12. Pruebas y evidencias
13. Cómo cambiar cosas (recetas)
14. Glosario

---

## 1. Las piezas básicas de ROS 2

ROS 2 no es un programa: es una forma de dividir un robot en **programas
pequeños que se comunican**.

| Concepto | Qué es | Ejemplo en este proyecto |
|---|---|---|
| **Nodo** | Un programa que hace una cosa | `gait_node` calcula los ángulos de las patas |
| **Tópico** | Un "canal" con nombre por donde viajan mensajes. Quien publica no sabe quién escucha | `/hexapod/cmd_vel` |
| **Mensaje** | El tipo de dato que viaja por un tópico | `geometry_msgs/Twist` = velocidad lineal y angular |
| **Publicar / suscribirse** | Enviar a un tópico / recibir de él | `joy_to_twist` publica en `/hexapod/cmd_vel`; `gait_node` se suscribe |
| **Parámetro** | Un valor de configuración de un nodo, se carga desde un YAML | `stand_height: 0.090` en `gait.yaml` |
| **Launch** | Un archivo Python que arranca varios nodos juntos con sus parámetros | `bringup.launch.py` |
| **Paquete** | Carpeta con código, configuración y un `package.xml` que dice qué necesita | `hexapod_control` |
| **TF** | El sistema que sabe dónde está cada pieza del robot respecto de las otras | dónde está el pie izquierdo delantero respecto del cuerpo |

Lo importante de esta forma de trabajar: **cada pieza se puede cambiar sin
tocar las otras**. El teclado y el joystick publican en el mismo tópico
(`/hexapod/cmd_vel`); a `gait_node` le da lo mismo quién le habla. En el
robot real, el mismo `gait_node` mandaría ángulos a los servos reales.

Comandos para mirar un sistema corriendo:

```bash
ros2 node list                 # qué nodos hay
ros2 topic list -t             # qué tópicos hay y su tipo
ros2 topic echo /hexapod/imu   # ver los mensajes de un tópico
ros2 topic hz /hexapod/imu     # a qué frecuencia llegan
rqt_graph                      # dibujo de nodos y tópicos
```

---

## 2. El workspace y los 3 paquetes

Un **workspace** es una carpeta con `src/` adentro. `colcon build` compila
todo lo de `src/` y deja el resultado en `install/`. `source
install/setup.bash` le dice a la terminal dónde están los paquetes.

```
Evaluacion1_version_Hexapodo/
├── src/
│   ├── hexapod_description/   EL CUERPO: cómo es el robot (URDF/xacro)
│   ├── hexapod_control/       EL CEREBRO: cinemática, marcha, teleoperación
│   └── hexapod_bringup/       EL ARRANQUE: Gazebo, controladores, configuración
├── firmware/                  programa de la ESP32 (joystick)
├── scripts/                   pruebas que miden las evidencias
├── analysis/                  gráficos y estudio de dimensiones
├── docs/                      guías e imágenes
├── instalar.sh                instala dependencias y compila (un comando)
└── DOCUMENTACION.md           registro de cada etapa
```

**Por qué 3 paquetes y no uno:** cada uno cambia por motivos distintos.

| Paquete | Cambia cuando… | Tipo | Depende de |
|---|---|---|---|
| `hexapod_description` | cambian las medidas o piezas del robot | `ament_cmake` (solo instala archivos, no hay código que compilar) | nada propio |
| `hexapod_control` | cambia cómo se mueve | `ament_python` | `hexapod_description` (solo para los tests) |
| `hexapod_bringup` | cambia cómo se simula o se arranca | `ament_python` | los otros dos |

Así, el mismo `hexapod_description` sirve para RViz, Gazebo y el robot real;
y la cinemática de `hexapod_control` no depende de Gazebo (se prueba sin
abrir el simulador).

**Archivos de cada paquete:**

- `package.xml`: nombre, versión y **dependencias**. Es lo que lee `rosdep`
  para instalar lo que falta. Ej.: `<exec_depend>ros_gz_sim</exec_depend>`.
- `setup.py` (paquetes Python): qué archivos propios se instalan (launch,
  config) y qué **nodos** se pueden ejecutar con `ros2 run`
  (`console_scripts`). *No instala dependencias del sistema.*
- `CMakeLists.txt` (paquete cmake): qué carpetas se instalan.

**Instalar en un comando** (`./instalar.sh`): instala `rosdep` y `colcon`,
corre `rosdep install --from-paths src --ignore-src -y` (lee todos los
`package.xml` e instala con apt lo que falte: Gazebo, ros2_control, numpy…)
y compila. Por eso es clave que cada `package.xml` declare todo: si falta
una dependencia, el proyecto funciona en tu PC pero no en el del profesor.

---

## 3. Cómo se describe el robot: URDF y xacro

**URDF** (*Unified Robot Description Format*) es un XML con dos elementos:

- **link**: una pieza rígida. Tiene `visual` (cómo se ve), `collision` (con
  qué forma choca, puede ser más simple) e `inertial` (masa e inercia, para
  la física).
- **joint**: une dos links. Tipo `revolute` (gira, con límites), `fixed`
  (unión rígida). Tiene `origin` (dónde está el eje respecto del link padre)
  y `axis` (en torno a qué eje gira).

El robot es un **árbol**: cada link tiene un solo padre. La raíz es
`base_link` (el centro del cuerpo).

**xacro** es URDF con superpoderes: variables (`property`), cuentas
(`${coxa_length/2}`), **macros** (plantillas reutilizables) e `include`
(dividir en archivos). Al lanzar, el comando `xacro` lo convierte en URDF
normal. Sin xacro, el hexápodo serían ~1500 líneas de XML repetido 6 veces.

### Los archivos y su rol

```
hexapod.urdf.xacro          ARMA EL ROBOT: incluye todo e instancia 6 patas
 ├── hexapod_params.xacro   todas las medidas y masas (lo único que se edita)
 ├── materials.xacro        colores
 ├── macros/inertial.xacro  fórmulas de inercia (caja, cilindro)
 ├── body.xacro             cuerpo hexagonal + servos de las coxas
 ├── components.xacro       batería, Raspberry, 2 PCA9685
 ├── sensors.xacro          marcos de la IMU y la cámara
 ├── leg.xacro              macro de UNA pata (coxa, fémur, tibia, pie)
 ├── ros2_control.xacro     interfaces de control de las 18 articulaciones
 ├── gazebo.xacro           fricción de los pies y odometría de referencia
 └── sensors_gazebo.xacro   plugins que hacen medir a la cámara y la IMU
leg_test.urdf.xacro         banco de prueba: una pata sola (E1)
```

### La macro de la pata (`leg.xacro`)

Se escribe una vez y se usa 6 veces cambiando solo `prefix` (nombre) y
`mount_angle` (en qué vértice va):

```xml
<xacro:hexapod_leg prefix="lf" parent="base_link" mount_angle="${30*PI/180}" radius="${body_radius}"/>
```

Cada pata crea 4 links y 4 joints:

```
base_link ─(lf_coxa_joint,  gira en z)──► lf_coxa_link    lleva el servo del fémur
          ─(lf_femur_joint, gira en −y)─► lf_femur_link   lleva el servo de la rodilla
          ─(lf_tibia_joint, gira en −y)─► lf_tibia_link
          ─(lf_foot_joint,  fija)───────► lf_foot_link    esfera de contacto
```

- El origen de `coxa_joint` está en el vértice: (R cos φ, R sin φ, 0) y
  girado φ. Así, **en el marco de cada pata, x siempre apunta hacia afuera**:
  la misma cinemática sirve para las 6 patas.
- Fémur y tibia giran en torno a **−y** para que un ángulo positivo **suba**
  el segmento (más intuitivo).
- Cada link pone su masa donde está realmente: el servo del fémur en la
  punta de la coxa, el de la rodilla en la punta del fémur. El centro de
  masa es el promedio ponderado.

### El cuerpo (`body.xacro`)

URDF no tiene hexágonos. Se arma con **3 rectángulos** de 2a × s (a =
apotema, s = lado) girados 0°, 60° y 120°: sus esquinas son los vértices,
así que la unión es el hexágono exacto. `base_link` no tiene masa porque la
librería KDL (que usa `robot_state_publisher`) no acepta masa en la raíz; la
masa va en `body_link`, unido con una junta fija.

---

## 4. Los parámetros (`hexapod_params.xacro`)

Es **la única fuente de verdad**: el URDF, la cinemática y los tests leen de
aquí. Cada valor dice de dónde sale: [medido], [hoja], [hipótesis] o
[provisorio] (lo reemplaza el estudio de dimensiones, E8).

| Grupo | Variable | Valor | Origen | Para qué |
|---|---|---|---|---|
| Servo | `servo_mass` | 0,055 kg | medido (con adaptadores) | masa de cada MG996R |
| | `servo_x/y/z` | 40,7 × 19,7 × 42,9 mm | hoja | tamaño para dibujar y para la inercia |
| | `servo_effort` | 1,08 N·m | hoja (6 V) | torque máximo: Gazebo no deja hacer más |
| | `servo_velocity` | 7,48 rad/s | hoja (0,14 s/60°) | velocidad máxima |
| | `servo_shaft_offset` | 10 mm | hoja aprox. | dónde está el eje dentro del servo |
| Pata | `coxa_length` L1 | 45 mm | provisorio | eje coxa → eje fémur |
| | `femur_length` L2 | 80 mm | provisorio | eje fémur → eje rodilla |
| | `tibia_length` L3 | 120 mm | provisorio | eje rodilla → punta del pie |
| | `foot_radius` | 10 mm | provisorio | esfera de contacto |
| | `*_bracket_mass`, `tibia_mass` | 8, 10, 15 g | hipótesis | piezas que vas a fabricar |
| Límites | `coxa_lower/upper` | ±45° | provisorio | para que patas vecinas no choquen |
| | `femur_lower/upper` | ±90° | provisorio | |
| | `tibia_lower/upper` | −2,5…0,6 rad | provisorio | cuánto se dobla la rodilla |
| Cuerpo | `body_radius` R | 100 mm | provisorio | centro → eje de cada coxa |
| | `body_plate_margin` | 15 mm | provisorio | cuánto sobresale la placa del eje |
| | `body_plate_gap` | 45 mm | del servo | espacio entre placas (cabe el servo de 42,9) |
| | `body_structure_mass` | 200 g | hipótesis | placas |
| Componentes | `battery_*` | 271 g; 137 × 43 × 22 mm | medido | |
| | `raspberry_*` | 86 g; 85 × 56 × 17 mm | medido / hoja | |
| | `pca_*` | 9 g; 62 × 25 × 15 mm | medido / hoja | |
| | `wiring_mass` | 100 g | hipótesis | cables y reguladores |
| Sensores | `imu_*`, `camera_*` | 3 g y 4 g | hipótesis / hoja | BNO085 y Pi Camera 3 |
| | `camera_pitch` | 0,17 rad (10°) | hipótesis | inclinación hacia el suelo |

Masa total resultante: **1,876 kg** (un test lo comprueba).

Otros parámetros, fuera del URDF, en `hexapod_bringup/config/`:

| Archivo | Qué configura |
|---|---|
| `gait.yaml` | postura de pie (pie a 70 mm del fémur, 90 mm bajo la coxa), postura recogida, tiempos, marcha (T = 1 s, β = 0,6, h = 30 mm, paso máx. 60 mm, rampas) |
| `joystick.yaml` | zona muerta, expo, velocidades máximas, sentido de los ejes |
| `controllers.yaml` | controladores de ros2_control y la ganancia del servo simulado |
| `gz_bridge.yaml` | qué tópicos de Gazebo pasan a ROS |

---

## 5. El árbol de marcos (TF)

Cada link tiene su **marco** (sistema de coordenadas). TF sabe en todo
momento la posición de cada marco respecto de los otros.

```
odom                          mundo (lo publica Gazebo: dónde está el robot)
 └── base_link                centro del cuerpo: x adelante, y izquierda, z arriba (REP-103)
      ├── body_link, battery_link, raspberry_link, pca_1_link, pca_2_link
      ├── imu_link
      ├── camera_link ── camera_optical_link   (z adelante, x derecha, y abajo)
      └── (×6) {pata}_coxa_servo_link
               {pata}_coxa_link ── femur_link ── tibia_link ── foot_link
```

- `robot_state_publisher` lee el URDF y los ángulos de `/joint_states`, y
  publica todos los marcos del robot en `/tf`.
- `odom → base_link` lo publica Gazebo (odometría de referencia), por eso en
  RViz el robot se ve avanzar.
- El marco óptico existe porque las imágenes usan otra convención.

Probar: `ros2 run tf2_ros tf2_echo base_link lf_foot_link`.

---

## 6. Simulación: Gazebo y ros2_control

- **Gazebo Harmonic** simula la física: gravedad, contactos, fricción,
  inercia. Lee el URDF convertido a SDF.
- **ros2_control** es el estándar para mover articulaciones. Separa:
  - el **hardware** (`ros2_control.xacro`): qué articulaciones hay y qué
    interfaces tienen: *command* `position` (lo que se ordena) y *state*
    `position`, `velocity`, `effort` (lo que se mide);
  - los **controladores** (`controllers.yaml`):
    - `joint_state_broadcaster`: publica `/joint_states`;
    - `legs_controller` (`JointGroupPositionController`): recibe 18 ángulos
      en `/legs_controller/commands` y se los pasa al hardware.
- En simulación el hardware es el plugin `gz_ros2_control`. En el robot real
  se cambiaría por un driver de las PCA9685 y **todo lo demás quedaría igual**.

**Base flotante:** el cuerpo no está fijo al mundo (a diferencia de un brazo
atornillado a la mesa). Se sostiene solo si los pies tocan el suelo con
fricción y los servos aguantan el peso. Por eso:

- los pies son esferas con fricción μ = 1 (`gazebo.xacro`);
- el robot aparece apoyado en la batería y se para con una secuencia suave
  (recoger → bajar los pies). Pedir la postura de golpe lo hace trabarse.

---

## 7. El cerebro: cinemática, marcha y nodos

Todo en `hexapod_control/`. La lógica está **separada de ROS** (archivos
sin `rclpy`) para poder probarla con tests rápidos; los nodos solo conectan
esa lógica con los tópicos.

| Archivo | ¿Usa ROS? | Qué hace |
|---|---|---|
| `kinematics.py` | no | Cinemática directa (ángulos → pie) e inversa (pie → ángulos) de una pata |
| `robot_model.py` | no | Lee del URDF dónde va cada pata; pasa puntos entre el marco del cuerpo y el de la pata |
| `gait.py` | no | Marcha trípode: dónde debe estar cada pie en cada instante |
| `gait_node.py` | sí | Nodo: postura suave, luego marcha. Junta todo lo anterior |
| `keyboard_teleop.py` | sí | Nodo: teclado → velocidad |
| `joy_to_twist.py` | sí | Nodo: joystick → velocidad |

### Cinemática de una pata

Directa (FK), con θ1 coxa, θ2 fémur, θ3 tibia:

```
r = L1 + L2 cos θ2 + L3 cos(θ2 + θ3)          (distancia horizontal desde la coxa)
x = r cos θ1     y = r sin θ1     z = L2 sin θ2 + L3 sin(θ2 + θ3)
```

Inversa (IK), por ley de cosenos, con la rodilla hacia arriba:

```
θ1 = atan2(y, x)
ρ = √(x² + y²) − L1         D² = ρ² + z²
cos θ3 = (D² − L2² − L3²) / (2 L2 L3)      θ3 = −arccos(…)
θ2 = atan2(z, ρ) − atan2(L3 sin θ3, L2 + L3 cos θ3)
```

Si el punto no se alcanza o un ángulo sale de su límite, `ik` lanza un
error en vez de mandar un ángulo imposible (esto detectó un error real en
E3).

### Marcha trípode (`gait.py`)

- Grupo A (LF, RM, LR) y grupo B (RF, LM, RR) se alternan; siempre hay un
  triángulo apoyado.
- Cada pata tiene una fase φ ∈ [0, 1). En **apoyo** (φ < β) el pie retrocede
  respecto del cuerpo (eso empuja al robot); en **vuelo** vuelve adelante
  por un arco de altura h.
- Vector de paso: Δ = βT (v + ω ẑ × p0). Incluye el giro: cada pata da un
  paso distinto según dónde está.
- Se calcula paso a paso (incremental): si cambia la velocidad a mitad de
  paso, el pie no salta. Con velocidad constante da exactamente las
  fórmulas cerradas del plan (un test lo comprueba).
- **Rampa** de aceleración: la velocidad cambia como máximo 0,4 m/s².
- **β = 0,6** (no 0,5): las 6 patas apoyan un instante en cada cambio de
  grupo; con 0,5 el cuerpo caía 5 mm en cada cambio.

### Velocidad máxima: por qué 10 cm/s

No es "la más estable": es el **tope geométrico**. Un pie apoyado recorre
Δ = βT·v por paso, y no puede pasar de `max_step` = 60 mm sin salir del
alcance de la pata. Entonces v_max = 0,06 / (0,6 × 1 s) = 0,10 m/s. Para ir
más rápido: pasos más rápidos (T menor) o patas más largas (E8).

### `gait_node`, paso a paso

1. Espera el reloj de la simulación y el primer `/joint_states` (punto de partida).
2. **Recoger** (1 s): interpola cada articulación hasta la postura recogida.
3. **Parar** (2,5 s): baja los pies en línea recta (IK en cada paso). El cuerpo sube de 47,5 a 100 mm.
4. **Marcha** (50 Hz): lee `/hexapod/cmd_vel`, avanza `gait.py`, hace la IK de las 6 patas y publica 18 ángulos.
5. Si no llegan comandos por 0,5 s, toma velocidad 0 y se detiene.

---

## 8. Nodos y tópicos: quién habla con quién

```
 ┌───────────────┐  /hexapod/joystick   ┌──────────────┐
 │ ESP32 (micro- │ ─── sensor_msgs/Joy ─►│ joy_to_twist │──┐
 │ ROS) vía      │      20 Hz            └──────────────┘  │  /hexapod/cmd_vel
 │ micro_ros_agent│                                        ├─ geometry_msgs/Twist
 └───────────────┘                      ┌───────────────┐  │
                                        │keyboard_teleop│──┘
                                        └───────────────┘
                                                │
                                                ▼
 /joint_states ──────────────────────────► ┌──────────┐  /legs_controller/commands
 (solo para partir)                        │ gait_node│ ── Float64MultiArray (18) ──┐
                                           └──────────┘        50 Hz                │
                                                                                    ▼
 ┌───────────────────────────────── Gazebo ─────────────────────────────────────────────┐
 │ controller_manager (gz_ros2_control): legs_controller → 18 juntas simuladas          │
 │                                      joint_state_broadcaster → /joint_states         │
 │ sensores: cámara, IMU          odometría de referencia                               │
 └──────────────────────────────────────────────────────────────────────────────────────┘
        │ ros_gz_bridge: /clock, /hexapod/imu, /hexapod/camera/image, /camera_info,
        │                /hexapod/odom, /tf (odom → base_link)
        ▼
 robot_state_publisher: URDF + /joint_states → /tf (marcos del robot)
 RViz: modelo, TF, imagen de la cámara
```

| Tópico | Tipo | Publica | Escucha | Frecuencia |
|---|---|---|---|---|
| `/hexapod/joystick` | `sensor_msgs/Joy` | ESP32 (o `ros2 topic pub`) | `joy_to_twist` | 20 Hz |
| `/hexapod/cmd_vel` | `geometry_msgs/Twist` | `joy_to_twist`, `keyboard_teleop` | `gait_node` | 10–20 Hz |
| `/legs_controller/commands` | `std_msgs/Float64MultiArray` | `gait_node` | `legs_controller` | 50 Hz |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` | `robot_state_publisher`, `gait_node` | 100 Hz |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | `robot_state_publisher`, bridge | RViz, cualquiera | — |
| `/hexapod/imu` | `sensor_msgs/Imu` | Gazebo (bridge) | (scripts) | 100 Hz |
| `/hexapod/camera/image` | `sensor_msgs/Image` | Gazebo (bridge) | RViz | 15 Hz |
| `/hexapod/camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo (bridge) | — | 15 Hz |
| `/hexapod/odom` | `nav_msgs/Odometry` | Gazebo (bridge) | scripts de medición | 50 Hz |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo (bridge) | todos (`use_sim_time`) | — |
| `/robot_description` | `std_msgs/String` | `robot_state_publisher` | Gazebo (para crear el robot), RViz | una vez |

**Tiempo simulado:** todos los nodos usan `use_sim_time: true`: toman la
hora de `/clock` (de Gazebo) y no la del PC. Si el PC es lento, la
simulación va más lenta pero todo sigue sincronizado.

---

## 9. Sensores

| | IMU (BNO085) | Cámara (Pi Camera Module 3) |
|---|---|---|
| Dónde | centro del cuerpo | lado frontal, 10° hacia el suelo |
| Mensaje | `sensor_msgs/Imu`: orientación, velocidad angular, aceleración | `sensor_msgs/Image` + `CameraInfo` |
| Frecuencia | 100 Hz | 15 Hz, 640 × 480 |
| Modelo | ruido gaussiano (giroscopio 0,005 rad/s, acelerómetro 0,03 m/s²) | estenopeica, HFOV 66° → fx ≈ 493 px |
| Archivos | `sensors.xacro` (dónde) + `sensors_gazebo.xacro` (qué mide) | ídem |

Cada sensor tiene dos partes: un **link** (dónde está) y un bloque
`<gazebo reference>` (qué mide y en qué tópico de Gazebo publica). El
bridge lo pasa a ROS.

---

## 10. micro-ROS y el joystick

```
joystick ─analógico─► ESP32 ─USB serial (XRCE-DDS)─► micro_ros_agent ─DDS─► /hexapod/joystick
```

- La ESP32 es muy chica para DDS; micro-ROS usa un protocolo liviano y el
  **agente** del PC traduce.
- La placa solo lee y normaliza los ejes; **las decisiones están en ROS**
  (`joy_to_twist`): zona muerta, curva expo, velocidades. Cambiar el
  comportamiento no requiere reprogramar la placa.
- **Control suave**: la palanca entrega valores continuos. Zona muerta con
  reescalado → curva expo (fina al centro, completa al tope) → velocidad
  proporcional → rampa de la marcha. Ver el gráfico en `DOCUMENTACION.md` (E6).
- El botón cambia el eje X entre girar y caminar de lado.
- Detalle de conexión, carga y prueba: [`joystick_esp32.md`](joystick_esp32.md).

---

## 11. Los launch: qué arranca cada uno

| Comando | Arranca | Para qué |
|---|---|---|
| `ros2 launch hexapod_description display.launch.py` | `robot_state_publisher`, sliders, RViz | ver el modelo y mover las 18 articulaciones a mano |
| `… display.launch.py model:=leg_test` | ídem con una pata | estudiar una pata sola |
| `ros2 launch hexapod_bringup gazebo.launch.py` | Gazebo, robot, controladores (en orden), `gait_node`, bridge, RViz | simulación sin joystick |
| `ros2 launch hexapod_bringup bringup.launch.py` | todo lo anterior + `joy_to_twist` (+ agente con `use_microros:=true`) | **el comando principal** |
| `ros2 run hexapod_control keyboard_teleop` | el teclado (en otra terminal) | manejar con w/a/s/d/q/e |

`gazebo.launch.py` usa **eventos** (`OnProcessExit`) para el orden: el robot
tiene que existir antes de activar `joint_state_broadcaster`, y este antes
que `legs_controller`, y este antes que `gait_node`.

---

## 12. Pruebas y evidencias

**Tests automáticos** (`colcon test`, 74 hoy): se corren sin abrir Gazebo.

| Archivo | Qué comprueba |
|---|---|
| `test_kinematics.py` | FK/IK ida y vuelta en 5000 puntos (< 0,1 mm), casos a mano, límites |
| `test_kinematics_urdf.py` | la pata del URDF coincide con la FK |
| `test_hexapod_model.py` | 18 articulaciones, patas en sus vértices, masa 1,876 kg, centro de masa centrado |
| `test_robot_model.py` | cambios de marco, posturas alcanzables, marcha a velocidad máxima dentro de los límites |
| `test_gait.py` | 3 patas apoyadas, pies en el suelo, igual a las fórmulas, sin saltos, vuelve a casa |
| `test_joy_to_twist.py` | zona muerta, expo, gradiente continuo, mapeo |
| `test_config_consistency.py` | nombres y órdenes iguales entre URDF y YAML, topes del teclado y joystick |
| `test_flake8`, `test_pep257`, `test_copyright` | estilo del código |

**Scripts de evidencia** (`scripts/`, con la simulación corriendo): miden en
Gazebo y dicen CUMPLE / NO CUMPLE.

| Script | Evidencia |
|---|---|
| `medir_postura.py` | D8: de pie 60 s a altura estable |
| `medir_marcha.py` | D11: velocidad real vs pedida en 5 direcciones |
| `medir_torque_tripode.py` | torque estático en 3 patas (para E8) |
| `verificar_sensores.py` | D12–D13: frecuencias, focal, gravedad, giro de la IMU |
| `probar_joystick.py` | D15: joystick simulado y gráfico del gradiente |

---

## 13. Cómo cambiar cosas (recetas)

| Quiero… | Cambio | Después |
|---|---|---|
| Patas más largas | `coxa/femur/tibia_length` en `hexapod_params.xacro` | `colcon build` y `colcon test` (los tests de alcance avisan si la marcha ya no cabe) |
| Cuerpo más grande | `body_radius` | ídem |
| Robot más alto o más bajo | `stand_height` en `gait.yaml` | relanzar |
| Caminar más rápido | bajar `period` en `gait.yaml` (y subir `v_max` en `joystick.yaml` y `V_MAX` del teclado; un test exige que calcen) | `colcon test` |
| Joystick más sensible al centro | bajar `expo` en `joystick.yaml` | relanzar |
| Invertir un eje del joystick | `axis_x_sign` / `axis_y_sign` = −1 | relanzar |
| Ver una pata sola | `display.launch.py model:=leg_test` | — |

Con `--symlink-install` (lo usa `instalar.sh`) los YAML y los .py se leen
directo desde `src/`: basta relanzar. Si se agregan archivos nuevos, hay que
volver a compilar.

---

## 14. Glosario

| Término | Significado |
|---|---|
| Coxa, fémur, tibia | Los 3 segmentos de la pata, desde el cuerpo hacia el pie |
| L1, L2, L3 | Largos de coxa, fémur y tibia |
| FK / IK | Cinemática directa (ángulos → posición) / inversa (posición → ángulos) |
| Marcha trípode | Dos grupos de 3 patas que se alternan |
| β (duty) | Fracción del ciclo que una pata pasa apoyada |
| T (period) | Duración de un ciclo de marcha |
| Δ | Vector de paso: lo que recorre el pie en un apoyo |
| Base flotante | El cuerpo no está unido al mundo; se sostiene con los pies |
| REP-103 | Convención de ROS: x adelante, y izquierda, z arriba; metros y radianes |
| TF | Sistema de marcos de coordenadas de ROS |
| URDF / xacro | Descripción del robot en XML / con variables y macros |
| ros2_control | Estándar de ROS para controlar articulaciones (hardware + controladores) |
| Bridge | Traductor de tópicos entre Gazebo y ROS |
| micro-ROS / agente | ROS para microcontroladores / programa del PC que lo conecta |
| Zona muerta | Rango cerca del centro de la palanca que se toma como 0 |
| Expo | Curva que hace la palanca más fina cerca del centro |
| rosdep | Herramienta que instala las dependencias declaradas en los `package.xml` |
| colcon | Herramienta que compila el workspace |
| `use_sim_time` | Usar el reloj de Gazebo en vez del del PC |
