# Plan de trabajo — Hexápodo simulado con ROS 2 y estudio de dimensiones

| | |
|---|---|
| Asignatura | Open Robotics — 2° semestre 2026 |
| Estudiante | Cesar Olivares Lavalovich |
| Profesor | José Pascal |
| Ubicación | `~/Open_robotics/Evaluacion1_version_Hexapodo/` |
| Relación con la entrega oficial | **Proyecto paralelo.** La entrega oficial de la Evaluación 1 sigue siendo el brazo Octavius (`Evaluacion_1/`, que no se modifica) |

> Documento para revisión **antes** de programar. Cuando esté aprobado, se ejecuta fase por fase.

---

## 0. Alcance y principios

### 0.1 La pauta manda

El alcance es **la pauta de la evaluación, completa y solo simulada**:

- Espacio de trabajo ROS 2.
- Gazebo y RViz.
- URDF/XACRO del robot.
- Sensores.
- Movimiento del robot simulado (`ros2_control` + teclado).
- micro-ROS.
- Launch.
- Presentación y tutorial.

Ninguna decisión de este plan puede dejar un punto de la pauta sin cumplir (ver la trazabilidad en §2).

### 0.2 El aporte propio: estudio de dimensiones

Además de cumplir la pauta, el proyecto usa la simulación para **calcular las dimensiones y el layout ideales del hexápodo** a partir de los componentes que ya existen:

- 18 × MG996R.
- Raspberry Pi 4.
- 2 × PCA9685.
- LiPo 3S 5200 mAh.
- IMU y cámara (por comprar).

El resultado es una hoja de especificación con largos de coxa, fémur y tibia, radio del cuerpo y ubicación de cada componente. Con esa hoja después se diseñan y maquinan las piezas reales.

### 0.3 Fuera de alcance

La construcción y puesta en marcha del robot real quedan fuera de este proyecto. Se dejan solo como recomendaciones en el anexo A.

### 0.4 Principios

1. **Nada de cajas negras.** Cada componente se documenta con:
   - qué es;
   - por qué se usa;
   - qué datos entran y salen;
   - cómo se procesan.
2. **Datos reales donde existan.** Masas, medidas, torque y velocidad de los componentes salen de la hoja de datos o de una medición, con la fuente indicada. Lo que se calcula (los largos de las patas) se marca como resultado del estudio.
3. **Un solo archivo de parámetros** (`hexapod_params.xacro`) alimenta el URDF, la cinemática y la marcha. Cambiar una medida es cambiar una línea.
4. **Primero lo que puede hacer fracasar el proyecto.** En un hexápodo el riesgo principal es la base flotante: que se sostenga de pie en Gazebo. Por eso ese hito va antes que la marcha y los extras.

---

## 1. Componentes del robot

### 1.1 Lo que ya existe

| Componente | Dato usado en el modelo | Fuente |
|---|---|---|
| 18 × **MG996R** (modificados con eje coaxial) | Masa 55 g; cuerpo 40,7 × 19,7 × 42,9 mm. Torque de bloqueo 9,4 kgf·cm a 4,8 V y 11 kgf·cm a 6 V. Velocidad 0,17 s/60° a 4,8 V y 0,14 s/60° a 6 V | Hoja de datos (Tower Pro) |
| **Raspberry Pi 4** | 85 × 56 × ~17 mm (masa en la fila de masas medidas) | Hoja de datos (Raspberry Pi Ltd.) |
| **2 × PCA9685** (16 canales c/u: 9 servos por lado) | Placa de ~62 × 25 mm, ~9 g c/u | Hoja de datos (Adafruit) |
| **LiPo 3S 5200 mAh 60C** | **271 g**; medidas L × A × H ❓ | Pesada por el usuario |
| Masas medidas | MG996R modificado **55 g** (¿incluye adaptadores? ❓) · Raspberry Pi 4 **86 g** · PCA9685 **9 g** | Pesadas por el usuario |
| Adaptadores coaxiales MG996R | A: 49 × 69 × 20 mm · B: 40 × 49 × 20 mm (*supuesto: son los adaptadores reales del eje coaxial*) | `MG996R_adapter-A/B.STEP` (repo `Hexodo_test`) |

> Las piezas estructurales del repositorio (`base_hexapodo`, `BICEP`, `PATA_1`, `Tapa superior`) **no se usan**: se diseñaron a ojo. Las cotas de la estructura salen del estudio de dimensiones (§4).

### 1.2 Sensores recomendados (por comprar; en la simulación se modelan con sus datos de hoja)

| Sensor | Modelo recomendado | Por qué | Alternativa |
|---|---|---|---|
| **IMU** (centro del cuerpo) | **BNO085** (breakout Adafruit 4754 o SparkFun) | Hace la fusión de sensores en el chip y entrega la orientación (cuaternión) lista, que es lo que necesita un hexápodo para saber su inclinación | **LSM6DSOX** 6 ejes (Adafruit 4438), con la fusión hecha en ROS (`imu_filter_madgwick`). Es más simple de conectar por I2C |
| **Cámara** (cara frontal) | **Raspberry Pi Camera Module 3**, versión estándar (sensor IMX708, 12 MP, campo de visión horizontal de 66°) | Es oficial, pequeña (25 × 24 mm, ~4 g) y va por CSI sin ocupar un USB | Una cámara USB UVC (p. ej. Logitech C270): es *plug and play* en Ubuntu, pero más grande y pesada |

Notas para cuando se compren (no afectan la simulación):

- **BNO085 en la Raspberry:** el BNO085 tiene problemas conocidos con el I2C de la Raspberry (*clock stretching*). Se conecta por SPI o UART.
- **Camera Module 3 con Ubuntu 24.04 + Jazzy:** suele requerir compilar el fork de `libcamera` de Raspberry Pi para usarla con `camera_ros`.
- **Magnetómetro:** no se usa. Cerca de 18 motores no es confiable.

### 1.3 Cómo aparecen en la simulación

| Componente | En el URDF | En Gazebo |
|---|---|---|
| Raspberry Pi 4 | `raspberry_link`: caja con sus medidas y masa, sobre el cuerpo. Representa al computador que en el robot real corre el control | Masa e inercia reales (influyen en el centro de masa) |
| 2 × PCA9685, batería | Cajas con sus medidas y masa. La batería va bajo el cuerpo para bajar el centro de masa | Ídem |
| IMU | `imu_link` en el centro geométrico del cuerpo | Plugin IMU de Gazebo, con el ruido de la hoja de datos |
| Cámara | `camera_link` + `camera_optical_link` (REP-103) en la cara frontal | Plugin de cámara de Gazebo con el FOV y la resolución de la Camera Module 3 (escalada para no cargar el PC) |

---

## 2. Trazabilidad de la pauta

| # | Requerimiento de la pauta | Cómo se cumple | Verificación |
|---|---|---|---|
| R1 | Paquete con sus directorios | Workspace con `hexapod_description`, `hexapod_control`, `hexapod_bringup` (§5) | `colcon build` sin errores |
| R2 | Robot articulado en URDF/XACRO | XACRO modular: cuerpo hexagonal + una **macro de pata** instanciada 6 veces → 18 articulaciones | `check_urdf`, árbol con `urdf_to_graphviz` |
| R3 | Modelo en RViz | `display.launch.py` | Captura |
| R4 | `joint_state_publisher` | `joint_state_publisher_gui` con **18 sliders** | Sliders mueven las patas |
| R5 | Percepción en RViz: RobotModel y TF | `.rviz` con RobotModel, TF, imagen de la cámara e IMU | Captura |
| R6 | Modelo en Gazebo | Base flotante, de pie sobre el suelo | Se sostiene ≥ 60 s sin hundirse ni deslizarse |
| R7 | ≥ 2 plugins de sensores | **IMU** + **cámara** frontal | `ros2 topic hz` de ambos |
| R8 | `ros2_control` + teclado | Teclado → velocidad del cuerpo → marcha trípode + cinemática inversa → 18 ángulos → `legs_controller` | Camina y gira con las teclas |
| R9 | Conectividad y control con micro-ROS | **ESP32 + joystick → micro-ROS → mueve al hexápodo en la simulación** (como Octavius). La ESP32 aún no se compra: paquete, tópico y firmware quedan listos y se prueba publicando el tópico a mano (§3.6) | Con la ESP32: el joystick hace caminar al robot |
| R10 | Todo con un launchfile | `bringup.launch.py` con argumentos | Un comando |
| R11 | Estructura de carpetas | Estándar ROS 2 (§5) | Revisión |
| R12 | Presentación PDF + editable, ≤ 15 min | Plantilla Beamer USACH de Octavius | Checklist §8 |
| R13 | Tutorial PDF + editable | `DOCUMENTACION.md` → `.docx` + `.pdf` | Checklist §8 |
| R14 | Entrega `.zip` | `make_release.sh` adaptado | Descomprimir y compilar en limpio |

> ⚠️ **R9 depende de comprar la ESP32 antes de la presentación.** Sin ella, el código queda completo y se demuestra con el tópico simulado, pero la pauta pide la conectividad real con micro-ROS. La misma ESP32 con su joystick sirve también para cerrar la Fase 4 de Octavius.

---

## 3. Diseño del robot

### 3.1 Cuerpo: hexágono regular horizontal

- El cuerpo es un **hexágono regular** (todos los lados y ángulos iguales, sin alargarlo) y su placa queda **horizontal**.
- Las 6 patas salen de los **6 vértices**.
- Para que haya 3 patas por lado, simétricas, el robot mira hacia una **arista**. Así, las patas quedan a ±30° (delanteras), ±90° (medias) y ±150° (traseras) del eje de avance.
- La cámara va centrada en la arista frontal.

```
              frente (cámara)
             ______________
       LF   /              \   RF        Patas en los vértices, a 60° entre sí
           /                \            Radio del hexágono R = distancia
     LM   <     (IMU)        >   RM        del centro a cada eje de coxa
           \                /            (lo calcula el estudio, §4)
       LR   \______________/   RR
```

### 3.2 Árbol cinemático

```
base_footprint
└── base_link (cuerpo hexagonal, base flotante)
    ├── imu_link, camera_link → camera_optical_link
    ├── raspberry_link, pca9685_left_link, pca9685_right_link, battery_link   (fijos, con masa)
    └── ×6 patas {lf, lm, lr, rf, rm, rr}
        └── {leg}_coxa_joint (Z) → coxa_link
            └── {leg}_femur_joint (horizontal) → femur_link
                └── {leg}_tibia_joint (horizontal) → tibia_link
                    └── {leg}_foot_link (esfera de contacto)
```

### 3.3 Decisiones de modelado

- **Una macro xacro de pata** con parámetros (nombre, ángulo de montaje y radio). Las 6 patas son la misma macro.
- **Servos como cajas con la masa y las medidas del MG996R** en su posición real dentro de cada pata. Así, la distribución de masas de la simulación es la del robot real.
- **Pie esférico** con fricción alta para un contacto estable.
- **Inercias** calculadas con las macros de Octavius (fórmulas en la documentación).
- **Límites** (`velocity`, `effort`) desde la hoja de datos del MG996R a 6 V.

### 3.4 Cinemática de una pata

- **Directa (FK):** ángulos θ₁, θ₂, θ₃ → posición del pie.
- **Inversa (IK) analítica:**
  - θ₁ = atan2(y, x).
  - θ₂ y θ₃ se obtienen resolviendo el triángulo fémur–tibia con la ley de cosenos.
- **Test unitario:** IK → FK vuelve al punto inicial, con error < 0,1 mm, en miles de puntos.

### 3.5 Marcha trípode

- Los grupos (LF, RM, LR) y (RF, LM, RR) se alternan. Siempre hay 3 patas apoyadas formando un triángulo.
- Cada pata alterna dos fases:
  - **apoyo:** el pie empuja hacia atrás en línea recta;
  - **vuelo:** el pie vuelve adelante por un arco.
- Entrada: velocidad del cuerpo (vx, vy, ωz). Salida: 18 ángulos a 50 Hz.

### 3.6 Control y flujo de datos

| Controlador | Tipo | Articulaciones |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster` | 18 → `/joint_states` |
| `legs_controller` | `position_controllers/JointGroupPositionController` | 18 |

```
 Teclado ─► keyboard_teleop ───────────────────────────────┐
                                                           ├─► /hexapod/cmd_vel (Twist) ─► gait_node ─► IK ─► /legs_controller/commands
 ESP32 + joystick ─(USB)─► micro_ros_agent ─► /hexapod/joystick (Joy) ─► joy_to_twist ─┘                          │
                                                                                                          gz_ros2_control (Gazebo)
                                                                                                                  │
                                                         RViz ◄── robot_state_publisher ◄── /joint_states ◄───────┘
```

Para probar sin la ESP32 se publica el mismo tópico a mano:

```bash
ros2 topic pub -r 20 /hexapod/joystick sensor_msgs/msg/Joy "{axes: [0.0, 0.5]}"
```

---

## 4. Estudio de dimensiones (el aporte propio)

**Pregunta:** con 18 MG996R, una LiPo 3S y la electrónica dada, ¿qué largo deben tener la coxa, el fémur y la tibia, y qué tamaño el hexágono, para que el robot camine con margen de torque y sin choques entre patas?

### 4.1 Entradas

- Masas y medidas de §1, y la geometría del bracket coaxial (desde el CAD).
- Torque de diseño del servo: **50 % del torque de bloqueo** (factor de seguridad 2). Los MG996R reales, sobre todo las copias, suelen rendir menos que su hoja.

### 4.2 Restricciones que se calculan

| Criterio | Qué se exige |
|---|---|
| **Torque estático** | Torque en el fémur (el más cargado) ≤ torque de diseño, con 3 patas sosteniendo todo el peso |
| **Torque dinámico** | El mismo criterio, medido en Gazebo durante la marcha (`gz_ros2_control` expone el esfuerzo de cada articulación) |
| **Espacio de trabajo** | El pie alcanza el paso deseado sin salirse de los límites articulares |
| **Altura libre** | El cuerpo queda a una altura mínima del suelo |
| **Sin choques** | Patas vecinas (60° entre sí) no se tocan en ningún punto del ciclo |
| **Estabilidad** | El centro de masa queda dentro del triángulo de apoyo con un margen mínimo |
| **Layout** | Batería, Raspberry y PCA9685 caben en el hexágono con el centro de masa en el centro |

### 4.3 Estimación preliminar (orden de magnitud)

- **Masa total:** ≈ 1,87 kg.
  - Medido: servos 18 × 55 = 990 g; batería 271 g; Raspberry 86 g; 2 PCA9685 18 g.
  - *Hipótesis:* estructura 400 g; cables y reguladores 100 g; cámara + IMU 7 g.
- **Carga por pata:** con 3 patas de apoyo, cada pata sostiene ≈ 18,4 N / 3 ≈ 6,1 N.
- **Torque de diseño:** el MG996R a 6 V tiene 11 kgf·cm ≈ 1,08 N·m de bloqueo. Al 50 %, son ≈ 0,54 N·m.

Entonces:

> **distancia horizontal del eje del fémur al pie ≤ 0,54 / 6,1 ≈ 8,8 cm** (a 4,8 V: ≤ 7,5 cm)

Esto ya limita bastante la geometría: patas largas y abiertas no son viables con este servo. El estudio refina este número, sumando el peso propio de la pata y la dinámica de la marcha.

### 4.4 Método y resultado

1. Script `dimension_study.py` (sin ROS, con tests):
   - barre combinaciones de largos y de radio del cuerpo;
   - aplica las restricciones de §4.2;
   - genera gráficos (mapa de torque vs. largos, espacio de trabajo, margen de estabilidad).
2. La mejor combinación se escribe en `hexapod_params.xacro`.
3. **Se verifica en Gazebo:** se mide el torque durante la marcha y se compara con el estático.
4. Entregable **`docs/especificacion_diseno.md`**:
   - tabla de cotas para maquinar (largos de eje a eje y radio del hexágono);
   - posición de cada componente sobre el cuerpo;
   - márgenes obtenidos.

---

## 5. Estructura del repositorio

```
Evaluacion1_version_Hexapodo/
├── README.md · PLAN_DE_TRABAJO.md · DOCUMENTACION.md
├── src/
│   ├── hexapod_description/        modelo: urdf/ (hexapod.urdf.xacro, hexapod_params.xacro ★,
│   │                               leg.xacro, body.xacro, components.xacro, sensors.xacro,
│   │                               ros2_control.xacro, macros/), rviz/, launch/display.launch.py
│   ├── hexapod_control/            cerebro: kinematics.py, gait.py (sin ROS, testeables),
│   │                               gait_node.py, keyboard_teleop.py, joy_to_twist.py, config/, test/
│   └── hexapod_bringup/            launch/ (gazebo, microros_agent, bringup), config/
│                                   (controllers.yaml, gz_bridge.yaml, *.rviz), worlds/
├── analysis/                       dimension_study.py, sus tests y figuras
├── firmware/hexapod_joystick/      ESP32 + micro_ros_arduino (a partir del de Octavius)
├── docs/                           especificacion_diseno.md, img/
├── presentacion/                   Beamer USACH
└── scripts/                        make_release.sh, clean_room_test.sh
```

---

## 6. Etapas y cronograma

El detalle de cada etapa está en [`docs/plan/plan_ataque.pdf`](docs/plan/plan_ataque.pdf): qué hay que entender, ecuaciones, tareas, entregables con evidencia y qué mostrar en la presentación. Las fechas suponen entrega el **domingo 29 nov 2026** (por confirmar).

| Etapa | Contenido | Cuándo | Pauta |
|---|---|---|---|
| **E0** — Workspace y parámetros | 3 paquetes; `hexapod_params.xacro` con masas reales y cota inicial d ≤ 8,8 cm | jue 26 noche | R1, R11 |
| **E1** — Cinemática de una pata | `kinematics.py` (FK/IK) + test; macro de pata en RViz | jue 26 – vie 27 | base de R8 |
| **E2** — Modelo completo en RViz | Cuerpo hexagonal, 6 patas, componentes, sensores; 18 sliders | vie 27 | R2–R5 |
| **E3** — De pie en Gazebo ⚠️ | Base flotante, contacto, `ros2_control`, postura suave | vie 27 | R6 |
| **E4** — Marcha trípode y teclado | `gait.py`, `gait_node`, `keyboard_teleop` | sáb 28 | R8 |
| **E5** — Sensores | IMU + cámara | sáb 28 | R5, R7 |
| **E6** — micro-ROS | `joy_to_twist`, tópico `/hexapod/joystick`, firmware compilado; prueba real al comprar la ESP32 | sáb 28 | R9 |
| **E7** — Integración y pruebas | `bringup.launch.py`, tests, prueba en limpio | sáb 28 | R10, R11 |
| **E8** — Estudio de dimensiones | Barrido de L1, L2, L3, R; gráficos; cotas ideales; verificación de torques en Gazebo (bajo la línea de corte) | dom 29 | aporte propio |
| **E9** — Documentación y entrega | Sección por etapa al cerrarla; tutorial, Beamer, `.zip` | cada cierre + dom 29 | R12–R14 |

El estudio de dimensiones **no bloquea** el modelo: E0 parte con la cota estática de §4.3 y E8 la reemplaza por las cotas finales.

---|---|---|
| **0** — Workspace y datos | 3 paquetes que compilan; `hexapod_params.xacro` con los datos de §1; leer el CAD del bracket coaxial | `colcon build` OK |
| **1** — Cinemática de una pata | `kinematics.py` + test IK↔FK; macro de pata en RViz (3 sliders) | Test pasa; RViz coincide con FK |
| **2** — Estudio de dimensiones | `dimension_study.py`, gráficos y primera versión de las cotas ideales | Largos y radio elegidos con todas las restricciones de §4.2 cumplidas |
| **3** — Hexápodo en RViz | Cuerpo hexagonal, 6 patas, componentes y sensores en el URDF; 18 sliders | R2–R5 |
| **4** — De pie en Gazebo ⚠️ | Base flotante, fricción, `ros2_control`, postura inicial suave | Se sostiene ≥ 60 s; R6 |
| **5** — Caminar con teclado | `gait.py`, `gait_node`, `keyboard_teleop` | Avanza, retrocede y gira; R8 |
| **6** — Sensores | IMU + cámara publicando y visibles en RViz | R5, R7 |
| **7** — Verificación de dimensiones | Torques medidos en Gazebo durante la marcha vs. torque de diseño; ajustar cotas si hace falta; `especificacion_diseno.md` | Margen ≥ 2 en todas las articulaciones |
| **8** — Integración y calidad | `bringup.launch.py`, tests, prueba en limpio en contenedor | R10, R11, R14 |
| **9** — micro-ROS *(última prioridad)* | Paquete y tópico `/hexapod/joystick` + `joy_to_twist` listos desde la Fase 5. Firmware ESP32 escrito y compilado. Prueba real **cuando se compre la ESP32** | R9 |
| **10** — Documentación y presentación | `DOCUMENTACION.md`, tutorial `.docx/.pdf`, Beamer | R12, R13 |

---

## 7. Lo que falta para partir

| Dato | Bloquea | Estado |
|---|---|---|
| Modelos CAD | — | ✅ recibidos (`Hexodo_test-main.zip`); se usan solo los adaptadores coaxiales |
| Masas de batería, Raspberry, servo y PCA9685 | — | ✅ |
| Medidas de la batería (137 × 43 × 22 mm); los 55 g del servo incluyen los adaptadores | — | ✅ |
| Fecha y hora exactas de entrega | Calendario | ❓ (supuesto: domingo 29 nov 2026) |
| Requisitos propios: altura libre mínima, largo de paso deseado (si no hay, se proponen valores típicos) | Fase 2 | ❓ opcional |
| ESP32 + joystick | Solo la prueba final de la Fase 9 | Por comprar |

---

## 8. Checklist de presentación y tutorial

Requisitos de formato:

- Membrete y logo USACH.
- Título, estudiante, profesor (José Pascal) y fecha.
- Numeración de páginas, tablas, figuras y ecuaciones.
- Letra uniforme y figuras legibles.
- Índice, introducción y conclusiones.
- Referencias IEEE.
- Máximo 15 minutos.
- PDF + editable.

Contenido propio:

- Árbol cinemático.
- Ecuaciones de FK/IK.
- Diagrama de la marcha.
- Estudio de dimensiones (restricciones, gráficos, cotas finales).
- Torques simulados vs. MG996R.
- Layout del cuerpo.

---

## Anexo A — Recomendaciones para el robot real (fuera de alcance)

- **PCA9685:** segundo módulo con el puente A0 soldado (dirección 0x41). Uno por lado del robot.
- **Alimentación:**
  - Los servos **nunca** van directo a la 3S (11,1–12,6 V). Se usan 2–3 reguladores UBEC/buck de 10–20 A a 6 V, cada uno para un grupo de patas.
  - La potencia va por cables gruesos y no por las pistas del PCA9685.
  - La Raspberry va con su propio regulador de 5 V y 3 A, con tierra común.
  - El dimensionamiento final de los reguladores se hace con los torques de la Fase 7.
- **Seguridad de la LiPo:** fusible, interruptor general y alarma de voltaje por celda (mínimo 3,3 V/celda).
- **Control en el robot real:** la Raspberry Pi 4 corre el mismo `hexapod_control` de la simulación y envía los ángulos a los PCA9685 por I2C. Cada servo necesita una tabla de calibración con su offset, su signo y su rango de pulso.
- **Puesta en marcha por etapas:**
  1. un servo;
  2. una pata en el aire;
  3. el robot sobre un soporte;
  4. de pie en el suelo;
  5. primeros pasos.
