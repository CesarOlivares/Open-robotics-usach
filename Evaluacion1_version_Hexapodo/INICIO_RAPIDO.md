# Inicio rápido

Requisito: **Ubuntu 24.04 con ROS 2 Jazzy** instalado
([guía oficial](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)).

## 1. Instalar y compilar (un comando)

```bash
cd Evaluacion1_version_Hexapodo
./instalar.sh              # o ./instalar.sh --test para correr además los tests
```

Instala con `rosdep` todo lo que declaran los `package.xml` (Gazebo,
ros2_control, …) y compila con `colcon`.

## 2. Ejecutar

En **cada terminal nueva**:

```bash
cd Evaluacion1_version_Hexapodo
source install/setup.bash
```

| Quiero… | Comando |
|---|---|
| **Simulación completa** (Gazebo + RViz + marcha + joystick) | `ros2 launch hexapod_bringup bringup.launch.py` |
| Manejar con el teclado (otra terminal) | `ros2 run hexapod_control keyboard_teleop` |
| Ver solo el modelo con 18 sliders | `ros2 launch hexapod_description display.launch.py` |
| Ver una pata sola | `ros2 launch hexapod_description display.launch.py model:=leg_test` |
| Joystick sin placa (simulado) | `ros2 topic pub -r 20 /hexapod/joystick sensor_msgs/msg/Joy "{axes: [0.0, 0.6], buttons: [0]}"` |
| Joystick con la ESP32 | `ros2 launch hexapod_bringup bringup.launch.py use_microros:=true serial_port:=/dev/ttyUSB0 agent:=docker` |
| Sin ventanas (PC lento / servidor) | `ros2 launch hexapod_bringup bringup.launch.py use_rviz:=false headless:=true` |

**Teclado:** `w`/`s` adelante/atrás · `a`/`d` lateral · `q`/`e` girar ·
espacio frena. Cada pulsación suma 2 cm/s (o 0,1 rad/s).

**Joystick:** eje Y avanza, eje X gira; apretar la palanca cambia a paso
lateral. La velocidad es proporcional a cuánto se mueve la palanca.

Al lanzar, el robot aparece apoyado en la batería y en ~4 s se pone de pie
solo. Después ya obedece al teclado o al joystick.

## 3. Comprobar que todo funciona

**Todo de una vez** (con `bringup.launch.py` corriendo, ~1 min): recorre la
cadena completa y dice CUMPLE / NO CUMPLE en 8 puntos.

```bash
python3 scripts/prueba_integracion.py
```

Pruebas por separado, con la simulación corriendo, en otra terminal:

```bash
python3 scripts/medir_postura.py --segundos 60    # de pie y estable
python3 scripts/medir_marcha.py                   # velocidad real vs pedida
python3 scripts/verificar_sensores.py             # IMU y cámara
python3 scripts/probar_joystick.py                # joystick simulado (usar bringup.launch.py)
```

Tests sin simulación: `colcon test && colcon test-result --verbose`.

Prueba en un PC limpio (Docker, ~15 min): `./scripts/prueba_en_limpio.sh`.

## 4. Mapa en 30 segundos

```
joystick ESP32 ─► /hexapod/joystick ─► joy_to_twist ─┐
teclado ──────────────────────────────────────────────┼─► /hexapod/cmd_vel ─► gait_node
                                                       │      (marcha trípode + IK de 6 patas)
                                                       │                │
                                                       │                ▼ /legs_controller/commands (18 ángulos)
                                                       │      Gazebo + ros2_control (18 servos MG996R simulados)
                                                       │                │
                         RViz ◄── /tf, cámara ◄────────┴────────────────┘ IMU, cámara, odometría
```

| Paquete | Qué tiene |
|---|---|
| `hexapod_description` | El robot: URDF/xacro. **Todas las medidas en `urdf/hexapod_params.xacro`** |
| `hexapod_control` | Cinemática, marcha y nodos (`gait_node`, `keyboard_teleop`, `joy_to_twist`) |
| `hexapod_bringup` | Launch, mundo de Gazebo y configuración (`config/*.yaml`) |

Para entenderlo a fondo: [`docs/GUIA_DESDE_CERO.md`](docs/GUIA_DESDE_CERO.md).
Registro de cada etapa con mediciones: [`DOCUMENTACION.md`](DOCUMENTACION.md).

## 5. Problemas comunes

| Síntoma | Solución |
|---|---|
| `Package 'hexapod_bringup' not found` | Falta `source install/setup.bash` en esa terminal |
| El robot no se para | Esperar ~4 s tras abrir Gazebo; ver que `ros2 control list_controllers` muestre ambos *active* |
| Gazebo muy lento | `headless:=true use_rviz:=false` |
| `keyboard_teleop necesita una terminal interactiva` | Correrlo con `ros2 run` en una terminal propia, no dentro de un launch |
| Dos simulaciones se mezclan | Cerrar la anterior; o usar otro `ROS_DOMAIN_ID` en cada terminal |
