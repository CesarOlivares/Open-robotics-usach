# Joystick con ESP32 / ESP32-S3 (micro-ROS)

Guía para cuando lleguen los componentes. Hoy todo el lado ROS está probado
con un joystick simulado; falta solo la prueba con la placa.

## 1. Qué se necesita

| Componente | Detalle |
|---|---|
| Placa | ESP32 DevKit V1 (ESP32-WROOM-32) **o** ESP32-S3 DevKitC-1 |
| Joystick | Módulo analógico de 2 ejes con botón (tipo KY-023): pines GND, +5V, VRx, VRy, SW |
| Cable USB | De datos, no solo de carga. La S3 tiene dos puertos: usar el marcado **UART** (o COM) |
| 5 cables dupont | Hembra-hembra |

## 2. Conexión

```
 Joystick            ESP32 DevKit        ESP32-S3 DevKitC-1
 ─────────           ────────────        ──────────────────
 GND        ───────  GND                 GND
 +5V        ───────  3V3   ← ¡no 5 V!    3V3   ← ¡no 5 V!
 VRx        ───────  GPIO34              GPIO1
 VRy        ───────  GPIO35              GPIO2
 SW         ───────  GPIO32              GPIO4
```

- **3,3 V, no 5 V:** el pin del módulo dice +5V, pero el ADC de la ESP32
  solo admite hasta 3,3 V. Alimentado con 3,3 V el potenciómetro entrega
  0–3,3 V y se usa todo el rango.
- **ADC1:** los pines elegidos son del ADC1. El ADC2 deja de funcionar si se
  activa el WiFi (por si después se usa micro-ROS por WiFi).
- **SW** se lee con la resistencia *pull-up* interna: suelto = 1, apretado = 0.

## 3. Qué hace el firmware

`firmware/hexapod_joystick/hexapod_joystick.ino` (un solo archivo para las
dos placas; los pines se eligen solos según la placa):

1. Al encender mide el centro real de cada eje (el joystick debe estar
   suelto). Ningún módulo da exactamente la mitad del rango.
2. Cada 50 ms (20 Hz): lee cada eje 8 veces y promedia (el ADC es ruidoso),
   lo normaliza a [-1, 1] escalando cada lado por separado desde el centro
   medido, y publica `sensor_msgs/Joy` en `/hexapod/joystick`:
   `axes = [x, y]`, `buttons = [botón]`.
3. Si se pierde el agente, libera todo y vuelve a esperarlo (reconexión).
4. LED: parpadea esperando al agente y queda fijo al conectar (en la S3, el
   LED RGB: ámbar parpadeando → verde fijo).

La placa **no** aplica zona muerta, curva ni velocidades: entrega el valor
continuo del eje y `joy_to_twist` decide (se ajusta en
`config/joystick.yaml` sin reprogramar).

## 4. Compilar y cargar

### Opción A — ESP32 clásica con Arduino IDE (verificada)

1. Arduino IDE 2 → *Boards Manager* → **esp32 by Espressif, versión 2.0.x**
   (se probó con 2.0.2).
2. Librería **micro_ros_arduino** versión *jazzy* (2.0.8-jazzy): descargar el
   .zip del release y *Sketch → Include Library → Add .ZIP Library*.
3. Abrir `firmware/hexapod_joystick/hexapod_joystick.ino`, placa
   **ESP32 Dev Module**, elegir el puerto y *Upload*.

Resultado de la compilación: **0 errores, 0 advertencias**; 30 % de la
memoria de programa y 13 % de la RAM.

### Opción B — ESP32-S3 (o ESP32) con PlatformIO

`micro_ros_arduino` trae micro-ROS **precompilado solo para la ESP32
clásica**; para la S3 hay que compilar micro-ROS para esa placa, y eso lo
hace PlatformIO con `micro_ros_platformio`. Usa el mismo `.ino`.

```bash
# Instalar PlatformIO (una vez)
curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
export PATH=$PATH:$HOME/.platformio/penv/bin

cd firmware
pio run -e esp32s3                 # compilar (la 1.ª vez tarda: compila micro-ROS)
pio run -e esp32s3 -t upload       # cargar
```

Para la ESP32 clásica con PlatformIO: `-e esp32dev`.

Resultado (`-e esp32s3`, probado en un contenedor Docker): compila y genera
`firmware.bin`; usa 12,8 % de la RAM y 10,7 % de la flash. La primera vez
tardó 7 min (compila micro-ROS para la S3).

## 5. Agente micro-ROS (en el PC)

La placa no habla DDS (es muy pesado para un microcontrolador). Habla
XRCE-DDS por el USB con el **agente**, que la presenta al resto de ROS 2
como un nodo más (`/hexapod_joystick`). Dos formas:

- **Docker** (sin compilar nada): `agent:=docker` en el launch.
- **Nativo:** compilarlo una vez según el tutorial oficial (mismos pasos que
  en Octavius, `Evaluacion_1/README.md` §4.5) y hacer
  `source ~/microros_ws/install/local_setup.bash`.

## 6. Prueba con la placa (cuando llegue)

```bash
ls /dev/ttyUSB* /dev/ttyACM*        # ver en qué puerto quedó
ros2 launch hexapod_bringup bringup.launch.py use_microros:=true serial_port:=/dev/ttyUSB0 agent:=docker
```

| # | Paso | Esperado | ✓ |
|---|---|---|---|
| 1 | Conectar la placa con el joystick suelto | LED parpadea (busca al agente) | ☐ |
| 2 | Lanzar el bringup con `use_microros:=true` | LED fijo; el agente muestra *session established* | ☐ |
| 3 | `ros2 node list` | aparece `/hexapod_joystick` | ☐ |
| 4 | `ros2 topic hz /hexapod/joystick` | ~20 Hz | ☐ |
| 5 | `ros2 topic echo /hexapod/joystick` con la palanca suelta | ejes cerca de 0 (\|x\| < 0,08) | ☐ |
| 6 | Palanca al tope en cada dirección | ejes llegan a ±1 | ☐ |
| 7 | Palanca arriba | el robot avanza; si retrocede, `axis_y_sign: -1.0` en `joystick.yaml` | ☐ |
| 8 | Palanca a la derecha | gira a la derecha; si no, `axis_x_sign: -1.0` | ☐ |
| 9 | Apretar la palanca y moverla a la derecha | paso lateral a la derecha | ☐ |
| 10 | Mover la palanca de a poco | la velocidad crece suave desde 0 | ☐ |
| 11 | Desconectar el USB caminando | el robot se detiene solo (< 1 s) | ☐ |
| 12 | Reconectar | LED vuelve a fijo y el joystick funciona sin relanzar | ☐ |

## 7. Probar sin la placa

Se publica lo mismo que publicaría la ESP32:

```bash
ros2 launch hexapod_bringup bringup.launch.py
ros2 topic pub -r 20 /hexapod/joystick sensor_msgs/msg/Joy "{axes: [0.0, 0.6], buttons: [0]}"
```

O la prueba completa con gráfico: `python3 scripts/probar_joystick.py`.
