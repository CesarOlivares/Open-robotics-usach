# Circuito y firmware del joystick (ESP32 + micro-ROS)

## 1. Materiales

| Componente | Cantidad | Nota |
|---|---|---|
| ESP32 DevKit (ESP32-WROOM-32) | 1 | Cualquier placa "ESP32 Dev Module" |
| Módulo joystick analógico de 2 ejes (tipo KY-023) | 1 | 2 potenciómetros + 1 pulsador |
| Cables dupont hembra-hembra | 5 | |
| Cable USB de datos | 1 | Alimenta la placa y transporta micro-ROS |

## 2. Conexiones

```
   Joystick KY-023                 ESP32 DevKit
  ┌───────────────┐              ┌──────────────┐
  │          GND ─┼──────────────┼─ GND         │
  │          +5V ─┼──────────────┼─ 3V3         │  ← alimentar con 3,3 V, no con 5 V
  │          VRx ─┼──────────────┼─ GPIO34      │  ← eje X → base
  │          VRy ─┼──────────────┼─ GPIO35      │  ← eje Y → hombro
  │           SW ─┼──────────────┼─ GPIO32      │  ← pulsador (reservado)
  └───────────────┘              │              │
                                 │   USB ───────┼──► PC (micro_ros_agent)
                                 └──────────────┘
```

| Pin joystick | Pin ESP32 | Motivo |
|---|---|---|
| GND | GND | Referencia común |
| +5V | **3V3** | El ADC de la ESP32 mide de 0 a 3,3 V. Con 5 V los extremos del joystick superarían ese rango y podrían dañar el pin. |
| VRx | GPIO34 | Pin del **ADC1**. El ADC2 no se puede usar cuando el Wi-Fi está activo, así que usar ADC1 deja el circuito listo también para micro-ROS por Wi-Fi. |
| VRy | GPIO35 | ADC1, igual que el anterior. GPIO34 y 35 son solo de entrada, ideales para leer señales analógicas. |
| SW | GPIO32 | El pulsador conecta a GND al presionar. Se activa la resistencia *pull-up* interna, así que la lectura es 1 suelto y 0 presionado. |

## 3. Cómo se procesa la señal

Cada potenciómetro del joystick es un divisor de tensión: su salida va de 0 V a 3,3 V según la inclinación. El ADC de 12 bits de la ESP32 la convierte en un número:

$$raw = \frac{V}{3{,}3\,\text{V}} \cdot 4095 \qquad raw \in [0,\,4095] \tag{1}$$

El firmware hace 4 pasos:

1. **Calibración del centro (al encender).** Promedia 64 lecturas con el joystick suelto. El centro real casi nunca es 2048, porque el ADC de la ESP32 no es lineal y los potenciómetros tienen tolerancia. Por eso **no hay que tocar el joystick mientras la placa arranca**.
2. **Filtrado.** Cada lectura es el promedio de 8 muestras, para reducir el ruido del ADC.
3. **Normalización a [-1, 1].** Cada lado se escala por separado respecto al centro medido $c$:

$$v = \begin{cases} \dfrac{raw - c}{4095 - c} & raw \ge c \\[2ex] \dfrac{raw - c}{c} & raw < c \end{cases} \tag{2}$$

   Así el reposo da 0 y ambos topes llegan exactamente a −1 y +1, aunque el centro no esté en la mitad.
4. **Publicación.** Cada 50 ms (20 Hz) publica un `sensor_msgs/Joy` en `/octavius/joystick` con `axes = [v_x, v_y]` y `buttons = [SW]`.

La **zona muerta** y la conversión a velocidad articular **no** están en el firmware: las hace `arm_commander` en ROS (`config/teleop.yaml`). Así se ajustan sin reprogramar la placa.

## 4. Cargar el firmware

Requisitos, ya usados en clases:
- Arduino IDE 2.
- Placas ESP32 (gestor de placas: *esp32 by Espressif*, versión 2.0.x).
- Librería `micro_ros_arduino` versión **jazzy** (desde [github.com/micro-ROS/micro_ros_arduino/releases](https://github.com/micro-ROS/micro_ros_arduino/releases), *Sketch → Include Library → Add .ZIP Library*).

Pasos:
1. Abrir `firmware/octavius_joystick/octavius_joystick.ino`.
2. *Herramientas → Placa*: **ESP32 Dev Module**.
3. *Herramientas → Puerto*: el de la ESP32 (`/dev/ttyUSB0` o `/dev/ttyACM0`).
4. **Subir**. Si no conecta, mantener presionado el botón *BOOT* de la placa al empezar la carga.
5. **Cerrar el Monitor Serie**: el puerto queda reservado para el agente micro-ROS.

## 5. Indicador LED (GPIO2)

| LED | Estado |
|---|---|
| Parpadeo (2 Hz) | Esperando al agente micro-ROS |
| Encendido fijo | Conectado: publicando el joystick |

Si se cierra el agente, la placa vuelve sola a esperar y se reconecta cuando el agente vuelve, sin necesidad de reiniciarla.
