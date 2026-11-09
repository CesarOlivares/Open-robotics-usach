# Protocolo de pruebas con hardware (ESP32 + joystick)

Pruebas pendientes de la Fase 4 que requieren la placa física. Todo lo que se pudo verificar sin hardware ya está probado (ver "Estado previo"). Se ejecutan en orden: si una falla, se resuelve antes de seguir.

## Estado previo (verificado sin hardware)

| Ítem | Resultado |
|---|---|
| Firmware compila (esp32 core 2.0.2, micro_ros_arduino 2.0.8-jazzy) | ✅ 0 errores, 0 advertencias |
| Agente micro-ROS abre un puerto serial y espera | ✅ probado con puerto virtual |
| `bringup.launch.py use_microros:=true` levanta todo junto | ✅ |
| `arm_commander` con joystick simulado (`ros2 topic pub`) | ✅ base y hombro se mueven, zona muerta y watchdog funcionan |

## Preparación

1. Armar el circuito según [`circuito_joystick.md`](circuito_joystick.md) (VCC → **3V3**).
2. Conectar la ESP32 por USB y anotar el puerto: `ls /dev/ttyUSB* /dev/ttyACM*`.
3. Cargar `firmware/octavius_joystick/octavius_joystick.ino` desde el Arduino IDE y **cerrar el Monitor Serie**.
4. **No tocar el joystick** durante el arranque de la placa (calibra el centro).

## Pruebas

| # | Prueba | Cómo | Resultado esperado | ✔ |
|---|---|---|---|---|
| H1 | LED sin agente | Placa alimentada, sin lanzar nada | LED parpadea (2 Hz) | ☐ |
| H2 | Conexión con el agente | `ros2 launch octavius_bringup bringup.launch.py use_microros:=true` (agregar `serial_port:=...` si no es `/dev/ttyUSB0`) | LED fijo; en el log del agente aparece `session established` | ☐ |
| H3 | Nodo visible en ROS 2 | `ros2 node list` | Aparece `/octavius_joystick` | ☐ |
| H4 | Frecuencia | `ros2 topic hz /octavius/joystick` | ≈ 20 Hz | ☐ |
| H5 | Reposo | `ros2 topic echo /octavius/joystick` con el joystick suelto | `axes` cerca de 0 (\|v\| < 0,05) | ☐ |
| H6 | Extremos | Llevar cada eje a sus topes | `axes` llega a −1 y +1 en cada eje | ☐ |
| H7 | Botón | Presionar el joystick | `buttons: [1]` | ☐ |
| H8 | Base | Mover eje X | La base gira en Gazebo y RViz; suelto, se detiene | ☐ |
| H9 | Hombro | Mover eje Y | El hombro se mueve; se detiene en su límite (±1,57 rad) | ☐ |
| H10 | Sentido de los ejes | Derecha / arriba | Giro en el sentido intuitivo. Si no, cambiar el signo en `joystick_axis_sign` de `config/teleop.yaml` y recompilar | ☐ |
| H11 | Joystick + teclado juntos | Mover el joystick mientras se usa `keyboard_teleop` | Ambos funcionan a la vez, ninguno anula al otro | ☐ |
| H12 | Desconexión del cable | Desconectar el USB en pleno movimiento | Base y hombro se detienen en ≤ 0,5 s (watchdog) | ☐ |
| H13 | Reconexión | Volver a conectar el USB | LED vuelve a fijo y el joystick funciona sin reiniciar nada (puede requerir relanzar el agente si cambió el puerto) | ☐ |
| H14 | Reinicio del agente | Cerrar y volver a lanzar solo el agente: `ros2 launch octavius_bringup microros_agent.launch.py` | La placa se reconecta sola | ☐ |
| H15 | Agente por Docker (opcional) | `... use_microros:=true agent:=docker` | Igual que H2–H4 | ☐ |

## Evidencia para la documentación

- Video corto: joystick moviendo base y hombro en Gazebo.
- Captura de `ros2 node list` y de `ros2 topic hz /octavius/joystick`.
- Captura de `rqt_graph` con `/octavius_joystick → /octavius/joystick → /arm_commander`.
- Foto del circuito armado.

## Registro de resultados

| Fecha | Pruebas | Observaciones |
|---|---|---|
| | | |
