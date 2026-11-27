# Hexápodo simulado con ROS 2 — versión paralela de la Evaluación 1

Proyecto personal de la asignatura **Open Robotics** (2° semestre 2026) — Cesar Olivares Lavalovich.

Hexápodo de 18 articulaciones (MG996R) con cuerpo hexagonal regular, modelado en URDF/XACRO y simulado en Gazebo Harmonic con `ros2_control`. Tiene IMU y cámara frontal, camina con marcha trípode controlada por teclado y por un joystick ESP32 vía micro-ROS. Incluye un estudio de dimensiones que calcula los largos de las patas y el layout ideales para los componentes reales.

> La entrega oficial de la Evaluación 1 es el brazo Octavius ([`../Evaluacion_1/`](../Evaluacion_1/)). Este proyecto cumple la misma pauta con un robot con patas. Todo es simulado; el robot real queda fuera de alcance.

## Empezar

```bash
./instalar.sh                                     # dependencias + compilación, un comando
source install/setup.bash
ros2 launch hexapod_bringup bringup.launch.py     # simulación completa
```

| Documento | Para qué |
|---|---|
| [`INICIO_RAPIDO.md`](INICIO_RAPIDO.md) | Instalar, ejecutar y comprobar en 5 minutos |
| [`docs/GUIA_DESDE_CERO.md`](docs/GUIA_DESDE_CERO.md) | Aprender el proyecto desde cero: paquetes, nodos, tópicos, xacro, variables |
| [`DOCUMENTACION.md`](DOCUMENTACION.md) | Registro de cada etapa: qué se hizo, por qué, problemas y evidencia medida |
| [`docs/joystick_esp32.md`](docs/joystick_esp32.md) | Conexión y prueba del joystick con ESP32 / ESP32-S3 |
| [`PLAN_DE_TRABAJO.md`](PLAN_DE_TRABAJO.md) y [`docs/plan/plan_ataque.pdf`](docs/plan/plan_ataque.pdf) | Plan, etapas y entregables |

**Estado:** E0–E7 y E10 listas (modelo, de pie, marcha, sensores, joystick
del lado ROS, integración, prueba en un PC limpio, guía). Pendiente: prueba
del joystick con la placa, E8 (estudio de dimensiones) y E9 (entrega).
