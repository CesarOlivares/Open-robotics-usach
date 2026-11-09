/*
 * Octavius — firmware del joystick (ESP32 + micro-ROS por puerto serial)
 * =====================================================================
 *
 * Qué hace:
 *   Lee los 2 ejes de un joystick analógico, los normaliza a [-1, 1] y los
 *   publica en ROS 2 como sensor_msgs/Joy en el tópico /octavius/joystick,
 *   20 veces por segundo. En ROS, el nodo arm_commander usa:
 *     axes[0] (eje X) → giro de la base
 *     axes[1] (eje Y) → hombro
 *
 * Qué NO hace:
 *   No calcula ángulos, límites ni zona muerta. Eso vive en arm_commander
 *   (ROS), donde se ajusta en config/teleop.yaml sin reprogramar la placa.
 *
 * Comunicación (micro-ROS):
 *   ESP32 ──USB serial 115200──► micro_ros_agent (PC) ──DDS──► nodos ROS 2
 *   La ESP32 no habla DDS directamente: el agente traduce entre el
 *   protocolo XRCE-DDS (liviano, para microcontroladores) y DDS.
 *
 * Conexiones (ver docs/circuito_joystick.md):
 *   Joystick VCC → 3V3   (¡no 5 V! el ADC de la ESP32 admite hasta 3,3 V)
 *   Joystick GND → GND
 *   Joystick VRx → GPIO34   (ADC1)
 *   Joystick VRy → GPIO35   (ADC1)
 *   Joystick SW  → GPIO32   (botón; se publica en buttons[0], hoy sin uso)
 *
 * LED integrado (GPIO2):
 *   parpadeando → esperando al agente
 *   encendido   → conectado y publicando
 *
 * Requisitos:
 *   Arduino IDE 2, placa "ESP32 Dev Module" (core esp32 2.0.x) y la
 *   librería micro_ros_arduino (versión jazzy).
 *
 * Basado en el ejemplo oficial micro-ros_reconnection_example de
 * micro_ros_arduino.
 */

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/joy.h>

#if !defined(ESP32)
#error Este firmware está pensado para una placa ESP32.
#endif

// ---------------------------------------------------------------------
// Configuración
// ---------------------------------------------------------------------
const int PIN_AXIS_X = 34;      // VRx
const int PIN_AXIS_Y = 35;      // VRy
const int PIN_BUTTON = 32;      // SW (activo en bajo)
const int LED_PIN = 2;          // LED integrado de la ESP32 DevKit

const int ADC_MAX = 4095;       // ADC de 12 bits: lecturas de 0 a 4095
const int SAMPLES_PER_READ = 8; // promedio para reducir el ruido del ADC
const int CALIBRATION_SAMPLES = 64;

const unsigned int PUBLISH_PERIOD_MS = 50;   // 50 ms → 20 Hz

// ---------------------------------------------------------------------
// Entidades micro-ROS
// ---------------------------------------------------------------------
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t publisher;

// Mensaje Joy. En micro-ROS no hay memoria dinámica: los arreglos del
// mensaje deben apuntar a buffers reservados de antemano.
sensor_msgs__msg__Joy msg;
float axes_buffer[2];
int32_t buttons_buffer[1];
char frame_id[] = "octavius_joystick";

// Lectura del joystick en reposo (centro real de cada eje).
int center_x = ADC_MAX / 2;
int center_y = ADC_MAX / 2;

// Si algo falla al crear entidades, se vuelve a esperar al agente.
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { return false; } }

// Errores que se ignoran a propósito: un mensaje perdido se reemplaza por el
// siguiente en 50 ms, y al desconectarse el agente la liberación de
// entidades falla porque ya no hay con quién hablar.
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void) temp_rc; }

// Ejecuta X cada MS milisegundos sin bloquear el loop.
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis(); } \
  if (uxr_millis() - init > MS) { X; init = uxr_millis(); } \
} while (0)

enum states {
  WAITING_AGENT,       // buscando al agente en el puerto serial
  AGENT_AVAILABLE,     // el agente respondió: crear nodo y publicador
  AGENT_CONNECTED,     // publicando
  AGENT_DISCONNECTED   // se perdió el agente: liberar y volver a esperar
} state;

// ---------------------------------------------------------------------
// Lectura y normalización del joystick
// ---------------------------------------------------------------------

// Promedio de varias lecturas del ADC: el ADC de la ESP32 es ruidoso.
int read_adc_average(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
  }
  return sum / samples;
}

/*
 * Convierte una lectura cruda a [-1, 1].
 *
 * El centro real del joystick casi nunca es 2048 (el ADC de la ESP32 no es
 * lineal y los potenciómetros tienen tolerancia). Por eso cada lado se
 * escala por separado respecto al centro medido:
 *
 *   raw >= centro:  v = (raw - centro) / (4095 - centro)
 *   raw <  centro:  v = (raw - centro) / centro
 *
 * Así el reposo da 0 y ambos extremos llegan exactamente a -1 y +1.
 */
float normalize_axis(int raw, int center) {
  float v;
  if (raw >= center) {
    v = (float)(raw - center) / (float)(ADC_MAX - center);
  } else {
    v = (float)(raw - center) / (float)center;
  }
  if (v > 1.0f) v = 1.0f;
  if (v < -1.0f) v = -1.0f;
  return v;
}

// Mide el centro de cada eje al encender (el joystick debe estar suelto).
void calibrate_center() {
  center_x = read_adc_average(PIN_AXIS_X, CALIBRATION_SAMPLES);
  center_y = read_adc_average(PIN_AXIS_Y, CALIBRATION_SAMPLES);
}

// ---------------------------------------------------------------------
// Publicación
// ---------------------------------------------------------------------
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer == NULL) {
    return;
  }

  msg.axes.data[0] = normalize_axis(read_adc_average(PIN_AXIS_X, SAMPLES_PER_READ), center_x);
  msg.axes.data[1] = normalize_axis(read_adc_average(PIN_AXIS_Y, SAMPLES_PER_READ), center_y);
  msg.buttons.data[0] = (digitalRead(PIN_BUTTON) == LOW) ? 1 : 0;

  // Sello de tiempo con el reloj del PC (sincronizado al conectar).
  int64_t now_ns = rmw_uros_epoch_nanos();
  msg.header.stamp.sec = (int32_t)(now_ns / 1000000000LL);
  msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);

  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
}

// Crea nodo, publicador, timer y executor. Puede tardar unos segundos.
bool create_entities() {
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "octavius_joystick", "", &support));

  // Publicador "reliable" (QoS por defecto), igual que la suscripción de
  // arm_commander. Uno best_effort no se conectaría con ella.
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
    "octavius/joystick"));

  // El último argumento (true) arranca el timer de inmediato.
  RCCHECK(rclc_timer_init_default2(
    &timer,
    &support,
    RCL_MS_TO_NS(PUBLISH_PERIOD_MS),
    timer_callback,
    true));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Sincroniza el reloj con el agente para que los sellos de tiempo
  // coincidan con los del PC.
  rmw_uros_sync_session(1000);
  return true;
}

// Libera todo lo creado (al perder la conexión con el agente).
void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCSOFTCHECK(rcl_publisher_fini(&publisher, &node));
  RCSOFTCHECK(rcl_timer_fini(&timer));
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));
}

// Asigna los buffers fijos a los arreglos del mensaje (una sola vez).
void init_message() {
  msg.axes.data = axes_buffer;
  msg.axes.size = 2;
  msg.axes.capacity = 2;

  msg.buttons.data = buttons_buffer;
  msg.buttons.size = 1;
  msg.buttons.capacity = 1;

  msg.header.frame_id.data = frame_id;
  msg.header.frame_id.size = strlen(frame_id);
  msg.header.frame_id.capacity = sizeof(frame_id);
}

// ---------------------------------------------------------------------
// Arduino
// ---------------------------------------------------------------------
void setup() {
  // Transporte serial por el USB de la placa (115200 baudios). Desde aquí
  // el puerto serial es de micro-ROS: no usar Serial.print().
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  analogReadResolution(12);         // 0..4095
  analogSetAttenuation(ADC_11db);   // rango de entrada 0..3,3 V

  delay(500);                       // deja estabilizar el joystick
  calibrate_center();
  init_message();

  state = WAITING_AGENT;
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      }
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }

  // LED: fijo si está conectado, parpadeo de 2 Hz si espera al agente.
  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, (millis() / 250) % 2);
  }
}
