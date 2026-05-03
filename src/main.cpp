#include <Arduino.h>
#include <Servo.h>
/* =========================
 * Defines generales
 * ========================= */
#define DEBUG_SERIAL_BAUDRATE      115200

#define MAX_STATES                 3
#define MAX_EVENTS                 4

#define SERVO_PIN                  18
#define ULTRASONIC_LEFT_TRIG_PIN   5
#define ULTRASONIC_LEFT_ECHO_PIN   17
#define ULTRASONIC_RIGHT_TRIG_PIN  16
#define ULTRASONIC_RIGHT_ECHO_PIN  4

#define PERSON_DETECTION_THRESHOLD_CM   80.0f
#define ALIGN_TOLERANCE_CM              5.0f

#define SERVO_CENTER_ANGLE              90
#define SERVO_MIN_ANGLE                 0
#define SERVO_MAX_ANGLE                 180
#define SERVO_STEP_ANGLE                2

#define INVALID_DISTANCE_CM            -1.0f

/* =========================
 * Estados y eventos
 * ========================= */
typedef enum
{
  ST_IDLE = 0,
  ST_ALIGNING,
  ST_ALIGNED
} state_t;

typedef enum
{
  EV_NO_PERSON = 0,
  EV_PERSON_DETECTED,
  EV_TARGET_MISALIGNED,
  EV_TARGET_ALIGNED
} event_t;

/* =========================
 * Tipo de transición
 * ========================= */
typedef void (*transition_t)(void);

/* =========================
 * Variables globales FSM
 * ========================= */
state_t current_state = ST_IDLE;
event_t new_event = EV_NO_PERSON;

/* =========================
 * Variables globales del sistema
 * ========================= */
Servo mirrorServo;

float left_distance_cm = INVALID_DISTANCE_CM;
float right_distance_cm = INVALID_DISTANCE_CM;
int current_servo_angle = SERVO_CENTER_ANGLE;

/* =========================
 * Strings para debug
 * ========================= */
const char* state_names[MAX_STATES] =
{
  "ST_IDLE",
  "ST_ALIGNING",
  "ST_ALIGNED"
};

const char* event_names[MAX_EVENTS] =
{
  "EV_NO_PERSON",
  "EV_PERSON_DETECTED",
  "EV_TARGET_MISALIGNED",
  "EV_TARGET_ALIGNED"
};

/* =========================
 * Prototipos de funciones
 * ========================= */

// FSM
void smart_mirror_fsm(void);
void get_new_event(void);

// Lectura y evaluación
float read_ultrasonic_distance_cm(uint8_t trig_pin, uint8_t echo_pin);
bool is_person_detected(float left_cm, float right_cm);
bool is_target_aligned(float left_cm, float right_cm);

// Acciones de transición
void action_idle(void);
void action_start_aligning(void);
void action_continue_aligning(void);
void action_hold_aligned(void);

// Acciones auxiliares
void move_servo_left(void);
void move_servo_right(void);
void hold_servo_position(void);
void center_servo(void);

// Utilidades
void debug_print_transition(state_t state, event_t event);

/* =========================
 * Tabla de transición
 * Filas = estados
 * Columnas = eventos
 * ========================= */
transition_t state_table[MAX_STATES][MAX_EVENTS] =
{
  // ST_IDLE
  {
    action_idle,              // EV_NO_PERSON
    action_start_aligning,    // EV_PERSON_DETECTED
    action_idle,              // EV_TARGET_MISALIGNED
    action_idle               // EV_TARGET_ALIGNED
  },

  // ST_ALIGNING
  {
    action_idle,              // EV_NO_PERSON
    action_continue_aligning, // EV_PERSON_DETECTED
    action_continue_aligning, // EV_TARGET_MISALIGNED
    action_hold_aligned       // EV_TARGET_ALIGNED
  },

  // ST_ALIGNED
  {
    action_idle,              // EV_NO_PERSON
    action_hold_aligned,      // EV_PERSON_DETECTED
    action_continue_aligning, // EV_TARGET_MISALIGNED
    action_hold_aligned       // EV_TARGET_ALIGNED
  }
};

/* =========================
 * Máquina de estados
 * ========================= */
void smart_mirror_fsm(void)
{
  get_new_event();

  if ((current_state >= 0) && (current_state < MAX_STATES) &&
      (new_event >= 0) && (new_event < MAX_EVENTS))
  {
    debug_print_transition(current_state, new_event);
    state_table[current_state][new_event]();
  }
}

/* =========================
 * Generación de eventos
 * ========================= */
void get_new_event(void)
{
  left_distance_cm = read_ultrasonic_distance_cm(ULTRASONIC_LEFT_TRIG_PIN, ULTRASONIC_LEFT_ECHO_PIN);
  right_distance_cm = read_ultrasonic_distance_cm(ULTRASONIC_RIGHT_TRIG_PIN, ULTRASONIC_RIGHT_ECHO_PIN);

  if (!is_person_detected(left_distance_cm, right_distance_cm))
  {
    new_event = EV_NO_PERSON;
    return;
  }

  if (is_target_aligned(left_distance_cm, right_distance_cm))
  {
    new_event = EV_TARGET_ALIGNED;
    return;
  }

  new_event = EV_TARGET_MISALIGNED;
}

/* =========================
 * Lectura de sensores
 * ========================= */
float read_ultrasonic_distance_cm(uint8_t trig_pin, uint8_t echo_pin)
{
  // TODO: implementar lectura real del HC-SR04
  return INVALID_DISTANCE_CM;
}

bool is_person_detected(float left_cm, float right_cm)
{
  // TODO:
  // true si al menos uno de los sensores detecta una distancia
  // menor o igual a PERSON_DETECTION_THRESHOLD_CM
  return false;
}

bool is_target_aligned(float left_cm, float right_cm)
{
  // TODO:
  // true si abs(left_cm - right_cm) <= ALIGN_TOLERANCE_CM
  return false;
}

/* =========================
 * Acciones de transición
 * ========================= */
void action_idle(void)
{
  // Opcional:
  // mantener servo quieto o volver al centro
  hold_servo_position();
  current_state = ST_IDLE;
}

void action_start_aligning(void)
{
  // TODO:
  // decidir hacia qué lado mover el servo
  current_state = ST_ALIGNING;
}

void action_continue_aligning(void)
{
  // TODO:
  // seguir corrigiendo según la diferencia entre sensores
  current_state = ST_ALIGNING;
}

void action_hold_aligned(void)
{
  hold_servo_position();
  current_state = ST_ALIGNED;
}

/* =========================
 * Acciones auxiliares sobre servo
 * ========================= */
void move_servo_left(void)
{
  // TODO
}

void move_servo_right(void)
{
  // TODO
}

void hold_servo_position(void)
{
  // TODO
}

void center_servo(void)
{
  current_servo_angle = SERVO_CENTER_ANGLE;
  mirrorServo.write(current_servo_angle);
}

/* =========================
 * Debug
 * ========================= */
void debug_print_transition(state_t state, event_t event)
{
  Serial.print("[FSM] State: ");
  Serial.print(state_names[state]);
  Serial.print(" | Event: ");
  Serial.println(event_names[event]);
}

/* =========================
 * Setup
 * ========================= */
void setup()
{
  Serial.begin(DEBUG_SERIAL_BAUDRATE);

  pinMode(ULTRASONIC_LEFT_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_LEFT_ECHO_PIN, INPUT);

  pinMode(ULTRASONIC_RIGHT_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_RIGHT_ECHO_PIN, INPUT);

  mirrorServo.attach(SERVO_PIN);
  mirrorServo.write(SERVO_CENTER_ANGLE);

  current_servo_angle = SERVO_CENTER_ANGLE;
  current_state = ST_IDLE;
  new_event = EV_NO_PERSON;
}

/* =========================
 * Loop principal
 * ========================= */
void loop()
{
  smart_mirror_fsm();
  delay(50); // No podemos usar delay, revisar.
}