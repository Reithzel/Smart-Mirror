#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
/* =========================
 * Definitions and constants
 * ========================= */
#define DEBUG_SERIAL_BAUDRATE      115200

#define MAX_STATES                 3
#define MAX_EVENTS                 4

#define SERVO_PIN                  5
#define ULTRASONIC_LEFT_TRIG_PIN   22
#define ULTRASONIC_LEFT_ECHO_PIN   23
#define ULTRASONIC_RIGHT_TRIG_PIN  19
#define ULTRASONIC_RIGHT_ECHO_PIN  21

#define PERSON_DETECTION_THRESHOLD_CM   80.0f
#define ALIGN_TOLERANCE_CM              5.0f

#define SERVO_CENTER_ANGLE              90
#define SERVO_MIN_ANGLE                 0
#define SERVO_MAX_ANGLE                 180
#define SERVO_STEP_ANGLE                2

#define SOUND_SPEED_CM_PER_US 0.0343f
#define MAX_TIMEOUT_US 5830UL
#define INVALID_DISTANCE_CM            -1.0f

#define LDR_PIN                     34

#define LED_STRIP_PIN               18
#define LED_STRIP_PIXEL_COUNT       16

#define LDR_DARK_VALUE              3000
#define LDR_BRIGHT_VALUE            800

#define LED_MIN_BRIGHTNESS          0
#define LED_MAX_BRIGHTNESS          255
#define LED_FADE_STEP               5

/* =========================
 * Enumerations for FSM states and events
 * ========================= */
typedef enum
{
  ST_IDLE = 0,
  ST_ALIGNING,
  ST_ALIGNED
} state_t;

typedef enum
{
  EV_NO_TARGET = 0,
  EV_TARGET_DETECTED,
  EV_TARGET_MISALIGNED,
  EV_TARGET_ALIGNED
} event_t;

/* =========================
 * Transition function type definition
 * ========================= */
typedef void (*transition_t)(void);

/* =========================
 * Global variables for FSM
 * ========================= */
state_t current_state = ST_IDLE;
event_t new_event = EV_NO_TARGET;

/* =========================
 * Global variables (Servo and Ledstrip)
 * ========================= */
Servo mirrorServo;
Adafruit_NeoPixel ledStrip(LED_STRIP_PIXEL_COUNT, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);

float left_distance_cm = INVALID_DISTANCE_CM;
float right_distance_cm = INVALID_DISTANCE_CM;
int current_servo_angle = SERVO_CENTER_ANGLE;
int current_led_brightness = 0;
int ldr_value = 0;

/* =========================
 * String for debug purposes
 * ========================= */
const char* state_names[MAX_STATES] =
{
  "ST_IDLE",
  "ST_ALIGNING",
  "ST_ALIGNED"
};

const char* event_names[MAX_EVENTS] =
{
  "EV_NO_TARGET",
  "EV_TARGET_DETECTED",
  "EV_TARGET_MISALIGNED",
  "EV_TARGET_ALIGNED"
};

/* =========================
 * Function declarations
 * ========================= */

// FSM
void smart_mirror_fsm(void);
void get_new_event(void);

// Reading sensors and evaluating
float read_ultrasonic_distance_cm(uint8_t trig_pin, uint8_t echo_pin);
bool is_person_detected(float left_cm, float right_cm);
bool is_target_aligned(float left_cm, float right_cm);

// State transition actions
void action_idle(void);
void action_start_aligning(void);
void action_continue_aligning(void);
void action_hold_aligned(void);

// Auxilliary actions
void move_servo_left(void);
void move_servo_right(void);
void hold_servo_position(void);
void center_servo(void);

// Utility 
void debug_print_transition(state_t state, event_t event);

// Light management functions declarations
void update_light_control(void);
void update_leds_from_ldr(void);
void fade_out_leds(void);
int read_ldr_value(void);
int calculate_led_brightness(int ldr_value);
void set_led_brightness(int brightness);

/* =========================
 * State transition table
 * Rows = States
 * Columns = Events
 * ========================= */
transition_t state_table[MAX_STATES][MAX_EVENTS] =
{
  // ST_IDLE
  {
    action_idle,              // EV_NO_TARGET
    action_start_aligning,    // EV_TARGET_DETECTED
    action_idle,              // EV_TARGET_MISALIGNED
    action_idle               // EV_TARGET_ALIGNED
  },

  // ST_ALIGNING
  {
    action_idle,              // EV_NO_TARGET
    action_continue_aligning, // EV_TARGET_DETECTED
    action_continue_aligning, // EV_TARGET_MISALIGNED
    action_hold_aligned       // EV_TARGET_ALIGNED
  },

  // ST_ALIGNED
  {
    action_idle,              // EV_NO_TARGET
    action_hold_aligned,      // EV_TARGET_DETECTED
    action_continue_aligning, // EV_TARGET_MISALIGNED
    action_hold_aligned       // EV_TARGET_ALIGNED
  }
};

/* =========================
 * FSM (Finite State Machine)
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
 * Event generation
 * ========================= */
void get_new_event(void)
{
  left_distance_cm = read_ultrasonic_distance_cm(ULTRASONIC_LEFT_TRIG_PIN, ULTRASONIC_LEFT_ECHO_PIN);
  right_distance_cm = read_ultrasonic_distance_cm(ULTRASONIC_RIGHT_TRIG_PIN, ULTRASONIC_RIGHT_ECHO_PIN);

  if (!is_person_detected(left_distance_cm, right_distance_cm))
  {
    new_event = EV_NO_TARGET;
    return;
  }
  else if(current_state == ST_IDLE)
  {
    new_event = EV_TARGET_DETECTED;
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
 * Sensor readings and evaluations
 * ========================= */
float read_ultrasonic_distance_cm(uint8_t trig_pin, uint8_t echo_pin)
{
  static constexpr float SOUND_SPEED_CM_PER_US_HALF_TRIP = SOUND_SPEED_CM_PER_US / 2.0f;
  static constexpr unsigned long ECHO_TIMEOUT_US = MAX_TIMEOUT_US; // Equivalent to 200cm.

  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  const unsigned long duration_us = pulseIn(echo_pin, HIGH, ECHO_TIMEOUT_US);

  if (duration_us == 0)
  {
    return INVALID_DISTANCE_CM;
  }

  return duration_us * SOUND_SPEED_CM_PER_US_HALF_TRIP;
}

bool is_person_detected(float left_cm, float right_cm)
{
  if((left_cm > INVALID_DISTANCE_CM) && (left_cm <= PERSON_DETECTION_THRESHOLD_CM))
  {
    return true;
  }

  if((right_cm > INVALID_DISTANCE_CM) && (right_cm <= PERSON_DETECTION_THRESHOLD_CM))
  {
    return true;
  }

  return false;
}

bool is_target_aligned(float left_cm, float right_cm)
{
  if(left_cm > INVALID_DISTANCE_CM && right_cm > INVALID_DISTANCE_CM)
  {
    if(abs(left_cm - right_cm) <= ALIGN_TOLERANCE_CM)
    {
      return true;
    }
  }

  return false;
}

/* =========================
 * State transition actions
 * ========================= */
void action_idle(void)
{
  center_servo();
  current_state = ST_IDLE;
}

void action_start_aligning(void)
{
  if(left_distance_cm == INVALID_DISTANCE_CM)
  {
    move_servo_right();
    current_state = ST_ALIGNING;
    return;
  }

  if(right_distance_cm == INVALID_DISTANCE_CM)
  {
    move_servo_left();
    current_state = ST_ALIGNING;
    return;
  }

  if (left_distance_cm < right_distance_cm)
  {
    move_servo_left();
  }
  else
  {
    move_servo_right();
  }

  current_state = ST_ALIGNING;
}

void action_continue_aligning(void)
{

  if(left_distance_cm == INVALID_DISTANCE_CM)
  {
    move_servo_right();
    current_state = ST_ALIGNING;
    return;
  }

  if(right_distance_cm == INVALID_DISTANCE_CM)
  {
    move_servo_left();
    current_state = ST_ALIGNING;
    return;
  }

  if (left_distance_cm < right_distance_cm)
  {
    move_servo_left();
  }
  else
  {
    move_servo_right();
  }

  current_state = ST_ALIGNING;
}

void action_hold_aligned(void)
{
  hold_servo_position();
  current_state = ST_ALIGNED;
}

/* =========================
 * Auxilliary servo transition functions (move actions)
 * ========================= */
void move_servo_left(void)
{
  current_servo_angle -= SERVO_STEP_ANGLE;

  if (current_servo_angle < SERVO_MIN_ANGLE)
  {
    current_servo_angle = SERVO_MIN_ANGLE;
  }

  mirrorServo.write(current_servo_angle);
}

void move_servo_right(void)
{
  current_servo_angle += SERVO_STEP_ANGLE;

  if (current_servo_angle > SERVO_MAX_ANGLE)
  {
    current_servo_angle = SERVO_MAX_ANGLE;
  }
  
  mirrorServo.write(current_servo_angle);
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
 * Light strip management
 * ========================= */

void update_light_control(void)
{
  update_leds_from_ldr();
}

void update_leds_from_ldr(void)
{
  static unsigned long last_light_debug_ms = 0;

  ldr_value = read_ldr_value();
  
  int target_brightness = calculate_led_brightness(ldr_value);

  if (millis() - last_light_debug_ms >= 500)
  {
    Serial.print("[LIGHT] LDR: ");
    Serial.print(ldr_value);
    Serial.print(" | Brightness: ");
    Serial.println(target_brightness);
    last_light_debug_ms = millis();
  }
  
  set_led_brightness(target_brightness);
}

void fade_out_leds(void)
{
  if (current_led_brightness <= LED_MIN_BRIGHTNESS)
  {
    current_led_brightness = LED_MIN_BRIGHTNESS;
    set_led_brightness(current_led_brightness);
    return;
  }

  current_led_brightness -= LED_FADE_STEP;

  if (current_led_brightness < LED_MIN_BRIGHTNESS)
  {
    current_led_brightness = LED_MIN_BRIGHTNESS;
  }

  set_led_brightness(current_led_brightness);
}

int read_ldr_value(void)
{
  return analogRead(LDR_PIN);
}

int calculate_led_brightness(int value)
{
  value = constrain(value, 0, 4095);
  return (4095 - value) >> 4;
}

void set_led_brightness(int brightness)
{
  static int last_written_brightness = -1;

  brightness = constrain(brightness, LED_MIN_BRIGHTNESS, LED_MAX_BRIGHTNESS);
  current_led_brightness = brightness;

  if (current_led_brightness == last_written_brightness)
  {
    return;
  }

  ledStrip.fill(
    ledStrip.Color(current_led_brightness, current_led_brightness, current_led_brightness),
    0,
    ledStrip.numPixels()
  );
  ledStrip.show();
  last_written_brightness = current_led_brightness;
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

  pinMode(LDR_PIN, INPUT);

  pinMode(LED_STRIP_PIN, OUTPUT);
  ledStrip.begin();
  set_led_brightness(0);

  mirrorServo.attach(SERVO_PIN);
  mirrorServo.write(SERVO_CENTER_ANGLE);

  current_servo_angle = SERVO_CENTER_ANGLE;
  current_state = ST_IDLE;
  new_event = EV_NO_TARGET;
}

/* =========================
 * Main loop
 * ========================= */
void loop()
{
  smart_mirror_fsm();
  update_light_control();
  delay(50); // No podemos usar delay, revisar.
}
