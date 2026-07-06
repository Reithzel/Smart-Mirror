#include <Arduino.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include <PubSubClient.h>
#include "WiFi.h"

/* =========================
 * Definitions and constants
 * ========================= */
#define DEBUG_SERIAL_BAUDRATE        115200
#define WIFI_SSID                    "SO Avanzados"
#define WIFI_PASSWORD                "SOA.2019"

#define MQTT_BROKER_HOST             "192.168.30.220"
#define MQTT_BROKER_PORT             1883
#define MQTT_CLIENT_ID               "smart_mirror_esp32"
#define MQTT_RECONNECT_DELAY_MS      3000
#define MQTT_LOOP_DELAY_MS           10
#define MQTT_TELEMETRY_INTERVAL_MS   1000

#define MQTT_TOPIC_SENSOR_DISTANCE_1   "espejo/sensor/distancia1"
#define MQTT_TOPIC_SENSOR_DISTANCE_2   "espejo/sensor/distancia2"
#define MQTT_TOPIC_SENSOR_LIGHT        "espejo/sensor/luz"
 
#define MQTT_TOPIC_CONTROL_MODE        "espejo/control/modo"
#define MQTT_TOPIC_CONTROL_SERVO       "espejo/control/servo"
#define MQTT_TOPIC_CONTROL_LIGHT       "espejo/control/luz"
#define MQTT_TOPIC_CONTROL_LED         "espejo/control/led"

#define WIFI_CONNECT_TIMEOUT_MS      10000
#define WIFI_RETRY_DELAY_MS            500

#define MAX_STATES                 3
#define MAX_EVENTS                 13

#define SERVO_PIN                  13
#define ULTRASONIC_LEFT_TRIG_PIN   15
#define ULTRASONIC_LEFT_ECHO_PIN   2  
#define ULTRASONIC_RIGHT_TRIG_PIN  23
#define ULTRASONIC_RIGHT_ECHO_PIN  22

#define PERSON_DETECTION_THRESHOLD_CM   80.0f
#define ALIGN_TOLERANCE_CM              5.0f

#define SERVO_CENTER_ANGLE              90
#define SERVO_MIN_ANGLE                 0
#define SERVO_MAX_ANGLE                 180
#define SERVO_STEP_ANGLE                1
#define SERVO_SETTLE_TIME_MS            20

#define SOUND_SPEED_CM_PER_US 0.0343f
#define MAX_TIMEOUT_US 4500UL
#define INVALID_DISTANCE_CM            -1.0f

#define LDR_PIN                     34

#define LED_STRIP_PIN               14
#define LED_STRIP_PIXEL_COUNT       16

#define LDR_DARK_VALUE              800
#define LDR_BRIGHT_VALUE            3000

#define LED_MIN_BRIGHTNESS          0
#define LED_MAX_BRIGHTNESS          255
#define LED_FADE_STEP               4

#define LED_WHITE_VALUE             255

#define STACK_SIZE_TASKS          2048
#define MQTT_STACK_SIZE_TASKS     4096
#define FSM_TASK_PRIORITY            3
#define ULTRASONIC_TASK_PRIORITY     2
#define LDR_TASK_PRIORITY            1
#define MQTT_TASK_PRIORITY           1

#define EVENT_QUEUE_LENGTH           1

#define ULTRASONIC_READ_INTERVAL_MS 30
#define LDR_READ_INTERVAL_MS        100

#define DISTANCE_CHANCE_THRESHOLD_CM  3.0f
#define LDR_CHANGE_THRESHOLD          25

#define LOOP_IDLE_DELAY_MS            1000

#define UNUSED_SERVO_ANGLE    -1
#define UNUSED_LED_BRIGHTNESS -1


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
  EV_CONT = 0,
  EV_NO_TARGET,
  EV_TARGET_DETECTED,
  EV_TARGET_MISALIGNED,
  EV_TARGET_ALIGNED,
  EV_FADE_OUT_LIGHT,
  EV_UPDATE_LIGHT,
  EV_MQTT_LIGHT_OFF,
  EV_MQTT_LIGHT_ON,
  EV_MQTT_MODE_MANUAL,
  EV_MQTT_MODE_AUTO,
  EV_MQTT_SET_SERVO,
  EV_MQTT_SET_LIGHT

} event_t;

typedef struct
{
  event_t type;
  float left_distance_cm;
  float right_distance_cm;
  int ldr_value;
  int target_led_brightness;
  int servo_angle;
  int led_brightness;
} fsm_event_t;
typedef enum
{
  CONTROL_MODE_AUTO = 0,
  CONTROL_MODE_MANUAL
} control_mode_t;

/* =========================
 * Transition function type definition
 * ========================= */
typedef void (*transition_t)(void);

/* =========================
 * Global variables for FSM
 * ========================= */
state_t current_state = ST_IDLE;
portMUX_TYPE fsm_state_mutex = portMUX_INITIALIZER_UNLOCKED;

/* =========================
 * Queues for servo and led events and for mqtt events
 * ========================= */
QueueHandle_t servo_event_queue = NULL;
QueueHandle_t light_event_queue = NULL;
QueueHandle_t mqtt_event_queue = NULL;

/* =========================
 * Global variables
 * ========================= */
Servo mirrorServo;
Adafruit_NeoPixel ledStrip(LED_STRIP_PIXEL_COUNT, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);
TaskHandle_t fsm_task_handle;
portMUX_TYPE servo_busy_mutex = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE led_brightness_mutex = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE control_mode_mutex = portMUX_INITIALIZER_UNLOCKED;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
control_mode_t current_control_mode = CONTROL_MODE_AUTO;

float left_distance_cm = INVALID_DISTANCE_CM;
float right_distance_cm = INVALID_DISTANCE_CM;
int current_servo_angle = SERVO_CENTER_ANGLE;
int current_led_brightness = 0;
int ldr_value = 0;
float previous_left_distance_cm = INVALID_DISTANCE_CM;
float previous_right_distance_cm = INVALID_DISTANCE_CM;
int previous_ldr_value = -1;
int target_led_brightness = 0;
unsigned long servo_busy_until_ms = 0;
int mqtt_servo_angle = UNUSED_SERVO_ANGLE;
int mqtt_led_brightness = UNUSED_LED_BRIGHTNESS;


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
  "EV_CONT",
  "EV_NO_TARGET",
  "EV_TARGET_DETECTED",
  "EV_TARGET_MISALIGNED",
  "EV_TARGET_ALIGNED",
  "EV_FADE_OUT_LIGHT",
  "EV_UPDATE_LIGHT",
  "EV_MQTT_LIGHT_OFF",
  "EV_MQTT_LIGHT_ON",
  "EV_MQTT_MODE_MANUAL",
  "EV_MQTT_MODE_AUTO",
  "EV_MQTT_SET_SERVO",
  "EV_MQTT_SET_LIGHT"
};

/* =========================
* Function declarations
* ========================= */

// FSM
void fsm_task(void* pvParameters);
void dispatch_event(const fsm_event_t& event);

// Reading sensors
float read_ultrasonic_distance_cm(uint8_t trig_pin, uint8_t echo_pin);
int read_ldr_value(void);

// Evaluating sensor readings
bool is_person_detected(float left_cm, float right_cm);
bool is_target_aligned(float left_cm, float right_cm);
bool has_relevant_distance_change(float left_cm, float right_cm);
bool has_relevant_ldr_change(int ldr_value);
bool has_time_elapsed(unsigned long now_ms, unsigned long target_ms);
bool is_servo_busy(void);

// State transition actions
void action_idle(void);
void action_start_aligning(void);
void action_continue_aligning(void);
void action_hold_aligned(void);
void action_none(void);
void action_update_light(void);
void action_fade_out_light(void);

void action_mqtt_light_off(void);
void action_mqtt_light_on(void);
void action_mqtt_mode_manual(void);
void action_mqtt_mode_auto(void);
void action_mqtt_set_servo(void);
void action_mqtt_set_light(void);

// Auxiliary servo actions
void move_servo_left(void);
void move_servo_right(void);
void hold_servo_position(void);
void center_servo(void);

// Utility 
void debug_print_transition(state_t state, event_t event);
void mark_servo_busy(void);
void create_queues(void);
void create_tasks(void);

// Getters and setters
state_t get_current_state(void);
void set_current_state(state_t state);
int get_current_led_brightness(void);
void set_current_led_brightness(int brightness);
control_mode_t get_current_control_mode(void);
void set_current_control_mode(control_mode_t mode);

// Light management functions declarations
int calculate_led_brightness(int ldr_value);
void set_led_brightness(int brightness);

// Queue functions and helpers
void enqueue_servo_event(const fsm_event_t& event);
void enqueue_light_event(const fsm_event_t& event);
void enqueue_mqtt_event(const fsm_event_t& event);
void enqueue_mqtt_event_type(event_t event_type);
void notify_fsm_task(void);

// Servo task
void ultrasonic_task(void* pvParameters);

// Light task
void ldr_task(void* pvParameters);

// MQTT tasks
void mqtt_task(void* pvParameters);
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void connect_wifi(void);
bool connect_mqtt(void);
void publish_mqtt_telemetry(void);


/* =========================
* State transition table
* Rows = States
* Columns = Events
 * ========================= */
transition_t state_table[MAX_STATES][MAX_EVENTS] =
{
  // ST_IDLE
  {
    action_none,               // EV_CONT
    action_idle,               // EV_NO_TARGET
    action_start_aligning,     // EV_TARGET_DETECTED
    action_none,               // EV_TARGET_MISALIGNED
    action_none,               // EV_TARGET_ALIGNED
    action_fade_out_light,     // EV_FADE_OUT_LIGHT
    action_none,               // EV_UPDATE_LIGHT
    action_mqtt_light_off,     // EV_MQTT_LIGHT_OFF
    action_mqtt_light_on,      // EV_MQTT_LIGHT_ON
    action_mqtt_mode_manual,   // EV_MQTT_MODE_MANUAL
    action_mqtt_mode_auto,      // EV_MQTT_MODE_AUTO
    action_mqtt_set_servo,     // EV_MQTT_SET_SERVO
    action_mqtt_set_light      // EV_MQTT_SET_LIGHT
  },

  // ST_ALIGNING
  {
    action_none,               // EV_CONT
    action_idle,               // EV_NO_TARGET
    action_continue_aligning,  // EV_TARGET_DETECTED
    action_continue_aligning,  // EV_TARGET_MISALIGNED
    action_hold_aligned,       // EV_TARGET_ALIGNED
    action_none,               // EV_FADE_OUT_LIGHT
    action_update_light,       // EV_UPDATE_LIGHT
    action_mqtt_light_off,     // EV_MQTT_LIGHT_OFF
    action_mqtt_light_on,      // EV_MQTT_LIGHT_ON
    action_mqtt_mode_manual,   // EV_MQTT_MODE_MANUAL
    action_mqtt_mode_auto,      // EV_MQTT_MODE_AUTO
    action_mqtt_set_servo,     // EV_MQTT_SET_SERVO
    action_mqtt_set_light      // EV_MQTT_SET_LIGHT
  },  

  // ST_ALIGNED
  { 
    action_none,               // EV_CONT
    action_idle,               // EV_NO_TARGET
    action_hold_aligned,       // EV_TARGET_DETECTED
    action_continue_aligning,  // EV_TARGET_MISALIGNED
    action_hold_aligned,       // EV_TARGET_ALIGNED
    action_none,               // EV_FADE_OUT_LIGHT 
    action_update_light,       // EV_UPDATE_LIGHT
    action_mqtt_light_off,     // EV_MQTT_LIGHT_OFF
    action_mqtt_light_on,      // EV_MQTT_LIGHT_ON
    action_mqtt_mode_manual,   // EV_MQTT_MODE_MANUAL
    action_mqtt_mode_auto,      // EV_MQTT_MODE_AUTO
    action_mqtt_set_servo,     // EV_MQTT_SET_SERVO
    action_mqtt_set_light      // EV_MQTT_SET_LIGHT
  }
};

/* We could also add "error" actions, for example if the current state is "IDLE" we could never trigger the "EV_TARGET_ALIGNED" event, that's an error. */

/* =========================
 * FSM (Finite State Machine)
 * ========================= */

void fsm_task(void* pvParameters)
{
  (void)pvParameters;

  fsm_event_t event;

  while(true)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    while(true)
    {
      if(xQueueReceive(servo_event_queue, &event, 0) == pdTRUE)
      {
        dispatch_event(event);
        continue;
      }

      if(xQueueReceive(mqtt_event_queue, &event, 0) == pdTRUE)
      {
        dispatch_event(event);
        continue;
      }

      if(xQueueReceive(light_event_queue, &event, 0) == pdTRUE)
      {
        dispatch_event(event);
        continue;
      }

      break;
    }
  }
}

void dispatch_event(const fsm_event_t& event)
{

  const state_t actual_state = get_current_state();

  if(event.type == EV_CONT)
  {
    return;
  }

  if ((actual_state >= 0) && (actual_state < MAX_STATES) &&
      (event.type >= 0) && (event.type < MAX_EVENTS))
  {
    left_distance_cm = event.left_distance_cm;
    right_distance_cm = event.right_distance_cm;
    ldr_value = event.ldr_value;
    target_led_brightness = event.target_led_brightness;
    mqtt_servo_angle = event.servo_angle;
    mqtt_led_brightness = event.led_brightness;

    debug_print_transition(actual_state, event.type);
    state_table[actual_state][event.type]();
  }
}

/* =========================
 * Event generation (tasks)
 * ========================= */

 void ultrasonic_task(void* pvParameters)
{
  (void)pvParameters;

  TickType_t last_wake_time = xTaskGetTickCount();

  while(true)
  {
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(ULTRASONIC_READ_INTERVAL_MS));

    
    if(is_servo_busy())
    continue;
    
    const float new_left_distance_cm = read_ultrasonic_distance_cm(ULTRASONIC_LEFT_TRIG_PIN, ULTRASONIC_LEFT_ECHO_PIN);
    const float new_right_distance_cm = read_ultrasonic_distance_cm(ULTRASONIC_RIGHT_TRIG_PIN, ULTRASONIC_RIGHT_ECHO_PIN);
    
    const state_t actual_state = get_current_state();
    
    fsm_event_t event =
    {
      EV_CONT,
      new_left_distance_cm,
      new_right_distance_cm,
      ldr_value,
      target_led_brightness,
      UNUSED_SERVO_ANGLE,
      UNUSED_LED_BRIGHTNESS
    };
    
    if(get_current_control_mode() == CONTROL_MODE_MANUAL)
      continue;

    if(!is_person_detected(new_left_distance_cm, new_right_distance_cm))
    {
      previous_left_distance_cm = new_left_distance_cm;
      previous_right_distance_cm = new_right_distance_cm;
      
      event.type = EV_NO_TARGET;
      enqueue_servo_event(event);
      continue;
    }
    else if(actual_state == ST_IDLE)
    {
      previous_left_distance_cm = new_left_distance_cm;
      previous_right_distance_cm = new_right_distance_cm;      

      event.type = EV_TARGET_DETECTED;
      enqueue_servo_event(event);
      continue;
    }

    if(!has_relevant_distance_change(new_left_distance_cm, new_right_distance_cm) && actual_state != ST_ALIGNING)
      continue;

    if (is_target_aligned(new_left_distance_cm, new_right_distance_cm))
    {
      previous_left_distance_cm = new_left_distance_cm;
      previous_right_distance_cm = new_right_distance_cm;
      
      event.type = EV_TARGET_ALIGNED;
      enqueue_servo_event(event);
      continue;
    }

    previous_left_distance_cm = new_left_distance_cm;
    previous_right_distance_cm = new_right_distance_cm;

    event.type = EV_TARGET_MISALIGNED;
    enqueue_servo_event(event);
  }
}

void ldr_task(void* pvParameters)
{
  (void)pvParameters;

  TickType_t last_wake_time = xTaskGetTickCount();
  int latest_target_led_brightness = target_led_brightness;

  while(true)
  {
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(LDR_READ_INTERVAL_MS));

    
    const int new_ldr_value = read_ldr_value();
    
    if(has_relevant_ldr_change(new_ldr_value))
    {
      previous_ldr_value = new_ldr_value;
      latest_target_led_brightness = calculate_led_brightness(new_ldr_value);
    }
    
    const state_t actual_state = get_current_state();
    const int current_brightness = get_current_led_brightness();
    
    fsm_event_t event =
    {
      EV_CONT,
      left_distance_cm,
      right_distance_cm,
      new_ldr_value,
      latest_target_led_brightness,
      UNUSED_SERVO_ANGLE,
      UNUSED_LED_BRIGHTNESS
    };
    
    if(get_current_control_mode() == CONTROL_MODE_MANUAL)
      continue;

    if(actual_state == ST_IDLE && current_brightness > 0)
    {
      event.type = EV_FADE_OUT_LIGHT;
      event.target_led_brightness = LED_MIN_BRIGHTNESS;
      enqueue_light_event(event);
      continue;
    }

    if(current_brightness != latest_target_led_brightness)
    {
      event.type = EV_UPDATE_LIGHT;
      enqueue_light_event(event);
    }
  }
}

void mqtt_task(void* pvParameters)
{
  (void)pvParameters;

  unsigned long last_telemetry_publish_ms = 0;

  while(true)
  {
    if(WiFi.status() != WL_CONNECTED)
    {
      connect_wifi();
    }

    if((WiFi.status() == WL_CONNECTED) && !mqttClient.connected())
    {
      connect_mqtt();
    }

    if(mqttClient.connected())
    {
      mqttClient.loop();

      const unsigned long now_ms = millis();

      if(now_ms - last_telemetry_publish_ms >= MQTT_TELEMETRY_INTERVAL_MS)
      {
        publish_mqtt_telemetry();
        last_telemetry_publish_ms = now_ms;
      }

    }

    vTaskDelay(pdMS_TO_TICKS(MQTT_LOOP_DELAY_MS));
  }
}

/* =========================
 * Queue functions and helpers
 * ========================= */

void enqueue_servo_event(const fsm_event_t& event)
{
  if(event.type == EV_CONT)
    return;

  if(servo_event_queue == NULL)
    return;

  xQueueOverwrite(servo_event_queue, &event);
  notify_fsm_task();
}

void enqueue_light_event(const fsm_event_t& event)
{
  if(event.type == EV_CONT)
    return;

  if(light_event_queue == NULL)
    return;

  xQueueOverwrite(light_event_queue, &event);
  notify_fsm_task();
}

void enqueue_mqtt_event(const fsm_event_t& event)
{
  if(event.type == EV_CONT)
    return;

  if(mqtt_event_queue == NULL)
    return;

  xQueueOverwrite(mqtt_event_queue, &event);
  notify_fsm_task();
}

void enqueue_mqtt_event_type(event_t event_type)
{
  fsm_event_t event = 
  {
    event_type,
    left_distance_cm,
    right_distance_cm,
    ldr_value,
    target_led_brightness,
    UNUSED_SERVO_ANGLE,
    UNUSED_LED_BRIGHTNESS
  };

  enqueue_mqtt_event(event);
}

void notify_fsm_task(void)
{
  if(fsm_task_handle == NULL)
    return;

  xTaskNotifyGive(fsm_task_handle);
}

/* =========================
 * Sensor readings and evaluations
 * ========================= */

 /** Servo and ultrasonic sensor related functions **/
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

void mark_servo_busy(void)
{
  const unsigned long busy_until_ms = millis() + SERVO_SETTLE_TIME_MS;

  portENTER_CRITICAL(&servo_busy_mutex);
  servo_busy_until_ms = busy_until_ms;
  portEXIT_CRITICAL(&servo_busy_mutex);
}

bool has_time_elapsed(unsigned long now_ms, unsigned long target_ms)
{
  return ((long)(now_ms - target_ms) >= 0);
}

bool is_servo_busy(void)
{
  const unsigned long now_ms = millis();

  portENTER_CRITICAL(&servo_busy_mutex);
  const unsigned long busy_until_ms = servo_busy_until_ms;
  portEXIT_CRITICAL(&servo_busy_mutex);

  return !has_time_elapsed(now_ms, busy_until_ms);
}

 /** Light sensor related functions **/

int read_ldr_value(void)
{
  return analogRead(LDR_PIN);
}

/* =========================
 * State transition actions
 * ========================= */

void action_idle(void)
{
  center_servo();
  set_current_state(ST_IDLE);
}

void action_start_aligning(void)
{
  if(left_distance_cm == INVALID_DISTANCE_CM)
  {
    move_servo_right();
    set_current_state(ST_ALIGNING);
    return;
  }

  if(right_distance_cm == INVALID_DISTANCE_CM)
  {
    move_servo_left();
    set_current_state(ST_ALIGNING);
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

  set_current_state(ST_ALIGNING);
}

void action_continue_aligning(void)
{

  if(left_distance_cm == INVALID_DISTANCE_CM)
  {
    move_servo_right();
    set_current_state(ST_ALIGNING);
    return;
  }

  if(right_distance_cm == INVALID_DISTANCE_CM)
  {
    move_servo_left();
    set_current_state(ST_ALIGNING);
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

  set_current_state(ST_ALIGNING);
}

void action_hold_aligned(void)
{
  hold_servo_position();
  set_current_state(ST_ALIGNED);
}

void action_fade_out_light(void)
{
  const int current_brightness = get_current_led_brightness();

  set_led_brightness(max(current_brightness - LED_FADE_STEP, LED_MIN_BRIGHTNESS));
}

void action_update_light(void)
{
  const int current_brightness = get_current_led_brightness();
  
  if (current_brightness < target_led_brightness)
  {
    set_led_brightness(min(current_brightness + LED_FADE_STEP, target_led_brightness));
  }
  else if (current_brightness > target_led_brightness)
  {
    set_led_brightness(max(current_brightness - LED_FADE_STEP, target_led_brightness));
  }
}

void action_mqtt_light_off(void)
{

  if(get_current_control_mode() != CONTROL_MODE_MANUAL)
    return;

  target_led_brightness = LED_MIN_BRIGHTNESS;
  set_led_brightness(LED_MIN_BRIGHTNESS); 
}

void action_mqtt_light_on(void)
{
  if(get_current_control_mode() != CONTROL_MODE_MANUAL)
    return;

  target_led_brightness = calculate_led_brightness(ldr_value);
  set_led_brightness(target_led_brightness);
}

void action_mqtt_mode_manual(void)
{
  set_current_control_mode(CONTROL_MODE_MANUAL);
}

void action_mqtt_mode_auto(void)
{
  set_current_control_mode(CONTROL_MODE_AUTO);
}

void action_mqtt_set_servo(void)
{
  if(get_current_control_mode() != CONTROL_MODE_MANUAL)
    return;

  if(mqtt_servo_angle == UNUSED_SERVO_ANGLE)
    return;

  const int previous_servo_angle = current_servo_angle;

  current_servo_angle = constrain(mqtt_servo_angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

  if(current_servo_angle != previous_servo_angle)
  {
    mirrorServo.write(current_servo_angle);
    mark_servo_busy();
  }  
}

void action_mqtt_set_light(void)
{
  if(get_current_control_mode() != CONTROL_MODE_MANUAL)
    return;

  if(mqtt_led_brightness == UNUSED_LED_BRIGHTNESS)
    return;

  target_led_brightness = constrain(mqtt_led_brightness, LED_MIN_BRIGHTNESS, LED_MAX_BRIGHTNESS);
  set_led_brightness(target_led_brightness);
}

void action_none(void)
{
  // No state change, no action.
}

/* =========================
 * Auxilliary servo transition functions (move actions)
 * ========================= */

void move_servo_left(void)
{
  const int previous_servo_angle = current_servo_angle;

  current_servo_angle -= SERVO_STEP_ANGLE;

  if (current_servo_angle < SERVO_MIN_ANGLE)
  {
    current_servo_angle = SERVO_MIN_ANGLE;
  }

  if(current_servo_angle != previous_servo_angle)
  {
    mirrorServo.write(current_servo_angle);
    mark_servo_busy();
  }
}

void move_servo_right(void)
{
  const int previous_servo_angle = current_servo_angle;

  current_servo_angle += SERVO_STEP_ANGLE;

  if (current_servo_angle > SERVO_MAX_ANGLE)
  {
    current_servo_angle = SERVO_MAX_ANGLE;
  }
  
  if(current_servo_angle != previous_servo_angle)
  {
    mirrorServo.write(current_servo_angle);
    mark_servo_busy();
  }
}

void hold_servo_position(void)
{
  mirrorServo.write(current_servo_angle);
}

void center_servo(void)
{
  //current_servo_angle = SERVO_CENTER_ANGLE;
  //mirrorServo.write(current_servo_angle);
}

/* =========================
 * Debug
 * ========================= */
void debug_print_transition(state_t state, event_t event)
{
  if(event != EV_CONT)
  {
    Serial.print("[FSM] State: ");
    Serial.print(state_names[state]);
    Serial.print(" | Event: ");
    Serial.println(event_names[event]);
  }
}

/* =========================
 * Getters and setters (with mutex protection)
 * ========================= */

state_t get_current_state(void)
{
  portENTER_CRITICAL(&fsm_state_mutex);
  const state_t state = current_state;
  portEXIT_CRITICAL(&fsm_state_mutex);
  return state;
  
}

void set_current_state(state_t state)
{
  portENTER_CRITICAL(&fsm_state_mutex);
  current_state = state;
  portEXIT_CRITICAL(&fsm_state_mutex);
}

int get_current_led_brightness(void)
{
  portENTER_CRITICAL(&led_brightness_mutex);
  const int brightness = current_led_brightness;
  portEXIT_CRITICAL(&led_brightness_mutex);
  return brightness;
}

void set_current_led_brightness(int brightness)
{
  portENTER_CRITICAL(&led_brightness_mutex);
  current_led_brightness = brightness;
  portEXIT_CRITICAL(&led_brightness_mutex);
}

control_mode_t get_current_control_mode(void)
{
  portENTER_CRITICAL(&control_mode_mutex);
  const control_mode_t mode = current_control_mode;
  portEXIT_CRITICAL(&control_mode_mutex);
  return mode;
}

void set_current_control_mode(control_mode_t mode)
{
  portENTER_CRITICAL(&control_mode_mutex);
  current_control_mode = mode;
  portEXIT_CRITICAL(&control_mode_mutex);
}

/* =========================
 * Light strip management
 * ========================= */

int calculate_led_brightness(int ldr_value)
{
  if (ldr_value >= LDR_BRIGHT_VALUE)
  {
    return LED_MIN_BRIGHTNESS;
  }

  if (ldr_value <= LDR_DARK_VALUE)
  {
    return LED_MAX_BRIGHTNESS;
  }

  return ((LDR_BRIGHT_VALUE - ldr_value) * LED_MAX_BRIGHTNESS) /
         (LDR_BRIGHT_VALUE - LDR_DARK_VALUE);
}

void set_led_brightness(int brightness)
{
  const int current_brightness = get_current_led_brightness();

  brightness = constrain(brightness, LED_MIN_BRIGHTNESS, LED_MAX_BRIGHTNESS);

  if (brightness == current_brightness)
  {
    return;
  }

  set_current_led_brightness(brightness);

  ledStrip.setBrightness(current_led_brightness);

  for (uint16_t i = 0; i < LED_STRIP_PIXEL_COUNT; i++)
  {
    ledStrip.setPixelColor(i, ledStrip.Color(LED_WHITE_VALUE, LED_WHITE_VALUE, LED_WHITE_VALUE));
  }

  ledStrip.show();
}

/* =========================
 * Deadband evaluation functions
 * ========================= */

bool has_relevant_distance_change(float left_cm, float right_cm)
{
  if (previous_left_distance_cm == INVALID_DISTANCE_CM || previous_right_distance_cm == INVALID_DISTANCE_CM)
  {
    return true;
  }

  if (abs(left_cm - previous_left_distance_cm) >= DISTANCE_CHANCE_THRESHOLD_CM ||
      abs(right_cm - previous_right_distance_cm) >= DISTANCE_CHANCE_THRESHOLD_CM)
  {
    return true;
  }

  return false;
}

bool has_relevant_ldr_change(int ldr_value)
{
  if (previous_ldr_value == -1)
  {
    return true;
  }

  if (abs(ldr_value - previous_ldr_value) >= LDR_CHANGE_THRESHOLD)
  {
    return true;
  }

  return false;
}

/* =========================
 * Setup auxiliary functions
 * ========================= */

void create_queues(void)
{
  servo_event_queue = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(fsm_event_t));
  light_event_queue = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(fsm_event_t));
  mqtt_event_queue = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(fsm_event_t));

  if (servo_event_queue == NULL || light_event_queue == NULL || mqtt_event_queue == NULL)
  {
    Serial.println("[Setup] Error creating queues");
  }
}

void create_tasks(void)
{
  xTaskCreate(fsm_task, "FSM Task", STACK_SIZE_TASKS, NULL, FSM_TASK_PRIORITY, &fsm_task_handle);
  xTaskCreate(ultrasonic_task, "Ultrasonic Task", STACK_SIZE_TASKS, NULL, ULTRASONIC_TASK_PRIORITY, NULL);
  xTaskCreate(ldr_task, "LED Task", STACK_SIZE_TASKS, NULL, LDR_TASK_PRIORITY, NULL);
  xTaskCreate(mqtt_task, "MQTT Task", MQTT_STACK_SIZE_TASKS, NULL, MQTT_TASK_PRIORITY, NULL);
}

/* =========================
 * MQTT and WiFi management
 * ========================= */

void connect_wifi(void)
{
  if(WiFi.status() == WL_CONNECTED)
    return;

  Serial.print("[WiFi] Connecting to..");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const unsigned long start_time_ms = millis();

  while((WiFi.status() != WL_CONNECTED) && (millis() - start_time_ms < WIFI_CONNECT_TIMEOUT_MS))
  {
    Serial.print(".");
    vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_DELAY_MS));
  }

  Serial.println();

  if(WiFi.status() == WL_CONNECTED)
  {
    Serial.print("[WiFi] Connected. IP: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("[WiFi] Connection timeout.");
  }
}

bool connect_mqtt(void)
{
  if(mqttClient.connected())
    return true;

  if(WiFi.status() != WL_CONNECTED)
  {
    Serial.println("[MQTT] WiFi not connected. Cannot connect to MQTT broker.");
    return false;
  }

  Serial.print("[MQTT] Connecting to broker ");
  Serial.print(MQTT_BROKER_HOST);
  Serial.print(":");
  Serial.println(MQTT_BROKER_PORT);

  mqttClient.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
  mqttClient.setCallback(mqtt_callback);

  if(mqttClient.connect(MQTT_CLIENT_ID))
  {
    Serial.println("[MQTT] Connected");

    mqttClient.subscribe(MQTT_TOPIC_CONTROL_LED);
    Serial.print("[MQTT] Subscribed to ");
    Serial.println(MQTT_TOPIC_CONTROL_LED);

    mqttClient.subscribe(MQTT_TOPIC_CONTROL_MODE);
    Serial.print("[MQTT] Subscribed to ");
    Serial.println(MQTT_TOPIC_CONTROL_MODE);

    mqttClient.subscribe(MQTT_TOPIC_CONTROL_SERVO);
    Serial.print("[MQTT] Subscribed to ");
    Serial.println(MQTT_TOPIC_CONTROL_SERVO);

    mqttClient.subscribe(MQTT_TOPIC_CONTROL_LIGHT);
    Serial.print("[MQTT] Subscribed to ");
    Serial.println(MQTT_TOPIC_CONTROL_LIGHT);

    return true;
  }

  Serial.print("[MQTT] Connection failed. State: ");
  Serial.println(mqttClient.state());

  return false;
}

void mqtt_callback(char* topic, byte* payload, unsigned int length)
{
  char message[32];

  const unsigned int copy_length = min(length, sizeof(message) - 1);

  memcpy(message, payload, copy_length);
  message[copy_length] = '\0';

  Serial.print("[MQTT] Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(" | Payload: ");
  Serial.println(message);

  if(strcmp(topic, MQTT_TOPIC_CONTROL_MODE) == 0)
  {
    if(strcmp(message, "manual") == 0)
    {
      Serial.println("[MQTT] Control mode: MANUAL");
      enqueue_mqtt_event_type(EV_MQTT_MODE_MANUAL);
      return;
    }

    if(strcmp(message, "automatico") == 0)
    {
      Serial.println("[MQTT] Control mode: AUTO");
      enqueue_mqtt_event_type(EV_MQTT_MODE_AUTO);
      return;
    }

    Serial.println("[MQTT] Unknown control mode command received.");
    return;
  }

  if(strcmp(topic, MQTT_TOPIC_CONTROL_LED) == 0)
  {
    if(strcmp(message, "0") == 0)
    {
      Serial.println("[MQTT] LED command: OFF");
      enqueue_mqtt_event_type(EV_MQTT_LIGHT_OFF);
      return;
    }

    if(strcmp(message, "1") == 0)
    {
      Serial.println("[MQTT] LED command: ON");
      enqueue_mqtt_event_type(EV_MQTT_LIGHT_ON);
      return;
    }

    Serial.println("[MQTT] Unknown LED command received.");
    return;
  }

  if(strcmp(topic, MQTT_TOPIC_CONTROL_SERVO) == 0)
  {
    const int servo_angle = atoi(message);

    fsm_event_t event =
    {
      EV_MQTT_SET_SERVO,
      left_distance_cm,
      right_distance_cm,
      ldr_value,
      target_led_brightness,
      servo_angle,
      UNUSED_LED_BRIGHTNESS
    };

    Serial.print("[MQTT] Servo command: Set angle to ");
    Serial.println(servo_angle);

    enqueue_mqtt_event(event);
    return;
  }

  if(strcmp(topic, MQTT_TOPIC_CONTROL_LIGHT) == 0)
  {
    const int led_brightness = atoi(message);

    fsm_event_t event =
    {
      EV_MQTT_SET_LIGHT,
      left_distance_cm,
      right_distance_cm,
      ldr_value,
      target_led_brightness,
      UNUSED_SERVO_ANGLE,
      led_brightness
    };

    Serial.print("[MQTT] LED command: Set brightness to ");
    Serial.println(led_brightness);

    enqueue_mqtt_event(event);
    return;
  }
}

void publish_mqtt_telemetry(void)
{
  if(!mqttClient.connected())
    return;

  if(get_current_control_mode() == CONTROL_MODE_MANUAL)
    return;

  char payload[24];

  snprintf(payload, sizeof(payload), "%.2f", left_distance_cm);
  mqttClient.publish(MQTT_TOPIC_SENSOR_DISTANCE_1, payload);

  snprintf(payload, sizeof(payload), "%.2f", right_distance_cm);
  mqttClient.publish(MQTT_TOPIC_SENSOR_DISTANCE_2, payload);

  snprintf(payload, sizeof(payload), "%d", ldr_value);
  mqttClient.publish(MQTT_TOPIC_SENSOR_LIGHT, payload);
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

  set_current_state(ST_IDLE);

  create_queues();

  create_tasks();
}

/* =========================
 * Main loop (not in use because we use tasks)
 * ========================= */
void loop()
{
  vTaskDelay(pdMS_TO_TICKS(LOOP_IDLE_DELAY_MS));
}
