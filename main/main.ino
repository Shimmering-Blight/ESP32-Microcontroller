TaskHandle_t Task1;
TaskHandle_t Task2;

// states
#define GRASPING 0b00
#define MOVING 0b01
#define RELEASING 0b10
#define HOMING 0b11

// constants
const int led_pin = 2;
//const int limit_switch_pin = -1;
//const int proximity_sensor_pin = -1;
//const int strain_sensor_pin = -1;


// variables
volatile int state = -1;
long led_on_timestamp;
long led_off_timestamp;
boolean led_on;

void setup() {
  Serial.begin(115200);
  Serial.println('Hello World!');
  setupCore1();
  setupCore2();
  wait_until_grasping();
  Serial.println('Here we go!');
}

void loop() {}

// --------------------------------- //
// ---------- Core 1 Code ---------- //
// --------------------------------- //
void setupCore1() {
  xTaskCreatePinnedToCore(
    core1Loop, /* Function to implement the task */
    "Task1", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    2,  /* Priority of the task */
    &Task1,  /* Task handle. */
    0); /* Core where the task should run */
}

void core1Loop(void * parameter) {
  for(;;) {
    switch (state) {
      case GRASPING:
        grasping();
        break;
      case MOVING:
        moving();
        break;
      case RELEASING:
        releasing();
        break;
      case HOMING:
        homing();
        break;
      default:
        Serial.println('Done.');
        for(;;){} // infinite loop
        break;
    }
    Serial.print('State changed: ');
    Serial.println(state);
  }
}

/**
 * This function waits until the object has been sensed.
 * Then jumps into GRASPING state
 */
void wait_until_grasping() {
  boolean object_is_present = false;
  while (!object_is_present) {
    // read a sensor to determine if object is present
  }
  state = GRASPING;
}

/**
 * This function drives the motor until 
 * a sensor indicates it is adequately grasped
 */
void grasping() {
  boolean grasped = false; // this might be the reading of a strain gauge
  while (!grasped) {
    // close the fingers until the object is grasped
  }
  state = MOVING;
}

/**
 * I think this function just applies constant 
 * torque to the gripper until it has arrived
 */
void moving() {
  boolean arrived = false; // this might be the reading of a proximity sensor
  while (!arrived) {
    // wait until the gripper has arrived
  }
  state = RELEASING;
}

/**
 * This function drives the motor until 
 * a sensor indicates there is enough clearance
 */
void releasing() {
  boolean released = false; // this might be the reading of a strain gauge
  while (!released) {
    // open the fingers until the object is released
  }
  state = HOMING;
}

/**
 * This function drives the motor until
 * a limit switch indicated the motor is at "home position"
 */
void homing() {
  boolean at_home = false; // this would be the reading of a limit switch
  while (!at_home) {
    // go home
  }
  state = -1; // end of cycle
}

// --------------------------------- //
// ---------- Core 2 Code ---------- //
// --------------------------------- //
void setupCore2() {
  xTaskCreatePinnedToCore(
    core2Loop, /* Function to implement the task */
    "Task2", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    1,  /* Priority of the task */
    &Task2,  /* Task handle. */
    1); /* Core where the task should run */
}

void core2Loop(void * parameter) {
  for(;;) {
    update_led();
    delay(50); // sleep for 50ms
  }
}

void update_led() {
  switch (state) {
    case GRASPING:
      blink_led(100,900);
      break;
    case MOVING:
      blink_led(400,600);
      break;
    case RELEASING:
      blink_led(1000,0);
      break;
    case HOMING:
      blink_led(700,300);
      break;
    default:
      break;
  }
}

void blink_led(long on_time, long off_time) {
  long curr_time = millis();
  if (led_on) {
    if (curr_time > led_on_timestamp + on_time) {
      digitalWrite(led_pin, LOW);
      led_off_timestamp = curr_time;
      led_on = false;
    }
  } else {
    if (curr_time > led_off_timestamp + off_time) {
      digitalWrite(led_pin, HIGH);
      led_on_timestamp = curr_time;
      led_on = true;
    }
  }
}
