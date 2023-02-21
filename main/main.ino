// states
#define HOMING 0b00
#define GRASPING 0b01
#define MOVING 0b10
#define RELEASING 0b11

// multitasking objects
TaskHandle_t Task1;
TaskHandle_t Task2;

// constants
const int led_pin = 2;                  // on-board LED
const int rotary_encoder_pin = 4;       // motor position sensor
const int voltage_converter_pin = 12;   // connected to VIN of LT8300 flyback converter
const int item_sensor_pin = 13;         // item detection
const int destination_sensor_pin = 14;  // destination detection
const int strain_sensor_pin = 33;       // gripper force detection
const int limit_switch_pin = 34;        // limit switch
const int homing_button = 5;            // go-to HOMING state
const int grasping_button = 18;         // go-to GRASPING state
const int moving_button = 19;           // go-to MOVING state
const int releasing_button = 21;        // go-to RELEASING state

// variables
volatile int state = HOMING;            // global FSM state
volatile int motor_position;            // reading from rotary_encoder_pin (set in ISR)
int pinState[] = {0,0,0,0};             // used for debouncing, etc...
int lastPinState[] = {0,0,0,0};
int lastDbTime = 0;

void setup() {
  Serial.begin(115200);   
  Serial.println("Hello World!");
  pinMode(led_pin, OUTPUT);
  pinMode(voltage_converter_pin, OUTPUT);
  pinMode(item_sensor_pin, INPUT);
  pinMode(destination_sensor_pin, INPUT);
  pinMode(strain_sensor_pin, INPUT);
  pinMode(limit_switch_pin, INPUT_PULLUP); // connect switch from input pin to ground
  pinMode(homing_button, INPUT_PULLUP); // connect button from input pin to ground
  pinMode(grasping_button, INPUT_PULLUP); // connect button from input pin to ground
  pinMode(moving_button, INPUT_PULLUP); // connect button from input pin to ground
  pinMode(releasing_button, INPUT_PULLUP); // connect button from input pin to ground
  setup1(); // initialize core 1
  setup2(); // initialize core 2
  Serial.println("Here we go!");
}

void setup1() {
  xTaskCreatePinnedToCore(
    loop1,    // Function to implement the task
    "Task1",  // Name of the task 
    10000,    // Stack size in words 
    NULL,     // Task input parameter 
    0,        // Priority of the task 
    &Task1,   // Task handle. 
    0);       // Core where the task should run
}

void setup2() {
  xTaskCreatePinnedToCore(
    loop2,
    "Task2", 
    10000,  
    NULL,  
    1,  
    &Task2,  
    1); 
}

void loop() {}

/**
 * FSM Controller
 */
void loop1(void * parameter) {
  for (;;) {
    switch (state) {
      case HOMING:
        homing();
        break;
      case GRASPING:
        grasping();
        break;
      case MOVING:
        moving();
        break;
      case RELEASING:
        releasing();
        break;
      default:
        Serial.println("Something went wrong!");
        exit(0);
    }
  }
}

/**
 * Interface Controller
 */
void loop2(void * parameter) {
  // local resources
  bool led_on = true;
  long next_led_update_time = millis() + get_state_period(led_on);

  for (;;) {
    // check if led needs to be updated
    if (millis() > next_led_update_time) {
      next_led_update_time = millis() + get_state_period(led_on);
      digitalWrite(led_pin, led_on);
      led_on = !led_on;
    }

    // check for button presses
    if (!debounceButton(homing_button, 0)) {
      state = HOMING;
      Serial.println("HOMING...");
    }
    if (!debounceButton(grasping_button, 1)) {
      state = GRASPING;
      Serial.println("GRASPING...");
    }
    if (!debounceButton(moving_button, 2)) {
      state = MOVING;
      Serial.println("MOVING...");
    }
    if (!debounceButton(releasing_button, 3)) {
      state = RELEASING;
      Serial.println("RELEASING...");
    }
  }
}

/**
 * drive the motor until the limit switch closes
 */
void homing() {
  bool at_home = false;
  while (!at_home && state == HOMING) {
    // go home
    at_home = !digitalRead(limit_switch_pin);
  }
  motor_position = 0;
}

/**
 * drive the motor until the item is grasped
 */
void grasping() {
  bool grasped = false;
  bool item_is_graspable = false;
  while (!grasped && state == GRASPING) {
    item_is_graspable = !digitalRead(item_sensor_pin); // && motor_position < 10*360;
    if (item_is_graspable) {
      // close the fingers until the object is grasped
    } else {
      // stop the motor. maybe open the fingers if needed?
    }
    grasped = !digitalRead(strain_sensor_pin);
  }
}

/**
 * drive the motor until arrived at destination
 */
void moving() {
  bool arrived = false;
  while (!arrived && state == MOVING) {
    // wait until the gripper has arrived
    arrived = !digitalRead(destination_sensor_pin); // true on falling edge
  }
}

/**
 * drive the motor until the item is released
 */
void releasing() {
  bool released = false;
  while (!released && state == RELEASING) {
    // open the fingers until the object is released
    released = digitalRead(strain_sensor_pin); // complement reading to `grasped` in grasping()
  }
}

/**
 * return the time until the led needs to be updated
 */
long get_state_period(bool ontime) {
  switch (state) {
    case GRASPING:
      return ontime ? 100 : 900; // 10% duty cycle
    case MOVING:
      return ontime ? 400 : 600; // 40% duty cycle
    case RELEASING:
      return ontime ? 999 : 1; // 100% duty cycle
    case HOMING:
      return ontime ? 700 : 300; // 70% duty cycle
    default:
      return 0;
  }
}

/**
 * return 0 if button is pressed; else return 1
 */
int debounceButton(int pin, int index)
{
  int reading = digitalRead(pin);

  // button state has changed
  if (reading != lastPinState[index])
    lastDbTime = millis();
  lastPinState[index] = reading;

  // see if new button state remains for longer than 50ms
  if ((millis() - lastDbTime) > 50) {
    if (reading != pinState[index]) {
      pinState[index] = reading;
      return reading; // 1 on release, 0 on press
    }
  }
  return 1; // stub
}
