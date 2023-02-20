// states
#define HOMING 0b00
#define GRASPING 0b01
#define MOVING 0b10
#define RELEASING 0b11

// multitasking objects
TaskHandle_t Task1;
TaskHandle_t Task2;

// constants
const int led_pin = 2;
const int homing_button = 5;
const int grasping_button = 18;
const int moving_button = 19;
const int releasing_button = 21;

// variables
volatile int state = HOMING;
int pinState[] = {0,0,0,0};
int lastPinState[] = {0,0,0,0};
int lastDbTime = 0;

void setup() {
  Serial.begin(115200);
  pinMode(led_pin, OUTPUT); // on-board led
  pinMode(homing_button, INPUT_PULLUP); // button from input pin to ground
  pinMode(grasping_button, INPUT_PULLUP); // etc...
  pinMode(moving_button, INPUT_PULLUP);
  pinMode(releasing_button, INPUT_PULLUP);
  setup1(); // initialize core 1 and
  setup2(); // core 2
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
        break;
    }
  }
}

void loop2(void * parameter) {
  // local resources
  boolean led_on = true;
  long next_led_update_time = millis() + get_state_period(led_on);

  for (;;) {
    if (millis() > next_led_update_time) {
      // update the led
      next_led_update_time = millis() + get_state_period(led_on);
      digitalWrite(led_pin, led_on);
      led_on = !led_on;
    }

    // listen for button presses
    if (!debounceButton(5)) { // a button connect to pin 5
      state = HOMING;
      Serial.println("HOMING...");
    }
    if (!debounceButton(18)) { // a button connect to pin 18, etc..
      state = GRASPING;
      Serial.println("GRASPING...");
    }
    if (!debounceButton(19)) {
      state = MOVING;
      Serial.println("MOVING...");
    }
    if (!debounceButton(21)) {
      state = RELEASING;
      Serial.println("RELEASING...");
    }
  }
}

/**
 * This function drives the motor until
 * a limit switch indicated the motor is at "home position"
 */
void homing() {
  boolean at_home = false; // this would be the reading of a limit switch
  while (!at_home && state == HOMING) {
    // go home
  }
}

/**
 * This function drives the motor until 
 * a sensor indicates it is adequately grasped
 */
void grasping() {
  boolean grasped = false; // this might be the reading of a strain gauge
  while (!grasped && state == GRASPING) {
    // close the fingers until the object is grasped
  }
}

/**
 * I think this function just applies constant 
 * torque to the gripper until it has arrived
 */
void moving() {
  boolean arrived = false; // this might be the reading of a proximity sensor
  while (!arrived && state == MOVING) {
    // wait until the gripper has arrived
  }
}

/**
 * This function drives the motor until 
 * a sensor indicates there is enough clearance
 */
void releasing() {
  boolean released = false; // this might be the reading of a strain gauge
  while (!released && state == RELEASING) {
    // open the fingers until the object is released
  }
}

/**
 * return the time until 
 * the led needs to be updated
 */
long get_state_period(boolean ontime) {
  switch (state) {
    case GRASPING:
      return ontime ? 100 : 900;
    case MOVING:
      return ontime ? 400 : 600;
      break;
    case RELEASING:
      return ontime ? 999 : 1;
      break;
    case HOMING:
      return ontime ? 700 : 300;
      break;
    default:
      return 100;
      break;
  }
}

/**
 * return 0 if pin is pulled low
 * else, return 1
 */
int debounceButton(int pin)
{
  int reading = digitalRead(pin);

  // overwrite pin with corresponding array index 
  switch (pin) {
    case homing_button:
      pin = 0;
      break;
    case grasping_button:
      pin = 1;
      break;
    case moving_button:
      pin = 2;
      break;
    case releasing_button:
      pin = 3;
      break;
  }

  // button state has changed
  if (reading != lastPinState[pin])
    lastDbTime = millis();
  lastPinState[pin] = reading;

  // see if new button state remains for longer than 50ms
  if ((millis() - lastDbTime) > 50) {
    if (reading != pinState[pin]) {
      pinState[pin] = reading;
      return reading; // 1 on release, 0 on press
    }
  }
  return 1;
}
