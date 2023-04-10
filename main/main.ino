/* CONSTANTS */
#define HOMING                    0b00
#define GRASPING                  0b01
#define MOVING                    0b10
#define RELEASING                 0b11
#define PULSES_PER_REV            948
#define UPDATE_DISPLAY_INTERVAL   50
#define MOTOR_PWM_CHANNEL_1       0
#define MOTOR_PWM_CHANNEL_2       1
#define FREQUENCY                 1000
// #define CONTROL_FREQUENCY 100 // not sure if this is needed for our method of position sensing

/* PINS */
#define LED               2     // on-board LED
#define ENC_A             3     // A channel of encoder
#define ENC_B             4     // B channel of encoder
#define ITEM_SENSOR       13    // camera within gripper
#define ITEM_FLAG         14    // item flag detection
#define DESTINATION_FLAG  33    // destination flag detection
#define LIMIT_SWITCH      34    // for homing
#define PWM_1             25    // First motor driver pin
#define PWM_2             24    // Second motor driver pin
#define HOMING_BUTTON     5     // for debugging
#define GRASPING_BUTTON   18    // for debugging
#define MOVING_BUTTON     19    // for debugging
#define RELEASING_BUTTON  21    // for debugging

/* MULTITASKING OBJECTS*/
TaskHandle_t task1;
TaskHandle_t task2;
// hw_timer_t * timer0; not sure if this is needed for our method of position sensing

/* VARIABLES */
int state = HOMING;                       // global FSM state
volatile long motor_position = 0;         // set in ISR
long last_motor_position = 0;
int pin_state[] = {0,0,0,0};              // rest used for debouncing button presses
int last_pin_state[] = {0,0,0,0};
unsigned long last_debounce_time = 0;

/* ISR */
void read_encoder() {
  if ((bool)digitalRead(ENC_B)) {
    motor_position++;
  }
  else {
    motor_position--;
  }
}

void setup() {
  Serial.begin(115200);

  // inputs
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(ITEM_SENSOR, INPUT);
  pinMode(ITEM_FLAG, INPUT);
  pinMode(DESTINATION_FLAG, INPUT);
  pinMode(LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(HOMING_BUTTON, INPUT_PULLUP);
  pinMode(GRASPING_BUTTON, INPUT_PULLUP);
  pinMode(MOVING_BUTTON, INPUT_PULLUP);
  pinMode(RELEASING_BUTTON, INPUT_PULLUP);

  // outputs
  pinMode(LED, OUTPUT);
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);

  //configure motor PWM functionalities
  ledcSetup(MOTOR_PWM_CHANNEL_1, FREQUENCY, 8);
  ledcSetup(MOTOR_PWM_CHANNEL_2, FREQUENCY, 8);

  ledcAttachPin(PWM_1, MOTOR_PWM_CHANNEL_1);
  ledcAttachPin(PWM_2, MOTOR_PWM_CHANNEL_2);

  ledcWrite(MOTOR_PWM_CHANNEL_1, 0);
  ledcWrite(MOTOR_PWM_CHANNEL_2, 0);

  //setup core 1 and 2 on ESP32
  setup_core1();
  setup_core2();

  // ISR (I think we need to use control frequency instead)
  attachInterrupt(digitalPinToInterrupt(ENC_A), read_encoder, RISING);
}

void setup_core1() {
  xTaskCreatePinnedToCore(
    loop1,    // function to implement the task
    "task1",  // name of the task 
    10000,    // stack size in words 
    NULL,     // task input parameter 
    0,        // priority of the task 
    &task1,   // task handle
    0);       // core where the task should run
}

void setup_core2() {
  xTaskCreatePinnedToCore(loop2, "task2", 10000, NULL, 1, &task2, 1);
  // setup_timer0(); // pin timer0 to core 2; not sure if this is needed for our method of position sensing
}

// not sure if this is needed for our method of position sensing
// /**
//  * documentation can be found at:
//  * https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/timer.html
//  */
// void setup_timer0() {
//   timer0 = timerBegin(0, 80, true);
//   timerAttachInterrupt(timer0, &read_encoder, true);
//   timerAlarmWrite(timer0, 1000000/CONTROL_FREQUENCY, true);
//   timerAlarmEnable(timer0);
// }

void loop() {}

/**
 * Motor Controller
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
  unsigned long curr_time = 0;
  unsigned long next_update_led_time = 0;
  unsigned long next_update_display_time = 0;

  for (;;) {
    curr_time = millis();

    // check if led needs to be updated
    if (curr_time > next_update_led_time) {
      next_update_led_time = curr_time + get_led_interval(update_led());
    }

    // update the display
    if (curr_time > next_update_display_time) {
      next_update_display_time = curr_time + UPDATE_DISPLAY_INTERVAL;
      update_display();
    }

    // check for button presses
    if (!debounceButton(curr_time, HOMING_BUTTON, 0)) {
      state = HOMING;
    }
    if (!debounceButton(curr_time, GRASPING_BUTTON, 1)) {
      state = GRASPING;
    }
    if (!debounceButton(curr_time, MOVING_BUTTON, 2)) {
      state = MOVING;
    }
    if (!debounceButton(curr_time, RELEASING_BUTTON, 3)) {
      state = RELEASING;
    }
  }
}

/**
 * updates the LED
 */
bool update_led() {
  bool led_state = digitalRead(LED);
  digitalWrite(LED, !led_state);
  return led_state;
}

/**
 * return the time until the led needs to be updated
 */
long get_led_interval(bool led_state) {
  switch (state) {
    case HOMING:
      return led_state ? 300 : 700; // 70% duty cycle
    case GRASPING:
      return led_state ? 900 : 100; // 10% duty cycle
    case MOVING:
      return led_state ? 600 : 400; // 40% duty cycle
    case RELEASING:
      return led_state ? 0 : 1000; // 100% duty cycle
    default:
      return 0;
  }
}

void update_display() {
  long angle = ((long)((float)motor_position / 948. * 360) % 360);
  if (motor_position < 0) {
    angle = 360 + angle;
  }
  long num_steps = abs(motor_position - last_motor_position);
  int direction = (motor_position > last_motor_position) ? 1 : -1;
  float rpm = (float)num_steps / (float)PULSES_PER_REV * 60000. / (float)UPDATE_DISPLAY_INTERVAL;
  last_motor_position = motor_position;

  // print the state
  Serial.print("STATE: ");
  switch (state) {
    case GRASPING:
      Serial.print("GRASPING");
      break;
    case MOVING:
      Serial.print("MOVING");
      break;
    case RELEASING:
      Serial.print("RELEASING");
      break;
    case HOMING:
      Serial.print("HOMING");
      break;
    default:
      Serial.print("UNDEFINED");
      break;
  }
  Serial.print('\t');

  //update the display
  Serial.print("POSITION: ");
  Serial.print(motor_position);
  Serial.print('\t');
  Serial.print("ANGLE: ");
  Serial.print(angle);
  Serial.print('\t');
  Serial.print(" SPEED: ");
  Serial.print(rpm);
  Serial.print(" RPM ");
  Serial.println((direction == 1) ? "CCW" : "CW");
}

/**
 * drive the motor until the limit switch closes
 */
void homing() {
  bool at_home = false;
  while (!at_home && state == HOMING) {
    ledcWrite(MOTOR_PWM_CHANNEL_1, 0);
    ledcWrite(MOTOR_PWM_CHANNEL_2, 0);
    at_home = !(bool)digitalRead(LIMIT_SWITCH);
  }
  digitalWrite(MOTOR_PIN_1, LOW);
  motor_position = 0;
}

/**
 * drive the motor until the item is grasped
 */
void grasping() {
  bool grasped = false;
  bool item_is_graspable = false;
  while (!grasped && state == GRASPING) {
    item_is_graspable = (bool)digitalRead(ITEM_SENSOR);
    if (item_is_graspable) {
      // TODO: close the gripper until the object is grasped 
      // TODO: need some way to detect when object is grasped, closing action is complete
      ledcWrite(MOTOR_PWM_CHANNEL_1, 127);
      ledcWrite(MOTOR_PWM_CHANNEL_2, 0);
    } else {
      // TODO: stop the motor. maybe open the fingers if needed? 
      ledcWrite(MOTOR_PWM_CHANNEL_1, 0);
      ledcWrite(MOTOR_PWM_CHANNEL_2, 0);
    }
    grasped = false; // TODO: implement detection for this
  }
}

/**
 * drive the motor until arrived at destination
 */
void moving() {
  bool arrived = false;
  while (!arrived && state == MOVING) {
    // TODO: wait until the gripper has arrived
    arrived = (bool)digitalRead(DESTINATION_FLAG);
  }
}

/**
 * drive the motor until the item is released
 */
void releasing() {
  bool released = false;
  while (!released && state == RELEASING) {
    // TODO: open the fingers until the object is released //done
    // TODO: need to detect when iris is completely open
    ledcWrite(MOTOR_PWM_CHANNEL_1, 0);
    ledcWrite(MOTOR_PWM_CHANNEL_2, 127);
    released = !(bool)digitalRead(LIMIT_SWITCH); // TODO: implement this
    //note on limit switch: shouldn't require debounce, is quite reliable when tested
    //imo should just completely open the thing, no need to detect when object has been let go
  }
}

/**
 * return 0 if button is pressed; else return 1
 */
int debounceButton(unsigned long curr_time, int pin, int index)
{
  int reading = digitalRead(pin);

  // button state has changed
  if (reading != last_pin_state[index])
    last_debounce_time = millis();
  last_pin_state[index] = reading;

  // see if new button state remains for longer than 50ms
  if ((curr_time - last_debounce_time) > 50) {
    if (reading != pin_state[index]) {
      pin_state[index] = reading;
      return reading; // 1 on release, 0 on press
    }
  }
  return 1; // stub
}
