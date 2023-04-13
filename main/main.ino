#include <SPI.h>
#include <MFRC522.h>
#include "FastIMU.h"

/* STATES */
#define HOMING      0b00
#define GRASPING    0b01
#define MOVING      0b10
#define RELEASING   0b11

/* PINS */
#define LED               2     // on-board LED 
#define ENC_A             3     // A channel of encoder
#define ENC_B             4     // B channel of encoder
#define LIMIT_SWITCH      34    // for homing
#define SENSOR_PIN        7     // proximinity
#define HBRIDGE_FORWARD   26    // First motor driver pin
#define HBRIDGE_REVERSE   24    // Second motor driver pin
#define AUDIO_STATE       29    // State sent through audio jack
#define RESET_HOMING      30    // Reset coming from audio jack

// TODO
#define RST_PIN  5             // RFID pin
#define SS_PIN      10            // RFID pin
// RFID/MPU6050 pins  ESP32 pins
// Vcc -              3.3V
// GND -              GND
// MISO -             38 (D19)
// MOSI -             36 (D23)
// SCK -              35 (D18)  
// SDA -              34 (D6)
// SCL -              39 (D22)

/* CONSTANTS */
#define IMU_ADDRESS 0x68
#define PULSES_PER_REV 700
#define UPDATE_DISPLAY_INTERVAL 50
String origin_uid = "9145341d";
String dest_uid = "904ad626";

/* OBJECTS */
TaskHandle_t task1;
TaskHandle_t task2;
MPU6050 IMU;
calData calib = { 0 };
GyroData gyroData;
MFRC522 rfid(SS_PIN, RST_PIN);  //create instance of MFRC522

/* VARIABLES */
int state = HOMING;                       // global FSM state
volatile bool at_home;
volatile long motor_position = 0;             // set in ISR
long last_motor_position = 0;
int pin_state[] = {0,0,0,0};              // rest used for debouncing button presses
int last_pin_state[] = {0,0,0,0};
unsigned long last_debounce_time = 0; 

/* motor_position ISR */
void read_encoder() {
  if ((bool)digitalRead(ENC_B)) {
    motor_position++;
  }
  else {
    motor_position--;
  }
}

/* at_home ISR */
void read_limit_switch() {
  at_home = (bool)digitalRead(LIMIT_SWITCH);
}

void read_reset_pin() {
    ESP.restart();
}

// call in setup()
void setupIMU() {
  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true);
  }

  // calibrate the IMU
  IMU.calibrateAccelGyro(&calib);
  delay(1000);

  // begin communication
  IMU.init(calib, IMU_ADDRESS);
  delay(1000);

  err = IMU.setGyroRange(500);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true);
  }
}

void setup() {
  Serial.begin(115200);

  // buttons
  pinMode(LIMIT_SWITCH, INPUT_PULLUP);

  // inputs
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(RESET_HOMING, INPUT);

  // outputs
  pinMode(LED, OUTPUT);
  pinMode(HBRIDGE_FORWARD, OUTPUT);
  pinMode(HBRIDGE_REVERSE, OUTPUT);
  pinMode(AUDIO_STATE, OUTPUT);

  //setup core 1 and 2 on ESP32
  setup_core1();
  setup_core2();

  SPI.begin();        // initialize SPI bus
  rfid.PCD_Init();     // initialize MFRC522
  setupIMU();
  
  attachInterrupt(digitalPinToInterrupt(ENC_A), read_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RESET_HOMING), read_reset_pin, RISING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH), read_limit_switch, CHANGE);
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
  stop_motor();

  if (!at_home) {
    drive_motor_reverse(255); // half max speed
  }
  while (!at_home);
  stop_motor();

  drive_motor_forward(255); // max speed
  while (at_home);
  delay(100); // overshoot
  stop_motor();

  drive_motor_reverse(255/4); // quarter max speed
  while(!at_home);
  stop_motor();

  motor_position = 0;
  state = GRASPING;
}

/**
 * drive the motor until the item is grasped
 */
void grasping() {
  drive_motor_forward(255);
  while(motor_position < 5 * PULSES_PER_REV);
  stop_motor();
  delay(100);
  while(read_RFID() != 1 );
  drive_motor_reverse(255);
  while (true) {
    if ((bool)digitalRead(SENSOR_PIN))
      break;
  }
  stop_motor();
  state = MOVING;
}

/**
 * drive the motor until arrived at destination
 */
void moving() {
  while(read_RFID() != -1 );
  state = RELEASING;
}

/**
 * drive the motor until the item is released
 */
void releasing() {
  drive_motor_forward(255);
  while (true) {
    if (!(bool)digitalRead(SENSOR_PIN))
      break;
  }
  delay(100);
  stop_motor();
  state = -1;  
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

void drive_motor_forward(int speed) {
  digitalWrite(HBRIDGE_FORWARD, LOW);
  if (speed < 0)
    return analogWrite(HBRIDGE_REVERSE, LOW);
  else if (speed > 255)
    return analogWrite(HBRIDGE_REVERSE, HIGH);
  analogWrite(HBRIDGE_REVERSE, speed);
}

void drive_motor_reverse(int speed) {
  digitalWrite(HBRIDGE_FORWARD, LOW);
  if (speed < 0)
    return analogWrite(HBRIDGE_REVERSE, LOW);
  else if (speed > 255)
    return analogWrite(HBRIDGE_REVERSE, HIGH);
  analogWrite(HBRIDGE_REVERSE, speed);
}

void stop_motor() {
  digitalWrite(HBRIDGE_FORWARD, LOW);
  digitalWrite(HBRIDGE_REVERSE, LOW);
}

int read_RFID()
{
  // Look for new cards
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    // Get the UID of the card
    String uid = "";
    for (byte i = 0; i < rfid.uid.size; i++) {
      uid += String(rfid.uid.uidByte[i], HEX);
    }

    // Check if card is origin or destination
    if (uid == origin_uid) {
      rfid.PICC_HaltA(); // Stop reading
      rfid.PCD_StopCrypto1(); // Stop encryption on PCD
      return 1;
    } else if (uid == dest_uid) {
      rfid.PICC_HaltA(); // Stop reading
      rfid.PCD_StopCrypto1(); // Stop encryption on PCD
      return -1;
    } else {
      rfid.PICC_HaltA(); // Stop reading
      rfid.PCD_StopCrypto1(); // Stop encryption on PCD
      return 0;
    }
  }
}

bool isStationary() {
  return (abs(gyroData.gyroX) < 1) && (abs(gyroData.gyroY) < 1) && (abs(gyroData.gyroZ) < 1);
}