#define GRASPING    0b00 // 10% duty cycle
#define MOVING      0b01 // 40% duty cycle
#define RELEASING   0b10 // 100% duty cycle
#define HOMING      0b11 // 70% duty cycle

// constants
const int led_pin = 2;

// variables
volatile int state;
long led_on_timestamp;
long led_off_timestamp;
boolean led_on;

void setup() {
  pinMode(led_pin, OUTPUT);
  Serial.begin(115200);

  // initialize variables
  state = GRASPING;
  led_on_timestamp = millis();
  led_off_timestamp = millis();
  led_on = false;
}

void loop() {
  // update the led if necessary
  update_led();
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