const int led_pin = 2;
hw_timer_t *My_timer = NULL;

void IRAM_ATTR onTimer() {
  Seriprint(millis());
}

void setup() {
  pinMode(led_pin, OUTPUT);
  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 250000, true);
  timerAlarmEnable(My_timer);
}

void loop(){}
