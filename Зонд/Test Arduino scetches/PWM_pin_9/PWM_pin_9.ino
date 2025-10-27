#include <GyverPWM.h>

void setup() {
  // put your setup code here, to run once:
  pinMode(9, OUTPUT);
  PWM_frequency(9, 20500, FAST_PWM);
  PWM_set(9, 128);
}

void loop() {
  // put your main code here, to run repeatedly:
}
