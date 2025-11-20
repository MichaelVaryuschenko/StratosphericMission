#include <GyverPWM.h>

//Voltage stabilizer PID
int32_t voltage = 0;
int32_t target_voltage = 350;
int32_t err = 175;
int32_t err_i = 0;
int32_t err_old = 0;
int pulse_width = 25;
const float k_p = 0.1;
const float k_i = 0.01;
const float k_d = 0;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(9, OUTPUT);
  PWM_frequency(9, 20500, FAST_PWM);
  PWM_set(9, pulse_width);
  Serial.println("MC setup is completed");
}

void loop() {
  // put your main code here, to run repeatedly:
  voltage = (float)analogRead(A0) / 1023 * 430;
  err = target_voltage - voltage;
  pulse_width += k_p * (target_voltage - voltage)  + k_i * err_i + k_d * (err - err_old);
  if(pulse_width > 128)
    pulse_width = 128;
  PWM_set(9, pulse_width);
  Serial.print("Dosimeter voltage: ");
  Serial.println(voltage);
  Serial.print("PWM fill coefficient: ");
  Serial.println((float)pulse_width / 255);
  Serial.print("Error: ");
  Serial.println(err);
  Serial.print("Integral error: ");
  Serial.println(err_i);
  err_old = err;
  err_i += min(err, 20);
  Serial.println("---------------------------");
  delay(3000);
}
