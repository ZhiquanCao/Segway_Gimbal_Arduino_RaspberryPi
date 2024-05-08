#include "Pins.h"
#include "mode.h"
#include "BalanceCar.h"

void setup()
{
  Serial.begin(9600);
  carInitialize();
  // Serial.print("jj");
}

void loop(){
  // Serial.print("kalmanfilter_angle is ");
  // Serial.println(kalmanfilter_angle);
  // Serial.print("balance_control_output is ");
  // Serial.println(balance_control_output);
  // Serial.print("AIN1 is ");
  // Serial.println(AIN1);

  // Serial.print("              pwm diff is ");
  // Serial.println(pwm_left-pwm_right);
  // Serial.print("                          pwm_right is ");
  // Serial.println(pwm_right);
    // static unsigned long print_time;
    // if (millis() - print_time > 100)
    // {
    //     print_time = millis();
    //     Serial.print("                            Angle is ");
    //     Serial.println(kalmanfilter.angle);
    // }
}