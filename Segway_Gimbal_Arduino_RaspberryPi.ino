#include "Pins.h"
#include "mode.h"
#include "BalanceCar.h"

char key_value = '\0';
unsigned long start_prev_time = 0;
unsigned long trig_prev_time = 0;
bool stop_flag = false;

void keyEventHandle()
{
  if (key_value != '\0')
  {
    switch (key_value)
    {
    case 's':
      motion_mode = STANDBY;
      key_value = "\0";
      break;
    case 'f':
      motion_mode = FORWARD;
      key_value = "\0";
      break;
    case 'b':
      motion_mode = BACKWARD;
      key_value = "\0";
      break;
    case 'l':
      turn_prev_time = millis();
      motion_mode = TURNLEFT;
      key_value = "\0";
      break;
    case 'r':
      turn_prev_time = millis();
      motion_mode = TURNRIGHT;
      key_value = "\0";
      break;
    default:
      break;
    }
  }
}

void setMotionState()
{
  switch (motion_mode)
  {
  case FORWARD:
    setting_car_speed = 20;
    setting_turn_speed = 0;
    break;
  case BACKWARD:
    setting_car_speed = -20;
    setting_turn_speed = 0;
    break;
  case TURNLEFT:
    setting_car_speed = 0;
    setting_turn_speed = 50;
    break;
  case TURNRIGHT:
    setting_car_speed = 0;
    setting_turn_speed = -50;
    break;
  case STANDBY:
    setting_car_speed = 0;
    setting_turn_speed = 0;
    break;
  case STOP:
    setting_car_speed = 0;
    setting_turn_speed = 0;
    // if (millis() - start_prev_time > 1000)
    // {
    //   if (balance_angle_min <= kalmanfilter_angle && kalmanfilter_angle <= balance_angle_max)
    //   {
    //     motion_mode = STANDBY;
    //   }
    // }
    break;
  case START:
    if (millis() - start_prev_time > 2000)
    {
      start_prev_time = millis();
      if (balance_angle_min <= kalmanfilter_angle && kalmanfilter_angle <= balance_angle_max)
      {
        car_speed_integeral = 0;
        setting_car_speed = 0;
        motion_mode = STANDBY;
      }
      else
      {
        motion_mode = STOP;
        carStop();
      }
    }
    break;
  default:
    break;
  }
}

void setup()
{
  Serial.begin(9600);
  start_prev_time = millis();
  trig_prev_time = millis();
  delay(1000);
  carInitialize();
}

void loop(){
  if (Serial.available() > 0) {  // Check if data is available to read
    key_value = Serial.read();  // Read the incoming byte
    // Serial.print("I received: ");
    // Serial.println(inputData);
  }
  keyEventHandle();
  setMotionState();
  // Serial.print("pwm_left is ");
  // Serial.println(pwm_left);
  if (millis() - trig_prev_time > 500) {
    trig_prev_time = millis();
    if (!digitalRead(TRIG_PIN)){
      
      stop_flag = !stop_flag;
      // Serial.print("stop ");
      // Serial.println(stop_flag);
      // Serial.println("Pressed");
      motion_mode = stop_flag? STOP : START;
      // Serial.print("Motion Mode ");
      // Serial.println(motion_mode);
      // Serial.println(motion_mode);
    }
  }
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