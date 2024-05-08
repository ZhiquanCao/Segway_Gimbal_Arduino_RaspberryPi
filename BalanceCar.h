/*
 * @Description: In User Settings Edit
 * @Author: your name
 * @Date: 2019-10-08 09:35:07
 * @LastEditTime: 2019-10-11 16:25:04
 * @LastEditors: Please set LastEditors
 */
#include <MsTimer2.h>
#include <Wire.h>
#include <MPU6050.h>
#include "KalmanFilter.h"

#define ENCODER_RIGHT_A_PIN 19
#define ENCODER_LEFT_A_PIN 18

// int AIN1 = 3;
// int AIN2 = 4;
// int PWMA_LEFT = 5;

// int BIN1 = 8;
// int BIN2 = 9;
// int PWMB_RIGHT = 10;

MPU6050 mpu;
KalmanFilter kalmanfilter;

double balance_control_output = 0;

//Setting PID parameters

double kp_balance = 55, kd_balance = 0.75;
double kp_speed = 10, ki_speed = 0.26;
double kp_turn = 2.5, kd_turn = 0.5;

//Setting MPU6050 calibration parameters
double angle_zero = 0;            //x axle angle calibration
double angular_velocity_zero = 0; //x axle angular velocity calibration

volatile unsigned long encoder_count_right_a = 0;
volatile unsigned long encoder_count_left_a = 0;
int16_t ax, ay, az, gx, gy, gz;
float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;

int encoder_left_pulse_num_speed = 0;
int encoder_right_pulse_num_speed = 0;
double speed_control_output = 0;
double rotation_control_output = 0;
double speed_filter = 0;
int speed_control_period_count = 0;
double car_speed_integeral = 0;
double speed_filter_old = 0;
int setting_car_speed = 0;
int setting_turn_speed = 0;
double pwm_left = 0;
double pwm_right = 0;
float kalmanfilter_angle;
// char balance_angle_min = -27;
// char balance_angle_max = 27;
char balance_angle_min = -12;
char balance_angle_max = 12;

void carStop()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  // digitalWrite(STBY_PIN, HIGH);
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
}

void carForward(unsigned char speed)
{
  digitalWrite(AIN1, 0);
  digitalWrite(AIN2, 1);
  digitalWrite(BIN1, 0);
  digitalWrite(BIN2, 1);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, speed);
}

void carBack(unsigned char speed)
{
  digitalWrite(AIN1, 1);
  digitalWrite(AIN2, 0);
  digitalWrite(BIN1, 1);
  digitalWrite(BIN2, 0);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, speed);
}

void forward(){
  digitalWrite(BIN1, 0);
  digitalWrite(BIN2, 1);
  digitalWrite(AIN1, 1);
  digitalWrite(AIN2, 0);
  analogWrite(PWMB_RIGHT, -pwm_right);
  analogWrite(PWMA_LEFT, -pwm_left);
}

void backward(){
  digitalWrite(BIN1, 1);
  digitalWrite(BIN2, 0);
  digitalWrite(AIN1, 0);
  digitalWrite(AIN2, 1);
  analogWrite(PWMB_RIGHT, pwm_right);
  analogWrite(PWMA_LEFT, pwm_left);
}

void balanceCar()
{
  // Serial.println("kkk");
  sei();
  // Serial.print("encoder_count_left_a is ");
  // Serial.println(encoder_count_left_a);
  // Serial.print("encoder_count_right_a is ");
  // Serial.println(encoder_count_right_a);
  encoder_left_pulse_num_speed += pwm_left < 0 ? -encoder_count_left_a : encoder_count_left_a;
  encoder_right_pulse_num_speed += pwm_right < 0 ? -encoder_count_right_a : encoder_count_right_a;
  Serial.print("encoder_count_left_a is ");
  Serial.println(encoder_count_left_a);
  Serial.print("encoder_count_right_a is ");
  Serial.println(encoder_count_right_a);
  encoder_count_left_a = 0;
  encoder_count_right_a = 0;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  kalmanfilter_angle = kalmanfilter.angle;
  // Serial.print("kalmanfilter_angle is ");
  // Serial.println(kalmanfilter_angle);
  // Serial.print("kalmanfilter.Gyro_x - angular_velocity_zero is ");
  // Serial.println(kalmanfilter.Gyro_x - angular_velocity_zero);
  balance_control_output = kp_balance * (kalmanfilter_angle - angle_zero) + kd_balance * (kalmanfilter.Gyro_x - angular_velocity_zero);
  // Serial.print("balance_control_output is ");
  // Serial.println(balance_control_output);

  speed_control_period_count++;
  if (speed_control_period_count >= 8)
  {
    speed_control_period_count = 0;
    double car_speed = (encoder_left_pulse_num_speed + encoder_right_pulse_num_speed) * 0.5;
    Serial.print("car_speed is ");
    Serial.println(car_speed);
    encoder_left_pulse_num_speed = 0;
    encoder_right_pulse_num_speed = 0;
    speed_filter = speed_filter_old * 0.7 + car_speed * 0.3;
    speed_filter_old = speed_filter;
    car_speed_integeral += speed_filter;
    car_speed_integeral += -setting_car_speed;
    car_speed_integeral = constrain(car_speed_integeral, -3000, 3000);
    speed_control_output = -kp_speed * speed_filter - ki_speed * car_speed_integeral;
    // Serial.print("speed_control_output is ");
    // Serial.println(speed_control_output);

    // rotation_control_output = setting_turn_speed + kd_turn * kalmanfilter.Gyro_z;
  }

  pwm_left = balance_control_output - speed_control_output - rotation_control_output;
  pwm_right = balance_control_output - speed_control_output + rotation_control_output;

  // Serial.print("original pwm_left is ");
  // Serial.println(pwm_left);
  pwm_left = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);

  // Serial.print("kalman filter is ");
  // Serial.println(kalmanfilter_angle);
  if (motion_mode != START && motion_mode != STOP && (kalmanfilter_angle < balance_angle_min || balance_angle_max < kalmanfilter_angle))
  {
    Serial.println("STOPPPP!");
    motion_mode = STOP;
    carStop();
  }

  if (motion_mode == STOP)
  {
    car_speed_integeral = 0;
    setting_car_speed = 0;
    pwm_left = 0;
    pwm_right = 0;
    carStop();
  } else
  if (motion_mode == STOP)
  {
    car_speed_integeral = 0;
    setting_car_speed = 0;
    pwm_left = 0;
    pwm_right = 0;
  }
  else
  {
    // Serial.print("              pwm_left is ");
    // Serial.println(pwm_left);
    // Serial.print("                          pwm_right is ");
    // Serial.println(pwm_right);
    if (pwm_left < 0)
    {
      // digitalWrite(AIN1, 1);
      // digitalWrite(AIN2, 0);
      // analogWrite(PWMA_LEFT, -pwm_left);
      forward();
    }
    else
    {
      // digitalWrite(AIN1, 0);
      // digitalWrite(AIN2, 1);
      // analogWrite(PWMA_LEFT, pwm_left);
      backward();
    }
    // if (pwm_right < 0)
    // {
    //   digitalWrite(BIN1, 0);
    //   digitalWrite(BIN2, 1);
    //   analogWrite(PWMB_RIGHT, -pwm_right);
    // }
    // else
    // {
    //   digitalWrite(BIN1, 1);
    //   digitalWrite(BIN2, 0);
    //   analogWrite(PWMB_RIGHT, pwm_right);
    // }
  }
}

void encoderCountRightA()
{
  encoder_count_right_a++;
}

void encoderCountLeftA()
{
  encoder_count_left_a++;
}

void carInitialize()
{
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA_LEFT, OUTPUT);
  
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB_RIGHT, OUTPUT);

  Wire.begin();
  mpu.initialize();
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), encoderCountLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A_PIN), encoderCountRightA, CHANGE);
  MsTimer2::set(5, balanceCar);
  MsTimer2::start();
}
