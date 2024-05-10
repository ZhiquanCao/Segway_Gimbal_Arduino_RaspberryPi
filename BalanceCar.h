#include <MsTimer2.h>
#include <Wire.h>
#include <MPU6050.h>
#include "KalmanFilter.h"

#define ENCODER_RIGHT_A_PIN 19
#define ENCODER_LEFT_A_PIN 18



//Setting PID parameters

double kp_balance = 55, kd_balance = 0.75;
double kp_speed = 10, ki_speed = 0.26;
double kp_turn = 2.5, kd_turn = 0.5;

//Setting MPU6050 calibration parameters
double angle_zero = 0;            //x axle angle calibration
double angular_velocity_zero = 0; //x axle angular velocity calibration

MPU6050 mpu;
KalmanFilter kalmanfilter;

volatile unsigned long encoder_count_right_a = 0;
volatile unsigned long encoder_count_left_a = 0;
int16_t ax, ay, az, gx, gy, gz;
float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;

unsigned long turn_prev_time = 0;
unsigned long move_prev_time = 0;
double balance_control_output = 0;
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
char balance_angle_min = -12;
char balance_angle_max = 12;

void carStop()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  // digitalWrite(STBY_PIN, HIGH);
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
}

void carForward(){
  digitalWrite(BIN1, 0);
  digitalWrite(BIN2, 1);
  digitalWrite(AIN1, 1);
  digitalWrite(AIN2, 0);
  analogWrite(PWMB_RIGHT, -pwm_right);
  analogWrite(PWMA_LEFT, -pwm_left);
}

void carBackward(){
  digitalWrite(BIN1, 1);
  digitalWrite(BIN2, 0);
  digitalWrite(AIN1, 0);
  digitalWrite(AIN2, 1);
  analogWrite(PWMB_RIGHT, pwm_right);
  analogWrite(PWMA_LEFT, pwm_left);
}

void balanceCar()
{
  sei();
  encoder_left_pulse_num_speed += pwm_left < 0 ? -encoder_count_left_a : encoder_count_left_a;
  encoder_right_pulse_num_speed += pwm_right < 0 ? -encoder_count_right_a : encoder_count_right_a;
  encoder_count_left_a = 0;
  encoder_count_right_a = 0;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  kalmanfilter_angle = kalmanfilter.angle;
  balance_control_output = kp_balance * (kalmanfilter_angle - angle_zero) + kd_balance * (kalmanfilter.Gyro_x - angular_velocity_zero);

  speed_control_period_count++;
  if (speed_control_period_count >= 8)
  {
    speed_control_period_count = 0;
    double car_speed = (encoder_left_pulse_num_speed + encoder_right_pulse_num_speed) * 0.5;
    encoder_left_pulse_num_speed = 0;
    encoder_right_pulse_num_speed = 0;
    speed_filter = speed_filter_old * 0.7 + car_speed * 0.3;
    speed_filter_old = speed_filter;
    car_speed_integeral += speed_filter;
    car_speed_integeral += -setting_car_speed;
    car_speed_integeral = constrain(car_speed_integeral, -3000, 3000);
    speed_control_output = -kp_speed * speed_filter - ki_speed * car_speed_integeral;

    if (millis() - move_prev_time > 3000){
      setting_car_speed = 0;
      motion_mode = CLEAR; // clear the modes but do nothing for the car speed or turn speed
    }
    rotation_control_output = setting_turn_speed + kd_turn * kalmanfilter.Gyro_z;
    if (millis() - turn_prev_time > 1000){
      setting_turn_speed = 0;
      motion_mode = CLEAR;
    }
  }

  pwm_left = balance_control_output - speed_control_output - rotation_control_output;
  pwm_right = balance_control_output - speed_control_output + rotation_control_output;

  pwm_left = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);

  if (motion_mode != START && motion_mode != STOP && (kalmanfilter_angle < balance_angle_min || balance_angle_max < kalmanfilter_angle))
  {
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
    if (pwm_left < 0)
    {
      digitalWrite(AIN1, 1);
      digitalWrite(AIN2, 0);
      analogWrite(PWMA_LEFT, -pwm_left);
      // forward();
    }
    else
    {
      digitalWrite(AIN1, 0);
      digitalWrite(AIN2, 1);
      analogWrite(PWMA_LEFT, pwm_left);
      // backward();
    }
    if (pwm_right < 0)
    {
      digitalWrite(BIN1, 0);
      digitalWrite(BIN2, 1);
      analogWrite(PWMB_RIGHT, -pwm_right);
    }
    else
    {
      digitalWrite(BIN1, 1);
      digitalWrite(BIN2, 0);
      analogWrite(PWMB_RIGHT, pwm_right);
    }
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

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), encoderCountLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A_PIN), encoderCountRightA, CHANGE);

  Wire.begin();
  mpu.initialize();
  MsTimer2::set(5, balanceCar);
  MsTimer2::start();
}
