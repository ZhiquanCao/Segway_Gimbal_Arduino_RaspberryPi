// #include <MsTimer2.h>
// #include <Wire.h>
// #include <MPU6050.h>
// #include "KalmanFilter.h"

// #define ENCODER_RIGHT_A_PIN 19
// #define ENCODER_LEFT_A_PIN 18

// int motor1pin1 = 3;
// int motor1pin2 = 4;
// int pwmA = 5;

// int motor2pin1 = 8;
// int motor2pin2 = 9;
// int pwmB = 10;

// volatile unsigned long encoder_count_right_a = 0;
// volatile unsigned long encoder_count_left_a = 0;

// MPU6050 mpu;
// KalmanFilter kalmanfilter;

// int16_t ax, ay, az; // Accelerometer data
// int16_t gx, gy, gz; // Gyroscope data

// float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;
// float kalmanfilter_angle;



// void encoderCountRightA()
// {
//   encoder_count_right_a++;
// }

// void encoderCountLeftA()
// {
//   encoder_count_left_a++;
// }

// void read_imu(){
//   sei();
//   mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//   kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
//   kalmanfilter_angle = kalmanfilter.angle;
// }

// void setup() {
//   Serial.begin(9600); // Start the Serial communication
//   Wire.begin(); // Start I2C communication

//   // Initialize the MPU6050
//   Serial.println("Initializing MPU6050...");
//   mpu.initialize();

//   pinMode(motor1pin1, OUTPUT);
//   pinMode(motor1pin2, OUTPUT);
//   pinMode(motor2pin1, OUTPUT);
//   pinMode(motor2pin2, OUTPUT);

//   attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), encoderCountLeftA, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A_PIN), encoderCountRightA, CHANGE);

//   MsTimer2::set(5, read_imu);
//   MsTimer2::start();
// }


// void forward(int speed) {
//   digitalWrite(motor1pin1, LOW);
//   digitalWrite(motor1pin2, HIGH);
//   analogWrite(pwmA, speed);
//   digitalWrite(motor2pin1, HIGH);
//   digitalWrite(motor2pin2, LOW);
//   analogWrite(pwmB, speed);
// }

// void backward(int speed) {
//   digitalWrite(motor1pin1, HIGH);
//   digitalWrite(motor1pin2, LOW);
//   analogWrite(pwmA, speed);
//   digitalWrite(motor2pin1, LOW);
//   digitalWrite(motor2pin2, HIGH);
//   analogWrite(pwmB, speed);
// }

// void loop() {
// //   static unsigned long print_time;
// //   if (millis() - print_time > 100)
// //   {
// //     print_time = millis();
// //     Serial.print("                            Angle is ");
// //     Serial.println(kalmanfilter.angle);
// //   }

//   delay(0); // Wait for a while before the next reading
// }
