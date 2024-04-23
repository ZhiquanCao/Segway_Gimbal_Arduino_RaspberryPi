#include <MsTimer2.h>
#include <Wire.h>
#include <MPU6050.h>
#include "KalmanFilter.h"

MPU6050 mpu;
KalmanFilter kalmanfilter;

int16_t ax, ay, az; // Accelerometer data
int16_t gx, gy, gz; // Gyroscope data

float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;
float kalmanfilter_angle;


void read_imu(){
  sei();

  // static unsigned long pre_time;
  // Serial.print("The time gap is ");
  // Serial.println(millis() - pre_time);
  // pre_time = millis();

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  kalmanfilter_angle = kalmanfilter.angle;
}

void setup() {
  Serial.begin(9600); // Start the Serial communication
  Wire.begin(); // Start I2C communication

  // Initialize the MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  // Check if the MPU6050 is connected
  if (mpu.testConnection()) {
    Serial.println("MPU6050 is connected.");
  } else {
    Serial.println("Failed to connect to MPU6050.");
    while (1) {
      // Loop forever if the MPU6050 connection fails
    }
  }
  MsTimer2::set(5, read_imu);
  MsTimer2::start();
}


void loop() {
  // Variables to store the data
  
  // Print the data to the Serial Monitor
  // Serial.print("Ax: "); Serial.print(ax);
  // Serial.print(" | Ay: "); Serial.print(ay);
  // Serial.print(" | Az: "); Serial.print(az);
  // Serial.print(" | Gx: "); Serial.print(gx);
  // Serial.print(" | Gy: "); Serial.print(gy);
  // Serial.print(" | Gz: "); Serial.println(gz);
  static unsigned long print_time;
  if (millis() - print_time > 100)
  {
    print_time = millis();
    Serial.print("                            Angle is ");
    Serial.println(kalmanfilter.angle);
  }

  delay(0); // Wait for a while before the next reading
}
