/*
  BALANCE BOT 
  PROGRAMMER: Arthur W. Aznive Jr.
  DATE:       3/11/2023
  NOTES:      Created for SNHU Control Systems Analysis Fall 2023
              Using the LSM9DS1 sensor.
  VERSION:    0.1
*/

#include <Arduino.h>

#include <Arduino_LSM9DS1.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");
}

void loop() {
  // put your main code here, to run repeatedly:
  float x, y, z;

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);

    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
  }
}