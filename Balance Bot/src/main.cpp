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

//GLOBAL VARIABLES
const char PWMB = 11;
const char BIN2 = 10;
const char BIN1 = 9;
const char STBY = 8;
const char AIN1 = 7;
const char AIN2 = 6;
const char PWMA = 5;

float accelAngle[2];
float gyroAngle[2];
float totalAngle[2];

unsigned long currTime, deltaTime, lastTime;

float PID, error, previousError;

//degrees = (radians * 4068) / 71

int kp = 0;
int ki = 0;
int kd = 0;

float targetAngle = 0;

void setup() {
  // put your setup code here, to run once:

  //Initialize arduino pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  //Set the starting time for well, timing! (in milliseconds).
  currTime = millis();

}

void loop() {
  // put your main code here, to run repeatedly:
  float GX, GY, GZ, AX, AY, AZ; //Gyro and Accel xyz variables.

  IMU.readAcceleration(AX, AY, AZ);
  IMU.readGyroscope(GX, GY, GZ);
  // Read the Gyro and Accel. Data
  //Insert into PID Control
  //Update Motor output accordingly,
   
}