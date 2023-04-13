#include "Wire.h"
#include "MPU6050.h"
#include "math.h"

#define Kp 40
#define Kd 0.05
#define Ki 40
#define sampleTime 0.005
#define targetAngle -2.5

// This is the lowest PWM value that spins the motors (I think), adjust based on your motors. (Test them)
int MotorALow = 25;
int MotorBLow = 25;

const int aIN1 = 7; // Left motor control pin 1
const int aIN2 = 6; // Left motor control pin 2
const int PWMA = 5; // Left motor PWM

const int STBY = 8; // Standby Pin of the Sparkfun Motor Driver, don't pull Low or the motors will stop

const int PWMB = 11; // Right Motor PWM
const int bIN1 = 9;  // Right Motor control pin 1
const int bIN2 = 10; // Right Motor control pin 2

int pwmOutA = 0;
int pwmOutB = 0;

bool aDir = true;
bool bDir = true;

// - - - - - - - END MOTOR PINS - - - - - - -

MPU6050 mpu;

int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle = 0, _error, prev_error = 0, _errorSum = 0;
volatile byte count = 0;
int distanceCm;

void setup()
{
  // Set Nano Pins
  pinMode(aIN1, OUTPUT);
  pinMode(aIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(bIN1, OUTPUT);
  pinMode(bIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  // set STBY HIGH and never touch it again, unless the robot falls over, duh? ;)
  digitalWrite(STBY, HIGH);

  // set the status LED to output mode
  pinMode(13, OUTPUT);
  // initialize the MPU6050 and set offset values
  mpu.initialize();
  mpu.setYAccelOffset(1593);
  mpu.setZAccelOffset(963);
  mpu.setXGyroOffset(40);
}

// The ISR will be called every 5 milliseconds
void calculatePID()
{
  // calculate the angle of inclination
  accAngle = atan2(accY, accZ) * RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate * sampleTime;
  currentAngle = 0.9934 * (prevAngle + gyroAngle) + 0.0066 * (accAngle);

  _error = currentAngle - targetAngle;
  _errorSum = _errorSum + _error;
  _errorSum = constrain(_errorSum, -300, 300);
  // calculate output from P, I and D values
  motorPower = Kp * (_error) + Ki * (_errorSum)*sampleTime - Kd * (currentAngle - prevAngle) / sampleTime;
  prevAngle = currentAngle;
  // toggle the led on pin13 every second
  count++;
  if (count == 200)
  {
    count = 0;
    digitalWrite(13, !digitalRead(13));
  }
}

void loop()
{
  // read acceleration and gyroscope values
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();
  gyroX = mpu.getRotationX();
  // set motor power after constraining it
  motorPower = constrain(motorPower, -255, 255);

  // Set your motor power here.

  calculatePID();
}
