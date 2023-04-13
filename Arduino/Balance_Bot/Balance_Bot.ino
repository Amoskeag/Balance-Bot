/*
  BALANCE BOT 
  PROGRAMMER: Arthur W. Aznive Jr.
  DATE:       3/11/2023
  NOTES:      Created for my personal understanding of Control Systems.
              Using the LSM9DS1 sensor.
             
  VERSION:    0.1
*/

#include <Arduino_LSM9DS1.h>

//Motor Control Variables

//This is the lowest PWM value that spins the motors (I think), adjust based on your motors. (Test them)
int MotorALow = 25;
int MotorBLow = 25;
              
const int aIN1 = 7;   // Left motor control pin 1
const int aIN2 = 6;   // Left motor control pin 2
const int PWMA = 5;   // Left motor PWM

const int STBY = 8;   // Standby Pin of the Sparkfun Motor Driver, don't pull Low or the motors will stop

const int PWMB = 11;  // Right Motor PWM 
const int bIN1 = 9;   // Right Motor control pin 1
const int bIN2 = 10;  // Right Motor control pin 2

int pwmOutA = 0;
int pwmOutB = 0;

bool aDir = true;
bool bDir = true;

// - - - - - - - END MOTOR PINS - - - - - - -

//Example PID constants
int Kp = 2.0;
int Ki = 0.5;
int Kd = 0.5;

float setPoint = 0.0;
float err = 0.0;
float integral = 0.0;
float prevErr = 0.0;
float deriv = 0.0;

// - - - - - - - END PID VARIABLEs - - - - - - - 

int motorPower;
int prevMotorPowers[2]; //Storage of previous motor powers.
float angleY;
int targetAngle = 0;
int sampleCounter = 0;
int8_t ctrlValues[] = {0,0,0,0}; //Target Pitch, Roll, Yaw, and Power.

const float samplingPeriod = 20;//in s
long int sampleTimer = 0;

long lastTime;
long lastInterval;

float Ax,            Ay,             Az,                       // units m/s/s i.e. accelZ if often 9.8 (gravity)
      Gx,            Gy,             Gz,                       // units dps (degrees per second)
      gyroDriftX,        gyroDriftY,         gyroDriftZ,        // units dps
      gyroRoll,          gyroPitch,          gyroYaw,           // units degrees (expect major drift)
      gyroCorrectedRoll, gyroCorrectedPitch, gyroCorrectedYaw,  // units degrees (expect minor drift)
      accRoll,           accPitch,           accYaw,            // units degrees (roll and pitch noisy, yaw not possible)
      complementaryRoll, complementaryPitch, complementaryYaw;  // units degrees (excellent roll, pitch, yaw minor drift)
    
int plusThreshold = 30, minusThreshold = -30; //Helps stop if a collision happens.

//Adafruit_LSM9DS1 IMU = Adafruit_LSM9DS1();

void setup() {
  // put your setup code here, to run once:
  if(!IMU.begin()){
    //Failed to initialize IMU!
    while(1);
  }

  // Set Nano Pins 
  pinMode(aIN1, OUTPUT);
  pinMode(aIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  
  pinMode(bIN1, OUTPUT);
  pinMode(bIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  // set STBY HIGH and never touch it again, unless the robot falls over, duh? ;)  
  digitalWrite(STBY, HIGH);

  //start in a Neutral position.
  SetMotorA(0, true);
  SetMotorB(0, true);

  calibrateIMU(250,250);

  lastTime = micros();
}

/*
 * SetMotorA Speed and Direction. True is forward.
 * Gets called by PIDBalance()
 * 
 *  the idea of the speed code is to allow the PID control 
 *  to use 0 - 255 but because the DC motors dont spin at all in a low range 
 *  say 0 - 25, the Map() utility lets us create an input scale (PID) to a mapped 
 *  output scale (DC Motor needs) 0 - 255 mapped to 25 - 255, so you always keep the
 *  robot dynamically moving. "constrain(abs(Output), 0, 255)" make sure the output
 *  doesnt go crazy.
 *             
 */
void SetMotorA(int pwmOutA, bool dir) {
  
  if(dir)
  {
    digitalWrite(aIN1, LOW);   // set leg 1 of the motor driver low
    digitalWrite(aIN2, HIGH);  // set leg 2 of the motor driver high
  }
  else
  {
    digitalWrite(aIN1, HIGH);   // set leg 1 of the motor driver low
    digitalWrite(aIN2, LOW);  // set leg 2 of the motor driver high
  }

  int speed = map(constrain(abs(pwmOutA), 0, 255), 0, 255, MotorALow, 255);  // Sets the PWM for Motor A
            
  analogWrite(PWMA, speed);
}

/*
 * SetMotorB Speed and Direction. True is forward.
 * Gets called by PIDBalance()
 */
void SetMotorB(int pwmOutB, bool dir) {
  
  if(dir)
  {
    digitalWrite(bIN1, LOW);   // set leg 1 of the motor driver low
    digitalWrite(bIN2, HIGH);  // set leg 2 of the motor driver high
  }
  else
  {
    digitalWrite(bIN1, HIGH);   // set leg 1 of the motor driver low
    digitalWrite(bIN2, LOW);  // set leg 2 of the motor driver high
  }

  // map the PID output to the DC motor outputs. If your DC motor has a different min value update it in global variables.
  int speed = map(constrain(abs (pwmOutB), 0, 255), 0, 255, MotorBLow, 255);  // Sets the PWM for Motor B
            
  analogWrite(PWMB, speed);
}

void loop() {
  // put your main code here, to run repeatedly:
  /*if(IMU.accelerationAvailable()){
    IMU.readAcceleration(Ax, Ay, Az);
  }*/
  
  if(IMU.gyroscopeAvailable()){
    long currentTime = micros();
    lastInterval = currentTime - lastTime;
    lastTime = currentTime;
  }

  IMUCalculation();

  PIDBalance();
 
}

/**
   Read accel and gyro data.
   returns true if value is 'new' and false if IMU is returning old cached data
*/
bool readIMU() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() ) {
    IMU.readAcceleration(Ax, Ay, Az);
    IMU.readGyroscope(Gx, Gy, Gz);
    return true;
  }
  return false;
}

/*
  the gyro's x,y,z values drift by a steady amount. if we measure this when arduino is still
  we can correct the drift when doing real measurements later
*/
void calibrateIMU(int delayMillis, int calibrationMillis) {

  int calibrationCount = 0;

  delay(delayMillis); // to avoid shakes after pressing reset button

  float sumX, sumY, sumZ;
  int startTime = millis();
  while (millis() < startTime + calibrationMillis) {
    IMU.readGyroscope(Gx, Gy, Gz);
    // in an ideal world gyroX/Y/Z == 0, anything higher or lower represents drift
    sumX += Gx;
    sumY += Gy;
    sumZ += Gz;

    calibrationCount++;
  
  }

  if (calibrationCount == 0) {
    Serial.println("Failed to calibrate");
  }

  gyroDriftX = sumX / calibrationCount;
  gyroDriftY = sumY / calibrationCount;
  gyroDriftZ = sumZ / calibrationCount;

}

void PIDBalance(){
  //Look at the IMUCalculations and see what needs to be done to hold position.
  //ctrlValues[] = {0,0,0,0}; //Target Pitch, Roll, Yaw, and Power.
  //Calculate the nessesary motor values based on the IMU readings for a target setPoint. (generally 0 if the idea is not to move around). I think.
  IMU.read();
  
  sensors_event_t a, m ,g, temp;
  
  IMU.getEvent(&a, &m, &g, &temp);
  
  deltaT = millis() - lastTime;
  
  float angle = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;

  err = angle - setPoint;

  integral += err * deltaT;

  float iTerm = Ki * integral;
  
  deriv = (err - prevErr) / deltaT;

  int motorSpeed = (Kp * err) + iTerm + (Kd * deriv);
  
}
void IMUCalculation() {
  IMU.readAcceleration(Ax, Ay, Az);
  IMU.readGyroscope(Gx, Gy, Gz);
  
  accRoll = atan2(Ay, Az) * 180 / M_PI;
  accPitch = atan2(-Ax, sqrt(Ay * Ay + Az * Az)) * 180 / M_PI;

  float lastFrequency = (float) 1000000.0/lastInterval;
  
  gyroRoll = gyroRoll + (Gx / lastFrequency);
  gyroPitch = gyroPitch + (Gy / lastFrequency);
  gyroYaw = gyroYaw + (Gz / lastFrequency);

  gyroCorrectedRoll = gyroCorrectedRoll + ((Gx - gyroDriftX) / lastFrequency);
  gyroCorrectedPitch = gyroCorrectedPitch + ((Gy - gyroDriftY) / lastFrequency);
  gyroCorrectedYaw = gyroCorrectedYaw + ((Gz - gyroDriftZ) / lastFrequency);

  complementaryRoll = complementaryRoll + ((Gx - gyroDriftX) / lastFrequency);
  complementaryPitch = complementaryPitch + ((Gy - gyroDriftY) / lastFrequency);
  complementaryYaw = complementaryYaw + ((Gz - gyroDriftZ) / lastFrequency);

  complementaryRoll = 0.98 * complementaryRoll + 0.02 * accRoll;
  complementaryPitch = 0.98 * complementaryPitch + 0.02 * accPitch;
  
}
