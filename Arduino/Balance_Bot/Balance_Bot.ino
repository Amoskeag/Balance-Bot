/*
  BALANCE BOT 
  PROGRAMMER: Arthur W. Aznive Jr.
  DATE:       3/11/2023
  NOTES:      Created for my personal understanding of Control Systems.
              Using the LSM9DS1 sensor.
             
  VERSION:    0.1
*/

//Playing around with how I interface with my hardware, stay tuned.
//#include <Arduino_LSM9DS1.h>
#include <Adafruit_LSM9DS1.h>

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

//Example PID constants
int Kp = 2.0;
int Ki = 0.5;
int Kd = 0.5;

float setPoint = 0.0;
float err = 0.0;
float integral = 0.0;
float prevErr = 0.0;
float deriv = 0.0;

unsigned long currTime;
unsigned long deltaT;
unsigned long lastTime;

Adafruit_LSM9DS1 IMU = Adafruit_LSM9DS1();

void setup() {
 
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

    //Set the starting time.
    currTime = millis();
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
void SetMotorA(int pwmOutA, bool dir)
{
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
void SetMotorB(int pwmOutB, bool dir)
{
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

/*
 * Using PID we will control the robot to target a specific angle (0 deg straight up). 
 *  If the robot leans we will measure the angle off the Z axis so stright up is 0, falling flat forward is 90, 
 *  and falling on its back is -90.
 *  
 *  If the robot goes beyond +/- 45 deg, turn STBY off and pray for the help of an engineer until they create
 *  a little arm to right yourself. 
 */
void PIDBalance()
{
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

  //Finally what direction should each motor be in?
  /*
   * if(-deg)
   * {
   *    //run motors backwards.
   *    aDir = false;
   *    bDir = false;
   * }
   * if(+deg)
   * {
   *    //motors forward
   *    aDir = true;
   *    bDir = true;
   *    
   * }
   * else //we are vertical?
   * {
   *    Dont move?
   * }
   * 
   * setMotorA(pidOutA, aDir);
   * setMotorB(pidOutB, bDir);
   * 
   * 
   * Other idea:
   * motorA->setSpeed(200 + motorSpeed);
   * motorB->setSpeed(200 - motorSpeed);
   */

   //Remember this for next call!
   prevErr = err;
   lastTime = millis();
}

void loop() {
  // Keep the robot in balance
  PIDBalance();
 
}
