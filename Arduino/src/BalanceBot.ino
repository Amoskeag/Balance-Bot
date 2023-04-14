#include "Wire.h"
#include "MPU6050.h"
#include "math.h"
#include "complementaryFilter.h"

void setup()
{
  Serial.begin(115200);

  // initialize the MPU6050 and set offset values
  mpu.initialize();
}

void loop()
{
  ;
}
