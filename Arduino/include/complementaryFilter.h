/**
 * @brief Complementary filter
 *
 * Ugh
 */

#include "Wire.h"
#include "MPU6050.h"

int lastTime = 0;  // Last time this was ran
int16_t accX, gyX; // only care about acceleration in x, and roll in y right now
int curpitch = 0;

MPU6050 mpu;

void filter(int dt)
{
    accX = mpu.getAccelerationX();
    gyX = mpu.getRotationX();

    curpitch = 0.98 * (curpitch + gyX * dt) + 0.02 * accX;
}
