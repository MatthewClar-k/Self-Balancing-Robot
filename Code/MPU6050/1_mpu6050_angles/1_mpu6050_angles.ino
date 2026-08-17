/*
 * Balance angle: complementary filter on accel Y/Z + gyro X.
 * No DMP. Offset registers forced to zero; bias handled in software.
 */

#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

/*--- From six-pose characterisation ---*/
const float AY_BIAS  = -9581.0f;
const float AZ_BIAS  = -11745.0f;
const float AY_SCALE =  16393.0f;   // counts per g, measured
const float AZ_SCALE =  16741.0f;
const float GX_BIAS  =   -166.0f;   // gyro LSB at rest

const float GYRO_LSB_PER_DPS = 131.0f;   // +/-250 deg/s

/* Pitch rate is rotation about the LATERAL axis = gyro X.
   Verify sign with the printout below; flip to -1.0f if needed. */
float GYRO_SIGN = -1.0f;

const float ALPHA = 0.98f;
const unsigned long LOOP_MS = 10;

float angle = 0.0f;
unsigned long tPrev;

void setup() {
  Wire.begin();
  Wire.setClock(100000);
  Serial.begin(115200);
  while (!Serial);

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println(F("MPU6050 connection failed"));
    while (true);
  }

  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setDLPFMode(3);                 // ~44Hz, kills motor vibration

  /* Force offsets to zero. Factory trim reloads on power-up, so this
     must be explicit or the software biases above stop matching. */
  mpu.setXAccelOffset(0); mpu.setYAccelOffset(0); mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);  mpu.setYGyroOffset(0);  mpu.setZGyroOffset(0);

  /* Seed the filter from the accelerometer so it starts converged */
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  angle = atan2((ay - AY_BIAS) / AY_SCALE,
                -(az - AZ_BIAS) / AZ_SCALE) * 180.0f / M_PI;

  tPrev = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - tPrev < LOOP_MS) return;
  float dt = (now - tPrev) / 1000.0f;
  tPrev = now;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accAngle = atan2((ay - AY_BIAS) / AY_SCALE,
                         -(az - AZ_BIAS) / AZ_SCALE) * 180.0f / M_PI;

  float rate = GYRO_SIGN * (gx - GX_BIAS) / GYRO_LSB_PER_DPS;   // deg/s

  angle = ALPHA * (angle + rate * dt) + (1.0f - ALPHA) * accAngle;

  Serial.print(F("acc: "));   Serial.print(accAngle, 2);
  Serial.print(F("\trate: ")); Serial.print(rate, 1);
  Serial.print(F("\tangle: ")); Serial.println(angle, 2);
}