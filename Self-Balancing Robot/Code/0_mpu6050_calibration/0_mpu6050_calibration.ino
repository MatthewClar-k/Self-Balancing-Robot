// Run this code once to generate offset values (then hardcode them into main program)

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Initialise I2C
  mpu.initialize(); // Initialise MPU6050

  // 1. (Optional) Reset offsets to 0 before calibration
  mpu.setXAccelOffset(0); mpu.setYAccelOffset(0); mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0); mpu.setYGyroOffset(0); mpu.setZGyroOffset(0);  

  Serial.println("Calibrating internal sensor offsets... Keep the sensor completely flat and still.");

  // 2. Automaitcally generate and apply the offsets 
  // The argument (6) represents the numbers of refinement loops. 6 is standard and highly accurate.
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);

  Serial.println("\nCalibration complete! Active Offsets:");

  Serial.println("Accel Offsets: ");
  Serial.print(mpu.getXAccelOffset()); Serial.print("\t");
  Serial.print(mpu.getYAccelOffset()); Serial.print("\t");
  Serial.print(mpu.getZAccelOffset());

  Serial.print("\n");

  Serial.println("Gyro Offsets: ");
  Serial.print(mpu.getXGyroOffset()); Serial.print("\t");
  Serial.print(mpu.getYGyroOffset()); Serial.print("\t");
  Serial.print(mpu.getZGyroOffset());

  Serial.print("\n");
}

void loop() {
  // Nothing to do here
}