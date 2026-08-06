#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Initialise I2C
  mpu.initialize(); // Initialise MPU6050

  // Get these values from mpu6050_calibration.ino 
  mpu.setXAccelOffset(-3442); 
  mpu.setYAccelOffset(998); 
  mpu.setZAccelOffset(3074);
  mpu.setXGyroOffset(46); 
  mpu.setYGyroOffset(-43); 
  mpu.setZGyroOffset(-68);

  Serial.println("MPU6050 initialised. Reading values..."); 
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Print accelerometer values
  Serial.print("Accel -> ");
  Serial.print("X: "); Serial.print(ax);
  Serial.print(" | Y: "); Serial.print(ay);
  Serial.print(" | Z: "); Serial.print(az);

  // Print gyroscope values
  Serial.print(" || Gyro -> ");
  Serial.print("X: "); Serial.print(gx);
  Serial.print(" | Y: "); Serial.print(gy);
  Serial.print(" | Z: "); Serial.print(gz);

  Serial.print("\n");

  delay(300); // Small delay for readability
}