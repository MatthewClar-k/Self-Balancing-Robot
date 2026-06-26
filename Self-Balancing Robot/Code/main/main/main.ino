#include <PID_v1.h>

// Define variables
double setpoint, input, output;
double Kp = 1.0, Ki = 0.0, Kd = 0.0;

// Create PID object
// Arguments: Input, Output, Setpoint, Kp, Ki, Kd, Direction
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Setpoint = 0.0; // Target angle (0 degrees = upright)
  
  myPID.SetMode(AUTOMATIC); // Start the PID running
  myPID.SetOutputLimits(-255, 255); // Match PWM range
  myPID.SetSampleTime(10); // Match loop time (ms)
}

void loop() {
  //1. Update input from complementary filter
  input = currentAngle; // degrees, from MPU6050

  // 2. Compute() returns true only when the sample time has elapsed
  if (myPID.Compute()) {
    // 3. Act on the output
    driveMotors(output);
  }

}
