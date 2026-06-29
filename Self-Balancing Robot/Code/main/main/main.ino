#include <PID_v1.h>

// Arduino pins
int IN1 = 10;
int IN2 = 9;
int IN3 = 6;
int IN4 = 5;

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

  void driveMotors(double output) {
    int pwm = abs((int(output)));
    pwm = constrain(pwm, 0, 255);

    if (output > 0) {
      // Forward
      analogWrite(IN1, pwm);
      digitalWrite(IN2, LOW);
      analogWrite(IN3, pwm);
      digitalWrite(IN4, LOW);
    }
    else {
      //Reverse
      digitalWrite(IN1, LOW);
      analogWrite(IN2, pwm);
      digitalWrite(IN3, LOW);
      analoglWrite(IN4, pwm);
    }
  }

}
