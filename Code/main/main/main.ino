#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

/*---Complementary filter: accel Y/Z + gyro X. No DMP.---*/
/*---Offset registers forced to zero; bias handled in software.---*/

/*--- From six-pose characterisation ---*/
const float AY_BIAS  = -9581.0f;
const float AZ_BIAS  = -11745.0f;
const float AY_SCALE =  16393.0f;   // counts per g, measured
const float AZ_SCALE =  16741.0f;
const float GX_BIAS  =   -166.0f;   // gyro LSB at rest

const float GYRO_LSB_PER_DPS = 131.0f;   // +/-250 deg/s
const float GYRO_SIGN        = -1.0f;    // inverted mount
const float ALPHA            = 0.98f;
const unsigned long LOOP_MS  = 10;

float angle = 0.0f;                 // filtered pitch, degrees
unsigned long tPrev;

/*---Arduino to DRV8833 Mapping---*/
int IN1 = 10; // Pin10 <---> IN1
int IN2 = 9; // Pin9 <---> IN2
int IN3 = 6; // Pin6 <---> IN3
int IN4 = 5; // Pin5 <---> IN4
// Batt+ <---> Vcc
// Batt- <---> Gnd
// MotorA <---> Out1
// MotorA <---> Out2
// MotorB <---> Out3
// MotorB <---> Out4

/*---Arduino to MP6050 Mapping---*/
// 5V <---> Vcc
// Gnd <---> Gnd
// SCL <---> SCL
// SDA <---> SDA

/*---Arduino to Battery Mapping---*/
// Vin <---> Batt+
// Gnd <---> Batt-

/*---Arduino to LED/Button Mapping---*/
const uint8_t BTN_GND = 4;   // driven LOW - acts as ground
// 1k ohm resistor in series
const uint8_t BTN_IN  = 7;   // input with internal pull-up
const uint16_t DEBOUNCE_MS = 25;
bool lastRaw = HIGH;
bool stable  = HIGH;
unsigned long lastChange = 0;

// Define variables
double setpoint, input, output;
double Kp = 10, Ki = 0.0, Kd = 0.0;

// Create PID object
// Arguments: Input, Output, Setpoint, Kp, Ki, Kd, Direction
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Wire.begin();
  Wire.setClock(400000); // OPTIONAL: Boost I2C speed to 400kHz for faster updates

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(BTN_GND, OUTPUT);
  digitalWrite(BTN_GND, LOW);
  pinMode(BTN_IN, INPUT_PULLUP);

  mpu.initialize();

  /*Verify connection*/
  if (mpu.testConnection() == false) {
    while (true);   // fatal: no MPU6050
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

  setpoint = 0.0; // Target angle (0 degrees = upright)

  myPID.SetMode(AUTOMATIC); // Start the PID running
  myPID.SetOutputLimits(-255, 255); // Match PWM range
  myPID.SetSampleTime(10); // Match loop time (ms)
}

const int MIN_PWM = 20; // lowest PWM that actually turns the wheels (measure it)

void driveMotors(double output) {
  int pwm = abs((int(output)));

  if (pwm > 0) {
    pwm = map(pwm, 1, 255, MIN_PWM, 255); // squeeze [1..255] into [MIN_PWM..255]
  }
  pwm = constrain(pwm, 0, 255);

  if (output > 0) {
    // Forward
    digitalWrite(IN1, LOW);
    analogWrite(IN2, pwm);
    analogWrite(IN3, pwm);
    digitalWrite(IN4, LOW);
  }
  else {
    //Reverse
    analogWrite(IN1, pwm);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    analogWrite(IN4, pwm);
  }
}

void loop() {

  /* Complementary filter tick, every LOOP_MS */
  unsigned long now = millis();
  if (now - tPrev >= LOOP_MS) {
    float dt = (now - tPrev) / 1000.0f;
    tPrev = now;

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float accAngle = atan2((ay - AY_BIAS) / AY_SCALE,
                           -(az - AZ_BIAS) / AZ_SCALE) * 180.0f / M_PI;

    float rate = GYRO_SIGN * (gx - GX_BIAS) / GYRO_LSB_PER_DPS;   // deg/s

    angle = ALPHA * (angle + rate * dt) + (1.0f - ALPHA) * accAngle;

    //1. Update input from filter
    input = angle; // degrees, from MPU6050
  }

  // 2. Compute() returns true only when the sample time has elapsed
  if (myPID.Compute()) {
    // 3. Act on the output
    driveMotors(output);
  }
}