#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

/*---Complementary filter: accel Y/Z + gyro X. No DMP.---*/
/*---Offset registers forced to zero; bias handled in software.---*/

/*--- From six-pose characterisation ---*/
const float AY_BIAS  = -9626.6f;
const float AZ_BIAS  = -11591.4f;
const float AY_SCALE =  16441.7f;   // counts per g, measured
const float AZ_SCALE =  16729.4f;
const float GX_BIAS  =   -154.8f;   // gyro LSB at rest

const float GYRO_LSB_PER_DPS = 131.0f;   // +/-250 deg/s
const float GYRO_SIGN        = -1.0f;    // inverted mount
const float ALPHA            = 0.98f;
const unsigned long LOOP_MS  = 10;

/* PID_v1 keeps its own millis() timer inside Compute(). Its guard is set
   below the loop period so it never skips a call, and the gains are
   rescaled to put the effective sample time back at LOOP_MS. See setup(). */
const int PID_GUARD_MS = 9;

float angle = 0.0f;                 // filtered pitch, degrees
unsigned long tPrev;

/*---Arduino to DRV8833 Mapping---*/
int IN1 = 10; // Pin10 <---> IN1
int IN2 = 9;  // Pin9  <---> IN2
int IN3 = 6;  // Pin6  <---> IN3
int IN4 = 5;  // Pin5  <---> IN4

/*---Arduino to LED/Button Mapping---*/
const uint8_t BTN_IN    = 7;   // input w/ internal pull-up, button to GND rail
const uint8_t RED_LED   = 12;  // power indicator, anode via 330R to GND rail
const uint8_t GREEN_LED = 8;   // balancing indicator, anode via 330R to GND rail

const uint16_t DEBOUNCE_MS = 25;
bool lastRaw = HIGH;
bool stable  = HIGH;
unsigned long lastChange = 0;

/*---Arm state---*/
bool balancing = false;
const float ARM_WINDOW_DEG = 20.0;  // must be this upright to arm
const float FALL_LIMIT_DEG = 35.0;  // auto-disarm past this tilt

// Define variables
double setpoint, input, output;
double Kp = 15.1, Ki = 33, Kd = 0.17;

// Create PID object
// Arguments: Input, Output, Setpoint, Kp, Ki, Kd, Direction
// NOTE: these constructor gains are superseded by SetTunings() in setup().
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, REVERSE);

const int MIN_PWM = 6; // lowest PWM that actually turns the wheels (measured)

/*---------------------------------------------------------*/

void stopMotors() {
  /* All four inputs LOW = outputs Hi-Z (coast). digitalWrite() also
     detaches the timer, so this kills any analogWrite() in progress.
     Coast rather than brake, so you can pick the robot up without the
     wheels fighting you. */
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void arm() {
  output = 0;                 // Initialize() seeds outputSum from this
  myPID.SetMode(AUTOMATIC);   // bumpless transfer: clears integral + d-history
  balancing = true;
  digitalWrite(GREEN_LED, HIGH);
  Serial.println(F("ARMED"));
}

void disarm() {
  myPID.SetMode(MANUAL);
  output = 0;
  stopMotors();
  balancing = false;
  digitalWrite(GREEN_LED, LOW);
  Serial.println(F("DISARMED"));
}

void fatalBlink() {            // visible fault code with no serial attached
  while (true) {
    digitalWrite(RED_LED, HIGH); delay(150);
    digitalWrite(RED_LED, LOW);  delay(150);
  }
}

void driveMotors(double output) {
  int pwm = abs((int(output)));

  if (pwm > 0) {
    pwm = map(pwm, 1, 255, MIN_PWM, 255); // squeeze [1..255] into [MIN_PWM..255]
  }
  pwm = constrain(pwm, 0, 255);

  if (output > 0) {
    // Forward - slow decay
    analogWrite(IN1, 255 - pwm);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    analogWrite(IN4, 255 - pwm);
  }
  else {
    // Reverse - slow decay
    digitalWrite(IN1, HIGH);
    analogWrite(IN2, 255 - pwm);
    analogWrite(IN3, 255 - pwm);
    digitalWrite(IN4, HIGH);
  }
}

/*---------------------------------------------------------*/

void setup() {
  /* Indicators and motor outputs first - red lights the instant power
     is applied, and the H-bridge is guaranteed idle before anything
     that can block. */
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotors();

  pinMode(BTN_IN, INPUT_PULLUP);

  Wire.begin();
  Wire.setClock(400000); // OPTIONAL: Boost I2C speed to 400kHz for faster updates

  Serial.begin(115200);

  mpu.initialize();

  /*Verify connection*/
  if (mpu.testConnection() == false) {
    Serial.println(F("MPU6050 connection failed"));
    fatalBlink();
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

  myPID.SetOutputLimits(-255, 255); // Match PWM range

  /* Compute() is called once per filter tick, so its internal guard must
     never reject a call. Consecutive calls land 10-11 ms apart in millis()
     terms, so a 9 ms guard always passes.

     SetTunings() derives its internal ki/kd from whatever SampleTime is
     current, so the two factors below cancel the 9 ms guard and restore
     exactly the scaling of a clean LOOP_MS sample. Change gains through
     SetTunings(), not by editing the constructor on line 71. */
  myPID.SetSampleTime(PID_GUARD_MS);
  myPID.SetTunings(Kp,
                   Ki * (double)LOOP_MS / (double)PID_GUARD_MS,
                   Kd * (double)PID_GUARD_MS / (double)LOOP_MS);

  myPID.SetMode(MANUAL);            // idle until the button says otherwise

  Serial.println(F("Ready - stand upright and press to arm."));
}

/*---------------------------------------------------------*/

void loop() {

  /* 1. Complementary filter tick, every LOOP_MS. Runs armed or not, so
        the angle is already converged by the time you press to arm.

        The control update lives in here too. Both used to run off their
        own millis() timer, which left the PID reading an input of
        arbitrary age and made the D-term see a doubled or missing
        difference whenever the two timers crossed. One tick, one
        sensor read, one Compute(). */
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

    input = angle; // degrees, from MPU6050

    if (balancing) {
      if (fabs(angle) > FALL_LIMIT_DEG) {
        disarm();
      }
      else {
        myPID.Compute();
        driveMotors(output);
      }
    }
  }

  /* 2. Button - polled every pass, not gated by LOOP_MS */
  bool raw = digitalRead(BTN_IN);
  if (raw != lastRaw) {
    lastRaw = raw;
    lastChange = millis();
  }
  if (raw != stable && (millis() - lastChange) >= DEBOUNCE_MS) {
    stable = raw;
    if (stable == LOW) {                    // falling edge = press
      if (balancing) {
        disarm();
      }
      else if (fabs(angle) <= ARM_WINDOW_DEG) {
        arm();
      }
      else {
        Serial.print(F("Too far off vertical to arm: "));
        Serial.println(angle);
      }
    }
  }
}
