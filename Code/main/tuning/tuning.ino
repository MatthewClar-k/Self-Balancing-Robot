/*
 * tuning.ino
 *
 * Balance firmware with live gain and setpoint editing over serial.
 * The control core is identical to main.ino. Everything added for
 * tuning is marked /*TUNING*\/ so it can be stripped back out.
 *
 * Serial Monitor: 115200 baud. Line ending "Newline" is recommended,
 * but single-character commands fire without it.
 */

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
   rescaled to put the effective sample time back at LOOP_MS. */
const int PID_GUARD_MS = 9;

float angle = 0.0f;                 // filtered pitch, degrees
unsigned long tPrev;

/*---Arduino to DRV8833 Mapping---*/
int IN1 = 10; // Pin10 <---> IN1
int IN2 = 9;  // Pin9  <---> IN2
int IN3 = 6;  // Pin6  <---> IN3
int IN4 = 5;  // Pin5  <---> IN4
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
const uint8_t BTN_IN    = 7;   // input w/ internal pull-up, button to GND rail
const uint8_t RED_LED   = 12;  // power indicator,     anode via 330R to GND rail
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
double Kp = 11.04, Ki = 0.0, Kd = 0.2;

// Create PID object
// Arguments: Input, Output, Setpoint, Kp, Ki, Kd, Direction
// NOTE: these constructor gains are superseded by applyTunings() in setup().
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, REVERSE);

const int MIN_PWM = 18; // lowest PWM that actually turns the wheels (measured)

/*TUNING------------------------------------------------------*/
bool     telemetry = true;      // stream sp/angle/output
double   trimStep  = 0.10;      // degrees per +/- nudge
const uint8_t TELEM_DIV = 5;    // print every Nth tick -> 100/5 = 20 Hz
uint8_t  telemCount = 0;

uint16_t minDt = 65535;         // loop health, reset on every status print
uint16_t maxDt = 0;
/*------------------------------------------------------------*/

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

/* Single point of entry for gains. The two factors cancel the shortened
   PID_v1 guard, so Kp/Ki/Kd mean exactly what they would at a clean
   LOOP_MS sample. Never call myPID.SetTunings() directly. */
void applyTunings() {
  myPID.SetTunings(Kp,
                   Ki * (double)LOOP_MS / (double)PID_GUARD_MS,
                   Kd * (double)PID_GUARD_MS / (double)LOOP_MS);
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

void toggleArm() {
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

/*TUNING - serial command interface------------------------------*/

void printHelp() {
  Serial.println(F("--- commands ---"));
  Serial.println(F("p<v>  Kp          p14.5"));
  Serial.println(F("i<v>  Ki          i120"));
  Serial.println(F("d<v>  Kd          d0.35"));
  Serial.println(F("s<v>  setpoint    s-0.40"));
  Serial.println(F("n<v>  trim step   n0.05"));
  Serial.println(F("+ -   nudge setpoint by trim step"));
  Serial.println(F("a     arm / disarm"));
  Serial.println(F("t     telemetry on / off"));
  Serial.println(F("?     status + help"));
  Serial.println(F("----------------"));
}

void printStatus() {
  Serial.print(F("Kp="));    Serial.print(Kp, 3);
  Serial.print(F(" Ki="));   Serial.print(Ki, 3);
  Serial.print(F(" Kd="));   Serial.print(Kd, 4);
  Serial.print(F(" sp="));   Serial.print(setpoint, 3);
  Serial.print(F(" step=")); Serial.print(trimStep, 3);
  Serial.print(balancing ? F(" ARMED") : F(" DISARMED"));

  /* Tick min/max since the last status print. Both should read 10-11.
     Anything larger means the loop stalled and the D-term saw a bad
     sample. */
  Serial.print(F(" tick="));
  if (maxDt == 0) {
    Serial.println(F("-"));
  } else {
    Serial.print(minDt); Serial.print('/'); Serial.print(maxDt);
    Serial.println(F("ms"));
  }
  minDt = 65535;
  maxDt = 0;
}

void doInstant(char c) {
  switch (c) {
    case '+': setpoint += trimStep; break;
    case '-': setpoint -= trimStep; break;
    case 'a': toggleArm(); return;
    case 't': telemetry = !telemetry;
              Serial.println(telemetry ? F("telemetry ON") : F("telemetry OFF"));
              return;
    case '?': printHelp(); break;
  }
  printStatus();
}

void doValue(char *s) {
  if (s[1] == '\0') {              // bare letter, no number
    Serial.println(F("no value - try ?"));
    return;
  }

  double v = atof(s + 1);

  switch (s[0]) {
    /* SetTunings() silently ignores negative gains, so catch them here
       rather than let a typo look like it worked. */
    case 'p': if (v < 0) { Serial.println(F("Kp must be >= 0")); return; }
              Kp = v; applyTunings(); break;

    /* Changing Ki does not disturb the accumulated integral: PID_v1
       stores it in output units, so there is no bump on the way in
       or out. */
    case 'i': if (v < 0) { Serial.println(F("Ki must be >= 0")); return; }
              Ki = v; applyTunings(); break;

    case 'd': if (v < 0) { Serial.println(F("Kd must be >= 0")); return; }
              Kd = v; applyTunings(); break;

    /* Safe to change while balancing: PID_v1 takes the derivative of the
       measurement, not the error, so a setpoint step causes no D kick. */
    case 's': setpoint = v; break;

    case 'n': if (v <= 0) { Serial.println(F("step must be > 0")); return; }
              trimStep = v; break;

    default:  Serial.println(F("unknown - try ?")); return;
  }
  printStatus();
}

/* Non-blocking. Handles at most one command per pass so a pasted burst
   cannot stall the control loop. */
void handleSerial() {
  static char buf[16];
  static uint8_t n = 0;

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (n == 0) continue;            // swallow CRLF pairs and blank lines
      buf[n] = '\0';
      n = 0;
      doValue(buf);
      return;
    }

    /* Single-character commands fire immediately, so they work with the
       Serial Monitor set to "No line ending" too. None of the value
       commands start with these characters. */
    if (n == 0 && (c == '?' || c == '+' || c == '-' || c == 'a' || c == 't')) {
      doInstant(c);
      return;
    }

    if (n < sizeof(buf) - 1) buf[n++] = c;
  }
}

void sendTelemetry() {
  /* Labelled and tab separated, so the IDE Serial Plotter picks up the
     legend. ~28 bytes at 20 Hz, well inside the 64-byte TX buffer. */
  Serial.print(F("sp:"));    Serial.print(setpoint, 2);
  Serial.print(F("\tang:")); Serial.print(angle, 2);
  Serial.print(F("\tout:")); Serial.println(output, 0);
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
  myPID.SetSampleTime(PID_GUARD_MS);
  applyTunings();
  myPID.SetMode(MANUAL);            // idle until the button says otherwise

  Serial.println(F("Ready - stand upright and press to arm."));
  printHelp();
  printStatus();
}

/*---------------------------------------------------------*/

void loop() {

  /* 1. Complementary filter tick, every LOOP_MS. Runs armed or not, so
        the angle is already converged by the time you press to arm.

        The control update lives in here too, so one tick means one
        sensor read and one Compute(). */
  unsigned long now = millis();
  if (now - tPrev >= LOOP_MS) {
    uint16_t dtMs = (uint16_t)(now - tPrev);       /*TUNING*/
    if (dtMs < minDt) minDt = dtMs;                /*TUNING*/
    if (dtMs > maxDt) maxDt = dtMs;                /*TUNING*/

    float dt = dtMs / 1000.0f;
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

    /* Telemetry last, so nothing delays the motor update. Runs disarmed
       too - that is how you read the hand-balanced angle for the coarse
       setpoint. */
    if (telemetry && ++telemCount >= TELEM_DIV) {   /*TUNING*/
      telemCount = 0;
      sendTelemetry();
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
      toggleArm();
    }
  }

  /* 3. Serial commands - also every pass, also outside the tick */
  handleSerial();                                   /*TUNING*/
}
