/* ============================================================================
   deadband_test.ino  --  measure the KINETIC PWM dead-band (MIN_PWM)

   WHAT IT MEASURES
   ----------------
   For each drive configuration it ramps PWM up until the wheels break away,
   overshoots slightly so the gearbox is properly turning, then ramps back
   DOWN until they stop.  Two numbers come out:

     break-away (static) : torque needed to START a stopped gearbox
     drop-out  (kinetic) : torque needed to KEEP a turning gearbox turning

   Static is always the larger and it is NOT the number your balancing loop
   lives in -- mid-balance the motors are already moving, or reversing through
   zero, not starting cold.  Use the drop-out value as MIN_PWM.

   Six blocks run: BOTH / LEFT / RIGHT motors, forward and reverse.  Left and
   right TT gearboxes routinely differ by 20+ counts, and that asymmetry is
   what makes a balancing robot veer, so it is worth measuring separately.

   HOW MOTION IS DETECTED  (read this, it decides which mode you want)
   ------------------------------------------------------------------
   There are no encoders, so there are two options:

   DETECT_BUTTON (default, recommended)
       You watch the wheels and tap the button the instant they start, and
       again the instant they stop.  Single-handed, unambiguous, nothing to
       tune.  Your eye is the best sensor on this robot for this job.

   DETECT_GYRO
       Uses the MPU6050 as a vibration pickup: it high-passes the gyro above
       roughly 16 Hz and looks for the buzz of a turning gearbox.  It does NOT
       threshold rotation rate -- a robot rolling at CONSTANT speed has almost
       no pitch rate, so a rate threshold would miss steady rolling entirely,
       and the hand holding the box would swamp whatever was left.  Startup
       measures quiet-versus-running contrast and drops back to button mode by
       itself if the contrast is too poor to trust.

   BEFORE YOU RUN IT
   -----------------
   1. Set the pin numbers below to match your wiring.  All four motor pins
      must be PWM-capable (Uno: 3, 5, 6, 9, 10, 11).
   2. Check FORWARD is really forward.  Uncomment the JOG block in setup(),
      upload, watch which way each wheel turns, set LEFT_INVERT / RIGHT_INVERT
      so both drive the robot the same way, then re-comment the JOG block.
   3. This sketch drives the DRV8833 in SLOW DECAY (one input held HIGH, the
      other PWM'd with inverted duty) at the stock Uno PWM frequency.  If your
      balancing firmware uses fast decay or a different timer frequency,
      change this sketch to match -- the dead-band depends on both, and a
      number measured under different drive conditions is worthless.
   4. Charge the pack and measure it with a multimeter.  Write the voltage
      down.  Dead-band in PWM counts scales roughly as 1/V_bat, so a number
      taken at 8.2 V reads about 12 % low compared with 7.4 V.

   HOW TO RUN IT
   -------------
   1. Put the robot on the floor you will actually balance on, wheels loaded
      with its own weight.  Carpet and tile give different answers.
   2. Clear about 2 m of floor.  Forward and reverse blocks alternate to keep
      it roughly in one place, but it will wander.
   3. Open Serial Monitor at 115200 baud.
   4. Hold the box lightly between two fingers near the TOP edge, just enough
      to keep it upright.  Do not grip the wheels and do not brace it against
      anything -- the wheels must roll freely, and any sideways force changes
      the friction you are trying to measure.
   5. Tap the button to start each block.  Hold it for 1.5 s at any time to
      abort and cut the motors.
   6. Reaction lag puts break-away about two counts high and drop-out about
      two counts low.  That is the safe direction to be wrong in -- a slightly
      low MIN_PWM costs you less gain inflation and a smaller step at zero
      error than a slightly high one.  Watch the dropMin/dropMax spread in the
      summary; if it is more than about 8 counts, slow STEP_MS down or do more
      REPEATS.

   AFTER
   -----
   Paste the recommended value into balancebot.py:  min_pwm: int = <value>
   then re-run it.  That updates the compensated gains and the generated
   firmware constants automatically.

   TWO FOLLOW-UPS WORTH DOING
   --------------------------
   * Repeat with the wheels off the ground.  The difference between the two
     runs is rolling resistance; what is left is gearbox stiction.  If the
     on-floor number is much larger, look at the tyres and the floor before
     you touch any gains.
   * If you want this measured rather than watched: two resistor dividers from
     the motor terminals to A0/A1 let you coast the bridge for a millisecond
     and read back-EMF.  That is real speed feedback, and it turns this sketch
     into something you can trust unattended.
   ========================================================================= */

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

/* ---------------------------------------------------------------- WIRING -- */
const uint8_t AIN1 = 5, AIN2 = 6;     // LEFT  motor, both must be PWM pins
const uint8_t BIN1 = 9, BIN2 = 10;    // RIGHT motor, both must be PWM pins
const bool LEFT_INVERT  = false;
const bool RIGHT_INVERT = false;

const uint8_t BUTTON_PIN = 7;         // to GND, uses INPUT_PULLUP

/* ------------------------------------------------------------- BEHAVIOUR -- */
#define DETECT_BUTTON 0
#define DETECT_GYRO   1
uint8_t DETECT_MODE = DETECT_BUTTON;

const int      MAX_PWM      = 200;    // never ramp past this
const int      STEP_MS      = 60;     // dwell per PWM count
const int      OVERSHOOT    = 25;     // counts above break-away before ramp down
const int      HOLD_MS      = 500;    // reach steady speed before ramping down
const int      CAL_PWM      = 200;    // drive level for the running-buzz sample
const int      HINT_MARGIN  = 40;     // how far below the last break-away to restart
const uint16_t AC_WINDOW_MS = 40;     // vibration integration window
const uint16_t AC_PRIME_MS  = 10;     // let the high-pass settle before counting
const uint16_t AC_PERIOD_US = 2000;   // 500 Hz paced sampling
const float    AC_ALPHA     = 0.20;   // EMA high-pass, corner ~16 Hz
const uint8_t  REPEATS      = 3;

/* ------------------------------------------------------------------ TESTS -- */
#define M_LEFT  0x01
#define M_RIGHT 0x02

struct Cfg { const char *name; uint8_t motors; int8_t dir; };
const Cfg CFGS[] = {
  { "BOTH  fwd", M_LEFT | M_RIGHT, +1 },
  { "BOTH  rev", M_LEFT | M_RIGHT, -1 },
  { "LEFT  fwd", M_LEFT,           +1 },
  { "LEFT  rev", M_LEFT,           -1 },
  { "RIGHT fwd", M_RIGHT,          +1 },
  { "RIGHT rev", M_RIGHT,          -1 },
};
const uint8_t NCFG = sizeof(CFGS) / sizeof(CFGS[0]);

long    sumBreak[NCFG], sumDrop[NCFG];
int     minDrop[NCFG], maxDrop[NCFG];
uint8_t nGood[NCFG];

MPU6050 mpu;
float acThreshold = 0;

/* ------------------------------------------------------------ MOTOR DRIVE -- */
// DRV8833 slow decay: hold one input HIGH, PWM the other with inverted duty.
// pwm == 0 leaves both inputs HIGH, which is BRAKE -- the correct idle state
// for slow-decay drive, and what the balancing firmware does at zero output.
void motorWrite(uint8_t in1, uint8_t in2, int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0)      { digitalWrite(in1, HIGH); analogWrite(in2, 255 - pwm); }
  else if (pwm < 0) { digitalWrite(in2, HIGH); analogWrite(in1, 255 + pwm); }
  else              { digitalWrite(in1, HIGH); digitalWrite(in2, HIGH); }
}

void motorCoast(uint8_t in1, uint8_t in2) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

// Unselected motors COAST, never brake -- a braked wheel drags and would
// corrupt the single-motor measurements.
void setDrive(uint8_t motors, int8_t dir, int pwm) {
  int cmd = dir * pwm;
  if (motors & M_LEFT)  motorWrite(AIN1, AIN2, LEFT_INVERT  ? -cmd : cmd);
  else                  motorCoast(AIN1, AIN2);
  if (motors & M_RIGHT) motorWrite(BIN1, BIN2, RIGHT_INVERT ? -cmd : cmd);
  else                  motorCoast(BIN1, BIN2);
}

void allStop() {
  motorCoast(AIN1, AIN2);
  motorCoast(BIN1, BIN2);
}

/* ---------------------------------------------------------------- BUTTON -- */
// 0 = nothing, 1 = tap, 2 = held (abort).  Blocks until release, so the mark
// lands on the release edge.  The same reaction lag applies to both ramps, so
// most of it cancels out of the break-away / drop-out difference.
uint8_t pollButton() {
  if (digitalRead(BUTTON_PIN) != LOW) return 0;
  unsigned long t0 = millis();
  while (digitalRead(BUTTON_PIN) == LOW) {
    if (millis() - t0 > 1500) {
      allStop();
      while (digitalRead(BUTTON_PIN) == LOW) { }
      return 2;
    }
  }
  return (millis() - t0 > 25) ? 1 : 0;
}

void waitTap(const char *prompt) {
  Serial.println();
  Serial.print(F(">>> "));
  Serial.print(prompt);
  Serial.println(F("   -- tap the button when you are ready"));
  for (;;) {
    uint8_t b = pollButton();
    if (b == 1) return;
    if (b == 2) { Serial.println(F("ABORTED")); allStop(); while (1) { } }
  }
}

// Swallow taps for `ms` so a double-press or a bounce after the break-away
// mark cannot be mistaken for the drop-out mark.
void flushButton(uint16_t ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) pollButton();
}

/* ------------------------------------------------------------- VIBRATION -- */
// Mean high-passed gyro deviation over `ms`, in deg/s.  This is a vibration
// meter, not a rate meter: the EMA tracks and subtracts anything slower than
// about 16 Hz, which is where hand tremor lives, leaving the gearbox buzz.
float acEnergy(uint16_t ms) {
  static float ex = 0, ey = 0, ez = 0;
  static bool primed = false;
  float sum = 0;
  uint16_t n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < (unsigned long)(ms + AC_PRIME_MS)) {
    unsigned long tick = micros();
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    float x = gx, y = gy, z = gz;
    if (!primed) { ex = x; ey = y; ez = z; primed = true; }
    ex += AC_ALPHA * (x - ex);
    ey += AC_ALPHA * (y - ey);
    ez += AC_ALPHA * (z - ez);
    // the first few samples after a gap are inflated while the EMA re-converges
    if (millis() - t0 >= (unsigned long)AC_PRIME_MS) {
      sum += fabs(x - ex) + fabs(y - ey) + fabs(z - ez);
      n++;
    }
    while (micros() - tick < AC_PERIOD_US) { }
  }
  return n ? sum / (n * 131.0) : 0.0;
}

void calibrateVibration() {
  Serial.println(F("calibrating vibration pickup -- hold the robot still"));
  allStop();
  delay(300);
  float quiet = acEnergy(700);

  // Spin in place (left forward, right reverse) so calibration does not send
  // the robot across the room.
  motorWrite(AIN1, AIN2, LEFT_INVERT  ? -CAL_PWM :  CAL_PWM);
  motorWrite(BIN1, BIN2, RIGHT_INVERT ?  CAL_PWM : -CAL_PWM);
  delay(350);
  float running = acEnergy(700);
  allStop();

  Serial.print(F("quiet "));
  Serial.print(quiet, 2);
  Serial.print(F("   running "));
  Serial.print(running, 2);
  Serial.print(F("   ratio "));
  Serial.println(quiet > 0 ? running / quiet : 0.0, 1);

  if (running < quiet * 2.5) {
    Serial.println(F("!! not enough contrast to detect motion reliably"));
    Serial.println(F("!! falling back to DETECT_BUTTON"));
    DETECT_MODE = DETECT_BUTTON;
    return;
  }
  acThreshold = sqrt(quiet * running);       // geometric mean of the two
  Serial.print(F("threshold "));
  Serial.println(acThreshold, 2);
}

/* ----------------------------------------------------------------- RAMPS -- */
// Sense one PWM step.  1 = moving, 0 = not moving, 2 = abort.
uint8_t senseStep() {
  if (DETECT_MODE == DETECT_BUTTON) {
    unsigned long t0 = millis();
    while (millis() - t0 < (unsigned long)STEP_MS) {
      uint8_t b = pollButton();
      if (b) return (b == 2) ? 2 : 1;
    }
    return 0;
  }
  if (STEP_MS > AC_WINDOW_MS) delay(STEP_MS - AC_WINDOW_MS);
  if (pollButton() == 2) return 2;
  return acEnergy(AC_WINDOW_MS) > acThreshold ? 1 : 0;
}

// First PWM count that produced motion.  -1 = never moved, -2 = aborted.
int rampUp(uint8_t motors, int8_t dir, int startPwm) {
  for (int p = startPwm; p <= MAX_PWM; p++) {
    setDrive(motors, dir, p);
    uint8_t s = senseStep();
    if (s == 2) return -2;
    if (s == 1) return p;
  }
  return -1;
}

// Lowest PWM count still producing motion.  -2 = aborted.
// In button mode the tap marks the moment it STOPS, so that count is the
// first dead one and the answer is the count above it.
int rampDown(uint8_t motors, int8_t dir, int startPwm) {
  int lastMoving = startPwm;
  uint8_t quiet = 0;
  for (int p = startPwm; p >= 0; p--) {
    setDrive(motors, dir, p);
    uint8_t s = senseStep();
    if (s == 2) return -2;
    if (DETECT_MODE == DETECT_BUTTON) {
      if (s == 1) return (p < MAX_PWM) ? p + 1 : p;
    } else {
      if (s == 1) { lastMoving = p; quiet = 0; }
      else if (++quiet >= 2) return lastMoving;   // one quiet window is noise
    }
  }
  return 0;
}

/* --------------------------------------------------------------- REPORT -- */
void summary() {
  Serial.println();
  Serial.println(F("=============================================="));
  Serial.println(F("config      n   break   drop   dropMin dropMax"));
  for (uint8_t c = 0; c < NCFG; c++) {
    Serial.print(CFGS[c].name);
    Serial.print(F("   "));
    Serial.print(nGood[c]);
    if (!nGood[c]) { Serial.println(F("   -- no data")); continue; }
    Serial.print(F("     "));
    Serial.print((int)(sumBreak[c] / nGood[c]));
    Serial.print(F("     "));
    Serial.print((int)(sumDrop[c] / nGood[c]));
    Serial.print(F("      "));
    Serial.print(minDrop[c]);
    Serial.print(F("      "));
    Serial.println(maxDrop[c]);
  }

  int lMax = 0, rMax = 0, bothSum = 0, bothN = 0;
  for (uint8_t c = 0; c < NCFG; c++) {
    if (!nGood[c]) continue;
    int m = (int)(sumDrop[c] / nGood[c]);
    if (CFGS[c].motors == M_LEFT  && m > lMax) lMax = m;
    if (CFGS[c].motors == M_RIGHT && m > rMax) rMax = m;
    if (CFGS[c].motors == (M_LEFT | M_RIGHT)) { bothSum += m; bothN++; }
  }
  int both = bothN ? bothSum / bothN : 0;

  Serial.println(F("----------------------------------------------"));
  Serial.print(F("MIN_PWM_LEFT  = ")); Serial.println(lMax);
  Serial.print(F("MIN_PWM_RIGHT = ")); Serial.println(rMax);
  if (abs(lMax - rMax) > 15)
    Serial.println(F("left/right differ by >15 counts -- use separate "
                     "constants in toPwm() or the robot will veer"));
  Serial.println();
  Serial.print(F("for balancebot.py:   min_pwm: int = "));
  Serial.println(both);
  Serial.print(F("gain multiplier 255/(255-MIN_PWM) = "));
  Serial.println(255.0 / (255.0 - both), 2);
  Serial.println(F("write the pack voltage down now -- this number is only "
                   "valid near it"));
  Serial.println(F("=============================================="));
}

/* ------------------------------------------------------------------ MAIN -- */
void setup() {
  Serial.begin(115200);
  while (!Serial) { }
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  allStop();

  Wire.begin();
  Wire.setClock(400000);
  mpu.initialize();
  // The gyro is used only as a high-passed vibration pickup, so the offset
  // registers, the factory trim reload and the railed accel X axis are all
  // irrelevant here.
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  Serial.println(F("\n--- kinetic dead-band test ---"));

  /* --- JOG: uncomment to check motor direction, then re-comment ----------
  Serial.println(F("JOG: left forward"));
  setDrive(M_LEFT,  +1, 180); delay(700); allStop(); delay(800);
  Serial.println(F("JOG: right forward"));
  setDrive(M_RIGHT, +1, 180); delay(700); allStop();
  while (1) { }
  ---------------------------------------------------------------------- */

  if (DETECT_MODE == DETECT_GYRO) {
    if (!mpu.testConnection()) {
      Serial.println(F("MPU6050 not responding -- using DETECT_BUTTON"));
      DETECT_MODE = DETECT_BUTTON;
    } else {
      calibrateVibration();
    }
  }
  if (DETECT_MODE == DETECT_BUTTON)
    Serial.println(F("BUTTON mode: tap the instant the wheels start, and "
                     "again the instant they stop"));

  for (uint8_t c = 0; c < NCFG; c++) {
    sumBreak[c] = 0; sumDrop[c] = 0;
    minDrop[c] = 999; maxDrop[c] = 0; nGood[c] = 0;
  }
}

void loop() {
  int hint = 0;

  for (uint8_t c = 0; c < NCFG; c++) {
    waitTap(CFGS[c].name);

    for (uint8_t rep = 0; rep < REPEATS; rep++) {
      int start = (hint > HINT_MARGIN) ? hint - HINT_MARGIN : 0;

      int brk = rampUp(CFGS[c].motors, CFGS[c].dir, start);
      if (brk == -2) { Serial.println(F("ABORTED")); allStop(); while (1) { } }
      if (brk < 0) {
        allStop();
        Serial.println(F("  never moved -- raise MAX_PWM or check wiring"));
        continue;
      }

      if (start > 0 && brk <= start + 5) {      // was already moving at `start`
        Serial.println(F("  suspect: moved on the first step, redoing from 0"));
        allStop(); delay(600);
        brk = rampUp(CFGS[c].motors, CFGS[c].dir, 0);
        if (brk == -2) { Serial.println(F("ABORTED")); allStop(); while (1) { } }
        if (brk < 0) { allStop(); continue; }
      }

      int top = min(brk + OVERSHOOT, MAX_PWM);
      setDrive(CFGS[c].motors, CFGS[c].dir, top);
      flushButton(HOLD_MS);

      int drop = rampDown(CFGS[c].motors, CFGS[c].dir, top);
      allStop();
      if (drop == -2) { Serial.println(F("ABORTED")); while (1) { } }
      if (drop >= top - 2) {
        Serial.println(F("  suspect: stopped immediately, rep discarded"));
        delay(700);
        continue;
      }

      Serial.print(F("  "));
      Serial.print(CFGS[c].name);
      Serial.print(F("  rep "));
      Serial.print(rep + 1);
      Serial.print(F(":  break-away "));
      Serial.print(brk);
      Serial.print(F("   drop-out "));
      Serial.println(drop);

      sumBreak[c] += brk;
      sumDrop[c]  += drop;
      if (drop < minDrop[c]) minDrop[c] = drop;
      if (drop > maxDrop[c]) maxDrop[c] = drop;
      nGood[c]++;
      hint = brk;

      delay(700);                   // let the robot and your hand settle
    }
  }

  summary();
  allStop();
  while (1) { }
}
