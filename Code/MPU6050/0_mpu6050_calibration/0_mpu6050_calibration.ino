/*
 * Six-pose calibration for the balance robot IMU.
 * Emits the constant block used by the balance firmware.
 *
 * POWER-CYCLE the board before running (unplug USB + LiPo for 5s)
 * so the MPU6050 reloads factory trim from a known state.
 */

#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

const uint8_t  NUM_POSES  = 6;
const uint16_t SAMPLES    = 200;
const float    GYRO_LSB   = 131.0f;    // +/-250 deg/s
const float    IDEAL_SCALE = 16384.0f; // counts per g at +/-2g

enum { P_UPRIGHT = 0, P_NOSE, P_BACK, P_LEFT, P_RIGHT, P_INVERTED };

const char* poseName[NUM_POSES] = {
  "upright at the balance point",
  "nose down   (front face flat on table)",
  "back down   (rear face flat on table)",
  "left side down",
  "right side down",
  "fully inverted (wheels up)"
};

const char axisLetter[3] = {'X', 'Y', 'Z'};

float A[NUM_POSES][3];
float G[NUM_POSES][3];
long  wob[NUM_POSES];
bool  clipped[NUM_POSES];

void waitForKey() {
  while (Serial.available()) Serial.read();
  while (!Serial.available()) ;
  while (Serial.available()) Serial.read();
}

void capture(uint8_t p) {
  long sa[3] = {0, 0, 0}, sg[3] = {0, 0, 0};
  int16_t mn[3] = { 32767,  32767,  32767};
  int16_t mx[3] = {-32768, -32768, -32768};
  clipped[p] = false;

  for (uint16_t i = 0; i < SAMPLES; i++) {
    int16_t a[3], g[3];
    mpu.getMotion6(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2]);

    for (uint8_t k = 0; k < 3; k++) {
      sa[k] += a[k];
      sg[k] += g[k];
      if (a[k] < mn[k]) mn[k] = a[k];
      if (a[k] > mx[k]) mx[k] = a[k];
      if (a[k] >= 32700 || a[k] <= -32700) clipped[p] = true;
    }
    delay(5);
  }

  long w = 0;
  for (uint8_t k = 0; k < 3; k++) {
    A[p][k] = sa[k] / (float)SAMPLES;
    G[p][k] = sg[k] / (float)SAMPLES;
    long d = (long)mx[k] - (long)mn[k];
    if (d > w) w = d;
  }
  wob[p] = w;
}

/* Which axis swings most between two poses = the axis that pair isolates */
uint8_t axisBetween(uint8_t pa, uint8_t pb) {
  uint8_t best = 0;
  float bestSwing = -1.0f;
  for (uint8_t k = 0; k < 3; k++) {
    float s = fabs(A[pa][k] - A[pb][k]);
    if (s > bestSwing) { bestSwing = s; best = k; }
  }
  return best;
}

float integrateGyro(uint8_t axis, float bias, uint16_t windowMs) {
  unsigned long t0 = millis(), tPrev = t0;
  float total = 0.0f;
  while (millis() - t0 < windowMs) {
    int16_t a[3], g[3];
    mpu.getMotion6(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2]);
    unsigned long now = millis();
    total += (G ? 1.0f : 1.0f) * (g[axis] - bias) / GYRO_LSB
             * ((now - tPrev) / 1000.0f);
    tPrev = now;
    delay(2);
  }
  return total;   // net degrees rotated
}

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

  /* Must match the firmware exactly */
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setDLPFMode(3);

  /* Registers zeroed: all correction lives in software */
  mpu.setXAccelOffset(0); mpu.setYAccelOffset(0); mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);  mpu.setYGyroOffset(0);  mpu.setZGyroOffset(0);

  Serial.println(F("=== six-pose IMU calibration ==="));
  Serial.println(F("Hold each pose still, then send any character.\n"));

  for (uint8_t p = 0; p < NUM_POSES; p++) {
    Serial.print(F("Pose ")); Serial.print(p + 1);
    Serial.print(F(" of 6: ")); Serial.println(poseName[p]);
    Serial.println(F("   send a character to capture"));
    waitForKey();
    Serial.println(F("   holding still..."));
    capture(p);
    Serial.print(F("   done, wobble = ")); Serial.println(wob[p]);
    if (wob[p] > 300) Serial.println(F("   ** high wobble, consider redoing **"));
    if (clipped[p])   Serial.println(F("   ** axis saturated in this pose **"));
    Serial.println();
  }

  /* --- work out which sensor axis plays which role --- */
  uint8_t foreAxis = axisBetween(P_NOSE, P_BACK);
  uint8_t vertAxis = axisBetween(P_UPRIGHT, P_INVERTED);
  uint8_t latAxis  = 3 - foreAxis - vertAxis;

  if (foreAxis == vertAxis) {
    Serial.println(F("ERROR: fore-aft and vertical resolved to the same axis."));
    Serial.println(F("Poses were likely confused. Rerun."));
    while (true);
  }

  float foreBias  = (A[P_NOSE][foreAxis] + A[P_BACK][foreAxis]) / 2.0f;
  float foreScale = fabs(A[P_NOSE][foreAxis] - A[P_BACK][foreAxis]) / 2.0f;
  float vertBias  = (A[P_UPRIGHT][vertAxis] + A[P_INVERTED][vertAxis]) / 2.0f;
  float vertScale = fabs(A[P_UPRIGHT][vertAxis] - A[P_INVERTED][vertAxis]) / 2.0f;

  float gyroBias[3] = {0, 0, 0};
  for (uint8_t k = 0; k < 3; k++) {
    for (uint8_t p = 0; p < NUM_POSES; p++) gyroBias[k] += G[p][k];
    gyroBias[k] /= NUM_POSES;
  }

  /* Sign conventions, derived not assumed */
  float vertSign = (A[P_UPRIGHT][vertAxis] - vertBias) < 0 ? -1.0f : 1.0f;
  float foreSign = (A[P_NOSE][foreAxis]  - foreBias) < 0 ? -1.0f : 1.0f;

  /* --- gyro sign needs motion --- */
  Serial.println(F("--- gyro direction check ---"));
  Serial.println(F("Stand the robot upright, then tip it NOSE DOWN through"));
  Serial.println(F("about 90 deg and leave it there. Send a character, then tip."));
  waitForKey();
  Serial.println(F("tip now... (4 seconds)"));
  float netRotation = integrateGyro(latAxis, gyroBias[latAxis], 4000);
  float gyroSign = (netRotation * foreSign) > 0 ? 1.0f : -1.0f;
  Serial.print(F("net rotation measured: ")); Serial.print(netRotation, 1);
  Serial.println(F(" deg"));
  if (fabs(netRotation) < 30.0f)
    Serial.println(F("** small rotation, gyro sign may be unreliable - rerun **"));
  Serial.println();

  /* --- report --- */
  Serial.println(F("===== PASTE INTO FIRMWARE ====="));
  Serial.print(F("/* fore-aft = sensor ")); Serial.print(axisLetter[foreAxis]);
  Serial.print(F(", vertical = sensor ")); Serial.print(axisLetter[vertAxis]);
  Serial.print(F(", pitch rate = gyro ")); Serial.print(axisLetter[latAxis]);
  Serial.println(F(" */"));

  Serial.print(F("const float AY_BIAS   = ")); Serial.print(foreBias, 1);  Serial.println(F("f;"));
  Serial.print(F("const float AY_SCALE  = ")); Serial.print(foreScale, 1); Serial.println(F("f;"));
  Serial.print(F("const float AZ_BIAS   = ")); Serial.print(vertBias, 1);  Serial.println(F("f;"));
  Serial.print(F("const float AZ_SCALE  = ")); Serial.print(vertScale, 1); Serial.println(F("f;"));
  Serial.print(F("const float GX_BIAS   = ")); Serial.print(gyroBias[latAxis], 1); Serial.println(F("f;"));
  Serial.print(F("const float GYRO_SIGN = ")); Serial.print(gyroSign, 1);  Serial.println(F("f;"));

  Serial.println();
  Serial.println(F("/* angle expression: */"));
  Serial.print(F("accAngle = atan2("));
  if (foreSign < 0) Serial.print(F("-"));
  Serial.print(F("(a")); Serial.print(axisLetter[foreAxis]);
  Serial.print(F(" - AY_BIAS) / AY_SCALE, "));
  if (vertSign < 0) Serial.print(F("-"));
  Serial.print(F("(a")); Serial.print(axisLetter[vertAxis]);
  Serial.println(F(" - AZ_BIAS) / AZ_SCALE) * 180.0f / M_PI;"));
  Serial.println(F("===== END ====="));

  /* --- health warnings --- */
  Serial.println();
  Serial.println(F("--- checks ---"));
  float fErr = 100.0f * (foreScale - IDEAL_SCALE) / IDEAL_SCALE;
  float vErr = 100.0f * (vertScale - IDEAL_SCALE) / IDEAL_SCALE;
  Serial.print(F("fore-aft scale error: ")); Serial.print(fErr, 1); Serial.println(F(" %"));
  Serial.print(F("vertical scale error: ")); Serial.print(vErr, 1); Serial.println(F(" %"));
  if (fabs(fErr) > 5.0f || fabs(vErr) > 5.0f)
    Serial.println(F("** scale error > 5%: a pose was probably off-axis **"));

  float latBias  = (A[P_LEFT][latAxis] + A[P_RIGHT][latAxis]) / 2.0f;
  float latScale = fabs(A[P_LEFT][latAxis] - A[P_RIGHT][latAxis]) / 2.0f;
  Serial.print(F("lateral axis bias ")); Serial.print(latBias, 0);
  Serial.print(F(", scale ")); Serial.println(latScale, 0);
  if (clipped[P_LEFT] || clipped[P_RIGHT] || latScale < 0.8f * IDEAL_SCALE)
    Serial.println(F("lateral axis saturates - expected on this unit, not used for pitch"));

  Serial.print(F("upright angle reads: "));
  float upFore = foreSign * (A[P_UPRIGHT][foreAxis] - foreBias) / foreScale;
  float upVert = vertSign * (A[P_UPRIGHT][vertAxis] - vertBias) / vertScale;
  Serial.print(atan2(upFore, upVert) * 180.0f / M_PI, 2);
  Serial.println(F(" deg  <- candidate PID setpoint"));
}

void loop() {}