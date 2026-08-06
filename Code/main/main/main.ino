#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---Orientation/Motion Variables---*/ 
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorFloat gravity;    // [x, y, z]            Gravity vector
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

double currentAngle = 0;

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
  
  Serial.begin(115200);
  while (!Serial);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(BTN_GND, OUTPUT);
  digitalWrite(BTN_GND, LOW);
  pinMode(BTN_IN, INPUT_PULLUP);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  /*Verify connection*/
  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }

  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  /* Supply your calibrated offsets here */
  mpu.setXAccelOffset(-3442); // YOUR VALUE HERE
  mpu.setYAccelOffset(998); // YOUR VALUE HERE
  mpu.setZAccelOffset(3074); // YOUR VALUE HERE
  mpu.setXGyroOffset(46); // YOUR VALUE HERE
  mpu.setYGyroOffset(-43); // YOUR VALUE HERE
  mpu.setZGyroOffset(-68); // YOUR VALUE HERE

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {

    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();

    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    DMPReady = true;
  } 
  else {
    Serial.print(F("DMP Initialization failed (code ")); //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }
  
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
    analogWrite(IN4, pwm);
  }
}

void loop() {
  if (!DMPReady) return; 
    
  /* Check for a fresh packet from the FIFO buffer */
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { 
    
    // Process angles using the dependency chain
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    // Print Yaw, Pitch, Roll in degrees
    // Serial.print("Yaw: ");   Serial.print(ypr[0] * 180/M_PI);
    // Serial.print("\tPitch: "); Serial.print(ypr[1] * 180/M_PI);
    // Serial.print("\tRoll: ");  Serial.println(ypr[2] * 180/M_PI);

    currentAngle = ypr[1] * 180/M_PI;
  }
  
  //1. Update input from filter
  input = currentAngle; // degrees, from MPU6050

  // 2. Compute() returns true only when the sample time has elapsed
  if (myPID.Compute()) {
    // 3. Act on the output
    driveMotors(output);
  }
}