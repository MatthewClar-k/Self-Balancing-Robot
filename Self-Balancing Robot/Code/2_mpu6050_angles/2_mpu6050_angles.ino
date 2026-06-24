// This program outputs yaw, pitch and roll of MPU6050

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

void setup() {
  Wire.begin();
  Wire.setClock(400000); // OPTIONAL: Boost I2C speed to 400kHz for faster updates
  
  Serial.begin(115200);
  while (!Serial);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  /*Verify connection*/
  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }

  /*Wait for Serial input*/
  Serial.println(F("\nSend any character to begin: "));
  while (Serial.available() && Serial.read()); // Empty buffer
  while (!Serial.available());                 // Wait for data
  while (Serial.available() && Serial.read()); // Empty buffer again

  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  /* Supply your calibrated offsets here */
  mpu.setXGyroOffset(0); // YOUR VALUE HERE
  mpu.setYGyroOffset(0); // YOUR VALUE HERE
  mpu.setZGyroOffset(0); // YOUR VALUE HERE
  mpu.setXAccelOffset(0); // YOUR VALUE HERE
  mpu.setYAccelOffset(0); // YOUR VALUE HERE
  mpu.setZAccelOffset(0); // YOUR VALUE HERE

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
    Serial.print("Yaw: ");   Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\tPitch: "); Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\tRoll: ");  Serial.println(ypr[2] * 180/M_PI);
  }
}