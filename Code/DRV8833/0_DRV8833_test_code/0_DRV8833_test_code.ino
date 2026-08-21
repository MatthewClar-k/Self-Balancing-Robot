// This program is used to test the motors (fast decay mode)

// Arduino pins
int IN1 = 10;
int IN2 = 9;
int IN3 = 6;
int IN4 = 5;

int speed = 60; // Value between 0 and 255 (Voltage must not exceed max motor voltage - measure this!)

void setup() {

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  
  /* Motor 1 */
  digitalWrite(IN1, LOW); // Fast decay mode
  analogWrite(IN2, speed);

  /* Motor 2 */
  analogWrite(IN3, speed);
  digitalWrite(IN4, LOW);
}
