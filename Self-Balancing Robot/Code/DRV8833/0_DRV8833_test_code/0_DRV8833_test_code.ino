// This program is used to test the motors

// Arduino pins
int IN1 = 10;
int IN2 = 9;
int IN3 = 6;
int IN4 = 5;

int speed = 0; // Value between 0 and 255 (Voltage must not exceed max motor voltage - measure this!)

void setup() {

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  
  /* Motor 1 */
  analogWrite(IN1, speed);
  digitalWrite(IN2, LOW);

  /* Motor 2 */
  analogWrite(IN3, speed);
  digitalWrite(IN4, LOW);
}
