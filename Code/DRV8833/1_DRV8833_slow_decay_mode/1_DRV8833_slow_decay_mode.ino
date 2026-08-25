// This program is used to test the motors (slow decay mode)

// Arduino pins
int IN1 = 10;
int IN2 = 9;
int IN3 = 6;
int IN4 = 5;

int speed = 25; // Value between 0 and 255 (Voltage must not exceed max motor voltage - measure this!)

void setup() {

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {

  /* Motor 1 */
  analogWrite(IN1, 255 - speed); // Slow decay mode
  digitalWrite(IN2, HIGH);

  /* Motor 2 */
  digitalWrite(IN3, HIGH);
  analogWrite(IN4, 255 - speed); // Slow decay mode
}