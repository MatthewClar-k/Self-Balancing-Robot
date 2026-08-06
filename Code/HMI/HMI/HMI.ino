const uint8_t BTN_GND = 4;   // driven LOW — acts as ground
// 1k ohm resistor in series
const uint8_t BTN_IN  = 7;   // input with internal pull-up

const uint8_t RedLED = 12; // Anode via 330R, cathode to GND rail
const uint8_t = 8; // Anode via 330R, cathode to GND rail

const uint16_t DEBOUNCE_MS = 25;

bool lastRaw = HIGH;
bool stable  = HIGH;
unsigned long lastChange = 0;

void setup() {
  Serial.begin(115200);

  pinMode(BTN_GND, OUTPUT);
  digitalWrite(BTN_GND, LOW);
  pinMode(BTN_IN, INPUT_PULLUP);

  pinMode(RedLED, OUTPUT);
  pinMode(GreenLED, OUTPUT);
  digitalWrite(RedLED, HIGH); // steady on
  digitalWrite(GreenLED, HIGH);
}

void loop() {
  bool raw = digitalRead(BTN_IN);

  if (raw != lastRaw) {
    lastRaw = raw;
    lastChange = millis();
  }

  if (raw != stable && (millis() - lastChange) >= DEBOUNCE_MS) {
    stable = raw;
    Serial.println(stable == LOW ? F("pressed") : F("released"));
  }
}