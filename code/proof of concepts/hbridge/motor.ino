// Pins aanpassen aan jouw board!
const int AIN1 = 22;   // motor A richting
const int AIN2 = 21;   // motor A PWM (of omgekeerd, beide kunnen PWM)
const int BIN1 = 19;   // motor B richting
const int BIN2 = 18;  // motor B PWM

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Motor A vooruit op 60% duty, Motor B achteruit op 30%
  digitalWrite(AIN1, HIGH);
  analogWrite(AIN2, 150); // ~60% van 255

  digitalWrite(BIN1, HIGH);
  analogWrite(BIN2, 150);  // ~30% van 255
}

void loop() {
  // jouw logica hier
}