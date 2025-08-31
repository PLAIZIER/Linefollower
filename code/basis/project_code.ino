/*
  ESP32 Line Follower – QTR-8A + DRV8833 + Bluetooth SPP
  - Sensors: QTR-8A (ANALOGE uitgangen)
  - Driver:  DRV8833 (dual H-bridge)
  - MCU:     ESP32-WROOM
  - BT:      Classic Bluetooth SPP (Android: b.v. "Serial Bluetooth Terminal")

  PINMAPPING (pas aan naar je eigen bekabeling):
  QTR-8A analoge uitgangen -> ESP32 ADC-pins:
    S1..S8 -> {32, 33, 34, 35, 36, 39, 25, 26}
    Let op: 34..39 zijn input-only. 25/26 zijn ADC2; werkt niet samen met Wi-Fi (Wi-Fi uitlaten).
    (Optioneel) IR/LEDON -> 27  (HIGH = LEDs aan; of los laten = altijd aan)

  DRV8833:
    AIN1=18, AIN2=19, BIN1=16, BIN2=17, nSLEEP=23 (hoog = driver aan)
    VM = motorvoeding (bijv. 6–9 V), GND gemeenschappelijk met ESP32.

  BT-naam: "ESP32-LineBot"
  Commando’s via BT:
    p=1.2    -> Kp instellen
    i=0.0    -> Ki instellen
    d=6.0    -> Kd instellen
    b=160    -> baseSpeed (0..255)
    s        -> start/stop togglen
    c        -> (her)kalibreren (1.5 s)
    r        -> waarden en PID printen (debug)

  Volg je een ZWARTE lijn op WIT? Laat LINE_IS_DARK = true.
*/

#include <BluetoothSerial.h>
BluetoothSerial SerialBT;

// ====== CONFIG ======
const bool LINE_IS_DARK = true;   // zwarte lijn = true, witte lijn = false
const int BASE_SPEED_DEFAULT = 160;  // 0..255

// QTR-8A sensorpins (8 kanalen)
const int sensorPins[8] = {34, 35, 32, 33, 25, 26, 27, 14}; // ADC1 + ADC2
// (Optioneel) IR/LEDEN/“IR” pin; zet op -1 als je hem niet gebruikt
const int IR_CTRL = 27; // zet HIGH voor IR aan; of -1

// DRV8833 pins
const int AIN1 = 22;
const int AIN2 = 21;
const int BIN1 = 19;
const int BIN2 = 18;
//const int NSLEEP = 23;

// LEDC PWM-instellingen
const int PWM_FREQ = 20000; // 20 kHz
const int PWM_RES  = 8;     // 0..255
const int CH_A1 = 0, CH_A2 = 1, CH_B1 = 2, CH_B2 = 3;

// ====== VARS ======
int baseSpeed = BASE_SPEED_DEFAULT;
float Kp = 1.2f, Ki = 0.0f, Kd = 6.0f;

int sensorRaw[8];
int sensorMin[8], sensorMax[8];
long lastPos = 3500; // 0..7000 (8 sensors → 7 segmenten)
bool running = false;

// PID state
float integral = 0.0f;
float lastError = 0.0f;

// ====== UTILS ======
/*void attachPWM() {
  ledcSetup(CH_A1, PWM_FREQ, PWM_RES); ledcAttachPin(AIN1, CH_A1);
  ledcSetup(CH_A2, PWM_FREQ, PWM_RES); ledcAttachPin(AIN2, CH_A2);
  ledcSetup(CH_B1, PWM_FREQ, PWM_RES); ledcAttachPin(BIN1, CH_B1);
  ledcSetup(CH_B2, PWM_FREQ, PWM_RES); ledcAttachPin(BIN2, CH_B2);
  ledcWrite(CH_A1, 0); ledcWrite(CH_A2, 0);
  ledcWrite(CH_B1, 0); ledcWrite(CH_B2, 0);
}*/

void motorWrite(int left, int right) {
  // left/right: -255..255 (positief = vooruit)
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  // Motor A = links; Motor B = rechts (pas aan als jouw bekabeling andersom is)
  int LA = abs(left), RA = abs(right);

  if (left >= 0) { ledcWrite(CH_A1, LA); ledcWrite(CH_A2, 0); }   // vooruit
  else           { ledcWrite(CH_A1, 0);  ledcWrite(CH_A2, LA); }  // achteruit

  if (right >= 0){ ledcWrite(CH_B1, RA); ledcWrite(CH_B2, 0); }
  else           { ledcWrite(CH_B1, 0);  ledcWrite(CH_B2, RA); }
}

void motorsCoast() { motorWrite(0, 0); }

// Lees alle 8 sensoren (raw 0..4095)
void readSensorsRaw() {
  for (int i = 0; i < 8; i++) {
    sensorRaw[i] = analogRead(sensorPins[i]);
  }
}

// Schaal naar 0..1000 per sensor o.b.v. min/max, eventueel inverteren
// Geeft ook de “lijnpositie” terug op 0..7000 (gewogen gemiddelde)
long readLinePosition() {
  readSensorsRaw();
  long weightedSum = 0;
  long sum = 0;

  for (int i = 0; i < 8; i++) {
    int v = sensorRaw[i];
    int minv = sensorMin[i], maxv = sensorMax[i];
    int scaled = 0;

    if (maxv > minv) {
      scaled = (int)((long)(v - minv) * 1000L / (maxv - minv));
      if (scaled < 0) scaled = 0; if (scaled > 1000) scaled = 1000;
    }
    // Invert indien we een donkere lijn volgen (donker = “meer line”)
    if (LINE_IS_DARK) scaled = 1000 - scaled;

    sum += scaled;
    weightedSum += (long)scaled * (i * 1000); // 0,1000,2000,...,7000
  }

  if (sum == 0) {
    // Lijn kwijt → gebruik laatste positie (houd koers)
    return lastPos;
  }

  lastPos = weightedSum / sum;
  return lastPos;
}

// Snelle auto-kalibratie (1.5 s “stilliggend”)
void calibrateSensors(unsigned long ms = 1500) {
  for (int i = 0; i < 8; i++) { sensorMin[i] = 4095; sensorMax[i] = 0; }

  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    readSensorsRaw();
    for (int i = 0; i < 8; i++) {
      if (sensorRaw[i] < sensorMin[i]) sensorMin[i] = sensorRaw[i];
      if (sensorRaw[i] > sensorMax[i]) sensorMax[i] = sensorRaw[i];
    }
    delay(5);
  }
}

// Verwerk eenvoudige BT-commando’s
void handleBT() {
  while (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();

    if (cmd == "s") { running = !running; SerialBT.printf("RUN=%d\n", running); }
    else if (cmd == "c") { motorsCoast(); calibrateSensors(); SerialBT.println("CAL OK"); }
    else if (cmd == "r") {
      SerialBT.printf("Kp=%.3f Ki=%.3f Kd=%.3f base=%d\n", Kp, Ki, Kd, baseSpeed);
    }
    else if (cmd.startsWith("p=")) { Kp = cmd.substring(2).toFloat(); SerialBT.printf("Kp=%.3f\n", Kp); }
    else if (cmd.startsWith("i=")) { Ki = cmd.substring(2).toFloat(); SerialBT.printf("Ki=%.3f\n", Ki); }
    else if (cmd.startsWith("d=")) { Kd = cmd.substring(2).toFloat(); SerialBT.printf("Kd=%.3f\n", Kd); }
    else if (cmd.startsWith("b=")) { baseSpeed = constrain(cmd.substring(2).toInt(), 0, 255); SerialBT.printf("base=%d\n", baseSpeed); }
  }
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32-LineBot");

  // Sensorgpio’s
  for (int i = 0; i < 8; i++) pinMode(sensorPins[i], INPUT);

  // (Optioneel) IR-control pin
  if (IR_CTRL >= 0) {
    pinMode(IR_CTRL, OUTPUT);           // voor 3.3 V gevoed QTR
    digitalWrite(IR_CTRL, HIGH);        // IR aan
    // Als je QTR op 5 V voedt en veilig wilt schakelen:
    // pinMode(IR_CTRL, OUTPUT_OPEN_DRAIN); digitalWrite(IR_CTRL, HIGH); // aan (hoog-impedant)
    // digitalWrite(IR_CTRL, LOW); // uit
  }

  // DRV8833 enable
//  pinMode(NSLEEP, OUTPUT);
 // digitalWrite(NSLEEP, HIGH); // driver aan

  //attachPWM();
  motorsCoast();

  // Startkalibratie
  Serial.println("Calibreren (1.5 s)...");
  calibrateSensors(1500);
  Serial.println("Klaar. Stuur 's' via BT om te starten.");
  SerialBT.println("LineBot klaar. 's' start/stop, 'c' calibrate, 'p=','i=','d=','b=' instellen, 'r' status.");
}

void loop() {
  handleBT();

  if (!running) {
    motorsCoast();
    delay(10);
    return;
  }

  // 1) Positie meten
  long pos = readLinePosition(); // 0..7000; midden ~3500
  long setpoint = 3500;

  // 2) PID
  float error = (float)setpoint - (float)pos;
  integral += error;
  float derivative = error - lastError;
  lastError = error;

  float correction = Kp*error + Ki*integral + Kd*derivative;

  // 3) Motorsnelheden
  int left  = baseSpeed + (int)correction;
  int right = baseSpeed - (int)correction;

  // 4) Output
  motorWrite(left, right);

  // (Optioneel) af en toe debug naar BT
  static uint32_t t0 = 0;
  if (millis() - t0 > 200) {
    t0 = millis();
    SerialBT.printf("pos=%ld err=%.0f L=%d R=%d\n", pos, error, left, right);
  }

  delay(5); // ~200 Hz loop
}
