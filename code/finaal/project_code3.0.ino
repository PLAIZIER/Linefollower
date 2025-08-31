/*
  ESP32 Line Follower – QTR-8A + DRV8833 + Bluetooth SPP
  ZONDER ledc* functies (software-PWM in loop)

  Pins (pas aan indien nodig):
    QTR-8A OUTs -> {34,35,32,33,25,26,14,39}  (34-39 input-only; 25/26/14 = ADC2, niet met WiFi)
    DRV8833: AIN1=22, AIN2=21, BIN1=19, BIN2=18, nSLEEP=23
    IR_CTRL = -1 (uit); zet naar b.v. 27 en zet HIGH in setup() als je IR-LEDs wil schakelen

  Bluetooth commando's:
    1      -> print éénmalig de 8 RAW sensorwaarden
    s      -> start/stop
    c      -> (her)kalibreren (beweeg over wit en zwart tijdens 2 s)
    r      -> toon PID/basespeed
    p= i= d= b=   -> stel PID en baseSpeed in (bv. p=1.2)
    b+  b-        -> baseSpeed +10 / -10
    max=###       -> absolute snelheidslimiet (0..255), b.v. max=220
    slow / fast   -> snelheids-presets
    stop          -> noodstop (running=false)
*/

#include <BluetoothSerial.h>
BluetoothSerial SerialBT;

// ====== CONFIG ======
// Jouw situatie: zwart ≈ 4095 (hoger), wit ≈ 2000 -> NIET inverteren:
const bool LINE_IS_DARK = false;

const int  BASE_SPEED_DEFAULT = 160;

// QTR-8A sensorpins
//const int sensorPins[8] = {34, 35, 32, 33, 25, 26, 27, 14};
const int sensorPins[8] = {14, 27, 26, 25, 33, 32, 35, 34};

const int IR_CTRL = -1;      // zet evt. naar 27 en zet HIGH in setup()

// DRV8833 pins
const int AIN1 = 22;
const int AIN2 = 21;
const int BIN1 = 19;
const int BIN2 = 18;
const int NSLEEP = 23;       // enable (hoog = aan)

// ====== Software PWM ======
const uint32_t PWM_PERIOD_US = 2000; // 500 Hz
uint8_t dutyA1=0, dutyA2=0, dutyB1=0, dutyB2=0;
uint32_t pwmStartUs = 0;

inline void updateSoftPWM() {
  uint32_t now = micros();
  uint32_t el  = now - pwmStartUs;
  if (el >= PWM_PERIOD_US) { pwmStartUs = now; el = 0; }

  uint32_t thA1 = (uint32_t)dutyA1 * PWM_PERIOD_US / 255;
  uint32_t thA2 = (uint32_t)dutyA2 * PWM_PERIOD_US / 255;
  uint32_t thB1 = (uint32_t)dutyB1 * PWM_PERIOD_US / 255;
  uint32_t thB2 = (uint32_t)dutyB2 * PWM_PERIOD_US / 255;

  digitalWrite(AIN1, (el < thA1) ? HIGH : LOW);
  digitalWrite(AIN2, (el < thA2) ? HIGH : LOW);
  digitalWrite(BIN1, (el < thB1) ? HIGH : LOW);
  digitalWrite(BIN2, (el < thB2) ? HIGH : LOW);
}

// ====== VARS ======
int   baseSpeed = BASE_SPEED_DEFAULT;
int   maxDuty   = 220;     // absolute begrenzer (0..255)
const int stepB = 10;      // stapgrootte voor b+/b-

float Kp = 1.2f, Ki = 0.0f, Kd = 6.0f;

int  sensorRaw[8];
int  sensorMin[8], sensorMax[8];
long lastPos = 3500;
bool running = false;

float integral = 0.0f;
float lastError = 0.0f;

// ====== Motor helpers (software-PWM) ======
void motorWrite(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  int LA = abs(left), RA = abs(right);

  if (left >= 0) { dutyA1 = LA; dutyA2 = 0; }
  else           { dutyA1 = 0;  dutyA2 = LA; }

  if (right >= 0){ dutyB1 = RA; dutyB2 = 0; }
  else           { dutyB1 = 0;  dutyB2 = RA; }
}

void motorsCoast() { dutyA1=dutyA2=dutyB1=dutyB2=0; }

// ====== Sensoren ======
void readSensorsRaw() {
  for (int i = 0; i < 8; i++) sensorRaw[i] = analogRead(sensorPins[i]);
}

long readLinePosition() {
  readSensorsRaw();
  long weightedSum = 0, sum = 0;

  for (int i = 0; i < 8; i++) {
    int v = sensorRaw[i];
    int minv = sensorMin[i], maxv = sensorMax[i];
    int scaled = 0;
    if (maxv > minv) {
      // schaal 0..1000: wit ~0, zwart ~1000 (zwart groter dan wit)
      scaled = (int)((long)(v - minv) * 1000L / (maxv - minv));
      if (scaled < 0) scaled = 0; if (scaled > 1000) scaled = 1000;
    }
    // niet inverteren (LINE_IS_DARK=false), maar laat de optie staan:
    if (LINE_IS_DARK) scaled = 1000 - scaled;

    sum += scaled;
    weightedSum += (long)scaled * (i * 1000); // 0..7000
  }
  if (sum == 0) return lastPos; // lijn kwijt -> houd laatste koers
  lastPos = weightedSum / sum;
  return lastPos;
}

void calibrateSensors(unsigned long ms = 2000) {
  for (int i = 0; i < 8; i++) { sensorMin[i] = 4095; sensorMax[i] = 0; }
  uint32_t t0 = millis();
  SerialBT.println("Kalibreren: beweeg over wit <-> zwart...");
  while (millis() - t0 < ms) {
    readSensorsRaw();
    for (int i = 0; i < 8; i++) {
      if (sensorRaw[i] < sensorMin[i]) sensorMin[i] = sensorRaw[i];
      if (sensorRaw[i] > sensorMax[i]) sensorMax[i] = sensorRaw[i];
    }
    // PWM in leven houden
    for (int k=0;k<3;k++){ updateSoftPWM(); delay(1); }
  }
  SerialBT.println("CAL OK");
}

// ====== Bluetooth commando's ======
void handleBT() {
  while (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n'); cmd.trim();

    if (cmd == "1") {
      // éénmalig RAW waarden
      readSensorsRaw();
      SerialBT.print("RAW: ");
      for (int i=0;i<8;i++) {
        SerialBT.print(sensorRaw[i]);
        if (i<7) SerialBT.print('\t');
      }
      SerialBT.print("\n");
    }
    else if (cmd == "s") { running = !running; SerialBT.printf("RUN=%d\n", running); }
    else if (cmd == "stop") { running = false; motorsCoast(); SerialBT.println("STOPPED"); }
    else if (cmd == "c") { motorsCoast(); calibrateSensors(); }
    else if (cmd == "r") { SerialBT.printf("Kp=%.3f Ki=%.3f Kd=%.3f base=%d max=%d\n", Kp, Ki, Kd, baseSpeed, maxDuty); }

    else if (cmd.startsWith("p=")) { Kp = cmd.substring(2).toFloat(); SerialBT.printf("Kp=%.3f\n", Kp); }
    else if (cmd.startsWith("i=")) { Ki = cmd.substring(2).toFloat(); SerialBT.printf("Ki=%.3f\n", Ki); }
    else if (cmd.startsWith("d=")) { Kd = cmd.substring(2).toFloat(); SerialBT.printf("Kd=%.3f\n", Kd); }
    else if (cmd.startsWith("b=")) { baseSpeed = constrain(cmd.substring(2).toInt(), 0, 255); SerialBT.printf("base=%d\n", baseSpeed); }

    else if (cmd == "b+") { baseSpeed = min(baseSpeed + stepB, 255); SerialBT.printf("base=%d\n", baseSpeed); }
    else if (cmd == "b-") { baseSpeed = max(baseSpeed - stepB, 0);   SerialBT.printf("base=%d\n", baseSpeed); }

    else if (cmd.startsWith("max=")) {
      maxDuty = constrain(cmd.substring(4).toInt(), 0, 255);
      SerialBT.printf("max=%d\n", maxDuty);
    }
    else if (cmd == "slow") {
      baseSpeed = 120; maxDuty = 160;
      SerialBT.printf("mode=slow base=%d max=%d\n", baseSpeed, maxDuty);
    }
    else if (cmd == "fast") {
      baseSpeed = 200; maxDuty = 255;
      SerialBT.printf("mode=fast base=%d max=%d\n", baseSpeed, maxDuty);
    }
  }
}

// ====== Setup/Loop ======
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32-LineBot");

  // Betere ADC-instellingen voor 0..3.3V
  analogReadResolution(12);           // 0..4095
  analogSetAttenuation(ADC_11db);     // volledige schaal ~3.3V

  for (int i = 0; i < 8; i++) pinMode(sensorPins[i], INPUT);
  if (IR_CTRL >= 0) { pinMode(IR_CTRL, OUTPUT); digitalWrite(IR_CTRL, HIGH); }

  pinMode(NSLEEP, OUTPUT); digitalWrite(NSLEEP, HIGH);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  motorsCoast();
  pwmStartUs = micros();

  Serial.println("Calibreren (2.0 s)...");
  calibrateSensors(2000);
  Serial.println("Klaar. Stuur 's' om te starten. '1' voor RAW, 'b+/b-' voor snelheid.");
  SerialBT.println("LineBot klaar. 's' start/stop, '1' RAW once, 'c' calibrate, 'p=','i=','d=','b=', 'b+','b-','max=','slow','fast','r','stop'.");
}

void loop() {
  handleBT();

  if (!running) {
    motorsCoast();
    updateSoftPWM();
    delay(1);
    return;
  }

  // 1) Positie op de lijn (0..7000; midden ~3500)
  long pos = readLinePosition();
  long setpoint = 3500;

  // 2) PID
  float error = (float)setpoint - (float)pos;
  integral += error;
  // anti-windup
  if (integral > 2000) integral = 2000;
  if (integral < -2000) integral = -2000;

  float derivative = error - lastError;
  lastError = error;

  float correction = Kp*error + Ki*integral + Kd*derivative;

  // 3) Snelheden
  int left  = baseSpeed + (int)correction;
  int right = baseSpeed - (int)correction;

  // Proportionele begrenzer op maxDuty (behoud links/rechts verhouding)
  int m = max(abs(left), abs(right));
  if (m > maxDuty && m > 0) {
    left  = (left  * maxDuty) / m;
    right = (right * maxDuty) / m;
  }

  // 4) Uitgang
  motorWrite(left, right);

  // Debug (optioneel, 5 Hz)
  static uint32_t t0=0;
  if (millis()-t0 > 200) {
    t0 = millis();
    SerialBT.printf("pos=%ld err=%.0f L=%d R=%d base=%d max=%d\n", pos, error, left, right, baseSpeed, maxDuty);
  }

  // PWM “tick”
  updateSoftPWM();
  delay(1);
}
