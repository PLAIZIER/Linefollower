// Pinnen toewijzing voor de QTR-8A sensoren
const int sensorPins[] = {34, 35, 32, 33, 25, 26, 27, 14}; 
const int numSensors = 8; // Aantal sensoren

void setup() {
  // Initialiseer de seriële monitor
  Serial.begin(115200);
  
  // Stel alle sensorpinnen in als invoer
  for (int i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  Serial.println("QTR-8A Sensor Waarden (elke seconde):");
}

void loop() {
  // Array om sensorwaarden op te slaan
  int sensorValues[numSensors];

  // Lees de waarden van alle sensoren
  for (int i = 0; i < numSensors; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }

  // Print de waarden naar de seriële monitor
  Serial.print("Sensor waarden: ");
  for (int i = 0; i < numSensors; i++) {
    Serial.print("D");
    //Serial.print(sensorPins[i]);
    Serial.print(": ");
    Serial.print(sensorValues[i]);
    if (i < numSensors - 1) Serial.print(", ");
  }
  Serial.println();

  // Wacht 1 seconde
  delay(20);
}
