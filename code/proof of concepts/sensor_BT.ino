// Pinnen toewijzing voor de QTR-8A sensoren
const int sensorPins[] = {34, 35, 32, 33, 25, 26, 27, 14}; 
const int numSensors = 8; // Aantal sensoren


#include "BluetoothSerial.h"
char cmd;
BluetoothSerial serialBT;


void setup() {
  serialBT.begin("Esp32-BT");
  pinMode(2, OUTPUT);


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



if(serialBT.available()){
    cmd = serialBT.read();
  }
  if (cmd == '1'){
    digitalWrite(2, HIGH);
    serialBT.println();

      serialBT.print("Sensor waarden: ");
  for (int i = 0; i < numSensors; i++) {
    Serial.print("D");
    //Serial.print(sensorPins[i]);
    serialBT.print(": ");
    serialBT.print(sensorValues[i]);
    if (i < numSensors - 1) Serial.print(", ");
  }
  serialBT.println();

  }
  if (cmd == '0'){
    digitalWrite(2, LOW);
  }





  // Wacht 1 seconde
  delay(100);
}
