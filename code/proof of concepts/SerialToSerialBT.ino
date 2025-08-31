//BLUETOOTH
  #include "BluetoothSerial.h"
  String device_name = "ESP32_slave_Lucas";
//SENSOR
  const int sensorPins[] = {34, 35, 32, 33, 25, 26, 27, 14}; 
  const int numSensors = 8; // Aantal sensoren

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif


#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

void setup() {
  //BLUETOOTH
    Serial.begin(9600);
    SerialBT.begin(device_name);  //Bluetooth device name
    Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
  //SENSORWAARDEN
    for (int i = 0; i < numSensors; i++) 
    {
    pinMode(sensorPins[i], INPUT);
    }

    
}

void loop() {
  //BLUETOOTH
    if (Serial.available()) {
      SerialBT.write(Serial.read());
      }
    if (SerialBT.available()) {
      Serial.write(SerialBT.read());
    }
      delay(20);

  // SENSORWAARDEN
    int sensorValues[numSensors];
    for (int i = 0; i < numSensors; i++) 
    {
    sensorValues[i] = analogRead(sensorPins[i]);
    }

    Serial.print("Sensor waarden: ");
    for (int i = 0; i < numSensors; i++) {
    Serial.print("D");
    Serial.print(": ");
    Serial.print(sensorValues[i]);
    if (i < numSensors - 1) Serial.print(", ");
    }
    Serial.println();
    delay(100);








}
