#include "BluetoothSerial.h"


char cmd;
BluetoothSerial serialBT;
void setup() {
  // put your setup code here, to run once:

  serialBT.begin("Esp32-BT");
  pinMode(2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(serialBT.available()){
    cmd = serialBT.read();
  }
  if (cmd == '1'){
    digitalWrite(2, HIGH);
    serialBT.println("2HIGH");

  }
  if (cmd == '0'){
    digitalWrite(2, LOW);
  }
  delay(100);
}
