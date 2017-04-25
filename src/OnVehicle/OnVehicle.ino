#include "VirtualWire.h"
#include "Transmitter.h"

void setup() {
  // put your setup code here, to run once:
  SetupTransmitter(8);
  pinMode(7,INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  bool isOpen = digitalRead(7);
  Serial.println(isOpen);
  if (isOpen)
  {
    SendMsg(101);
  }
  else
  {
    SendMsg(100);
  }
  
}
