#include "VirtualWire.h"
#include "Transmitter.h"

void setup() {
  // put your setup code here, to run once:
  SetupTransmitter(8);
  pinMode(7,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  bool isOpen = digitalRead(7);
  if (isOpen)
  {
    SendMsg(101);
  }
  else
  {
    SendMsg(100);
  }
  
}
