#include "VirtualWire.h"
#include "Transmitter.h"

void setup() {
  // put your setup code here, to run once:
  SetupTransmitter(2);
}

void loop() {
  // put your main code here, to run repeatedly:
  SendMsg(100);
}
