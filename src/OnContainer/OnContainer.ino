#include "VirtualWire.h"
#include "Receiver.h"
#include "ServoTimer2.h"

ServoTimer2 gateServo;
bool doorState;

void setup() {
  // put your setup code here, to run once:
  SetupReceiver(8);
  gateServo.attach(9);
  //Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned int reading = ReceiveMsg();

  if (reading == 100)
  {
    doorState = 0;
  }
  else if (reading == 101)
  {
    doorState = 1;
  }
  //Serial.println(doorState);

  //Serial.println(doorState);
  
  if (doorState == 1)
  {
    gateServo.write(110);
  }
  else
  {
    gateServo.write(10);
  }
}
