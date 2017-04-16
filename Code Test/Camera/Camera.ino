#include <Camera.h>
#include "SPI.h"


void setup()
{
  Camera pixy;
  Serial.begin(9600);
}


void loop()
{
  int result;
  result = Camera::ScanBall();
  Serial.println(result);
}
