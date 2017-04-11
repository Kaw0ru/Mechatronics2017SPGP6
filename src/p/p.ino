#include <Camera.h>



Camera pixy

void setup()
{ 
  Serial.begin(9600);
  pixy.Initialize();
}

void loop()
{ 
  int result[4];
  result = pixy.ScanBall();
  Serial.println(result);
}
