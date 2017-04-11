#include <Servo.h>

Servo pixyCtl;

void setup()
{
  pixyCtl.attach(8);
}

void loop()
{
  for (int pos = 0; pos < 180; pos++)
  {
    pixyCtl.write(pos);
    delay(15);
  }
  for (int pos = 180; pos > 0; pos--)
  {
    pixyCtl.write(pos);
    delay(15);
  }
}

