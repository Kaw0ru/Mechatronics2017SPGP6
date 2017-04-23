#include <Servo.h>

Servo pixyCtl;

void setup()
{
  pixyCtl.attach(8);
}

void loop()
{
  pixyCtl.write(0);

  delay(2000);

  pixyCtl.write(150);
  delay(2000);
}

