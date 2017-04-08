#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int laserSensor = 0;  // analog pin used to detect laser
int laser = 3;   // digital pin used to turn laser on

int const gateIdle = 100;
int const gateShut = 101;
int state = gateIdle;

void setup()
{
  Serial.begin(9600);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  pinMode(laser, OUTPUT);   // sets the laser pin as an output
}

void loop()
{
  digitalWrite (laser, HIGH); // open the laser head
  int value = analogRead(laserSensor);   // reads the laser detection value
  Serial.println(value);    // outputs the laser detection value to the serial monitor
  delay(15);                           // waits for the servo to get there

  switch (state) {
    case gateIdle:
      openGate();                  // sets the servo position according to the scaled value
      if (value < 300) {
        state = gateShut;
      }
      break;
    case gateShut:
      closeGate();                  // sets the servo position according to the scaled value
      break;
  }

}

void openGate() {
  myservo.write(140);
}

void closeGate() {
  myservo.write(20);
}

