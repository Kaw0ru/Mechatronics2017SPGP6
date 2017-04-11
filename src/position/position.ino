#include "math.h"
#include "Servo.h"
#include "Pixy.h"
#include "SPI.h"

Servo myServo;
Pixy pixy;
int DesiredY=150;
int CurrentY;
int x=0;
int y=0;
int width=0;
int height=0;
double angle=0;
int  i=0;
int  Dec=150; // a constant which reflects the relationship between Y and angle
int signature;
int error;

void setup() {
  // put your setup code here, to run once:
   myServo.attach(9);
   Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Scan();
  Serial.println(y);
  angle=myServo.read();
  PixyYCtrl(DesiredY,y,angle);
  myServo.write(angle);
  delay(10);

}

void PixyYCtrl(int DesiredY, int CurrentY, double angle)
{
  error=DesiredY-CurrentY;
  if (abs(error)>=5)
  {
  angle=angle+atan(-error/Dec);
  }
}

void Scan()
{
  uint16_t blocks;
  blocks = pixy.getBlocks();  //receive data from pixy 
  signature = pixy.blocks[i].signature;    //get object's signature
  x = pixy.blocks[i].x;                    //get x position
  y = pixy.blocks[i].y;                    //get y position
  width = pixy.blocks[i].width;            //get width
  height = pixy.blocks[i].height;          //get height
}

