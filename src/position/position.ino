
#include "Servo.h"
#include "Pixy.h"
#include "SPI.h"

Servo myServo;
Pixy pixy;
const int DesiredY=100; //Desired y value of the ball (Max y=199)
const int DesiredX=160; //Desired x value of the ball (Max x=319)
int x=0;
int y=0;
int width=0;
int height=0;
double angle=100;
int  i=0;
int  Dec=100; // a constant which reflects the relationship between Y and angle
int signature;
int errorX;
int errorY;
int k=0;
int last_y;
double delta;

void setup() {
  // put your setup code here, to run once:
   myServo.attach(9);
   Serial.begin(9600);
   pixy.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  Scan();
  if (last_y==y){
  k=k+1;
  }
  else
  {k=0;}// to check how many times y keeps immutable
   Serial.println(x);
  Serial.println(y);
  angle=myServo.read();
  
   if (k>=10){
    angle=100;
  } 
  //if y is immutable, which means the ball is out of the view, 
  // we need to reset the value of the angle
  
  angle=PixyYCtrl(DesiredY,y,angle);// new angle
  
  myServo.write(angle);//angle output
  
  Serial.println(angle);
  Serial.println('\n');
  last_y=y;//
  delay(10);

}

double PixyYCtrl(int DesiredY, int CurrentY, double angle)
// control the y value of the ball
{
  errorY=DesiredY-CurrentY;// error between desired_y and y
  
  if (abs(errorY)>=8) //to avoid the tiny disturbance
  {
  delta=double(errorY)/double(Dec);
  delta=9*atan(delta)/3.14; // delta angle
  angle=angle+delta;
  return angle;
  }
}
 
void PixyXCtrl(int DesiredX, int CurrentX) 
// control the x value of the ball
{
  errorX=DesiredX-CurrentX;// error between desired_x and x
  if (abs(errorX)>=10) //to avoid the tiny disturbance
  {
    if (errorX>0){
      // turn left
    }
    else{
      //turn right
    }
 
  }
  else{
    // go straight
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

