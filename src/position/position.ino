
#include "Servo.h"
#include "Pixy.h"
#include "SPI.h"
#include "Far.h"
#include "CarGateCtl.h"
#include "PixyServoCtl.h"
#include "Reset.h"
#include "LaserCtl.h"

Servo myServoY, myServoGate;
Pixy pixy;
int laserPin[2] = {3, A0};   // digital pin used to turn laser on
//int const DGateIdle = 100;
//int const DGateShut = 101;
//int DGatestate = DGateShut;
//int value;
//int Delay_time=0;

const int DesiredLoc[2] = {160, 100}; //Desired y value of the ball (Max y=199)
// const int DesiredX=160; //Desired x value of the ball (Max x=319)
int x=0;
int y=0;
double angle=93;
int  i=0;
int signature;
int k=0;
int scanResult[4] = {0, 0, 0, 0};
int *P_scanRe=&scanResult[0];
int scanResultHist[4] ={0, 0, 0, 0};
int *P_scanReHis=&scanResultHist[0];
int CtlCmd[4]={0,0,0,0};
int *P_CtlCmd=&CtlCmd[0];

//int Close=0;
int Close_count=0;
//int Stop=0;
//int Stop_count=0;
//int RST=0;

int isBlocked = 0;

void setup() {
   pinMode(laser, OUTPUT);
   pinMode(laserSensor, INPUT);
   myServoY.attach(9);
   myServoGate.attach(10);
   Serial.begin(9600);
   pixy.init();
}

void loop() {
  Scan(P_scanRe, P_scanReHis); // Get current scan reults
  Far(P_scanRe, P_scanReHis, &DesiredLoc[0], k, &angle, P_CtlCmd);// give the control cmd
  k = CtlCmd[3]; //Shift register of Far function

  if (k>=10)
  {
    Reset(); //Reset pixy servo, speed & car gate.
  }

  //Serial.println(k);
  angle = double(CtlCmd[2])/double(100);

  angle = PixyServoCtl(angle); // control tilting angle of pixy

  if (angle < 88)
  {
    Close_count++;
  }
  else
  { 
    if (Close_count<5)
    {
    Close_count=0;
    }
  }

  if (Close_count >= 5) // ball is near the vehicle
  {
    CarGateCtl(1); // open gate
    LaserCtl(&laserPin[0], 1, &isBlocked); //enable laser
  }
  else
  {
    CarGateCtl(0); //Close gate
    LaserCtl(&laserPin[0], 0, &isBlocked); //disable laser
  }

  if (isBlocked)
  {
    Reset();
  }
}




void Scan(int* p, int* p_hist)
{ 
  for(int j = 0; j<4; j++)
   { 
      *(p_hist+j) = *(p+j);
   }// Store results
  int width=0;
  int height=0;
  uint16_t blocks;
  blocks = pixy.getBlocks();  //receive data from pixy 
  signature = pixy.blocks[i].signature;    //get object's signature
  x = pixy.blocks[i].x;                    //get x position
  y = pixy.blocks[i].y;                    //get y position
  width = pixy.blocks[i].width;            //get width
  height = pixy.blocks[i].height;          //get height
  int result[4]={x,y,width,height};
  for (int j=0;j<4;j++)
  {
    *(p+j)=result[j];
  }
}
