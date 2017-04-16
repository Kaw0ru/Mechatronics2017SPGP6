
#include "Servo.h"
#include "Pixy.h"
#include "SPI.h"
#include "Far.h"

Servo myServo;
Pixy pixy;
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

void setup() {
   myServo.attach(9);
   Serial.begin(9600);
   pixy.init();
}

void loop() {
   myServo.write(angle);// write the angle
   Serial.println(angle);
   delay(10);
   
   angle=100*myServo.read();// read the angle
   
   for(int j = 0; j<4; j++)
   { 
  *(P_scanReHis+j) = *(P_scanRe+j);
   }// Store results

    Scan(P_scanRe); // Get current scan reults
    
  Serial.println(scanResult[1]);
  Serial.println(scanResultHist[1]);

  Far(P_scanRe, P_scanReHis, &DesiredLoc[0], k, &angle, P_CtlCmd);// give the control cmd
  
  k = CtlCmd[3];
  Serial.println(k);
  double  Angle = double(CtlCmd[2]); 
  angle=double(Angle)/double(100);
  Serial.println(' ');
}




void Scan(int* p)
{
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

