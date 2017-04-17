
#include "Servo.h"
#include "Pixy.h"
#include "SPI.h"
#include "Far.h"
#include "DCMotorCtl.h"

Servo myServoY, myServoGate;
Pixy pixy;
int laserSensor = 0;  // analog pin used to detect laser
int laser = 3;   // digital pin used to turn laser on
int const DGateIdle = 100;
int const DGateShut = 101;
int DGatestate = DGateShut;
int value;
int Delay_time=0;

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

int Close=0;
int Close_count=0;
int Stop=0;
int Stop_count=0;
int RST=0;
int Speed=0;

int DCMotorPin[4]={2,25,7,24}; //2 for EN1, 25 for DIR1, 7 for EN2, 24 for DIR2

void setup() {
   pinMode(laser, OUTPUT);
   pinMode(laserSensor, INPUT);
   myServoY.attach(9);
   myServoGate.attach(10);
   Serial.begin(9600);
   pixy.init();
}

void loop() {
   if (!Stop)
   {
    RST=0;
    if (!Close){
    digitalWrite (laser, LOW);// Laser
    }
   myServoY.write(angle);// write the angle
   Serial.println(angle);
   angle=100*myServoY.read();// read the angle
   
   if (angle<=8800)
   {
   
     Close_count++;
   }
   else
   {
    Close_count=0;
    Speed=1;
   } // judge if the car is close to the balls

    if (angle<=7800)
    {
      Stop_count++;
    }
    else{
      Stop_count=0;
    }// judge if the car is very close to the balls
    
   if (Close_count>=5)
   {
    Close=1;
    Speed=0;
   }

   if (Stop_count>=5)
   {
    Stop=1;
   }

   
   for(int j = 0; j<4; j++)
   { 
  *(P_scanReHis+j) = *(P_scanRe+j);
   }// Store results

    Scan(P_scanRe); // Get current scan reults
    
 // Serial.println(scanResult[1]);
//  Serial.println(scanResultHist[1]);

  Far(P_scanRe, P_scanReHis, &DesiredLoc[0], k, &angle, P_CtlCmd);// give the control cmd
  
  k = CtlCmd[3];
  if (k>=10)
  {
    RST=1;
  }
  //Serial.println(k);
  double  Angle = double(CtlCmd[2]); 
  angle=double(Angle)/double(100);
  Serial.println(' ');
   }//Stop
   else{
    myServoY.write(93);
   }
   if (Close==1)
   {
     digitalWrite (laser, HIGH); // open the laser head
   }
  value = analogRead(laserSensor);   // reads the laser detection value
  //Serial.println(value);    // outputs the laser detection value to the serial monitor
  //Serial.println(RST);
  //Serial.println(DGatestate);
  OpenCarGate(Close);
  
  if (RST==1)
  {
    Close=0;
    Stop=0;
    k=10;
    angle=93;
  }
  DCMotorCtl(CtlCmd[0], Speed, CtlCmd[1], Stop, &DCMotorPin[0]);
  delay(10);
  Serial.println(Stop);
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

void OpenCarGate(int if_close)
{
  if ((if_close)&&(RST==0))
  {
    DGatestate=DGateIdle;
   // Serial.println("Idle");
   }
   else
    {
    DGatestate=DGateShut;
    }
  switch (DGatestate) {
    case DGateIdle:
      openGate();                  // sets the servo position according to the scaled value
      if (value < 300) {
   //     Serial.println("ready");
        Delay_time=300;
        DGatestate = DGateShut;
        RST=1;
      }
      break;
    case DGateShut:
      closeGate();
      //delay(Delay_time);
       Delay_time=0;
    //  Serial.println(DGatestate);
       // sets the servo position according to the scaled value
      break;
  }
}
void openGate() {
  myServoGate.write(150);
}

void closeGate() {
  myServoGate.write(20);
}


