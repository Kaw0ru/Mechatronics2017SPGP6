#include "Servo.h"
#include "Pixy.h"
#include "SPI.h"
#include "GateCmd.h"
#include "PID_v1.h"

Servo myServoY;
Pixy pixy;

const int DesiredLoc[3] = {160, 100, 10000}; //Desired y value of the color bar (Max y=199)
                                            //Desired x value of the color bar (Max x=319)
											//Desired area value of the color bar (Max area= 64000)
int x=0;
int y=0;
int width=0;
int height=0;
double angle=110;
int  i=0;
int signature;
int k=0;
int scanResult[4] = {0, 0, 0, 0};
int *P_scanRe=&scanResult[0];
int scanResultHist[4] ={0, 0, 0, 0};
int *P_scanReHis=&scanResultHist[0];
int CtlCmd[7]={0,0,0,0,0,0,0};
int *P_CtlCmd=&CtlCmd[0];
int RST = 0;

// Motor set
int DCMotorEncP[2]={32,30}; 
// define Motors
//Motor LeftMotor(13,12), RightMotor(11,10);
int EN1=13;
int DIR1=12;
int EN2=11;
int DIR2=10;//13 for EN1, 12 for DIR1, 11 for EN2, 10 for DIR2

// Motor control
//Left encoder def
const byte leftEncoderPin1 = 2;
const byte leftEncoderPin2 = 4;

//Right encoder def
const byte rightEncoderPin1 = 3;
const byte rightEncoderPin2 = 5;

//Left motor
int E_left = 13; //The enabling of L298PDC motor driver board connection to the digital interface port 13
int M_left = 12; //The enabling of L298PDC motor driver board connection to the digital interface port 12

//Right motor
int E_right = 11; //The enabling of L298PDC motor driver board connection to the digital interface port 11
int M_right = 10; //The enabling of L298PDC motor driver board connection to the digital interface port 10

byte LeftEncoderPin1Last;
byte RightEncoderPin1Last;

double leftDuration, leftAbs_duration;//the number of the pulses
double rightDuration, rightAbs_duration;//the number of the pulses

bool leftDirection;//the rotation direction 
bool rightDirection;//the rotation direction 

bool leftResult;
bool rightResult;

double leftThrottle;//Power supplied to the motor PWM value.
double rightThrottle;

double leftSetpoint;
double rightSetpoint;

double leftKs[3] = {1, 10, 0.2};
double rightKs[3] = {1, 10, 0.2};

PID leftPID(&leftAbs_duration, &leftThrottle, &leftSetpoint, leftKs[0], leftKs[1], leftKs[2], DIRECT);
PID rightPID(&rightAbs_duration, &rightThrottle, &rightSetpoint, rightKs[0], rightKs[1], rightKs[2],DIRECT);

void setup() {
  
   myServoY.attach(26);
   Serial.begin(9600);
   pixy.init();
  leftIni(0);
  rightIni(0);
   
}

void loop() {

	myServoY.write(angle);// write the angle


   Scan(P_scanRe,P_scanReHis); // Get and Store the scan reults
    
  GateCmd(P_scanRe, P_scanReHis, &DesiredLoc[0], k, P_CtlCmd);// give the control cmd
  k = CtlCmd[6]; // robustness para


  
  DCMotorCtl(P_CtlCmd);
  delay(10); //sample period


}

void Scan(int* p1, int* p2)
{ 
 for(int j = 0; j<4; j++)
   { 
  *(p2+j) = *(p1+j);
   }// Store results
  uint16_t blocks;
  blocks = pixy.getBlocks();  //receive data from pixy 
  signature = pixy.blocks[i].signature;    //get object's signature
  x = pixy.blocks[i].x;                    //get x position
  y = pixy.blocks[i].y;                    //get y position
 // Serial.println(y);
  width = pixy.blocks[i].width;            //get width
  height = pixy.blocks[i].height;          //get height
// Serial.println(height);
  int result[4]={x,y,width,height};
  for (int j=0;j<4;j++)
  {
    *(p1+j)=result[j];
  }
}

void DCMotorCtl(int* pointer)
{

   int FWD = *(pointer);
   int BWD = *(pointer+1);
   int Speed  = *(pointer+2);
   int Reached = *(pointer+3);
   int degree = *(pointer+4);
  double  DesiredSpeed1;
  double  DesiredSpeed2;

  if (Reached) 
  {
   DesiredSpeed1 = 0;
   DesiredSpeed2 = 0;
  }
  else {
    if (FWD){
      if (Speed){// High Speed
      DesiredSpeed1 = 120;
      DesiredSpeed2 = 120;
      }
      else{ // Low Speed
        DesiredSpeed1 = 20;
        DesiredSpeed2 = 20;
      }
      if (BWD == 1){
      	DesiredSpeed1 = -DesiredSpeed1;
        DesiredSpeed2 = -DesiredSpeed2;
      }
    }
    else{
     if (BWD == 0){
      if (degree>0){ // turn right
        if (degree>40)
        {
          DesiredSpeed1 = 60;
          DesiredSpeed2 = 0;
        }
        else
        {
         DesiredSpeed1 = 20;
          DesiredSpeed2 = 0;
        }
      }
      else{ // turn left
        if (abs(degree)>40)
        {
          DesiredSpeed1 = 0;
          DesiredSpeed2 = 60;
        }
        else
        {
         DesiredSpeed1 =  0;
          DesiredSpeed2 = 20;
        }
      }
  }
   else{
   	if (degree>0){ // turn right
        if (degree>40)
        {
          DesiredSpeed1 = 0;
          DesiredSpeed2 = 60;
        }
        else
        {
         DesiredSpeed1 = 0;
          DesiredSpeed2 = 20;
        }
      }
      else{ // turn left
        if (abs(degree)>40)
        {
          DesiredSpeed1 = 60;
          DesiredSpeed2 = 0;
        }
        else
        {
         DesiredSpeed1 =  20;
          DesiredSpeed2 = 0;
        }
      }
   }

    }
  }
  
  Move(DesiredSpeed1,DesiredSpeed2);

  
}

// Motor control
void leftIni(int setPt)
{
  // Serial.begin(9600);//Initialize the serial port
  pinMode(M_left, OUTPUT);   //L298P Control port settings DC motor driver board for the output mode
  pinMode(E_left, OUTPUT); 
  leftSetpoint = setPt;  //Set the output value of the PID
  leftPID.SetMode(AUTOMATIC);//PID is set to automatic mode
  leftPID.SetSampleTime(10);//Set PID sampling frequency is 100ms
  leftDirection = true;//default -> Forward  
  pinMode(leftEncoderPin2,INPUT);  
  attachInterrupt(0, leftWheelSpeed, CHANGE);
}

void rightIni(int setPt)
{
  pinMode(M_right, OUTPUT);   //L298P Control port settings DC motor driver board for the output mode
  pinMode(E_right, OUTPUT); 
  rightSetpoint = setPt;  //Set the output value of the PID
  rightPID.SetMode(AUTOMATIC);//PID is set to automatic mode
  rightPID.SetSampleTime(10);//Set PID sampling frequency is 100ms
  rightDirection = true;//default -> Forward  
  pinMode(leftEncoderPin2,INPUT);  
  attachInterrupt(1, rightWheelSpeed, CHANGE);
}

void leftWheelSpeed()
{
  int leftLstate = digitalRead(leftEncoderPin1);
  if((LeftEncoderPin1Last == LOW) && leftLstate==HIGH)
  {
    int val = digitalRead(leftEncoderPin2);
    if(val == LOW && leftDirection)
    {
      leftDirection = false; //Reverse
    }
    else if(val == HIGH && !leftDirection)
    {
      leftDirection = true;  //Forward
    }
  }
  LeftEncoderPin1Last = leftLstate;
 
  if(!leftDirection)  leftDuration++;
  else  leftDuration--;
}

void rightWheelSpeed()
{
  int rightLstate = digitalRead(rightEncoderPin1);
  if((RightEncoderPin1Last == LOW) && rightLstate == HIGH)
  {
    int val = digitalRead(rightEncoderPin2);
    if(val == LOW && rightDirection)
    {
      rightDirection = false; //Reverse
    }
    else if(val == HIGH && !rightDirection)
    {
      rightDirection = true;  //Forward
    }
  }
  RightEncoderPin1Last = rightLstate;
 
  if(!rightDirection)  rightDuration++;
  else  rightDuration--;

}

void leftMoveFwd()//Motor Forward
{
     digitalWrite(M_left,HIGH);
     analogWrite(E_left,leftThrottle);
}

void rightMoveFwd()//Motor Forward
{
     digitalWrite(M_right,HIGH);
     analogWrite(E_right,rightThrottle);
}

void leftMoveBwd()//Motor reverse
{
     digitalWrite(M_left,LOW);
     analogWrite(E_left,leftThrottle);
}

void rightMoveBwd()//Motor Forward
{
     digitalWrite(M_right,LOW);
     analogWrite(E_right,rightThrottle);
}

void leftStop()//Motor stops
{
     digitalWrite(E_left, LOW); 
}

void rightStop()
{
  digitalWrite(E_right, LOW);
}

void Move(int LsetPt,int RsetPt)
{
   if (LsetPt>0)
   {
    leftMoveFwd();
   }
   else if(LsetPt<0){
    leftMoveBwd();
   }
   else{
    leftStop();
   }
    leftSetpoint=abs(LsetPt);
    
    leftAbs_duration = abs(leftDuration);
    leftResult = leftPID.Compute();//PID conversion is complete and returns 1
    if(leftResult)
    {
//      Serial.print("Left Throttle: ");
//      Serial.println(leftThrottle); 
//      Serial.print("Left Pulse: ");
//      Serial.println(leftDuration); 
//       Serial.println(" "); 
      leftDuration = 0; //Count clear, wait for the next count
    }
   


    if (RsetPt>0)
   {
    rightMoveFwd();
   }
   else if(RsetPt<0){
    rightMoveBwd();
   }
   else{
    rightStop();
   }
    rightSetpoint=abs(RsetPt);
    
    rightAbs_duration = abs(rightDuration);
    rightResult = rightPID.Compute();//PID conversion is complete and returns 1
    if(rightResult)
    {

      rightDuration = 0; //Count clear, wait for the next count
    }
}



