
#include "ServoTimer2.h"
#include "Pixy.h"
#include "SPI.h"
#include "Far.h"
#include "PID_v1.h"

#include "RF/RH_ASK.h"

RH_ASK driver (2000, 44,8,45,true);

ServoTimer2 myServoY, myServoGate;
Pixy pixy;


// laser set
int laserSensor = A0;  // analog pin used to detect laser
int laser = 22;   // digital pin used to turn laser on
int const DGateIdle = 100;
int const DGateShut = 101;
int DGatestate = DGateShut; // initial state of the gate is closed
int value;

// pixy set
const int DesiredLoc[2] = {160, 100}; //Desired y value of the ball (Max y=199)
                                      //Desired x value of the ball (Max x=319)
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

//judge if the robot is close to the ball
int Close=0;
int Close_count=0;
int Stop=0;
int Stop_count=0;
int RST=0; // if RST=1, reset the robot
int Speed=0; // High or Low Speed

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
int E_left = 13; //The enabling of L298PDC motor driver board connection to the digital interface port 5
int M_left = 12; //The enabling of L298PDC motor driver board connection to the digital interface port 4

//Right motor
int E_right = 11; //The enabling of L298PDC motor driver board connection to the digital interface port 5
int M_right = 10; //The enabling of L298PDC motor driver board connection to the digital interface port 4

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

// structure parameters
bool exeOnce = true;
unsigned long stopStartingTime = 0;
int globalState = 0; // global state, 0 for initial advance, 1 for search and catch the ball, 2 for search the gate, 3 for release the ball
unsigned long outOfViewStartingTime= millis();

void setup() {
   pinMode(laser, OUTPUT);
   pinMode(laserSensor, INPUT);
//   pinMode(EN1,OUTPUT);
//   pinMode(DIR1,OUTPUT);
//   pinMode(EN2,OUTPUT);
//   pinMode(DIR2,OUTPUT);
   myServoY.attach(26);
   myServoGate.attach(27);
   Serial.begin(9600);
   pixy.init();
  leftIni(0);
  rightIni(0);
  driver.init();
   
}

void loop() {
  Serial.println(globalState);
  if (globalState == 0)
  {
    Move(200,200);
    delay(2000);
    globalState = 1;   
  }
  else if (globalState == 1)
  {
    ScanAndCatchBall();
  }
  else if (globalState == 2)
  {
    leftStop();
    rightStop();
    //OpenGarageDoor(true);
    // SearchGate(); //=======================================
  }
}

void ScanAndCatchBall()
{
  if (!Stop)
   {
    RST=0;
    myServoY.write(angle);// write the angle
   //Serial.println(angle);
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

    if (angle<=7000)
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


   Scan(P_scanRe,P_scanReHis); // Get and Store the scan reults
    


  Far(P_scanRe, P_scanReHis, &DesiredLoc[0], k, &angle, P_CtlCmd, &outOfViewStartingTime, &globalState);// give the control cmd
  
  k = CtlCmd[3]; // robustness para
  angle=double(CtlCmd[2])/double(100); // ServoY angle
  
  if (k>=10) // the ball is out of the view
  {
    RST=1;
  }
  
  exeOnce = true;
  }
  else{ //Stop pixy's scaning
    myServoY.write(93);
   }

   if (exeOnce)
   {
      stopStartingTime = millis();
      exeOnce = false;
    }
  long stopCurrentTime = millis();
  long stopDeltaT = stopCurrentTime - stopStartingTime;
  Serial.println(stopDeltaT);
  Serial.println(" ");
  Serial.println(stopDeltaT);
  Serial.println(" ");
    Serial.println(stopDeltaT);
  Serial.println(" ");
    Serial.println(stopDeltaT);
  Serial.println(" ");
    Serial.println(stopDeltaT);
  Serial.println(" ");  
  Serial.println(stopDeltaT);
  Serial.println(" ");
  

  
  if (stopDeltaT > 3000)
    {
      RST=1;
    }
   
   if (Close==1)
   {
     digitalWrite (laser, HIGH); // open the laser head
   }
   else{
     digitalWrite (laser, LOW); // close the laser head
   }
   
  value = analogRead(laserSensor);   // reads the laser detection value
      // outputs the laser detection value to the serial monitor
  

  CarGateCtl(Close);
  
  if (RST==1) //reset
  {
    Close=0;
    Stop=0;
    Close_count=0;
    Stop_count=0;
    angle=93;
  }
  
  DCMotorCtl(CtlCmd[0], CtlCmd[1]);
  delay(10); //sample period
}




void Scan(int* p1, int* p2)
{ 
 for(int j = 0; j<4; j++)
   { 
  *(p2+j) = *(p1+j);
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
    *(p1+j)=result[j];
  }
}


void CarGateCtl(int if_close)
{
  if ((if_close)&&(RST==0))
  {
    DGatestate=DGateIdle;
   }
   else
    {
    DGatestate=DGateShut;
    }
  switch (DGatestate) {
    case DGateIdle:
      myServoGate.write(150); //openGate;                  
      // sets the servo position according to the scaled value
      if (value < 300) {
        DGatestate = DGateShut;
        RST=1;
      }
      break;
    case DGateShut:
      myServoGate.write(20); //closeGate;
       // sets the servo position according to the scaled value
      break;
  }
}



void DCMotorCtl(bool FWD, int degree)
{
  double  DesiredSpeed1;
  double  DesiredSpeed2;
  if (Stop) 
  {
   DesiredSpeed1 = 25;
   DesiredSpeed2 = 25;
  }
  else {
    if (FWD){
      if (Speed){// High Speed
      DesiredSpeed1 = 120;
      DesiredSpeed2 = 120;
      }
      else{ // Low Speed
        DesiredSpeed1 = 40;
        DesiredSpeed2 = 40;
      }
    }
    else{
      if (degree>0){ // turn right
        if (degree>20)
        {
          DesiredSpeed1 = 60;
          DesiredSpeed2 = 0;
        }
        else
        {
         DesiredSpeed1 = 30;
          DesiredSpeed2 = 0;
        }
      }
      else{ // turn left
        if (abs(degree)>20)
        {
          DesiredSpeed1 = 0;
          DesiredSpeed2 = 60;
        }
        else
        {
         DesiredSpeed1 =  0;
          DesiredSpeed2 = 30;
        }
      }
    }
  }
  
  Move(DesiredSpeed1,DesiredSpeed2);

  
}

//Garage Door Ctl

void OpenGarageDoor(bool toOpen)
{
  const uint8_t* op = 1;
  const uint8_t* cl = 0;
  if(toOpen)
  {
    driver.send((uint8_t *)op, strlen(op));
  }
  else
  {
    driver.send((uint8_t *)cl, strlen(cl));
  }
  Serial.println("OPEN");
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
//      Serial.print("Right Throttle: ");
//      Serial.println(rightThrottle); 
//     
//      Serial.print("Right Pulse: ");
//      Serial.println(rightDuration); 
//      Serial.println("  "); 
      rightDuration = 0; //Count clear, wait for the next count
    }
}
