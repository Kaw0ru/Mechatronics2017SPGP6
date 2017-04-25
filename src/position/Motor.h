#include "PID_v1.h"

//Left encoder def
const byte leftEncoderPin1 = 2;
const byte leftEncoderPin2 = 4;

//Right encoder def
const byte leftEncoderPin1 = 3;
const byte leftEncoderPin2 = 5;

//Left motor
int E_left = 33; //The enabling of L298PDC motor driver board connection to the digital interface port 5
int M_left = 34; //The enabling of L298PDC motor driver board connection to the digital interface port 4

//Right motor
int E_right = 35; //The enabling of L298PDC motor driver board connection to the digital interface port 5
int M_right = 36; //The enabling of L298PDC motor driver board connection to the digital interface port 4

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

double leftKs[3] = {0.6, 5, 0};
double rightKs[3] = {0.6, 5, 0};

PID leftPID(&leftAbs_duration, &leftThrottle, &leftSetpoint, leftKs[0], leftKs[1], leftKs[2], DIRECT);
PID rightPID(&rightAbs_duration, &rightThrottle, &rightSetpoint, rightKs[0], rightKs[1], rightKs[2],DIRECT);

void leftIni(int setPt)
{
	// Serial.begin(9600);//Initialize the serial port
	pinMode(M_left, OUTPUT);   //L298P Control port settings DC motor driver board for the output mode
	pinMode(E_left, OUTPUT); 
	leftSetpoint = setPt;  //Set the output value of the PID
	leftPID.SetMode(AUTOMATIC);//PID is set to automatic mode
	leftPID.SetSampleTime(100);//Set PID sampling frequency is 100ms
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
	rightPID.SetSampleTime(100);//Set PID sampling frequency is 100ms
	rightDirection = true;//default -> Forward  
	pinMode(leftEncoderPin2,INPUT);  
	attachInterrupt(1, rightWheelSpeed, CHANGE);
}

void leftWheelSpeed()
{
  int leftLstate = digitalRead(leftEncoderPin1);
  if((leftEncoderPin1Last == LOW) && leftLstate==HIGH)
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
  leftEncoderPin1Last = leftLstate;
 
  if(!leftDirection)  leftDuration++;
  else  leftDuration--;
}

void rightWheelSpeed()
{
  int rightLstate = digitalRead(rightEncoderPin1);
  if((rightEncoderPin1Last == LOW) && rightLstate == HIGH)
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
  rightEncoderPin1Last = rightLstate;
 
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

void Move(int setPt)
{
	  leftMoveFwd();
    leftAbs_duration = abs(leftDuration);
    leftResult = leftPID.Compute();//PID conversion is complete and returns 1
    if(leftResult)
    {
    	Serial.print("Left Pluse: ");
    	Serial.println(leftDuration); 
    	leftDuration = 0; //Count clear, wait for the next count
    }

    rightMoveFwd();
    rightAbs_duration = abs(rightDuration);
    rightResult = rightPID.Compute();//PID conversion is complete and returns 1
    if(rightResult)
    {
    	Serial.print("Right Pluse: ");
    	Serial.println(rightDuration); 
    	rightDuration = 0; //Count clear, wait for the next count
    }
}
