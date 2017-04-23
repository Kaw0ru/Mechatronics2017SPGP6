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

void setup() {
  // put your setup code here, to run once:
  leftIni(0);
  rightIni(0);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i = 1; i < 255; i++)
  {
  rightThrottle = i;
  leftThrottle = 0;
  //leftMoveFwd();
  rightMoveFwd();
  rightWheelSpeed();
  //leftWheelSpeed();
  Serial.print(rightThrottle);
  Serial.println(rightDuration);
  //Serial.println(leftThrottle);
  //Serial.println(leftDuration);
  delay(500);
  }
}


// Motor control
void leftIni(int setPt)
{
  // Serial.begin(9600);//Initialize the serial port
  pinMode(M_left, OUTPUT);   //L298P Control port settings DC motor driver board for the output mode
  pinMode(E_left, OUTPUT); 
  leftSetpoint = setPt;  //Set the output value of the PID
  //leftPID.SetMode(AUTOMATIC);//PID is set to automatic mode
  //leftPID.SetSampleTime(10);//Set PID sampling frequency is 100ms
  leftDirection = true;//default -> Forward  
  pinMode(leftEncoderPin2,INPUT);  
  attachInterrupt(0, leftWheelSpeed, CHANGE);
}

void rightIni(int setPt)
{
  pinMode(M_right, OUTPUT);   //L298P Control port settings DC motor driver board for the output mode
  pinMode(E_right, OUTPUT); 
  rightSetpoint = setPt;  //Set the output value of the PID
  //rightPID.SetMode(AUTOMATIC);//PID is set to automatic mode
  //rightPID.SetSampleTime(10);//Set PID sampling frequency is 100ms
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
