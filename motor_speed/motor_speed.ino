const byte encA = 2;
const byte encB = 4;

const double wheelDis = 0.01767; // units are meters per pulses 
const int E1 = 6;
const int M1 = 7;
long count = 0;
long currentTime = 0;
long lastTime = 0;
int deltaT = 0;
double avgDeltaT = 0;
int lastDeltaT = 0;
double speed1 = 0;

boolean encAState = HIGH;
boolean lencAState = HIGH;

void setup(){
  pinMode(encA, INPUT);
  pinMode(encB, INPUT);
  pinMode(M1, OUTPUT);
  Serial.begin(9600);
}

void loop(){
  analogWrite(E1, 255/2);
  digitalWrite(M1, HIGH);
  encAState = digitalRead(encA);
  if (encAState != lencAState){
    count++;
    lastTime = currentTime;
    currentTime = millis(); 
  
  // calculating the average time
  lastDeltaT = deltaT;
  deltaT = currentTime - lastTime;
  avgDeltaT = (double(deltaT) + double(lastDeltaT))/double(2); // units are in pulses/milliseconds

  //calculating the speed
  if (avgDeltaT == 0){
    Serial.println("Error in calculation of avgDeltaT");
  }
  else{
    speed1 = (wheelDis*(double(1)/avgDeltaT))*double(1000); // units are in m/s
  }
  
  lencAState = encAState;
  Serial.print("deltaT = ");
  Serial.println(deltaT);
  Serial.print("lastDeltaT = ");
  Serial.println(lastDeltaT);
  Serial.print("Speed: ");
  Serial.println(speed1);
  }
}

