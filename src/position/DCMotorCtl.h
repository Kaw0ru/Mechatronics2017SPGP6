void DCMotorCtl(bool FWD, int Speed, int degree, int Stop, int* pin)
// FWD == go straight
// degree == rotating degree
// pin == pointer to the pins of Motors
{
  int Coeffi=1;// reflect the relationship between the degree and the time
//  if (IR==1)
//     {
//      analogWrite(*(pin),45);
//      analogWrite(*(pin+1),45); // Low Speed
//      digitalWrite(*(pin+2),HIGH);
//      digitalWrite(*(pin+3),HIGH);  // go straight backward
//      delay(500);
//     }
     
  if(Stop==1) // the pixy is not working
  {
     
      digitalWrite(*(pin+2),HIGH); 
      digitalWrite(*(pin+3),HIGH); // go straight forward
      analogWrite(*(pin),45);
      analogWrite(*(pin+1),45);
      Serial.println("Stop");
     
  }
  else 
  
    if (FWD==1)
   {
      digitalWrite(*(pin+2),HIGH); 
      digitalWrite(*(pin+3),HIGH); // go straight forward
      if (Speed==1)// High Speed
      {
        analogWrite(*(pin),155);
        analogWrite(*(pin+1),155);
        Serial.println("FFD");
      }
     else // Low Speed
      {
        analogWrite(*(pin),45);
        analogWrite(*(pin+1),45);
        Serial.println("SFD");
      }
  }
   else
   {
    if (degree>0)
    {
          analogWrite(*(pin),105);
        analogWrite(*(pin+1),0); //turn right
        Serial.println("RT");
    }
    else
    {
      analogWrite(*(pin),0);
        analogWrite(*(pin+1),105); //turn left
        Serial.println("LT");
    }
   }
  
  
  Serial.println("  ");
}

	
