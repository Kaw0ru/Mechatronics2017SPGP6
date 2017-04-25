void ContainerCmd(int* scanResult, int* scanResultHist, int* DesiredLoc, int k, double* angle, int* P_CtlCmd)
{
  bool FWD = 0;
  int angX = 0;
  int angY = 0;
  int errorX = *(DesiredLoc) - *(scanResult);
  int errorY = *(1+DesiredLoc) - *(1+scanResultHist);
  
  if (*(1+scanResult)==*(1+scanResultHist)) // to check how many times y keeps immutable
  {
    k++ ;
   
  } 
  else
  {
    k = 0;
  }

  // Determin angX
  if (k>10) // the ball is out of the view
  {
   // unsigned long outofviewcurrenttime = millis();

//    if (outofviewcurrenttime - *(outOfViewStartingTime) > 5000)
//    {
//      *(globalState) = 2;
//    }
    
    if (errorX > 0)
    {
      FWD = 0;
      angX = 30; //Turn left
    }
    else
    {
      FWD = 0;
      angX = -30; //Turn right
    }
  }
  else
  {
   // *(outOfViewStartingTime) = millis();
    if (abs(errorX)>=10) //to avoid the tiny disturbance
    {
      if (errorX>0)
      {
        FWD = 0;// turn left
        angX = -1;
      }
      else
      {
        FWD = 0;//turn right
        angX = 1;
      } 
    }
    else
    {
      FWD = 1; //go straight
      angX = 0;
    }
  }

 // determine angY
  double ang= *angle;
  if (abs(errorY)>=8) //to avoid the tiny disturbance
  {
    double Dec = 100; // a constant which reflects the relationship between Y and angle
    double delta = double(errorY) / Dec;
    delta = 9*atan(delta)/3.14; // delta angle
    ang = ang + delta*100;
  }
  angY=int(ang);
  if (k>=10)
  {
    angY=*angle+(9300-*angle)/2;
  }
  int result[4] = {FWD, angX, angY, k};
 
  for(int j=0;j<4;j++)
  {
  *(P_CtlCmd+j)=result[j];
  }
  //return result; //angle = 100 * angle as a int.
}
