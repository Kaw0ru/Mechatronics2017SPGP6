void GateCmd(int* scanResult, int* scanResultHist, int* DesiredLoc, int k, double* angle, int* P_CtlCmd)
{
	bool FWD = 0; // =1 Go straight
	bool BWD = 0; // =0 Go backward
	bool Speed = 0; // =0 Low speed
	bool Reached = 0; // =1 the robot reached the desired place in front of the container
	int angX = 0;
	int errorX = *(DesiredLoc) - *(scanResult);
	int errorY = *(1+DesiredLoc) - *(1+scanResultHist);
	int errorArea = (*(2+DesiredLoc) **(3+DesiredLoc))- *(2+scanResultHist);

	if (*(scanResult+1)==*(scanResultHist+1)) // to check how many times y keeps immutable
	{
		k++ ;
   
	} 
	else
	{
		k = 0;
	}

	// Determin angX
	if (k>10) // the container is out of the view
  {
  
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
    angY=*angle+(10000-*angle)/2;
  }

  // determine the distance and speed
  {
  	if (abs(errorArea) > 100) //to avoid the tiny disturbance
  	{
  		if(errorArea > 0)// robot needs to get closer
  		{
  			BWD = 0;
  		}
  		else
  		{
  			BWD = 1;
  		}

  		if(errorArea < 500) //robot is very close to the diresed position in front of the container
  		{
            Speed = 0; // low speed
  		}
  		else
  		{
  			Speed = 1; // high speed
  		}

  	}
  	else
  	{
      Reached = 1;
  	}
  }
int result[7] = {FWD, BWD, Speed, Reached, angX, angY, k};

for(int j=0;j<7;j++)
  {
  *(P_CtlCmd+j)=result[j];
  }
	//return result;

}