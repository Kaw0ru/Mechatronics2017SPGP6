// WheelCtl.h

#ifndef _WHEELCTL_h
#define _WHEELCTL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class WheelCtl
{
 private:
	 double desiredSpeed;
	 double currentSpeed;
	 double throttle;
	 int pin;

 public:
	 WheelCtl(int pin); // Constructor
	 ~WheelCtl(); // Destructor
	 void SetSpeed(double v_d); // Set desired speed
	 double CruiseControl(double v_c); // Vary from 0 to 1
};


#endif

