#include "WheelCtl.h"

WheelCtl::WheelCtl(int pin_num)
{
	pin = pin_num;
}

WheelCtl::~WheelCtl(void)
{
	// Keep it blank
}

void WheelCtl::SetSpeed(double v_d)
{
	desiredSpeed = v_d;
}

double WheelCtl::CruiseControl(double v_c)
{
	currentSpeed = v_c;
	double error = desiredSpeed - currentSpeed;
	// Controller code comes here

	return throttle;
}