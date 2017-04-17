#include <iostream>
#include "PID.h"
using namespace std;


int main()
{
	Motor myMotor(1,2);
	myMotor.ChangeKs(1,2,3);
	myMotor.MotorRun(0,1,0.1);
}