//#include "Arduino.h"

using namespace std;

class PID
{
private:
	double Kp, Ki, Kd;
	double setPoint;
	double ek, ek_1, u, up, ui, ud, dt;
	int deadband[2], rst;
public:
	PID(double const &kp, double const &ki, double const &kd, double const &setPt)
	{
		this -> Kp = kp;
		this -> Ki = ki;
		this -> Kd = kd;
		this -> setPoint = setPt;
		ek_1 = setPoint;
	}

	~PID(){}

	void GetCtl(double const &currentInput, double &throttle, const double &dT, double const &setPt)
	{

		this -> setPoint = setPt;
	

		ek_1 = ek;
		ek = setPoint - currentInput;

		up = (ek - ek_1) * Kp;
		ui += ek * Ki * dT;
		ud = Kd / dT * (ek - ek_1);

		u = up + ui + ud;
		throttle = u;
	}

	void ChangePara(double const &kp, double const &ki, double const &kd)
	{
		this -> Kp = kp;
		this -> Ki = ki;
		this -> Kd = kd;
	}
};

class Motor
{
private:
	int enablePin;
	int dirPin;
	double Throttle;
	// bool whellDir;
	int CtlEffort;
	PID* myCtl;

public:
	Motor(const int &enablepin, const int &dirpin)
	{
		//pinMode(enablePin, OUTPUT);
		//pinMode(dirPin, OUTPUT);
		myCtl = new PID(0,0,0,0);
		enablePin = enablepin;
		dirPin = dirpin;
	}

	~Motor(){}

	void MotorRun(const double &currentSpeed, const double &setSpeed, const double &dT)
	{
		myCtl -> GetCtl(currentSpeed, Throttle, dT, setSpeed);
		if (Throttle > 0)
		{
			//digitalWrite(dirPin, HIGH);
			cout << "GO" << endl;
		}
		else
		{
			//digitalWrite(dirPin, LOW);
			cout << "BackUp" << endl;
		}

		CtlEffort = Throttle / 100 * 255;
		// analogWrite(enablePin, CtlEffort);
		cout << CtlEffort << endl;
	}

	void ChangeKs(double KpNew, double KiNew, double KdNew)
	{
		myCtl -> ChangePara(KpNew, KiNew, KdNew);
	}

};