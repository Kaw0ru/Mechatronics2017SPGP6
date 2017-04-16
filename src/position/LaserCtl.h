void LaserCtl(int* pin, bool enable, bool* isBlocked)
{
	int laserPin = *pin;
	int photoPin = *(pin + 1);

	if(enable)
	{
		digitalWrite (laserPin, HIGH);
		// delay(10); // Time for laser & photo resistor to resp
		*isBlocked = analogRead(photoPin) < 300 ? 1 : 0;
	}
	else
	{
		digitalWrite (laserPin, LOW);
		*isBlocked = 0;
	}
}