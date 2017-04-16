double PixyServoCtl(double ang)
{
	if (ang < 70)
	{
		myServoY.write(70);
	}
	else
	{
		myServoY.write(ang);
	}
	return myServoY.read();
}
