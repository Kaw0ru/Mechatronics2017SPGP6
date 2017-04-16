void CarGateCtl(int state)
{
	if (state == 1)
	{
		openGate();
	}
	else
	{
		closeGate();
	}

}

void openGate() {
  myServoGate.write(150);
}

void closeGate() {
  myServoGate.write(20);
}