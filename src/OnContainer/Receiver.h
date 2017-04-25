void SetupReceiver(int pin)
{
	vw_set_rx_pin(pin);
    
    /* Receive at 2000 bits per second */
    vw_setup(2000);
    
    /* Enable the receiver */
    vw_rx_start(); 
}

unsigned int ReceiveMsg()
{
	uint8_t Buffer_Size = 2;
  
	/* Holds the recived data */
	unsigned int Data;
  
	/* The receive buffer */
	uint8_t RxBuffer[Buffer_Size];

	/* Has a message been received? */
	if (vw_get_message(RxBuffer, &Buffer_Size)) // Non-blocking
	{
		/* If so, then turn on the LED connected to DIO 13 
           to indicate this */
   
        /* Store the received high and low byte data */
		Data = RxBuffer[0] << 8 | RxBuffer[1];
	}
	return Data;

}