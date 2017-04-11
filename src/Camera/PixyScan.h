// PixyScan.h

#ifndef _PIXYSCAN_h
#define _PIXYSCAN_h

#include "SPI.h"
#include "Pixy.h"

   // #include <SPI.h>  
   // #include <Pixy.h>
class Scan
{
private:
	double x;
	double y;
	double height;
	double width;
	double area;
	double max_area;
	double min_area;
	int signature;

public:
    void P_scan();
    void cal_area(height,width);// calculate the area
    //void backward();//backward
    //void forward();//forward
    //void right();//turn right
    // void left();//turn left
    void PixyPosition(y);
    void upward();// the direction of pixy will be upward
    void downward();// the direction of pixy will be downward
    // void Stop();//stop

}
