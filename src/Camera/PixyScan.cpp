 #include"PixyScan.h"

void PixyScan::P_scan()
{
  i=0;
  uint16_t blocks;
  blocks = pixy.getBlocks();  //receive data from pixy 
  signature = pixy.blocks[i].signature;    //get object's signature
  x = pixy.blocks[i].x;                    //get x position
  y = pixy.blocks[i].y;                    //get y position
  width = pixy.blocks[i].width;            //get width
  height = pixy.blocks[i].height;          //get height
}

void PixyScan::cal_area(height;width)
{
	area = width * height; //calculate the object area 
	max_area = area + 1000;
    min_area = area – 1000;
}

void PixyScan::upward()
{
  //digitaiWirte(HIGH);
}

void PixyScan::downward()
{
  //digitaiWirte(LOW);
}

void PixyScan::PixyPosition(y)
{ if y>=200
    {
      PixyScan::upward();
    }
  if y<=70
    {
      PixyScan::downward();
    }
}

