 #include"PixyScan.h"

void P_scan()
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

void cal_area(height;width)
{
	area = width * height; //calculate the object area 
	max_area = area + 1000;
    min_area = area – 1000;
}

void upward()
{
  //digitaiWirte(HIGH);
}

void downward()
{
  //digitaiWirte(LOW);
}

void PixyPosition(y)
{ if y>=200
    {
      upward();
    }
  if y<=70
    {
      downward();
    }
}

