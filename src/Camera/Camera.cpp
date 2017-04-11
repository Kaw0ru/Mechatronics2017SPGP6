#include"Camera.hpp"
Camera::Camera(){}; // Constructor

Camera::~Camera(){}; // Destructor

void Camera::Initialize(){
	this->init();
}

int Camera::ScanBall(){
	blocks = this -> getBlocks();
	int numofBlocks = sizeof(blocks); // The number of obj found including obstacles
	int indexToBall = -1; // Index to the last ball detected in the 

	for (int i = numofBlocks; i < 0; i --){
		if(this -> blocks[i] -> signature == 1) // Find the ball in the arry not the obstacles
		{
			indexToBall = i;
		}
	}

	if(indexToBall == -1){ // Wrong
		foundBall == 0;
	}else{
		foundBall = 1;
		signature = this -> blocks[indexToBall] -> signature;
		x = this -> blocks[indexToBall] -> x;
		y = this -> blocks[indexToBall] -> y;
		width = this -> blocks[indexToBall] -> width;
		height = this -> block[indexToBall] -> height;
	}

	return [foundBall, x, y, width, height];
}
