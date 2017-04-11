#include "SPI.h"  
#include "Pixy.h"

class Camera: public Pixy {
	public:
		Camera(); // Constructor
		~Camera(); // Destructor
		int ScanBall(); // Get ball location & size
		void Initialize();
	private:
		uint16_t blocks;
		uint8_t signature;
		uint8_t x;
		uint8_t y;
		uint8_t width;
		uint8_t height;
		bool foundBall;
}