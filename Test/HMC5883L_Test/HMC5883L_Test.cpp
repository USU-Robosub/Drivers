#include <iostream>
#include "HMC5883L.h"

int main() {
	std::cout << "Begin test!\n";
	std::cout << "Initializing...\n";
	HMC5883L sensor;
	sensor.setMode(HMC5883L::Mode::Continuous);
	std::cout << "Initialization finished!\n";
	std::cout << "Collecting Data...\n";
	for(int i; i < 100; i++) {
		while(!sensor.isReady()) {}
		std::cout << "X: ";
		std::cout << sensor.X();
		std::cout << " Y: ";
		std::cout << sensor.Y();
		std::cout << " Z: ";
		std::cout << sensor.Z() << "\n";
	}
	sensor.setMode(HMC5883L::Mode::Idle);
	std::cout << "Test finished!\n";
}
