#include <iostream>
#include "BMP085.h"

int main() {
	std::cout << "Begin test!\n";
	std::cout << "Initializing...\n";
	BMP085 sensor;
	sensor.initialize();
	std::cout << "Initialization finished!\n";
	std::cout << "Collecting Data...\n";
	for(int i; i < 100; i++) {
		std::cout << "Temperature: ";
		std::cout << sensor.readTemperature();
		std::cout << " Altitude: ";
		std::cout << sensor.readAltitude() << "\n";
	}
	std::cout << "Test finished!\n";
}
