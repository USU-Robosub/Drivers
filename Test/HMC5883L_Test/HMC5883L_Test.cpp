#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include "HMC5883L.h"

int main() {
	std::cout << "Begin test!\n";
	std::cout << "Initializing...\n";
	HMC5883L sensor;
	sensor.setOutputRate(HMC5883L::Rate::Hz75);
	sensor.setMode(HMC5883L::Mode::Continuous);
	std::cout << "Initialization finished!\n";
	std::cout << "Collecting Data...\n";
	std::cout << std::setprecision(2) << std::fixed;
	for(int i; i < 100; i++) {
		std::cout << "X: ";
		std::cout << sensor.X();
		std::cout << " Y: ";
		std::cout << sensor.Y();
		std::cout << " Z: ";
		std::cout << sensor.Z() << "\n";
		
		// wait for new data
		std::this_thread::sleep_for(std::chrono::milliseconds(1000/75));
	}
	sensor.setMode(HMC5883L::Mode::Idle);
	std::cout << "Test finished!\n";
}
