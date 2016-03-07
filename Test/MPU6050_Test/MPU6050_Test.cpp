#include <iostream>
#include "MPU6050.h"

int main() {
	std::cout << "Begin test!\n";
	std::cout << "Initializing...\n";
	HMC5883L sensor;
	sensor.awake();
	std::cout << "Initialization finished!\n";
	std::cout << "Collecting Data...\n";
	for(int i; i < 100; i++) {
		std::cout << "Accel[X: ";
		std::cout << sensor.accel_X();
		std::cout << " Y: ";
		std::cout << sensor.accel_Y();
		std::cout << " Z: ";
		std::cout << sensor.accel_Z();
		std::cout << "] Gyro[X: ";
		std::cout << sensor.gyro_X();
		std::cout << " Y: ";
		std::cout << sensor.gyro_Y();
		std::cout << " Z: ";
		std::cout << sensor.gyro_Z();
		std::cout << "] Temperature[";
		std::cout << sensor.temperature();
		std::cout << "]\n";
	}
	sensor.sleep();
	std::cout << "Test finished!\n";
}
