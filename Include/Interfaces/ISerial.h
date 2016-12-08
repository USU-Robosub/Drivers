#ifndef ISERIAL_H
#define ISERIAL_H

#include <string>

namespace Drivers {
	class ISerial {
	public:
		virtual int readData(char* buffer, int length) = 0;
		virtual void readData(std::string& buffer) = 0;
		virtual int writeData(char* buffer, int length) = 0;
		virtual void writeData(std::string buffer) = 0;
	};
}

#endif