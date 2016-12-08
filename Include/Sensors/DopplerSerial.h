#ifndef DOPPLER_SERIAL_H
#define DOPPLER_SERIAL_H

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <unistd.h> // F_OK

#include "ISerial.h"

namespace Drivers {
    class DopplerSerial : public ISerial {
    public:
        DopplerSerial(std::string deviceName);
        ~DopplerSerial();
        int readData(char* buffer, int length);
        void readData(std::string& buffer);
        int writeData(char* buffer, int length);
        void writeData(std::string buffer);
    
    private:
        std::shared_ptr<std::ifstream> _input_;
        std::shared_ptr<std::ofstream> _output_;
    };
}

#endif