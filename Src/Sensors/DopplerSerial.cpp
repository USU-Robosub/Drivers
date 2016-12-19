#include "DopplerSerial.h"

using namespace Drivers;

Drivers::DopplerSerial::DopplerSerial(std::string deviceName, int baud) {
    // check if file exists
    if(!( access( deviceName.c_str(), F_OK ) != -1 )) {
        throw -1;
    }

    // generate command string and configure serial device
    std::stringstream ss;
    ss << "stty -F " << deviceName << " cs8 " << baud;
    ss << " ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig \
          -icanon -iexten -echo -echoe -echok -echoctl -echoke \
          noflsh -ixon -crtscts";
    std::system(ss.str().c_str());

    _input_ = std::make_shared<std::ifstream>(deviceName.c_str());
    _output_ = std::make_shared<std::ofstream>(deviceName.c_str());
}

Drivers::DopplerSerial::~DopplerSerial() {
    _input_->close();
    _output_->close();
}

int Drivers::DopplerSerial::readData(char* buffer, int length) {
    if(length == 0)
        return 0;

    int t = 0;
    do{
        // NOTE: if the program halts here
        // it is because the device didn't
        // write anything to the stream.
        _input_->get(buffer[t]);
        t++;
    }while(t < length);

    return t;
}

void Drivers::DopplerSerial::readData(std::string& buffer) {
    std::stringstream ss;
    char c;
    do {
        c = _input_->get();
        ss << c;
    }while(c != '>');
    buffer = ss.str();
}

int Drivers::DopplerSerial::writeData(char* buffer, int length) {
    _output_->write(buffer, length);
    _output_->flush();
    return length;
}

void Drivers::DopplerSerial::writeData(std::string buffer) { (void)buffer; }