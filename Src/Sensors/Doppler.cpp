#include "Doppler.h"
#include <iomanip>

#define LOG(x, y) if(_log_ != nullptr) \
                    _log_->log(x, y)

using namespace Drivers;

Drivers::Doppler::Doppler(std::string deviceName) {
    _dvl_ = std::make_shared<DopplerSerial>(deviceName);
}

Drivers::Doppler::Doppler(std::shared_ptr<ISerial> serial) {
    _dvl_ = serial;
}

int Drivers::Doppler::WhoAmI() {
    return 0x95B6027; // P/N 95B-6027-00
}

void Drivers::Doppler::_initVars() {
    // set to defeault factory settings
    // (these are used primarily for functions that use config-specfic ops.)
    _fc_.autoEnsembleCycling = true;
    _fc_.autoPingCycling     = true;
    _fc_.binaryDataOutput    = true;
    _fc_.enableSerialOutput  = true;
    _fc_.enableDataRecording = false;
    
    _dof_ = DataOutputFormat::PD0;
}

void Drivers::Doppler::_readBinaryEnsemble(std::string& ensemble, int& length) {
    char buffer[4];
    int bufferSize = 64;
    int count = 4;
    
    // read echo "CS"
    _dvl_->readData(buffer, 2);
    
    // read header tag
    _dvl_->readData(buffer, count);
    int elen = ((buffer[3] << 8) | buffer[2]) - 2; // ((MSB << 8) | LSB) - 2
    length = count + elen;
    
    char* buf2 = new char[length];
    for(int i = 0; i < count; i++)
        buf2[i] = buffer[i];
    
    // read elen of bytes into buf2
    _dvl_->readData(buf2+count, elen);
    
    std::string out(buf2, length);
    ensemble = out;
    delete buf2;
    
    // read '>' character
    std::string temp;
    _dvl_->readData(temp);
}

void Drivers::Doppler::sendBreak() {
    std::string buffer;
    _dvl_->writeData((char*)"===", 3);
    _dvl_->readData(buffer);
    LOG(buffer, "Doppler.sendBreak()");
}

void Drivers::Doppler::sendUserReset() {
    std::string buffer;
    _dvl_->writeData((char*)"CR0\r", 4);
    _dvl_->readData(buffer);
    LOG(buffer, "Doppler.sendUserReset()");
}

void Drivers::Doppler::sendFactoryReset() {
    std::string buffer;
    _dvl_->writeData((char*)"CR1\r", 4);
    _dvl_->readData(buffer);
    LOG(buffer, "Doppler.sendFactoryReset()");
}

void Drivers::Doppler::sendPingStart(std::string& ensemble) {
    int ref = 0;
    sendPingStart(ensemble, ref);
}

void Drivers::Doppler::sendPingStart(std::string& ensemble, int& length) {
    _dvl_->writeData((char*)"CS\r", 3);
    if(_fc_.binaryDataOutput) {
        _readBinaryEnsemble(ensemble, length);
    }
    else {
        _dvl_->readData(ensemble);
        length = ensemble.length();
    }
    LOG(ensemble, "Doppler.sendPingStart(string)");
}

void Drivers::Doppler::setFlowControl(FlowControl config) {
    std::string buffer;
    const int LEN = 8;
    char command[LEN];
    
    _fc_ = config;
    
    command[0] = 'C';
    command[1] = 'F';
    command[2] = _fc_.autoEnsembleCycling?'1':'0';
    command[3] = _fc_.autoPingCycling    ?'1':'0';
    command[4] = _fc_.binaryDataOutput   ?'1':'0';
    command[5] = _fc_.enableSerialOutput ?'1':'0';
    command[6] = _fc_.enableDataRecording?'1':'0';
    command[7] = '\r';
    
    _dvl_->writeData((char*)command, LEN);
    _dvl_->readData(buffer);
    LOG(buffer, "Doppler.setFlowControl(FlowControl)");
}

void Drivers::Doppler::setDataOutputFormat(DataOutputFormat format) {
    std::string buffer;
    switch(format) {
        case PD0: _dvl_->writeData((char*)"#PD0\r", 5); break;
        case PD4: _dvl_->writeData((char*)"#PD4\r", 5); break;
        case PD5: _dvl_->writeData((char*)"#PD5\r", 5); break;
        case PD6: _dvl_->writeData((char*)"#PD6\r", 5); break;
        default: LOG("Illegal Data Output Format Detected!",
            "Doppler.setDataOutputFormat(DataOutputFormat)"); return;
    }
    _dof_ = format;
    _dvl_->readData(buffer);
    LOG(buffer, "Doppler.setDataOutputFormat(DataOutputFormat)");
}

void Drivers::Doppler::setLogger(std::shared_ptr<ILogger> logger) {
    _log_ = logger;
}
