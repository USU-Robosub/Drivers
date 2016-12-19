#include "Doppler.h"

#define BAUD 9600
#define LOG(x, y) if(_log_ != nullptr) \
                    _log_->log(x, y)

using namespace Drivers;

Drivers::Doppler::Doppler(std::string deviceName) {
    _dvl_ = std::make_shared<DopplerSerial>(deviceName, BAUD);
    _initVars();
}

Drivers::Doppler::Doppler(std::shared_ptr<ISerial> serial) {
    _dvl_ = serial;
    _initVars();
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
    _mm_ = MassMode::M_OFF;
    _ts_ = TurnkeyState::T_ON;

    _transform_.type = TransformType::EARTH;
    _transform_.useTilts = true;
    _transform_.allow3Beam = true;
    _transform_.allowBinMapping = true;
}

void Drivers::Doppler::_readBinaryEnsemble(std::string& ensemble, int& length) {
    char buffer[4];
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

char Drivers::Doppler::_numToChar(unsigned int n) {
    switch(n) {
        case 1: return '1';
        case 2: return '2';
        case 3: return '3';
        case 4: return '4';
        case 5: return '5';
        case 6: return '6';
        case 7: return '7';
        case 8: return '8';
        case 9: return '9';
        default: return '0';
    }
}

int Drivers::Doppler::WhoAmI() {
    return 0x95B6027; // P/N 95B-6027-00
}

void Drivers::Doppler::setLogger(std::shared_ptr<ILogger> logger) {
    _log_ = logger;
}



/* ------------------------------------------------------- *\
|                      Rank 1 COMMANDS                      |
\* ------------------------------------------------------- */

void Drivers::Doppler::sendReset(ResetType type) {
    std::string buffer;
    switch(type) {
    case ResetType::USER:       _dvl_->writeData((char*)"CR0\r", 4); break;
    case ResetType::FACTORY:    _dvl_->writeData((char*)"CR1\r", 4); break;
    }
    _dvl_->readData(buffer);
    LOG(buffer, "Doppler.sendReset(ResetType)");
}

void Drivers::Doppler::saveRemoteSettings() {
    std::string buffer;
    _dvl_->writeData((char*)"CK\r", 3);
    _dvl_->readData(buffer);
    LOG(buffer, "Doppler.saveRemoteSettings()");
}

void Drivers::Doppler::sendPingStart(std::string& ensemble) {
    int ref = 0;
    sendPingStart(ensemble, ref);
}

void Drivers::Doppler::sendPingStart(std::string& ensemble, int& length) {
    _dvl_->writeData((char*)"CS\r", 3);
    if(_fc_.binaryDataOutput && (_dof_ != DataOutputFormat::PD6)) {
        _readBinaryEnsemble(ensemble, length);
    }
    else {
        _dvl_->readData(ensemble);

        int len = ensemble.length();
        int start = (_dof_==PD0)?4:2;            // PD0 starts with 2 extra chars
        length = len - start - (_dof_!=PD6?3:5); // ignore CS\r and \r\n>
        ensemble = ensemble.substr(start, length);
    }
    LOG(ensemble, "Doppler.sendPingStart(string)");
}



/* ------------------------------------------------------- *\
|                      Rank 2 COMMANDS                      |
\* ------------------------------------------------------- */

void Drivers::Doppler::sendBreak() {
    std::string buffer;
    _dvl_->writeData((char*)"===", 3);
    _dvl_->readData(buffer);
    LOG(buffer, "Doppler.sendBreak()");
}

void Drivers::Doppler::setBaudRate(
    BaudRate rate, Parity parity, StopBits bits) {
    // sets the baud rate from 300 to 115200, default: 9600 (CB411)

    const int LEN = 6;
    char command[LEN];
    command[0] = 'C';
    command[1] = 'B';
    command[2] = _numToChar((int)rate);
    command[3] = _numToChar((int)parity);
    command[4] = _numToChar((int)bits);
    command[5] = '\r';

    std::string buffer;
    _dvl_->writeData((char*)command, LEN);
    _dvl_->readData(buffer);
    LOG(buffer, "Doppler.setBaudRate()");
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

void Drivers::Doppler::setBTPingsPerEnsemble(unsigned int pings) {
    const int LEN = 6;
    char command[LEN];
    command[0] = 'B';
    command[1] = 'P';
    command[2] = _numToChar((pings/100)%10);
    command[3] = _numToChar((pings/10)%10);
    command[4] = _numToChar((pings)%10);
    command[5] = '\r';

    std::string buffer;
    _dvl_->writeData((char*)command, LEN);
    _dvl_->readData(buffer);
    LOG(buffer, "Doppler.setBTPingsPerEnsemble(unsigned int)");
}

void Drivers::Doppler::setMaxTrackingDepth(unsigned int decimeters) {
    // default: BX01000 (1000 decimeters, 100 meters)
    const int LEN = 8;
    char command[LEN];
    command[0] = 'B';
    command[1] = 'X';
    command[2] = _numToChar((decimeters/10000)%10);
    command[3] = _numToChar((decimeters/1000)%10);
    command[4] = _numToChar((decimeters/100)%10);
    command[5] = _numToChar((decimeters/10)%10);
    command[6] = _numToChar((decimeters)%10);
    command[7] = '\r';

    std::string buffer;
    _dvl_->writeData((char*)command, LEN);
    _dvl_->readData(buffer);
    LOG(buffer, "Doppler.setMaxTrackingDepth(unsigned int)");
}

void Drivers::Doppler::setHeadingAlignment(float degrees) {
    bool flag = degrees >= 0;
    unsigned int deg = (unsigned int)(degrees*100 * (flag?1:-1));

    const int LEN = 9;
    char command[LEN];
    command[0] = 'E';
    command[1] = 'A';
    command[2] = flag ? '+' : '-';
    command[3] = _numToChar((deg/10000)%10);
    command[4] = _numToChar((deg/1000)%10);
    command[5] = _numToChar((deg/100)%10);
    command[6] = _numToChar((deg/10)%10);
    command[7] = _numToChar((deg)%10);
    command[8] = '\r';

    std::string buffer;
    _dvl_->writeData((char*)command, LEN);
    _dvl_->readData(buffer);
    LOG(buffer, "Doppler.setHeadingAlignment(float)");
}

void Drivers::Doppler::setTransducerDepth(unsigned int decimeters) {
    const int LEN = 8;
    char command[LEN];
    command[0] = 'E';
    command[1] = 'D';
    command[2] = _numToChar((decimeters/10000)%10);
    command[3] = _numToChar((decimeters/1000)%10);
    command[4] = _numToChar((decimeters/100)%10);
    command[5] = _numToChar((decimeters/10)%10);
    command[6] = _numToChar((decimeters)%10);
    command[7] = '\r';

    std::string buffer;
    _dvl_->writeData((char*)command, LEN);
    _dvl_->readData(buffer);
    LOG(buffer, "Doppler.setTransducerDepth(unsigned int)");
}

void Drivers::Doppler::setSalinity(unsigned char partsPerK) {
    const int LEN = 5;
    char command[LEN];
    command[0] = 'E';
    command[1] = 'S';
    command[2] = _numToChar((partsPerK/10)%10);
    command[3] = _numToChar((partsPerK)%10);
    command[4] = '\r';

    std::string buffer;
    _dvl_->writeData((char*)command, LEN);
    _dvl_->readData(buffer);
    LOG(buffer, "Doppler.setSalinity(unsigned char)");
}

void Drivers::Doppler::setCoordTransform(Transformation transform) {
    _transform_ = transform;

    const int LEN = 8;
    char command[LEN];
    command[0] = 'E';
    command[1] = 'X';
    switch(_transform_.type) {
    case TransformType::NONE:       command[2] = '0'; command[3] = '0'; break;
    case TransformType::INSTRUMENT: command[2] = '0'; command[3] = '1'; break;
    case TransformType::SHIP:       command[2] = '1'; command[3] = '0'; break;
    default:                        command[2] = '1'; command[3] = '1'; break;
    }
    command[4] = _transform_.useTilts?'1':'0';
    command[5] = _transform_.allow3Beam?'1':'0';
    command[6] = _transform_.allowBinMapping?'1':'0';
    command[7] = '\r';

    std::string buffer;
    _dvl_->writeData((char*)command, LEN);
    _dvl_->readData(buffer);
    LOG(buffer, "Doppler.setCoordTransform(Transformation)");
}

void Drivers::Doppler::setTimePerEnsemble(unsigned char hours,
    unsigned char minutes, unsigned char seconds, unsigned char hundredths) {

    const int LEN = 14;
    char command[LEN];
    command[0] = 'T';
    command[1] = 'E';
    command[2] = _numToChar((hours/10)%10);
    command[3] = _numToChar((hours)%10);
    command[4] = ':';
    command[5] = _numToChar((minutes/10)%10);
    command[6] = _numToChar((minutes)%10);
    command[7] = ':';
    command[8] = _numToChar((seconds/10)%10);
    command[9] = _numToChar((seconds)%10);
    command[10]= '.';
    command[11]= _numToChar((hundredths/10)%10);
    command[12]= _numToChar((hundredths)%10);
    command[13]= '\r';

    std::string buffer;
    _dvl_->writeData((char*)command, LEN);
    _dvl_->readData(buffer);
    LOG(buffer, "Doppler.setTimePerEnsemble(unsigned char,unsigned char, unsigned char, unsigned char)");
}

void Drivers::Doppler::setTimeBetweenPings(unsigned char minutes,
    unsigned char seconds, unsigned char hundredths) {

    const int LEN = 11;
    char command[LEN];
    command[0] = 'T';
    command[1] = 'P';
    command[2] = _numToChar((minutes/10)%10);
    command[3] = _numToChar((minutes)%10);
    command[4] = ':';
    command[5] = _numToChar((seconds/10)%10);
    command[6] = _numToChar((seconds)%10);
    command[7] = '.';
    command[8] = _numToChar((hundredths/10)%10);
    command[9] = _numToChar((hundredths)%10);
    command[10] = '\r';

    std::string buffer;
    _dvl_->writeData((char*)command, LEN);
    _dvl_->readData(buffer);
    LOG(buffer, "Doppler.setTimeBetweenPings(unsigned char, unsigned char, unsigned char)");
}



/* ------------------------------------------------------- *\
|                      Rank 3 COMMANDS                      |
\* ------------------------------------------------------- */

void Drivers::Doppler::setMassMode(MassMode mode) {
    _mm_ = mode;

    const int LEN = 5;
    char command[LEN];
    command[0] = '#';
    command[1] = 'B';
    command[2] = 'K';
    command[3] = _numToChar((int)_mm_);
    command[4] = '\r';

    std::string buffer;
    _dvl_->writeData((char*)command, LEN);
    _dvl_->readData(buffer);
    LOG(buffer, "Doppler.setMassMode(MassMode)");
}

void Drivers::Doppler::setMassLayer(unsigned int minSize,
    unsigned int nearBoundary, unsigned int farBoundary) {
    const int LEN = 18;
    char command[LEN];
    command[0] = '#';
    command[1] = 'B';
    command[2] = 'L';
    command[3] = _numToChar((minSize/1000)%10);
    command[4] = _numToChar((minSize/100)%10);
    command[5] = _numToChar((minSize/10)%10);
    command[6] = _numToChar((minSize)%10);
    command[7] = ',';
    command[8] = _numToChar((nearBoundary/1000)%10);
    command[9] = _numToChar((nearBoundary/100)%10);
    command[10]= _numToChar((nearBoundary/10)%10);
    command[11]= _numToChar((nearBoundary)%10);
    command[12]= ',';
    command[13]= _numToChar((farBoundary/1000)%10);
    command[14]= _numToChar((farBoundary/100)%10);
    command[15]= _numToChar((farBoundary/10)%10);
    command[16]= _numToChar((farBoundary)%10);
    command[17] = '\r';

    std::string buffer;
    _dvl_->writeData((char*)command, LEN);
    _dvl_->readData(buffer);
    LOG(buffer, "Doppler.setMassLayer(unsigned int, unsigned int, unsigned int)");
}

void Drivers::Doppler::setTurnkeyOp(TurnkeyState state) {
    _ts_ = state;

    const int LEN = 5;
    char command[LEN];
    command[0] = '#';
    command[1] = 'C';
    command[2] = 'T';
    command[3] = _numToChar((int)state);
    command[4] = '\r';

    std::string buffer;
    _dvl_->writeData((char*)command, LEN);
    _dvl_->readData(buffer);
    LOG(buffer, "Doppler.setTurnkeyOp(TurnkeyState)");
}

void Drivers::Doppler::setHeadingVariation(float degrees) {
    bool flag = degrees >= 0;
    unsigned int deg = (degrees*100*(flag?1:-1));

    const int LEN = 10;
    char command[LEN];
    command[0] = '#';
    command[1] = 'E';
    command[2] = 'V';
    command[3] = flag?'+':'-';
    command[4] = _numToChar((deg/10000)%10);
    command[5] = _numToChar((deg/1000)%10);
    command[6] = _numToChar((deg/100)%10);
    command[7] = _numToChar((deg/10)%10);
    command[8] = _numToChar((deg)%10);
    command[9] = '\r';

    std::string buffer;
    _dvl_->writeData((char*)command, LEN);
    _dvl_->readData(buffer);
    LOG(buffer, "Doppler.set()");
}

void Drivers::Doppler::setDataOutputFormat(DataOutputFormat format) {
    _dof_ = format;

    const int LEN = 5;
    char command[LEN];
    command[0] = '#';
    command[1] = 'P';
    command[2] = 'D';
    command[3] = _numToChar((int)format);
    command[4] = '\r';

    std::string buffer;
    _dvl_->writeData((char*)command, LEN);
    _dvl_->readData(buffer);
    LOG(buffer, "Doppler.setDataOutputFormat(DataOutputFormat)");
}



/* ------------------------------------------------------- *\
|                      Other  COMMANDS                      |
\* ------------------------------------------------------- */

void Drivers::Doppler::sendUserCommand(std::string command) {
    _dvl_->writeData((char*)command.c_str(), command.length());
}

void Drivers::Doppler::readString(std::string& data) {
    std::string buffer;
    _dvl_->readData(buffer);
    data = buffer;
}

Drivers::Doppler::DataOutputFormat Drivers::Doppler::getOutputFormat() {
    return _dof_;
}

bool Drivers::Doppler::isBinaryOutput() {
    return _fc_.binaryDataOutput;
}