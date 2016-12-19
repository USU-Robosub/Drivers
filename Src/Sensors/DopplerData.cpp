#include "DopplerData.h"

using namespace Drivers;

#define DEBUG
#ifdef DEBUG
#define DMSG(x) std::cerr<<x;
#else
#define DMSG(x)
#endif

unsigned char Drivers::DopplerData::hexToByte(char HEX) {
    if(HEX >= '0' && HEX <= '9')
        return HEX - '0';
    if(HEX >= 'A' && HEX <= 'F')
        return HEX - 'A' + 10;
    if(HEX >= 'a' && HEX <= 'f')
        return HEX - 'a' + 10;

    DMSG("Invalid Hex Char: " << std::hex << (int)HEX << "\n")
    throw 1;
}

unsigned char Drivers::DopplerData::hexToByte(char MSB, char LSB) {
    try {
        return hexToByte(MSB) * 16 + hexToByte(LSB);
    } catch (int e) {
        DMSG("Error converting hex to byte\n")
        throw e;
    }
}

std::string Drivers::DopplerData::hexToBinary(std::string stream) {
    int len = stream.length()/2;
    if(!len) return "";

    std::istringstream iss(stream);
    char* buffer = new char[len];

    unsigned char msb, lsb;
    for(int i = 0; i < len; i++) {
        iss >> msb >> lsb;
        buffer[i] = (char)hexToByte(msb, lsb);
    }

    std::string result(buffer, len);
    delete buffer;
    return result;
}

void Drivers::DopplerData::scrubPD6(std::string& stream) {
    // removes whitespace and \r\n chars
    const int LEN = 4;
    char chars[LEN];
    chars[0] = ' ';
    chars[1] = '\r';
    chars[2] = '\n';
    chars[3] = '\t';
    for (int i = 0; i < LEN; i++)
        stream.erase(std::remove(stream.begin(), stream.end(), chars[i]), stream.end());
}



Drivers::FixedLeader Drivers::DopplerData::_cds0_getFixedLeader(
    int offset, std::string data) {

    FixedLeader result; // 58 bytes
    int idx = offset;
    result.id                  = uShort(data, idx); // 0x0000
    result.firmwareVersion     = uChar (data, idx);
    result.firmwareRevision    = uChar (data, idx);
    result.sysConfig           = uShort(data, idx);
    result.realSimFlag         = uChar (data, idx);
    result.lagLength           = uChar (data, idx);
    result.numOfBeams          = uChar (data, idx);
    result.numOfCells          = uChar (data, idx);
    result.pingsPerEnsemble    = uShort(data, idx);
    result.depthCellLength     = uShort(data, idx);
    result.blankAfterTransmit  = uShort(data, idx);
    result.profilingMode       = uChar (data, idx);
    result.lowCorrThresh       = uChar (data, idx);
    result.numOfCodeReps       = uChar (data, idx); idx++; // ignore reserved byte
    result.errMaxVelocity      = uShort(data, idx);
    result.tppMinutes          = uChar (data, idx);
    result.tppSeconds          = uChar (data, idx);
    result.tppHundredths       = uChar (data, idx);
    result.coorTransform       = uChar (data, idx);
    result.headingAlignment    = uShort(data, idx);
    result.headingBias         = uShort(data, idx);
    result.sensorSource        = uChar (data, idx);
    result.sensorAvailable     = uChar (data, idx);
    result.bin1Dist            = uShort(data, idx);
    result.xmitPulseLength     = uShort(data, idx);
    result.wpRefLayerAverage   = uShort(data, idx);
    result.falseTargetThresh   = uChar (data, idx); idx++;
    result.transmitLagDistance = uShort(data, idx); idx+=8;
    result.sysBandwidth        = uShort(data, idx); idx+=2;
    result.sysSerialNumber     = (unsigned int)(data[idx] | (data[idx+1]<<8) | (data[idx+2]<<16) | (data[idx+3]<<24));
    return result;
}

Drivers::VariableLeader Drivers::DopplerData::_cds0_getVariableLeader(
    int offset, std::string data) {

    VariableLeader result; // 60 bytes
    int idx = offset;
    result.id              = uShort(data, idx); // 0x8000
    result.ensembleTag     = uShort(data, idx);
    result.rtcYear         = uChar (data, idx);
    result.rtcMonth        = uChar (data, idx);
    result.rtcDay          = uChar (data, idx);
    result.rtcHour         = uChar (data, idx);
    result.rtcMinute       = uChar (data, idx);
    result.rtcSecond       = uChar (data, idx);
    result.rtcHundredths   = uChar (data, idx);
    result.ensembleTagMsb  = uChar (data, idx);
    result.bitResult       = uShort(data, idx);
    result.speedOfSound    = uShort(data, idx);
    result.transducerDepth = uShort(data, idx);
    result.heading         = sShort(data, idx);
    result.pitch           = sShort(data, idx);
    result.roll            = sShort(data, idx);
    result.salinity        = sShort(data, idx);
    result.temperature     = sShort(data, idx);
    result.mptMinutes      = uChar (data, idx);
    result.mptSeconds      = uChar (data, idx);
    result.mptHundredths   = uChar (data, idx);
    result.headingStdDev   = uChar (data, idx);
    result.pitchStdDev     = uChar (data, idx);
    result.rollStdDev      = uChar (data, idx);
    result.adc0            = uChar (data, idx);
    result.adc1            = uChar (data, idx);
    result.adc2            = uChar (data, idx);
    result.adc3            = uChar (data, idx);
    result.adc4            = uChar (data, idx);
    result.adc5            = uChar (data, idx);
    result.adc6            = uChar (data, idx);
    result.adc7            = uChar (data, idx);
    result.errStatus       = uInt  (data, idx); idx+=2;
    result.pressure        = uInt  (data, idx);
    result.pressureVar     = uInt  (data, idx);
    return result;
}

PTR(Drivers::BottomTrack) Drivers::DopplerData::_cds0_getBottomTrack(
    int offset, std::string data) {

    PTR(BottomTrack) result = std::make_shared<BottomTrack>();
    int idx = offset;
    result->id                   = uShort(data, idx);
    result->pingsPerEnsemble     = uShort(data, idx);
    result->delayBeforeReAquire  = uShort(data, idx);
    result->corrMagMin           = uChar (data, idx);
    result->evalAmpMin           = uChar (data, idx);
    result->percentGoodMin       = uChar (data, idx);
    result->mode                 = uChar (data, idx);
    result->errVelMax            = uShort(data, idx); idx+=4;
    for(int i = 0; i < 4; i++) {
        result->range[i]         = uShort(data, idx);
    }
    for(int i = 0; i < 4; i++) {
        result->velocity[i]      = uShort(data, idx);
    }
    for(int i = 0; i < 4; i++) {
        result->correlation[i]   = uChar (data, idx);
    }
    for(int i = 0; i < 4; i++) {
        result->evalAmp[i]       = uChar (data, idx);
    }
    for(int i = 0; i < 4; i++) {
        result->percentGood[i]   = uChar (data, idx);
    }
    result->refLayerMin          = uShort(data, idx);
    result->refLayerNear         = uShort(data, idx);
    result->refLayerFar          = uShort(data, idx);
    for(int i = 0; i < 4; i++) {
        result->refLayerVel[i]   = uShort(data, idx);
    }
    for(int i = 0; i < 4; i++) {
        result->refCorrelation[i]= uChar (data, idx);
    }
    for(int i = 0; i < 4; i++) {
        result->refInt[i]        = uChar (data, idx);
    }
    for(int i = 0; i < 4; i++) {
        result->refPercentGood[i]= uChar (data, idx);
    }
    result->maxDepth             = uShort(data, idx);
    for(int i = 0; i < 4; i++) {
        result->rssiAmp[i]       = uChar (data, idx);
    }
    result->gain                 = uChar (data, idx);
    for(int i = 0; i < 4; i++) {
        result->rangeMSB[i]      = uChar (data, idx);
    }
    return result;
}

int Drivers::DopplerData::_cds6_getTag(
    char* tag, std::stringstream & data) {
    // return 0 for sucess, -1 otherwise
    if(!data) return -1;
    int idx = 0;
    while(idx < 3 && data) {
        data >> tag[idx];
        idx++;
    }
    if(idx < 3) return -1;
    else return 0;
}

int Drivers::DopplerData::_cds6_getValue(
    char  & value, std::stringstream & data) {
    if(!data) return -1;
    char msb, lsb;
    if(!(data >> msb >> lsb)) return -1;

    msb -= '0';
    lsb -= '0';
    value = msb*10 + lsb;

    return 0;
}



PTR(Drivers::DataStream0) Drivers::DopplerData::composeDataStream0(
    std::string data) {
    PTR(DataStream0) result = std::make_shared<DataStream0>();
    result->raw = data;
    int idx = 0, jdx = 0, length = data.length();
    short tag;

    // parse header
    result->headerId  = uChar (data, idx);
    result->id        = uChar (data, idx);
    result->length    = uShort(data, idx); idx++;
    result->typeCount = uChar (data, idx);

    // get checksum
    result->checksum  = (unsigned short)(data[length-2] | (data[length-1]<<8));

    // get type indices
    unsigned short * indices = new unsigned short[result->typeCount];
    for(int i = 0; i < result->typeCount; i++) {
        indices[i] = uShort(data, idx);
    }

    result->fixedLeader    = _cds0_getFixedLeader(indices[jdx], data); jdx++;
    result->variableLeader = _cds0_getVariableLeader(indices[jdx], data); jdx++;

    // conditional types
    int noc = result->fixedLeader.numOfCells;
    PTR(DepthCell) temp(new DepthCell[noc], std::default_delete<DepthCell[]>());

    // if id == velocity 0x0100
    idx = indices[jdx];
    if((data[idx]|(data[idx+1]<<8)) == 0x0100) {
        // only set depthCells if at least 1 of the four types are used to avoid
        // wasting memory by carying around (20 x numOfCells) worth of 0's
        result->depthCells = temp;

        idx+=2; // skip id
        for(int i = 0; i < noc; i++) {
            result->depthCells.get()[i].velocity[0] = uShort(data, idx);
            result->depthCells.get()[i].velocity[1] = uShort(data, idx);
            result->depthCells.get()[i].velocity[2] = uShort(data, idx);
            result->depthCells.get()[i].velocity[3] = uShort(data, idx);
        }

        jdx++;
        if(jdx >= result->typeCount) goto END_OF_TYPES;
        idx = indices[jdx];
    }

    // if id == correlation magnitude 0x2000
    if((data[idx]|(data[idx+1]<<8)) == 0x2000) {
        if(!result->depthCells)
            result->depthCells = temp;

        idx+=2;
        for(int i = 0; i < noc; i++) {
            result->depthCells.get()[i].corrMagnitude[0] = uChar(data, idx);
            result->depthCells.get()[i].corrMagnitude[1] = uChar(data, idx);
            result->depthCells.get()[i].corrMagnitude[2] = uChar(data, idx);
            result->depthCells.get()[i].corrMagnitude[3] = uChar(data, idx);
        }

        jdx++;
        if(jdx >= result->typeCount) goto END_OF_TYPES;
        idx = indices[jdx];
    }

    // if id == echo intensity 0x3000
    if((data[idx]|(data[idx+1]<<8)) == 0x3000) {
        if(!result->depthCells)
            result->depthCells = temp;

        idx+=2;
        for(int i = 0; i < noc; i++) {
            result->depthCells.get()[i].echoIntensity[0] = uChar(data, idx);
            result->depthCells.get()[i].echoIntensity[1] = uChar(data, idx);
            result->depthCells.get()[i].echoIntensity[2] = uChar(data, idx);
            result->depthCells.get()[i].echoIntensity[3] = uChar(data, idx);
        }

        jdx++;
        if(jdx >= result->typeCount) goto END_OF_TYPES;
        idx = indices[jdx];
    }

    // if id == percent good 0x0400
    if((data[idx]|(data[idx+1]<<8)) == 0x0400) {
        if(!result->depthCells)
            result->depthCells = temp;

        idx+=2;
        for(int i = 0; i < noc; i++) {
            result->depthCells.get()[i].percentGood[0] = uChar(data, idx);
            result->depthCells.get()[i].percentGood[1] = uChar(data, idx);
            result->depthCells.get()[i].percentGood[2] = uChar(data, idx);
            result->depthCells.get()[i].percentGood[3] = uChar(data, idx);
        }

        jdx++;
        if(jdx >= result->typeCount) goto END_OF_TYPES;
        idx = indices[jdx];
    }

    // if id == bottom track
    if((data[idx]|(data[idx+1]<<8)) == 0x0600) {
        result->bottomTrack = _cds0_getBottomTrack(indices[jdx], data);

        // this is last type
    }

END_OF_TYPES:
    delete[] indices;
    return result;
}

PTR(Drivers::DataStream4) Drivers::DopplerData::composeDataStream4(
    std::string data) {
    const char* ds = data.c_str();
    int idx = 0, length = data.length();

    if(length < 47) { // DataStream4 is exactly 47 bytes in length
        DMSG("DataStream Too Short!\n")
        throw -1;
    }

    PTR(DataStream4) result  = std::make_shared<DataStream4>();
    result->raw = data;

    result->id               = uChar (data, idx);
    result->structure        = uChar (data, idx);
    result->length           = uShort(data, idx);
    result->sysConfig        = uChar (data, idx);

    result->xVelBtm          = sShort(data, idx);
    result->yVelBtm          = sShort(data, idx);
    result->zVelBtm          = sShort(data, idx);
    result->eVelBtm          = sShort(data, idx);

    result->bm1RngBtm        = uShort(data, idx);
    result->bm2RngBtm        = uShort(data, idx);
    result->bm3RngBtm        = uShort(data, idx);
    result->bm4RngBtm        = uShort(data, idx);

    result->btmStatus        = uChar (data, idx);

    result->xVelRef          = sShort(data, idx);
    result->yVelRef          = sShort(data, idx);
    result->zVelRef          = sShort(data, idx);
    result->eVelRef          = sShort(data, idx);

    result->refLayerStart    = uShort(data, idx);
    result->refLayerEnd      = uShort(data, idx);
    result->refLayerStatus   = uChar (data, idx);

    result->tofpHour         = uChar (data, idx);
    result->tofpMinute       = uChar (data, idx);
    result->tofpSecond       = uChar (data, idx);
    result->tofpHundredths   = uChar (data, idx);

    result->bitResults       = uShort(data, idx);
    result->speedOfSound     = uShort(data, idx);
    result->temperature      = 0.01F*sShort(data, idx);

    result->checksum         = (unsigned short)(ds[length-2] | (ds[length-1]<<8));
    return result;
}

PTR(Drivers::DataStream5) Drivers::DopplerData::composeDataStream5(
    std::string data) {
    const char* ds = data.c_str();
    int idx = 45, length = data.length();

    if(length < 88) { // DataStream5 is exactly 88 bytes in length
        DMSG("DataStream Too Short!\n")
        throw -1;
    }

    PTR(DataStream5) result  = std::make_shared<DataStream5>();

    result->velocity         = composeDataStream4(data);

    result->salinity         = uChar (data, idx);
    result->depth            = uShort(data, idx);
    result->pitch            = 0.01F*sShort(data, idx);
    result->roll             = 0.01F*sShort(data, idx);
    result->heading          = 0.01F*sShort(data, idx);

    result->eastBtmDistGood  = uInt(data, idx);
    result->northBtmDistGood = uInt(data, idx);
    result->upBtmDistGood    = uInt(data, idx);
    result->errBtmDistGood   = uInt(data, idx);

    result->eastRefDistGood  = uInt(data, idx);
    result->northRefDistGood = uInt(data, idx);
    result->upRefDistGood    = uInt(data, idx);
    result->errRefDistGood   = uInt(data, idx);

    return result;
}

PTR(Drivers::DataStream6) Drivers::DopplerData::composeDataStream6(
    std::string data) {
    PTR(DataStream6) result = std::make_shared<DataStream6>();
    result->raw = data;
    std::stringstream ss(data);
    // unsigned int idx = 0;
    char tag[3];
    char c; // ignore ','

    // find first tag
    if(_cds6_getTag(tag, ss) != 0) goto END_OF_SEARCH;

    // match tag to next parsing algorithm
    if(tag[1] == 'S' && tag[2] == 'A') {
        // System Attitude Data
        float value;
        ss>>c; ss>>value;
        result->fmap.insert(std::pair<std::string, float>(":SA:Pitch", value));
        ss>>c; ss>>value;
        result->fmap.insert(std::pair<std::string, float>(":SA:Roll", value));
        ss>>c; ss>>value;
        result->fmap.insert(std::pair<std::string, float>(":SA:Heading", value));

        if(_cds6_getTag(tag, ss) != 0) goto END_OF_SEARCH;
    }
    if(tag[1] == 'T' && tag[2] == 'S') {
        // Timing and Scaling Data
        float value; unsigned int ival;
        ss>>c;
        _cds6_getValue(c, ss); result->imap.insert(std::pair<std::string, int>(":TS:YY", (int)c));
        _cds6_getValue(c, ss); result->imap.insert(std::pair<std::string, int>(":TS:MM", (int)c));
        _cds6_getValue(c, ss); result->imap.insert(std::pair<std::string, int>(":TS:DD", (int)c));
        _cds6_getValue(c, ss); result->imap.insert(std::pair<std::string, int>(":TS:HH", (int)c));
        _cds6_getValue(c, ss); result->imap.insert(std::pair<std::string, int>(":TS:mm", (int)c));
        _cds6_getValue(c, ss); result->imap.insert(std::pair<std::string, int>(":TS:ss", (int)c));
        _cds6_getValue(c, ss); result->imap.insert(std::pair<std::string, int>(":TS:hh", (int)c));
        ss>>c; ss>>value; result->fmap.insert(std::pair<std::string, float>(":TS:Salinity", value));
        ss>>c; ss>>value; result->fmap.insert(std::pair<std::string, float>(":TS:Temperature", value));
        ss>>c; ss>>value; result->fmap.insert(std::pair<std::string, float>(":TS:SensorDepth", value));
        ss>>c; ss>>value; result->fmap.insert(std::pair<std::string, float>(":TS:SpeedOfSound", value));
        ss>>c; ss>>ival ; result->imap.insert(std::pair<std::string, int>(":TS:BIT", ival));

        if(_cds6_getTag(tag, ss) != 0) goto END_OF_SEARCH;
    }
    if(tag[1] == 'W' && tag[2] == 'I') {
        // Water-Mass, Instrument-Referenced Velocity Data
        int value;
        ss>>c; ss>>value; result->imap.insert(std::pair<std::string, int>(":WI:X", value));
        ss>>c; ss>>value; result->imap.insert(std::pair<std::string, int>(":WI:Y", value));
        ss>>c; ss>>value; result->imap.insert(std::pair<std::string, int>(":WI:Z", value));
        ss>>c; ss>>value; result->imap.insert(std::pair<std::string, int>(":WI:E", value));
        ss>>c; ss>>c;     result->cmap.insert(std::pair<std::string, char>(":WI:Status", c));

        if(_cds6_getTag(tag, ss) != 0) goto END_OF_SEARCH;
    }
    if(tag[1] == 'B' && tag[2] == 'I') {
        // Bottom-Track, Instrument-Referenced Velocity Data
        int value;
        ss>>c; ss>>value; result->imap.insert(std::pair<std::string, int>(":BI:X", value));
        ss>>c; ss>>value; result->imap.insert(std::pair<std::string, int>(":BI:Y", value));
        ss>>c; ss>>value; result->imap.insert(std::pair<std::string, int>(":BI:Z", value));
        ss>>c; ss>>value; result->imap.insert(std::pair<std::string, int>(":BI:E", value));
        ss>>c; ss>>c;     result->cmap.insert(std::pair<std::string, char>(":BI:Status", c));

        if(_cds6_getTag(tag, ss) != 0) goto END_OF_SEARCH;
    }
    if(tag[1] == 'W' && tag[2] == 'S') {
        // Water-Mass, Ship-Referenced Velocity Data
        int value;
        ss>>c; ss>>value; result->imap.insert(std::pair<std::string, int>(":WS:T", value));
        ss>>c; ss>>value; result->imap.insert(std::pair<std::string, int>(":WS:L", value));
        ss>>c; ss>>value; result->imap.insert(std::pair<std::string, int>(":WS:N", value));
        ss>>c; ss>>c;     result->cmap.insert(std::pair<std::string, char>(":WS:Status", c));

        if(_cds6_getTag(tag, ss) != 0) goto END_OF_SEARCH;
    }
    if(tag[1] == 'B' && tag[2] == 'S') {
        // Bottom-Track, Ship-Referenced Velocity Data
        int value;
        ss>>c; ss>>value; result->imap.insert(std::pair<std::string, int>(":BS:T", value));
        ss>>c; ss>>value; result->imap.insert(std::pair<std::string, int>(":BS:L", value));
        ss>>c; ss>>value; result->imap.insert(std::pair<std::string, int>(":BS:N", value));
        ss>>c; ss>>c;     result->cmap.insert(std::pair<std::string, char>(":BS:Status", c));

        if(_cds6_getTag(tag, ss) != 0) goto END_OF_SEARCH;
    }
    if(tag[1] == 'W' && tag[2] == 'E') {
        // Water-Mass, Earth-Referenced Velocity Data
        int value;
        ss>>c; ss>>value; result->imap.insert(std::pair<std::string, int>(":WE:E", value));
        ss>>c; ss>>value; result->imap.insert(std::pair<std::string, int>(":WE:N", value));
        ss>>c; ss>>value; result->imap.insert(std::pair<std::string, int>(":WE:U", value));
        ss>>c; ss>>c;     result->cmap.insert(std::pair<std::string, char>(":WE:Status", c));

        if(_cds6_getTag(tag, ss) != 0) goto END_OF_SEARCH;
    }
    if(tag[1] == 'B' && tag[2] == 'E') {
        // Bottom-Track, EarthReferenced Velocity Data
        int value;
        ss>>c; ss>>value; result->imap.insert(std::pair<std::string, int>(":BE:E", value));
        ss>>c; ss>>value; result->imap.insert(std::pair<std::string, int>(":BE:N", value));
        ss>>c; ss>>value; result->imap.insert(std::pair<std::string, int>(":BE:U", value));
        ss>>c; ss>>c;     result->cmap.insert(std::pair<std::string, char>(":BE:Status", c));

        if(_cds6_getTag(tag, ss) != 0) goto END_OF_SEARCH;
    }
    if(tag[1] == 'W' && tag[2] == 'D') {
        // Water-Mass, Earth-Referenced Distance Data
        float value;
        ss>>c; ss>>value; result->fmap.insert(std::pair<std::string, float>(":WD:E", value));
        ss>>c; ss>>value; result->fmap.insert(std::pair<std::string, float>(":WD:N", value));
        ss>>c; ss>>value; result->fmap.insert(std::pair<std::string, float>(":WD:U", value));
        ss>>c; ss>>value; result->fmap.insert(std::pair<std::string, float>(":WD:D", value));
        ss>>c; ss>>value; result->fmap.insert(std::pair<std::string, float>(":WD:T", value));

        if(_cds6_getTag(tag, ss) != 0) goto END_OF_SEARCH;
    }
    if(tag[1] == 'B' && tag[2] == 'D') {
        // Bottom-Track, Earth-Referenced Distance Data
        float value;
        ss>>c; ss>>value; result->fmap.insert(std::pair<std::string, float>(":BD:E", value));
        ss>>c; ss>>value; result->fmap.insert(std::pair<std::string, float>(":BD:N", value));
        ss>>c; ss>>value; result->fmap.insert(std::pair<std::string, float>(":BD:U", value));
        ss>>c; ss>>value; result->fmap.insert(std::pair<std::string, float>(":BD:D", value));
        ss>>c; ss>>value; result->fmap.insert(std::pair<std::string, float>(":BD:T", value));

        // because this should be the last tag
        // there is no need to 'find' the next tag
    }

END_OF_SEARCH:
    return result;
}

PTR(Drivers::IEnsemble) Drivers::DopplerData::composeEnsemble(
    Drivers::Doppler & dvl) {
    int type = (int)dvl.getOutputFormat();
    bool isBin = dvl.isBinaryOutput();
    // todo check: is continuous ping (continuous ping not yet supported)

    std::string ensemble;
    dvl.sendPingStart(ensemble);

    if(!isBin && type != 6) {
        ensemble = hexToBinary(ensemble);
    }

    switch(type) {
    case 0: return composeDataStream0(ensemble);
    case 4: return composeDataStream4(ensemble);
    case 5: return composeDataStream5(ensemble);
    case 6: return composeDataStream6(ensemble);
    default: std::cerr << "Illegal type!\n"; break;
    }
    return NULL;
}



void Drivers::DopplerData::printDataStream0(PTR(Drivers::DataStream0) data) {
    LOG("Header ID:                  "); LOGN(std::hex << (int)data->headerId);
    LOG("Ensemble ID:                "); LOGN(std::hex << (int)data->id);
    LOG("Ensemble Length:            "); LOGN(std::hex << data->length);
    LOG("Number of Types:            "); LOGN(std::hex << (int)data->typeCount);
    LOG("Checksum:                   "); LOGN(std::hex << data->checksum);

    // print fixed leader
    {
        LOGN("- FIXED LEADER VELOCITY -");
        LOG("ID:                         "); LOGN(std::hex << data->fixedLeader.id);
        LOG("Firmware Version:           "); LOGN(std::hex << (int)data->fixedLeader.firmwareVersion);
        LOG("Firmware Revision:          "); LOGN(std::hex << (int)data->fixedLeader.firmwareRevision);
        LOG("System Config:              "); LOGN(std::hex << data->fixedLeader.sysConfig);
        LOG("Real/Simulation Flag:       "); LOGN(std::hex << (int)data->fixedLeader.realSimFlag);
        LOG("Lag Length:                 "); LOGN(std::hex << (int)data->fixedLeader.lagLength);
        LOG("Number of Beams:            "); LOGN(std::hex << (int)data->fixedLeader.numOfBeams);
        LOG("Number of Cells:            "); LOGN(std::hex << (int)data->fixedLeader.numOfCells);
        LOG("Pings Per Ensemble:         "); LOGN(std::hex << data->fixedLeader.pingsPerEnsemble);
        LOG("Depth Cell Length:          "); LOGN(std::hex << data->fixedLeader.depthCellLength);
        LOG("Blank After Transmit:       "); LOGN(std::hex << data->fixedLeader.blankAfterTransmit);
        LOG("Profiling Mode:             "); LOGN(std::hex << (int)data->fixedLeader.profilingMode);
        LOG("Low Correlation Threshold:  "); LOGN(std::hex << (int)data->fixedLeader.lowCorrThresh);
        LOG("Number of Code Repetitions: "); LOGN(std::hex << (int)data->fixedLeader.numOfCodeReps);
        LOG("Max Velocity Error:         "); LOGN(std::hex << data->fixedLeader.errMaxVelocity);
        LOG("Time Between Pings Minutes: "); LOGN(std::hex << (int)data->fixedLeader.tppMinutes);
        LOG("Time Between Pings Seconds: "); LOGN(std::hex << (int)data->fixedLeader.tppSeconds);
        LOG("Time Between Pings Hundred: "); LOGN(std::hex << (int)data->fixedLeader.tppHundredths);
        LOG("Coordinate Transformation:  "); LOGN(std::hex << (int)data->fixedLeader.coorTransform);
        LOG("Heading Alignment:          "); LOGN(std::hex << data->fixedLeader.headingAlignment);
        LOG("Heading Bias:               "); LOGN(std::hex << data->fixedLeader.headingBias);
        LOG("Sensor Source:              "); LOGN(std::hex << (int)data->fixedLeader.sensorSource);
        LOG("Sensors Available:          "); LOGN(std::hex << (int)data->fixedLeader.sensorAvailable);
        LOG("Bin 1 Distance:             "); LOGN(std::hex << data->fixedLeader.bin1Dist);
        LOG("XMIT Pulse Length:          "); LOGN(std::hex << data->fixedLeader.xmitPulseLength);
        LOG("WP Referance Layer Average: "); LOGN(std::hex << data->fixedLeader.wpRefLayerAverage);
        LOG("False Target Threshold:     "); LOGN(std::hex << (int)data->fixedLeader.falseTargetThresh);
        LOG("Transmit Lag Distance:      "); LOGN(std::hex << data->fixedLeader.transmitLagDistance);
        LOG("System Bandwidth:           "); LOGN(std::hex << data->fixedLeader.sysBandwidth);
        LOG("System Serial Number:       "); LOGN(std::hex << data->fixedLeader.sysSerialNumber);
    }

    // print variable leader
    {
        LOGN("- VARIABLE LEADER VELOCITY -");
        LOG("ID:                         "); LOGN(std::hex << data->variableLeader.id);
        LOG("Ensemble Tag:               "); LOGN(std::hex << data->variableLeader.ensembleTag);
        LOG("Real Time Clock Year:       "); LOGN(std::hex << (int)data->variableLeader.rtcYear);
        LOG("Real Time Clock Month:      "); LOGN(std::hex << (int)data->variableLeader.rtcMonth);
        LOG("Real Time Clock Day:        "); LOGN(std::hex << (int)data->variableLeader.rtcDay);
        LOG("Real Time Clock Hour:       "); LOGN(std::hex << (int)data->variableLeader.rtcHour);
        LOG("Real Time Clock Minute:     "); LOGN(std::hex << (int)data->variableLeader.rtcMinute);
        LOG("Real Time Clock Second:     "); LOGN(std::hex << (int)data->variableLeader.rtcSecond);
        LOG("Real Time Clock Hundredths: "); LOGN(std::hex << (int)data->variableLeader.rtcHundredths);
        LOG("Ensemble Tag MSB:           "); LOGN(std::hex << (int)data->variableLeader.ensembleTagMsb);
        LOG("Bit Result:                 "); LOGN(std::hex << data->variableLeader.bitResult);
        LOG("Speed Of Sound:             "); LOGN(std::hex << data->variableLeader.speedOfSound);
        LOG("Transducer Depth:           "); LOGN(std::hex << data->variableLeader.transducerDepth);
        LOG("Heading:                    "); LOGN(std::hex << data->variableLeader.heading);
        LOG("Pitch:                      "); LOGN(std::hex << data->variableLeader.pitch);
        LOG("Roll:                       "); LOGN(std::hex << data->variableLeader.roll);
        LOG("Salinity:                   "); LOGN(std::hex << data->variableLeader.salinity);
        LOG("Temperature:                "); LOGN(std::hex << data->variableLeader.temperature);
        LOG("MPT Minutes:                "); LOGN(std::hex << (int)data->variableLeader.mptMinutes);
        LOG("MPT Seconds:                "); LOGN(std::hex << (int)data->variableLeader.mptSeconds);
        LOG("MPT Hundredths:             "); LOGN(std::hex << (int)data->variableLeader.mptHundredths);
        LOG("Heading Standard Deviation: "); LOGN(std::hex << (int)data->variableLeader.headingStdDev);
        LOG("Pitch Standard Deviation:   "); LOGN(std::hex << (int)data->variableLeader.pitchStdDev);
        LOG("Roll Standard Deviation:    "); LOGN(std::hex << (int)data->variableLeader.rollStdDev);
        LOG("ADC0:                       "); LOGN(std::hex << (int)data->variableLeader.adc0);
        LOG("ADC1:                       "); LOGN(std::hex << (int)data->variableLeader.adc1);
        LOG("ADC2:                       "); LOGN(std::hex << (int)data->variableLeader.adc2);
        LOG("ADC3:                       "); LOGN(std::hex << (int)data->variableLeader.adc3);
        LOG("ADC4:                       "); LOGN(std::hex << (int)data->variableLeader.adc4);
        LOG("ADC5:                       "); LOGN(std::hex << (int)data->variableLeader.adc5);
        LOG("ADC6:                       "); LOGN(std::hex << (int)data->variableLeader.adc6);
        LOG("ADC7:                       "); LOGN(std::hex << (int)data->variableLeader.adc7);
        LOG("Error Status:               "); LOGN(std::hex << data->variableLeader.errStatus);
        LOG("Pressure:                   "); LOGN(std::hex << data->variableLeader.pressure);
        LOG("Pressure Variance:          "); LOGN(std::hex << data->variableLeader.pressureVar);
    }

    // conditional printing:
    if(data->depthCells && data->fixedLeader.numOfCells) {
        for(int i = 0; i < data->fixedLeader.numOfCells; i++) {
            LOG("Velocity 1:                 "); LOGN(std::hex << data->depthCells->velocity[0]);
            LOG("Velocity 2:                 "); LOGN(std::hex << data->depthCells->velocity[1]);
            LOG("Velocity 3:                 "); LOGN(std::hex << data->depthCells->velocity[2]);
            LOG("Velocity 4:                 "); LOGN(std::hex << data->depthCells->velocity[3]);
            LOG("Correlation Magnitude 1:    "); LOGN(std::hex << data->depthCells->corrMagnitude[0]);
            LOG("Correlation Magnitude 2:    "); LOGN(std::hex << data->depthCells->corrMagnitude[1]);
            LOG("Correlation Magnitude 3:    "); LOGN(std::hex << data->depthCells->corrMagnitude[2]);
            LOG("Correlation Magnitude 4:    "); LOGN(std::hex << data->depthCells->corrMagnitude[3]);
            LOG("Echo Intensity 1:           "); LOGN(std::hex << data->depthCells->echoIntensity[0]);
            LOG("Echo Intensity 2:           "); LOGN(std::hex << data->depthCells->echoIntensity[1]);
            LOG("Echo Intensity 3:           "); LOGN(std::hex << data->depthCells->echoIntensity[2]);
            LOG("Echo Intensity 4:           "); LOGN(std::hex << data->depthCells->echoIntensity[3]);
            LOG("Percent Good 1:             "); LOGN(std::hex << data->depthCells->percentGood[0]);
            LOG("Percent Good 2:             "); LOGN(std::hex << data->depthCells->percentGood[1]);
            LOG("Percent Good 3:             "); LOGN(std::hex << data->depthCells->percentGood[2]);
            LOG("Percent Good 4:             "); LOGN(std::hex << data->depthCells->percentGood[3]);
        }
    }

    if(data->bottomTrack) {
        LOGN("- BOTTOM TRACK -");
        LOG("ID:                         "); LOGN(std::hex << data->bottomTrack->id);
        LOG("Pings Per Ensemble:         "); LOGN(std::hex << data->bottomTrack->pingsPerEnsemble);
        LOG("Delay Before Re-Aquire:     "); LOGN(std::hex << data->bottomTrack->delayBeforeReAquire);
        LOG("Minimum Correlation Mag:    "); LOGN(std::hex << (int)data->bottomTrack->corrMagMin);
        LOG("Evaluation Amplitude Min:   "); LOGN(std::hex << (int)data->bottomTrack->evalAmpMin);
        LOG("Minimum Percent Good:       "); LOGN(std::hex << (int)data->bottomTrack->percentGoodMin);
        LOG("Mode:                       "); LOGN(std::hex << (int)data->bottomTrack->mode);
        LOG("Maximum Error Velocity:     "); LOGN(std::hex << data->bottomTrack->errVelMax);
        LOG("Range 1:                    "); LOGN(std::hex << data->bottomTrack->range[0]);
        LOG("Range 2:                    "); LOGN(std::hex << data->bottomTrack->range[1]);
        LOG("Range 3:                    "); LOGN(std::hex << data->bottomTrack->range[2]);
        LOG("Range 4:                    "); LOGN(std::hex << data->bottomTrack->range[3]);
        LOG("Velocity 1:                 "); LOGN(std::hex << data->bottomTrack->velocity[0]);
        LOG("Velocity 2:                 "); LOGN(std::hex << data->bottomTrack->velocity[1]);
        LOG("Velocity 3:                 "); LOGN(std::hex << data->bottomTrack->velocity[2]);
        LOG("Velocity 4:                 "); LOGN(std::hex << data->bottomTrack->velocity[3]);
        LOG("Correlation 1:              "); LOGN(std::hex << (int)data->bottomTrack->correlation[0]);
        LOG("Correlation 2:              "); LOGN(std::hex << (int)data->bottomTrack->correlation[1]);
        LOG("Correlation 3:              "); LOGN(std::hex << (int)data->bottomTrack->correlation[2]);
        LOG("Correlation 4:              "); LOGN(std::hex << (int)data->bottomTrack->correlation[3]);
        LOG("Evaluation Amplitude 1:     "); LOGN(std::hex << (int)data->bottomTrack->evalAmp[0]);
        LOG("Evaluation Amplitude 2:     "); LOGN(std::hex << (int)data->bottomTrack->evalAmp[1]);
        LOG("Evaluation Amplitude 3:     "); LOGN(std::hex << (int)data->bottomTrack->evalAmp[2]);
        LOG("Evaluation Amplitude 4:     "); LOGN(std::hex << (int)data->bottomTrack->evalAmp[3]);
        LOG("Percent Good 1:             "); LOGN(std::hex << (int)data->bottomTrack->percentGood[0]);
        LOG("Percent Good 2:             "); LOGN(std::hex << (int)data->bottomTrack->percentGood[1]);
        LOG("Percent Good 3:             "); LOGN(std::hex << (int)data->bottomTrack->percentGood[2]);
        LOG("Percent Good 4:             "); LOGN(std::hex << (int)data->bottomTrack->percentGood[3]);
        LOG("Reference Layer Minimum:    "); LOGN(std::hex << data->bottomTrack->refLayerMin);
        LOG("Reference Layer Near:       "); LOGN(std::hex << data->bottomTrack->refLayerNear);
        LOG("Reference Layer Far:        "); LOGN(std::hex << data->bottomTrack->refLayerFar);
        LOG("Reference Layer Velocity 1: "); LOGN(std::hex << data->bottomTrack->refLayerVel[0]);
        LOG("Reference Layer Velocity 2: "); LOGN(std::hex << data->bottomTrack->refLayerVel[1]);
        LOG("Reference Layer Velocity 3: "); LOGN(std::hex << data->bottomTrack->refLayerVel[2]);
        LOG("Reference Layer Velocity 4: "); LOGN(std::hex << data->bottomTrack->refLayerVel[3]);
        LOG("Reference Correlation 1:    "); LOGN(std::hex << (int)data->bottomTrack->refCorrelation[0]);
        LOG("Reference Correlation 2:    "); LOGN(std::hex << (int)data->bottomTrack->refCorrelation[1]);
        LOG("Reference Correlation 3:    "); LOGN(std::hex << (int)data->bottomTrack->refCorrelation[2]);
        LOG("Reference Correlation 4:    "); LOGN(std::hex << (int)data->bottomTrack->refCorrelation[3]);
        LOG("Reference Intensity 1:      "); LOGN(std::hex << (int)data->bottomTrack->refInt[0]);
        LOG("Reference Intensity 2:      "); LOGN(std::hex << (int)data->bottomTrack->refInt[1]);
        LOG("Reference Intensity 3:      "); LOGN(std::hex << (int)data->bottomTrack->refInt[2]);
        LOG("Reference Intensity 4:      "); LOGN(std::hex << (int)data->bottomTrack->refInt[3]);
        LOG("Reference Percent Good 1:   "); LOGN(std::hex << (int)data->bottomTrack->refPercentGood[0]);
        LOG("Reference Percent Good 2:   "); LOGN(std::hex << (int)data->bottomTrack->refPercentGood[1]);
        LOG("Reference Percent Good 3:   "); LOGN(std::hex << (int)data->bottomTrack->refPercentGood[2]);
        LOG("Reference Percent Good 4:   "); LOGN(std::hex << (int)data->bottomTrack->refPercentGood[3]);
        LOG("Maximum Depth:              "); LOGN(std::hex << data->bottomTrack->maxDepth);
        LOG("RSSI Amplitude:             "); LOGN(std::hex << (int)data->bottomTrack->rssiAmp[0]);
        LOG("RSSI Amplitude:             "); LOGN(std::hex << (int)data->bottomTrack->rssiAmp[1]);
        LOG("RSSI Amplitude:             "); LOGN(std::hex << (int)data->bottomTrack->rssiAmp[2]);
        LOG("RSSI Amplitude:             "); LOGN(std::hex << (int)data->bottomTrack->rssiAmp[3]);
        LOG("Gain:                       "); LOGN(std::hex << (int)data->bottomTrack->gain);
        LOG("Range MSB 1:                "); LOGN(std::hex << (int)data->bottomTrack->rangeMSB[0]);
        LOG("Range MSB 2:                "); LOGN(std::hex << (int)data->bottomTrack->rangeMSB[1]);
        LOG("Range MSB 3:                "); LOGN(std::hex << (int)data->bottomTrack->rangeMSB[2]);
        LOG("Range MSB 4:                "); LOGN(std::hex << (int)data->bottomTrack->rangeMSB[3]);
    }
}

void Drivers::DopplerData::printDataStream4(PTR(Drivers::DataStream4) data) {
    LOG("ID (0x7D):                  "); LOGN(std::hex << (int)data->id);
    LOG("Structure:                  "); LOGN(std::hex << (int)data->structure);
    LOG("Length:                     "); LOGN(std::hex << (int)data->length);
    LOG("System Config:              "); LOGN(std::hex << (int)data->sysConfig);
    LOG("X Bottom Velocity:          "); LOGN(std::hex << (short)data->xVelBtm);
    LOG("Y Bottom Velocity:          "); LOGN(std::hex << (short)data->yVelBtm);
    LOG("Z Bottom Velocity:          "); LOGN(std::hex << (short)data->zVelBtm);
    LOG("Err Bottom Velocity:        "); LOGN(std::hex << (short)data->eVelBtm);
    LOG("Beam 1 Range Bottom:        "); LOGN(std::hex << (int)data->bm1RngBtm);
    LOG("Beam 2 Range Bottom:        "); LOGN(std::hex << (int)data->bm2RngBtm);
    LOG("Beam 3 Range Bottom:        "); LOGN(std::hex << (int)data->bm3RngBtm);
    LOG("Beam 4 Range Bottom:        "); LOGN(std::hex << (int)data->bm4RngBtm);
    LOG("Bottom Status:              "); LOGN(std::hex << (int)data->btmStatus);
    LOG("X Reference Velocity:       "); LOGN(std::hex << (short)data->xVelRef);
    LOG("Y Reference Velocity:       "); LOGN(std::hex << (short)data->yVelRef);
    LOG("Z Reference Velocity:       "); LOGN(std::hex << (short)data->zVelRef);
    LOG("Err Reference Velocity:     "); LOGN(std::hex << (short)data->eVelRef);
    LOG("Reference Layer Start:      "); LOGN(std::hex << (int)data->refLayerStart);
    LOG("Reference Layer End:        "); LOGN(std::hex << (int)data->refLayerEnd);
    LOG("Reference Layer Status:     "); LOGN(std::hex << (int)data->refLayerStatus);
    LOG("Time Of The First Ping H:   "); LOGN(std::hex << (int)data->tofpHour);
    LOG("Time Of The First Ping M:   "); LOGN(std::hex << (int)data->tofpMinute);
    LOG("Time Of The First Ping S:   "); LOGN(std::hex << (int)data->tofpSecond);
    LOG("Time Of The First Ping +:   "); LOGN(std::hex << (int)data->tofpHundredths);
    LOG("Bit Results:                "); LOGN(std::hex << (int)data->bitResults);
    LOG("Speed Of Sound:             "); LOGN(std::dec << data->speedOfSound);
    LOG("Temperature:                "); LOGN(std::dec << data->temperature);
    LOG("Checksum:                   "); LOGN(std::hex << (int)data->checksum);
}

void Drivers::DopplerData::printDataStream5(PTR(Drivers::DataStream5) data) {
    printDataStream4(data->velocity);
    LOG("Salinity:                   "); LOGN(std::hex << (int)data->salinity);
    LOG("Depth:                      "); LOGN(std::hex << (int)data->depth);
    LOG("Pitch:                      "); LOGN(std::dec << data->pitch);
    LOG("Roll:                       "); LOGN(std::dec << data->roll);
    LOG("Heading:                    "); LOGN(std::dec << data->heading);
    LOG("East Bottom Distance Good:  "); LOGN(std::hex << data->eastBtmDistGood);
    LOG("North Bottom Distance Good: "); LOGN(std::hex << data->northBtmDistGood);
    LOG("Up Bottom Distance Good:    "); LOGN(std::hex << data->upBtmDistGood);
    LOG("Error Bottom Distance Good: "); LOGN(std::hex << data->errBtmDistGood);
    LOG("East Reference Dist. Good:  "); LOGN(data->eastRefDistGood);
    LOG("North Reference Dist. Good: "); LOGN(data->northRefDistGood);
    LOG("Up Reference Dist. Good:    "); LOGN(data->upRefDistGood);
    LOG("Error Reference Dist. Good: "); LOGN(data->errRefDistGood);
}

void Drivers::DopplerData::printDataStream6(PTR(Drivers::DataStream6) data) {
    for(std::map<std::string,float>::iterator it = data->fmap.begin();
        it != data->fmap.end(); it++) {
        LOG(it->first); LOG(" "); LOGN(it->second);
    }
    for(std::map<std::string,int>::iterator it = data->imap.begin();
        it != data->imap.end(); it++) {
        LOG(it->first); LOG(" "); LOGN(it->second);
    }
    for(std::map<std::string,char>::iterator it = data->cmap.begin();
        it != data->cmap.end(); it++) {
        LOG(it->first); LOG(" "); LOGN(it->second);
    }
}

void Drivers::DopplerData::printEnsemble(PTR(Drivers::IEnsemble) package) {
    int type = package->getType();
    switch(type) {
    case 0: printDataStream0(CAST(DataStream0,package)); break;
    case 4: printDataStream4(CAST(DataStream4,package)); break;
    case 5: printDataStream5(CAST(DataStream5,package)); break;
    case 6: printDataStream6(CAST(DataStream6,package)); break;
    default: std::cerr << "Illegal type!\n"; break;
    }
}